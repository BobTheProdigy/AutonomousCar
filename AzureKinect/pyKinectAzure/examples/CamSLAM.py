

import time
import copy
import os
from collections import deque

import numpy as np
import cv2
import open3d as o3d
import pykinect_azure as pykinect

# -----------------------
# Parameters (tune these)
# -----------------------
USE_WFOV_UNBINNED = True      # set False for WFOV_2X2BINNED (faster)
FPS = 15 if USE_WFOV_UNBINNED else 30

KEYFRAME_EVERY_N = 5         # store a keyframe every N frames
KEYFRAME_SEARCH_RADIUS = 30  # how many previous keyframes to try for loop closure
TSDF_VOXEL_LENGTH = 0.02     # meters (2 cm). reduce for more detail (memory ↑)
TSDF_SDF_TRUNC = 0.04        # truncation (m)
DEPTH_TRUNC = 3.0            # max depth used by Open3D integration (m)
ODOMETRY_MAX_DISTANCE = 1.0  # max distance between frames (m) to accept odometry
MIN_LOOP_SCORE = 1e-3        # low threshold for adding loop constraints (info matrix check)
KEYFRAME_MIN_TIME = 0.1      # seconds between saved keyframes (helps avoid near duplicates)

# -----------------------
# Initialize Kinect
# -----------------------
pykinect.initialize_libraries()
device_config = pykinect.default_configuration
device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
# device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
# device_config.camera_fps = pykinect.K4A_FRAMES_PER_SECOND_15 if USE_WFOV_UNBINNED else pykinect.K4A_FRAMES_PER_SECOND_30

device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
device_config.camera_fps = pykinect.K4A_FRAMES_PER_SECOND_30


print("Starting device with depth_mode:", device_config.depth_mode, "fps:", device_config.camera_fps)
device = pykinect.start_device(config=device_config)

calib = device.get_calibration(device_config.depth_mode, device_config.color_resolution)
intr = calib.depth_params
fx, fy, cx, cy = intr.fx, intr.fy, intr.cx, intr.cy
print("Intrinsics fx,fy,cx,cy:", fx, fy, cx, cy)

# Try to get transformation helper (fast)
transformation = None
try:
    transformation = device.create_transformation()
    print("Using device.create_transformation()")
except Exception:
    transformation = getattr(device, "transformation", None)
    if transformation is not None:
        print("Using device.transformation")
    else:
        trans_class = getattr(pykinect, "Transformation", None)
        if trans_class is not None:
            try:
                transformation = trans_class(calib)
                print("Using pykinect.Transformation(calib)")
            except Exception:
                transformation = None

if transformation is None:
    print("WARNING: No fast transformation helper. Aligned color will use resize/nearest fallback.")

# -----------------------
# Open3D intrinsics
# -----------------------
# We'll use depth resolution reported by capture (but assume WFOV_UNBINNED=1024)
# We'll set width/height dynamically once first depth frame is read.
camera_intrinsic = None

# -----------------------
# TSDF volume (live)
# -----------------------
def make_tsdf_volume(voxel_length=TSDF_VOXEL_LENGTH, sdf_trunc=TSDF_SDF_TRUNC):
    return o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )

tsdf_volume = make_tsdf_volume()

# -----------------------
# Pose graph for keyframes
# -----------------------
pose_graph = o3d.pipelines.registration.PoseGraph()
# node 0 = origin
pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))

# store keyframe data
keyframes = []  # list of dicts: { 'rgbd':rgbd, 'pose':4x4, 'index':i, 'timestamp':t, 'color':..., 'depth':... }

# store all frames (optional if you want to re-integrate)
frames = []     # list of dicts: { 'rgbd', 'timestamp' }

# utility: create Open3D RGBD image from color+depth numpy arrays
def make_o3d_rgbd(color, depth):
    # ---- Fix 1: convert BGR→RGB properly ----
    color_rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    color_rgb = np.ascontiguousarray(color_rgb)

    # ---- Fix 2: make depth contiguous ----
    depth = np.ascontiguousarray(depth)

    # ---- Convert to Open3D Images ----
    color_o3d = o3d.geometry.Image(color_rgb)
    depth_o3d = o3d.geometry.Image(depth)

    # ---- Create RGBD image ----
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1000.0,       # Kinect depth is in millimeters
        depth_trunc=4.0,          # max range
        convert_rgb_to_intensity=False
    )
    return rgbd


# incremental odometry state
prev_rgbd = None
prev_pose = np.eye(4)
frame_idx = 0
last_keyframe_time = -9999.0
start_time = time.time()

# visualization window (optional)
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Live TSDF", width=1280, height=720)
geom_added = False

print("Starting main SLAM loop. Press 's' to save mesh/pcd. Press 'q' to quit.")

try:
    while True:
        capture = device.update()
        # acquire frames
        ok_color, color = capture.get_color_image()
        ok_depth, depth = capture.get_depth_image()

        if not ok_color or not ok_depth:
            continue

        # get aligned color in depth space (fast path)
        aligned_color = None
        if transformation is not None:
            # try a few common API signatures; wrappers differ
            try:
                fn = getattr(transformation, "color_image_to_depth_camera", None)
                if callable(fn):
                    res = fn(color, depth)
                    if isinstance(res, tuple):
                        ok, aligned_color = res if len(res) == 2 else (True, res[0])
                    elif isinstance(res, np.ndarray):
                        aligned_color = res
                else:
                    # try depth_image_to_color_camera
                    fn2 = getattr(transformation, "depth_image_to_color_camera", None)
                    if callable(fn2):
                        res = fn2(depth, color)
                        if isinstance(res, tuple):
                            ok, aligned_color = res if len(res) == 2 else (True, res[0])
                        elif isinstance(res, np.ndarray):
                            aligned_color = res
            except Exception:
                aligned_color = None

        if aligned_color is None:
            # fallback: naive resize (not ideal but keeps pipeline running)
            aligned_color = cv2.resize(color, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_LINEAR)

        # prepare Open3D intrinsics once
        if camera_intrinsic is None:
            h, w = depth.shape
            camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width=w, height=h, fx=fx, fy=fy, cx=cx, cy=cy)
            print("Set camera_intrinsic:", w, h, fx, fy, cx, cy)

        # create rgbd
        rgbd = make_o3d_rgbd(aligned_color, depth)

        # save frame (for later re-integration if needed)
        frames.append({'rgbd': rgbd, 'timestamp': time.time(), 'index': frame_idx, 'color': aligned_color.copy(), 'depth': depth.copy()})

        # frame-to-frame odometry (estimate transform T_curr_from_prev)
        success = False
        if prev_rgbd is None:
            T_curr_from_prev = np.eye(4)
            success = True
        else:
            option = o3d.pipelines.odometry.OdometryOption()
            odo_method = o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm()
            success, T_curr_from_prev, info = o3d.pipelines.odometry.compute_rgbd_odometry(
                rgbd, prev_rgbd, camera_intrinsic, np.identity(4), odo_method, option)

        if not success:
            # if odometry failed, we keep previous pose (could try feature-based fallback)
            print(f"[Frame {frame_idx}] Odometry failed; using previous pose")
            T_curr_from_prev = np.eye(4)

        # update global pose
        curr_pose = prev_pose @ np.linalg.inv(T_curr_from_prev) if prev_rgbd is not None else prev_pose.copy()
        # note: depending on compute_rgbd_odometry conventions you may need transpose or invert; test and adjust if clouds flip
        prev_pose = curr_pose.copy()

        # integrate to TSDF immediately for live view
        tsdf_volume.integrate(rgbd, camera_intrinsic, np.linalg.inv(curr_pose))

        # visualize periodically: extract pointcloud (cheap) or mesh (expensive)
        if frame_idx % 10 == 0:
            pcd_live = tsdf_volume.extract_point_cloud()
            pcd_live.paint_uniform_color([0.6, 0.6, 0.6])
            if not geom_added:
                vis.add_geometry(pcd_live)
                geom_added = True
            else:
                vis.clear_geometries()
                vis.add_geometry(pcd_live)
            vis.poll_events()
            vis.update_renderer()

        # Keyframe handling
        now = time.time()
        if (frame_idx % KEYFRAME_EVERY_N == 0) and (now - last_keyframe_time > KEYFRAME_MIN_TIME):
            # add keyframe to list with current estimated pose
            kf = {'rgbd': rgbd, 'pose': curr_pose.copy(), 'index': frame_idx, 'timestamp': now, 'color': aligned_color.copy(), 'depth': depth.copy()}
            keyframes.append(kf)
            # add pose graph node (pose is inverse of camera pose? Open3D PoseGraph uses node pose as camera->world; we use camera->world = curr_pose)
            pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(curr_pose)))
            last_keyframe_time = now
            print(f"Added keyframe {len(keyframes)-1} (frame {frame_idx})")

            # Try loop closure: attempt odometry between this keyframe and a window of previous keyframes
            start_search = max(0, len(keyframes) - 1 - KEYFRAME_SEARCH_RADIUS)
            for j in range(start_search, len(keyframes)-1):
                kf_prev = keyframes[j]
                # attempt RGBD odometry from current keyframe to kf_prev
                option = o3d.pipelines.odometry.OdometryOption()
                ok_loop, trans_loop, info_loop = o3d.pipelines.odometry.compute_rgbd_odometry(
                    kf['rgbd'], kf_prev['rgbd'], camera_intrinsic, np.identity(4),
                    o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option)
                if not ok_loop:
                    continue
                # check translational magnitude to avoid trivial neighbors
                tvec = trans_loop[:3, 3]
                dist = np.linalg.norm(tvec)
                if dist < 0.1:  # too close (likely same viewpoint)
                    continue
                # add loop edge between node j and last node
                node_idx_cur = len(pose_graph.nodes) - 1  # just appended
                node_idx_prev = j
                information = np.identity(6)  # basic information; can use 'info_loop' to build a better matrix
                pose_graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(node_idx_prev, node_idx_cur, trans_loop, information, uncertain=False))
                print(f"Added loop constraint between keyframe {j} and {len(keyframes)-1}, dist={dist:.2f}")
                # optionally break after first strong match; continue to add multiple matches for robustness

            # After adding edges we can attempt to optimize if we have >1 edges
            if len(pose_graph.edges) > 0:
                print("Optimizing pose graph with {} nodes and {} edges...".format(len(pose_graph.nodes), len(pose_graph.edges)))
                option = o3d.pipelines.registration.GlobalOptimizationOption(max_correspondence_distance=0.05,
                                                                              edge_prune_threshold=0.25,
                                                                              reference_node=0)
                o3d.pipelines.registration.global_optimization(
                    pose_graph,
                    o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                    o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                    option)
                print("Pose graph optimized. Rebuilding TSDF from keyframes with optimized poses...")

                # rebuild TSDF from keyframes (to remove drift)
                tsdf_volume = make_tsdf_volume()  # fresh
                # integrate every keyframe using optimized node poses
                for node_id, kf in enumerate(keyframes):
                    node_pose = np.linalg.inv(pose_graph.nodes[node_id].pose)  # node.pose is camera->world?
                    tsdf_volume.integrate(kf['rgbd'], camera_intrinsic, np.linalg.inv(node_pose))
                print("TSDF rebuilt after optimization.")

        # advance
        prev_rgbd = rgbd
        frame_idx += 1

        # controls (also allow keyboard detection via OpenCV window)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('s'):
            print("Saving mesh and pointcloud...")
            mesh = tsdf_volume.extract_triangle_mesh()
            mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh("scene_mesh.ply", mesh)
            pcd_out = tsdf_volume.extract_point_cloud()
            o3d.io.write_point_cloud("scene_pointcloud.ply", pcd_out)
            print("Saved scene_mesh.ply and scene_pointcloud.ply")
        if k == ord('q'):
            break

finally:
    print("Cleaning up...")
    vis.destroy_window()
    cv2.destroyAllWindows()
    print("Done.")
