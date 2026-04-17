# live_tsdf_fusion.py
#
# Azure Kinect DK  →  Open3D TSDF Fusion  →  Mesh (.ply)
#
# Replaces ORB-SLAM sparse mapping with dense TSDF volumetric fusion.
# Pose estimation is kept via ORB + PnP (same as before), but every
# RGB-D frame is now integrated into a ScalableTSDFVolume so we get
# a dense, coloured mesh at the end instead of a sparse point cloud.
#
# Dependencies:
#   pip install open3d pykinect-azure opencv-python numpy
#
# Usage:
#   python live_tsdf_fusion.py
#   Press  ESC  in the "Color" window to stop and save mesh.
#   Press  Ctrl+C  in the terminal to force-quit.
#
# Output files (saved on exit):
#   tsdf_mesh.ply   – watertight coloured mesh (open with MeshLab / CloudCompare)
#   trajectory.ply  – camera path as a line set

import time
import numpy as np
import cv2
import pykinect_azure as pykinect
import open3d as o3d

# ============================================================
# TUNABLE PARAMETERS
# ============================================================
VOXEL_SIZE        = 0.02      # metres per voxel (2 cm → good indoor detail)
SDF_TRUNC         = 0.04      # truncation distance = 2× voxel size (metres)
DEPTH_MAX         = 3.0       # ignore depth beyond this (metres)
DEPTH_SCALE       = 1000.0    # Kinect depth is in mm → divide by 1000 for metres
FRAME_SKIP        = 0         # integrate every (FRAME_SKIP+1)-th frame (0 = every frame)
VIS_EVERY         = 5         # refresh Open3D window every N frames
MAX_MAP_PTS       = 50_000    # cap on sparse trajectory preview points
MESH_OUTPUT       = "tsdf_mesh.ply"
TRAJ_OUTPUT       = "trajectory.ply"
# ============================================================

# ─────────────────────────────────────────────
# 1.  Initialise Azure Kinect
# ─────────────────────────────────────────────
pykinect.initialize_libraries()

device_config = pykinect.default_configuration
device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
device_config.depth_mode       = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
device_config.camera_fps       = pykinect.K4A_FRAMES_PER_SECOND_30

device = pykinect.start_device(config=device_config)

# Pull calibrated intrinsics from the SDK
calibration = device.get_calibration(
    pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED,
    pykinect.K4A_COLOR_RESOLUTION_720P,
)
p = calibration._handle.color_camera_calibration.intrinsics.parameters.param
fx, fy, cx, cy = p.fx, p.fy, p.cx, p.cy

K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]], dtype=np.float64)

# Open3D intrinsic object (needed for TSDF integration)
color_w = calibration._handle.color_camera_calibration.resolution_width
color_h = calibration._handle.color_camera_calibration.resolution_height
o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(color_w, color_h, fx, fy, cx, cy)

print("Camera intrinsics  fx={:.2f}  fy={:.2f}  cx={:.2f}  cy={:.2f}".format(fx, fy, cx, cy))
print("Image size  {}×{}".format(color_w, color_h))

# ─────────────────────────────────────────────
# 2.  TSDF Volume
# ─────────────────────────────────────────────
# ScalableTSDFVolume grows on demand → no need to pre-declare a bounding box.
tsdf_volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=VOXEL_SIZE,
    sdf_trunc=SDF_TRUNC,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
)
print("TSDF volume ready  (voxel={} m  trunc={} m)".format(VOXEL_SIZE, SDF_TRUNC))

# ─────────────────────────────────────────────
# 3.  ORB feature tracker (for pose estimation)
# ─────────────────────────────────────────────
orb = cv2.ORB_create(5000)
bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# ─────────────────────────────────────────────
# 4.  Open3D live visualiser  (trajectory preview)
# ─────────────────────────────────────────────
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="TSDF Fusion – Trajectory Preview", width=960, height=540)

preview_pcd  = o3d.geometry.PointCloud()   # sparse preview cloud
traj_lineset = o3d.geometry.LineSet()      # camera path
vis.add_geometry(preview_pcd)
vis.add_geometry(traj_lineset)

render_opt = vis.get_render_option()
render_opt.point_size        = 3.0
render_opt.background_color  = np.array([0.05, 0.05, 0.05])

def refresh_visualiser(map_pts, traj_pts):
    """Push latest sparse preview + trajectory into the Open3D window."""
    # --- sparse preview cloud ---
    if len(map_pts) > 0:
        preview_pcd.points = o3d.utility.Vector3dVector(np.array(map_pts))
        preview_pcd.paint_uniform_color([0.2, 0.9, 0.2])
    else:
        preview_pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))

    # --- trajectory line set ---
    if len(traj_pts) > 1:
        pts_arr = np.array(traj_pts)
        lines   = [[i, i + 1] for i in range(len(pts_arr) - 1)]
        traj_lineset.points = o3d.utility.Vector3dVector(pts_arr)
        traj_lineset.lines  = o3d.utility.Vector2iVector(lines)
        traj_lineset.colors = o3d.utility.Vector3dVector(
            [[1.0, 0.2, 0.2]] * len(lines)
        )
    else:
        traj_lineset.points = o3d.utility.Vector3dVector(np.zeros((2, 3)))
        traj_lineset.lines  = o3d.utility.Vector2iVector([[0, 1]])
        traj_lineset.colors = o3d.utility.Vector3dVector([[1.0, 0.2, 0.2]])

    vis.update_geometry(preview_pcd)
    vis.update_geometry(traj_lineset)
    vis.poll_events()
    vis.update_renderer()

# ─────────────────────────────────────────────
# 5.  SLAM state
# ─────────────────────────────────────────────
prev_kp    = None
prev_des   = None
prev_depth = None

camera_pose    = np.eye(4, dtype=np.float64)   # world ← camera (4×4)
traj_pts_world = []                            # camera centres in world frame
map_points     = []                            # sparse preview points
frames_fused   = 0
frame_idx      = 0

# ─────────────────────────────────────────────
# 6.  Main loop
# ─────────────────────────────────────────────
print("\nStreaming – press ESC in the colour window to stop.\n")

try:
    t0 = time.time()

    while True:
        frame_idx += 1

        # ── grab frame ──────────────────────────────────────────────────────
        capture = device.update()
        ret_c, color_image = capture.get_color_image()
        ret_d, depth_image = capture.get_transformed_depth_image()  # depth in mm, aligned to colour

        if not ret_c or not ret_d:
            continue

        color = color_image.copy()   # BGR uint8  H×W×3
        depth = depth_image.copy()   # uint16     H×W   (mm)

        # ── ORB keypoints ────────────────────────────────────────────────────
        gray   = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        kp, des = orb.detectAndCompute(gray, None)

        # ── pose estimation via ORB + PnP ────────────────────────────────────
        pose_updated = False

        if (des is not None and len(kp) >= 10
                and prev_des is not None
                and frame_idx % (FRAME_SKIP + 1) == 0):

            matches = bf.match(prev_des, des)
            matches = sorted(matches, key=lambda m: m.distance)[:200]

            pts3d, pts2d = [], []
            for m in matches:
                u_p, v_p = prev_kp[m.queryIdx].pt
                u_c, v_c = kp[m.trainIdx].pt

                ui, vi = int(round(u_p)), int(round(v_p))
                if not (0 <= vi < prev_depth.shape[0] and 0 <= ui < prev_depth.shape[1]):
                    continue

                d_mm = prev_depth[vi, ui]
                if d_mm == 0 or d_mm > DEPTH_MAX * 1000:
                    continue

                z = d_mm / 1000.0
                pts3d.append([(u_p - cx) * z / fx,
                               (v_p - cy) * z / fy,
                               z])
                pts2d.append([u_c, v_c])

            if len(pts3d) >= 6:
                pts3d = np.array(pts3d, dtype=np.float64)
                pts2d = np.array(pts2d, dtype=np.float64)

                success, rvec, tvec, inliers = cv2.solvePnPRansac(
                    pts3d, pts2d, K, None,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                    reprojectionError=8.0,
                    iterationsCount=100,
                    confidence=0.99,
                )

                if success and inliers is not None and len(inliers) >= 6:
                    R, _ = cv2.Rodrigues(rvec)
                    t    = tvec.reshape(3)

                    T_prev_to_cur = np.eye(4, dtype=np.float64)
                    T_prev_to_cur[:3, :3] = R
                    T_prev_to_cur[:3,  3] = t

                    # accumulate world pose
                    camera_pose = camera_pose @ np.linalg.inv(T_prev_to_cur)
                    pose_updated = True

                    # record trajectory
                    traj_pts_world.append(camera_pose[:3, 3].copy())

                    # sparse preview points (world frame)
                    ones = np.ones((pts3d.shape[0], 1))
                    pts_w = (camera_pose @ np.hstack([pts3d, ones]).T).T[:, :3]
                    map_points.extend(pts_w.tolist())
                    if len(map_points) > MAX_MAP_PTS:
                        map_points = map_points[-MAX_MAP_PTS:]

        # ── TSDF integration ─────────────────────────────────────────────────
        # Integrate on every frame that has a valid (or identity) pose.
        # For the very first frame pose_updated is False but camera_pose = I,
        # which is a perfectly valid starting pose.
        depth_f32 = depth.astype(np.float32) / DEPTH_SCALE   # metres, float32

        # mask out pixels beyond DEPTH_MAX (set to 0 → ignored by TSDF)
        depth_f32[depth_f32 > DEPTH_MAX] = 0.0

        # build Open3D RGB-D image
        color_rgb  = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        o3d_color  = o3d.geometry.Image(color_rgb)
        o3d_depth  = o3d.geometry.Image(depth_f32)
        rgbd       = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color,
            o3d_depth,
            depth_scale=1.0,          # already in metres
            depth_trunc=DEPTH_MAX,
            convert_rgb_to_intensity=False,
        )

        # extrinsic = world ← camera  →  TSDF needs camera ← world (inverse)
        extrinsic = np.linalg.inv(camera_pose)

        tsdf_volume.integrate(rgbd, o3d_intrinsic, extrinsic)
        frames_fused += 1

        # ── OpenCV display ───────────────────────────────────────────────────
        vis_img = cv2.drawKeypoints(color, kp, None, color=(0, 255, 0), flags=0)
        status  = "Fused: {}  Pose OK: {}".format(frames_fused, pose_updated)
        cv2.putText(vis_img, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        cv2.imshow("Color + ORB Keypoints", vis_img)
        if cv2.waitKey(1) == 27:   # ESC
            break

        # update previous frame data
        prev_kp    = kp
        prev_des   = des
        prev_depth = depth

        # ── Open3D trajectory preview ────────────────────────────────────────
        if frame_idx % VIS_EVERY == 0:
            refresh_visualiser(map_points, traj_pts_world)

except KeyboardInterrupt:
    pass

# ─────────────────────────────────────────────
# 7.  Extract mesh and save
# ─────────────────────────────────────────────
finally:
    elapsed = time.time() - t0
    print("\n--- Capture finished ---")
    print("Frames captured : {}".format(frame_idx))
    print("Frames fused    : {}".format(frames_fused))
    print("Elapsed         : {:.1f} s  ({:.1f} fps)".format(
        elapsed, frame_idx / max(elapsed, 1)))

    print("\nExtracting mesh from TSDF volume …")
    mesh = tsdf_volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    # optional: clean up the mesh
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    print("Saving mesh  →  {}".format(MESH_OUTPUT))
    o3d.io.write_triangle_mesh(MESH_OUTPUT, mesh)

    # also save the camera trajectory as a line-set PLY
    if len(traj_pts_world) > 1:
        traj_out = o3d.geometry.LineSet()
        pts_arr  = np.array(traj_pts_world)
        lines    = [[i, i + 1] for i in range(len(pts_arr) - 1)]
        traj_out.points = o3d.utility.Vector3dVector(pts_arr)
        traj_out.lines  = o3d.utility.Vector2iVector(lines)
        traj_out.colors = o3d.utility.Vector3dVector(
            [[1.0, 0.0, 0.0]] * len(lines)
        )
        print("Saving trajectory  →  {}".format(TRAJ_OUTPUT))
        o3d.io.write_line_set(TRAJ_OUTPUT, traj_out)

    print("Shutting down …")
    vis.destroy_window()
    device.close()
    cv2.destroyAllWindows()

    print("\nDone.  Open {} in MeshLab or CloudCompare to inspect the result.".format(MESH_OUTPUT))
