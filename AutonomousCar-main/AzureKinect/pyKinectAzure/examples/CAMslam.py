# live_slam_trajectory.py
import time
import numpy as np
import cv2
import pykinect_azure as pykinect
import open3d as o3d

# -------- init camera --------
pykinect.initialize_libraries()

device_config = pykinect.default_configuration
device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
device_config.camera_fps = pykinect.K4A_FRAMES_PER_SECOND_30

device = pykinect.start_device(config=device_config)

# get calibration (intrinsics) and build camera matrix
calibration = device.get_calibration(
    pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED,
    pykinect.K4A_COLOR_RESOLUTION_720P
)

intrinsics = calibration._handle.color_camera_calibration.intrinsics.parameters.param

fx = intrinsics.fx
fy = intrinsics.fy
cx = intrinsics.cx
cy = intrinsics.cy

K = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]
])

print("Camera matrix:")
print(K)
print("Camera intrinsics fx,fy,cx,cy:", fx, fy, cx, cy)

# -------- ORB + matcher --------
orb = cv2.ORB_create(1500)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# -------- Open3D visualizer setup --------
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Live SLAM Trajectory", width=960, height=540)
pcd = o3d.geometry.PointCloud()            # sparse map points
traj = o3d.geometry.LineSet()              # camera trajectory as line set
traj_pts = []                              # trajectory points (list of [x,y,z])
traj_lines = []                            # line connections
traj_colors = []                           # line colors

vis.add_geometry(pcd)
vis.add_geometry(traj)

# helper to update visualizer
def update_vis(pcd_np, traj_pts_np, traj_lines_np):
    if pcd_np.shape[0] > 0:

        # convert numpy → Open3D point cloud
        pcd.points = o3d.utility.Vector3dVector(pcd_np)

        # -------- voxel downsample --------
        pcd_down = pcd.voxel_down_sample(voxel_size=0.02)

        # -------- remove outliers --------
        pcd_down, _ = pcd_down.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=2.0
        )

        # replace displayed cloud
        pcd.points = pcd_down.points

    else:
        pcd.points = o3d.utility.Vector3dVector(np.zeros((1,3)))

    pcd.paint_uniform_color([0.2,0.9,0.2])

    # update point cloud
    if pcd_np.shape[0] > 0:
        pcd.points = o3d.utility.Vector3dVector(pcd_np)
        vis.update_geometry(pcd)
        vis.reset_view_point(True)
        vis.poll_events()
        vis.update_renderer()
    else:
        pcd.points = o3d.utility.Vector3dVector(np.zeros((1,3)))
    pcd.paint_uniform_color([0.2, 0.9, 0.2])

    # update trajectory lines
    if traj_pts_np.shape[0] > 0:
        traj.points = o3d.utility.Vector3dVector(traj_pts_np)
        if len(traj_lines_np) > 0:
            traj.lines = o3d.utility.Vector2iVector(traj_lines_np)
            traj.colors = o3d.utility.Vector3dVector([[1.0, 0.0, 0.0] for _ in traj_lines_np])
        else:
            traj.lines = o3d.utility.Vector2iVector([[0,0]])  # dummy
            traj.colors = o3d.utility.Vector3dVector([[1.0,0.0,0.0]])
    else:
        traj.points = o3d.utility.Vector3dVector(np.zeros((1,3)))
        traj.lines = o3d.utility.Vector2iVector([[0,0]])
        traj.colors = o3d.utility.Vector3dVector([[1.0,0.0,0.0]])


#rendering
    render = vis.get_render_option()
    render.point_size = 4
    render.background_color = np.array([0,0,0])
    vis.update_geometry(pcd)
    vis.update_geometry(traj)
    vis.poll_events()
    vis.update_renderer()

# -------- SLAM state --------
prev_kp = None
prev_des = None
prev_color = None
prev_depth = None
prev_pts3d = None

camera_pose = np.eye(4)      # world <- camera (camera pose in world frame)
map_points = []              # list of 3D points (sparse)
traj_pts_world = []          # list of camera centers (world coords)

FRAME_SKIP = 0               # set >0 to skip frames (reduce CPU)
frame_idx = 0

try:
    t0 = time.time()
    while True:
        frame_idx += 1
        capture = device.update()
        ret_color, color_image = capture.get_color_image()
        ret_depth, depth_image = capture.get_transformed_depth_image() # depth in mm

        if not ret_color or not ret_depth:
            continue

        color = color_image.copy()
        depth = depth_image.copy()

        # convert to grayscale for ORB
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        # detect features
        kp, des = orb.detectAndCompute(gray, None)
        if des is None or len(kp) < 10:
            # show frames even if no features
            cv2.imshow("Color", color)
            if cv2.waitKey(1) == 27:
                break
            continue

        if prev_des is not None and frame_idx % (FRAME_SKIP + 1) == 0:
            # match descriptors
            matches = bf.match(prev_des, des)
            matches = sorted(matches, key=lambda x: x.distance)[:200]

            pts3d = []
            pts2d = []

            for m in matches:
                qidx = m.queryIdx  # prev
                tidx = m.trainIdx  # current

                u_prev, v_prev = prev_kp[qidx].pt
                u_cur,  v_cur  = kp[tidx].pt

                u_prev_i = int(round(u_prev))
                v_prev_i = int(round(v_prev))

                if u_prev_i < 0 or v_prev_i < 0 or v_prev_i >= prev_depth.shape[0] or u_prev_i >= prev_depth.shape[1]:
                    continue

                d_prev = prev_depth[v_prev_i, u_prev_i]

                if d_prev == 0 or d_prev > 4000:   # ignore >4 meters
                    continue

                z = float(d_prev) / 1000.0
                x = (u_prev - cx) * z / fx
                y = (v_prev - cy) * z / fy

                pts3d.append([x, y, z])
                pts2d.append([u_cur, v_cur])

            if len(pts3d) >= 6:
                pts3d = np.array(pts3d, dtype=np.float64)
                pts2d = np.array(pts2d, dtype=np.float64)

                # solvePnP (R: body->cam because points are in prev camera frame)
                success, rvec, tvec, inliers = cv2.solvePnPRansac(
                    pts3d, pts2d, K, None, flags=cv2.SOLVEPNP_ITERATIVE,
                    reprojectionError=8.0, iterationsCount=100, confidence=0.99
                )

                if success and inliers is not None and len(inliers) >= 6:
                    R, _ = cv2.Rodrigues(rvec)
                    t = tvec.reshape(3)

                    # build transform T_prev_to_cur
                    T_prev_to_cur = np.eye(4, dtype=np.float64)
                    T_prev_to_cur[:3, :3] = R
                    T_prev_to_cur[:3, 3] = t

                    # update world pose: pose_cur = pose_prev * T_prev_to_cur^{-1}
                    camera_pose = camera_pose @ np.linalg.inv(T_prev_to_cur)

                    # record camera center (world coords)
                    cam_center = camera_pose[:3, 3].copy()
                    traj_pts_world.append(cam_center)

                    # append pts3d transformed to world and keep sparse map
                    # transform pts3d from prev camera frame into world frame:
                    ones = np.ones((pts3d.shape[0],1))
                    pts3d_h = np.hstack([pts3d, ones])
                    pts_in_world = (camera_pose @ pts3d_h.T).T[:, :3]
                    map_points.extend(pts_in_world.tolist())

                    # trim map size
                    if len(map_points) > 50000:
                        map_points = map_points[-50000:]

        # draw keypoints on color
        vis_kp_img = cv2.drawKeypoints(color, kp, None, color=(0,255,0), flags=0)
        cv2.imshow("ORB Keypoints", vis_kp_img)
        if cv2.waitKey(1) == 27:
            break

        # update previous frame
        prev_kp = kp
        prev_des = des
        prev_color = color
        prev_depth = depth

        # update open3d window every N frames
        if frame_idx % 2 == 0:
            pcd_np = np.array(map_points) if len(map_points) > 0 else np.zeros((0,3))
            traj_pts_np = np.array(traj_pts_world) if len(traj_pts_world) > 0 else np.zeros((0,3))
            # build line list for trajectory: connect sequential points
            traj_lines = [[i, i+1] for i in range(len(traj_pts_np)-1)] if len(traj_pts_np) > 1 else []
            update_vis(pcd_np, traj_pts_np, traj_lines)

except KeyboardInterrupt:
    pass
finally:
    print("Shutting down...")
    device.stop()
    cv2.destroyAllWindows()
    vis.destroy_window()
