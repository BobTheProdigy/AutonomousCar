import cv2
import pykinect_azure as pykinect
import numpy as np
import open3d as o3d


# Save Gaussian Splat PLY
def save_gaussian_splat_ply(path, pts, colors, scales):
    print(f"Saving {pts.shape[0]} splats → {path}")

    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("property float scale_x\nproperty float scale_y\nproperty float scale_z\n")
        f.write("end_header\n")

        for i in range(len(pts)):
            x, y, z = pts[i]
            r, g, b = colors[i]
            sx, sy, sz = scales[i]
            f.write(f"{x} {y} {z} {r} {g} {b} {sx} {sy} {sz}\n")

    print("Saved.")


# Kinect Initialization
pykinect.initialize_libraries()

device_config = pykinect.default_configuration
device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
device_config.depth_mode       = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
device_config.camera_fps       = pykinect.K4A_FRAMES_PER_SECOND_30

device = pykinect.start_device(config=device_config)

calib = device.get_calibration(device_config.depth_mode,
                               device_config.color_resolution)

intr = calib.depth_params
fx, fy, cx, cy = intr.fx, intr.fy, intr.cx, intr.cy
print("Depth intrinsics:", fx, fy, cx, cy)


# Manual depth -> color mapping (correct for your SDK)
def map_depth_to_color(depth, calib, rgb):
    h, w = depth.shape
    N = h * w

    mapped = np.zeros((N, 2), dtype=np.int32)
    valid  = np.zeros(N,   dtype=bool)

    idx = 0
    for yy in range(h):
        for xx in range(w):
            d = depth[yy, xx]
            if d == 0:
                idx += 1
                continue

            # Depth pixel → 2D
            p2d = pykinect.k4a_float2_t()
            p2d.xy.x = float(xx)
            p2d.xy.y = float(yy)

            # Depth pixel → 3D (correct 4-arg version)
            p3d = calib.convert_2d_to_3d(
                p2d,
                float(d),
                pykinect.K4A_CALIBRATION_TYPE_DEPTH,
                pykinect.K4A_CALIBRATION_TYPE_DEPTH
            )

            # 3D depth point → color pixel (correct 3-arg version)
            p2d_color = calib.convert_3d_to_2d(
                p3d,
                pykinect.K4A_CALIBRATION_TYPE_DEPTH,
                pykinect.K4A_CALIBRATION_TYPE_COLOR
            )

            cx2 = int(p2d_color.xy.x)
            cy2 = int(p2d_color.xy.y)

            if 0 <= cx2 < rgb.shape[1] and 0 <= cy2 < rgb.shape[0]:
                mapped[idx] = (cx2, cy2)
                valid[idx] = True

            idx += 1

    return mapped, valid


# ICP algorithm
def apply_transform(T, pts):
    pts_h = np.hstack([pts, np.ones((len(pts), 1))])
    pts_world = (T @ pts_h.T).T[:, :3]
    return pts_world


# Main Loop
print("Press S to save splat, Q to quit.")
prev_pc = None

T_world_cam = np.eye(4)
accumulated_splats = [] # list of tuples (xyz_world, colors, scales)




while True:
    

    capture = device.update()

    ret_depth_vis, depth_vis = capture.get_colored_depth_image()
    ret_depth, depth         = capture.get_depth_image()
    ret_rgb, rgb             = capture.get_color_image()

    if not ret_depth or not ret_rgb:
        continue

    h, w = depth.shape

    # Show previews
    if ret_depth_vis:
        cv2.imshow("Depth", depth_vis)
    cv2.imshow("RGB", rgb)

    # Map depth → color
    mapped_uv, valid = map_depth_to_color(depth, calib, rgb)

    # Build XYZ
    xs, ys = np.meshgrid(np.arange(w), np.arange(h))
    z = depth.astype(np.float32) / 1000.0
    x = (xs - cx) * z / fx
    y = (ys - cy) * z / fy

    xyz_all = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    xyz = xyz_all[valid]

    #set up the point cloud the fram makes
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz)


    # FML ICP algorithm
    if prev_pc is not None:
        # sike open3d has it 
        reg = o3d.pipelines.registration.registration_icp(
            pc, prev_pc,
            max_correspondence_distance=0.15,  # 5 cm, adjust
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        T = reg.transformation 
        # Update global world transform

        T_world_cam = T_world_cam @ T

    # Transform current points to world frame
    xyz_world = apply_transform(T_world_cam, xyz)

    


    # Extract RGB
    uv = mapped_uv[valid]
    colors = rgb[uv[:,1], uv[:,0]][:, ::-1]  # BGR → RGB

    # Splat size
    scales = np.full((xyz.shape[0], 3), 0.01, dtype=np.float32)


    accumulated_splats.append((xyz_world, colors, scales))
    key = cv2.waitKey(1)
    if key == ord('s'):
        all_xyz = np.vstack([a[0] for a in accumulated_splats])
        all_colors = np.vstack([a[1] for a in accumulated_splats])
        all_scales = np.vstack([a[2] for a in accumulated_splats])
        save_gaussian_splat_ply("accumulated_splats.ply", all_xyz, all_colors, all_scales)

    if key == ord('q'):
        break

    prev_pc = pc

cv2.destroyAllWindows()

