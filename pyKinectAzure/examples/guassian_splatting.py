# import cv2
# import pykinect_azure as pykinect
# import numpy as np
# import time
# import socket


# # ==========================================================
# # Save Gaussian Splat Point Cloud to PLY
# # ==========================================================
# def save_gaussian_splat_ply(path, pts, colors, scales):
#     print(f"Saving {pts.shape[0]} splats → {path}")

#     with open(path, "w") as f:
#         f.write("ply\n")
#         f.write("format ascii 1.0\n")
#         f.write(f"element vertex {len(pts)}\n")
#         f.write("property float x\nproperty float y\nproperty float z\n")
#         f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
#         f.write("property float scale_x\nproperty float scale_y\nproperty float scale_z\n")
#         f.write("end_header\n")

#         for i in range(len(pts)):
#             x, y, z = pts[i]
#             r, g, b = (colors[i] * 255).astype(np.uint8)
#             sx, sy, sz = scales[i]

#             f.write(f"{x} {y} {z} {r} {g} {b} {sx} {sy} {sz}\n")

#     print("Saved.")


# # ==========================================================
# # Kinect Initialization
# # ==========================================================
# pykinect.initialize_libraries()

# device_config = pykinect.default_configuration
# device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
# device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

# device = pykinect.start_device(config=device_config)
# depth_mode = device_config.depth_mode

# # Field of view lookup
# FOV_TABLE = {
#     pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED: 75,
#     pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED: 75,
#     pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED: 120,
#     pykinect.K4A_DEPTH_MODE_WFOV_UNBINNED: 120,
# }

# fov = FOV_TABLE.get(depth_mode, None)
# print("Depth FOV:", fov)

# # Camera intrinsics
# calib = device.get_calibration(device_config.depth_mode, device_config.color_resolution)

# intr = calib.depth_params   # <— CORRECT FOR YOUR VERSION

# fx = intr.fx
# fy = intr.fy
# cx = intr.cx
# cy = intr.cy

# print("Kinect Depth Intrinsics:")
# print(" fx =", fx)
# print(" fy =", fy)
# print(" cx =", cx)
# print(" cy =", cy)

# time.sleep(1)


# # ==========================================================
# # Main Loop
# # ==========================================================
# while True:
#     capture = device.update()

#     ret_img, depth_color = capture.get_colored_depth_image()
#     ret_depth, depth = capture.get_depth_image()

#     if not ret_depth:
#         continue

#     # Depth → XYZ
#     h, w = depth.shape

#     xs, ys = np.meshgrid(np.arange(w), np.arange(h))
#     z = depth.astype(np.float32) / 1000.0  # meters

#     x = (xs - cx) * z / fx
#     y = (ys - cy) * z / fy

#     points = np.stack((x, y, z), axis=-1).reshape(-1, 3)

#     # Remove invalid z
#     mask = z > 0
#     mask = mask.reshape(-1)
#     points = points[mask]

#     # Color = normalized depth grayscale
#     color = (depth.reshape(-1, 1) / depth.max()).astype(np.float32)[mask]
#     color = np.repeat(color, 3, axis=1)

#     # Gaussian size per point
#     scale = np.full((points.shape[0], 3), 0.02, dtype=np.float32)

#     # Show depth image
#     if ret_img:
#         cv2.imshow("Depth Image", depth_color)

#     key = cv2.waitKey(1)

#     if key == ord('q'):
#         break

#     # Save only when user presses "s"
#     if key == ord('s'):
#         save_gaussian_splat_ply("frame_splat.ply", points, color, scale)


# cv2.destroyAllWindows()







#!/usr/bin/env python3
# gaussian_splat_capture.py
#
# Requirements:
#  - Python 3.10
#  - pykinect_azure installed and the Azure Kinect SDK on your system
#  - numpy, opencv-python (for visualization)
#
# Press 's' to save a PLY (frame_splat_color.ply). Press 'q' to quit.
import cv2
import pykinect_azure as pykinect
import numpy as np


# ==========================================================
# Save Gaussian Splat PLY
# ==========================================================
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


# ==========================================================
# Kinect Initialization
# ==========================================================
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


# ==========================================================
# Manual depth → color mapping (correct for your SDK)
# ==========================================================
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


# ==========================================================
# Main Loop
# ==========================================================
print("Press S to save splat, Q to quit.")

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

    # Extract RGB
    uv = mapped_uv[valid]
    colors = rgb[uv[:,1], uv[:,0]][:, ::-1]  # BGR → RGB

    # Splat size
    scales = np.full((xyz.shape[0], 3), 0.01, dtype=np.float32)

    key = cv2.waitKey(1)
    if key == ord('s'):
        save_gaussian_splat_ply("frame_splat_color.ply", xyz, colors, scales)

    if key == ord('q'):
        break

cv2.destroyAllWindows()

