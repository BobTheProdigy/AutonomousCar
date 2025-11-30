# import cv2
# import pykinect_azure as pykinect
# import numpy as np
# import serial
# import time
# import socket

# # Initialize the library
# pykinect.initialize_libraries()

# # Camera configuration
# device_config = pykinect.default_configuration
# device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
# device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

# UDP_IP = "192.168.43.1"
# UDP_PORT = 5005
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# # Start device
# device = pykinect.start_device(config=device_config)

# # FOV lookup
# FOV_TABLE = {
#     pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED: 75,
#     pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED: 75,
#     pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED: 120,
#     pykinect.K4A_DEPTH_MODE_WFOV_UNBINNED: 120,
# }
# fov = FOV_TABLE.get(device_config.depth_mode, None)
# print("fov:" + str(fov))

# # Serial setup
# # ser = serial.Serial('COM3', 115200, timeout=1)  
# time.sleep(2)

# cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)

# while True:
#     capture = device.update()

#     ret_color, depth_image = capture.get_colored_depth_image()
#     ret_depth, depth = capture.get_depth_image()

#     if not ret_depth:
#         continue

#     # ---------------- OLD TURN ANGLE METHOD ----------------
#     column_avg = np.mean(depth, axis=0)
#     closestColumn = np.argmax(column_avg)
#     rowsAway = 256 - closestColumn
#     turnAngle = int((fov / 2) * (rowsAway / 256))
#     print("old turn angle:" + str(turnAngle))

#     # ============================================================
#     # 👉 NEW: NEAREST OBJECT DETECTION USING CONNECTED COMPONENTS
#     # ============================================================

#     # 1. Find threshold for “nearest" pixels (lowest 5% depth)
#     cutoff = np.percentile(depth, 5)

#     # 2. Binary mask of only near pixels
#     mask = (depth < cutoff).astype(np.uint8)

#     # 3. Connected components
#     num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

#     if num_labels > 1:
#         # Ignore background label 0
#         areas = stats[1:, cv2.CC_STAT_AREA]
#         largest_label = 1 + np.argmax(areas)

#         cx, cy = centroids[largest_label]
#         cx = int(cx)
#         cy = int(cy)

#         object_distance = depth[cy, cx]

#         print("NEAREST OBJECT → centroid:", (cx, cy), " distance:", object_distance)

#         # Draw centroid on depth image
#         cv2.circle(depth_image, (cx, cy), 6, (0, 0, 255), -1)

#     else:
#         print("No object detected")

#     # Send UDP angle
#     sock.sendto(str(float(turnAngle)).encode(), (UDP_IP, UDP_PORT))

#     # Show image
#     cv2.imshow('Depth Image', depth_image)
#     # time.sleep(0.5)

#     # Quit
#     if cv2.waitKey(1) == ord('q'):
#         break









import cv2
import pykinect_azure as pykinect
import numpy as np
import serial
import time
import socket
from sklearn.cluster import KMeans
import seaborn as sns
from sklearn.preprocessing import StandardScaler

# Initialize the library, if the library is not found, add the library path as argument
pykinect.initialize_libraries()

# Modify camera configuration
device_config = pykinect.default_configuration
device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

UDP_IP = "192.168.43.1"     # Control Hub default IP
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Start device
device = pykinect.start_device(config=device_config)

depth_mode = device_config.depth_mode

# Map depth modes to FOV (degrees)
FOV_TABLE = {
    pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED: 75,
    pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED: 75,
    pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED: 120,
    pykinect.K4A_DEPTH_MODE_WFOV_UNBINNED: 120,
}

fov = FOV_TABLE.get(depth_mode, None)
print("fov:" + str(fov))

##
## SERIAL READ FOR REV CONTROL HUB
##
# Replace COM3 with the port your USB-serial is on
# ser = serial.Serial('COM3', 115200, timeout=1)  
time.sleep(2)  # wait for connection to establish


cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)


totalMaxDepth = 0

while True:
    # Get capture
    capture = device.update()
    
    # Get the color depth image from the capture
    ret, depth_image = capture.get_colored_depth_image()
    ret2, depth = capture.get_depth_image() # depth is a np array, therefore can do classification of points, cut out undesired columns, etc!!!!
    # size is 512 x 512
    # work on getting angle measurements of pixels, classifying average pixel distances (based on column average w/ numpy), and finding angle heading of most open spot
    
    column_avg = np.mean(depth, axis=0)
    closestColumn = np.argmax(column_avg)
    # print("closest column" + str(closestColumn))
    rowsAway = 256 - closestColumn # turn left is negative
    turnAngle = int((fov/2) * (rowsAway/256))
    print("old turn angle:" + str(turnAngle))
    
    
    # # DEPTH IMAGE TO ARRAY (RESHAPING)
    # h, w = depth.shape
    # ys, xs = np.indices((h, w)) # X&Y ARRAYS
    # pixel_data = np.column_stack((xs.ravel(), ys.ravel(), depth.ravel())) # CREATING 3D ARRAY
    # scaled = StandardScaler().fit_transform(pixel_data)
    
    # model = KMeans(n_clusters=1, n_init='auto')
    # model.fit(depth)
    # cluster_labels = model.predict(depth)

    # # There will be k centers
    # cluster_centers = model.cluster_centers_

    # # inertia_ is the sse
    # sse = model.inertia_
    
    # sns.scatterplot(data = depth, x = 'longitude', y = 'latitude', hue = 'median_house_value')
    
    
    
    
    time.sleep(0.5)
    # ser.write(f"{turnAngle}\n".encode())  # send as line
    # assume turnAngle is an int
    # sock.sendto(str(float(turnAngle)).encode(), (UDP_IP, UDP_PORT))
    
    if not ret2:
        continue
    
    # Plot the image
    cv2.imshow('Depth Image', depth_image)
    
    # Press q key to stop
    if cv2.waitKey(1) == ord('q'):
        print(np.shape(depth)) # 512 x 512
        # print("max number:" + str(totalMaxDepth))
        # 14000 mm max
        # 128 characters max
        break