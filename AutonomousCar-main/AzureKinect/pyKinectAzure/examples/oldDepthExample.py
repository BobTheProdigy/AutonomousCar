import cv2
import pykinect_azure as pykinect
import numpy as np

if __name__ == "__main__":
	# Initialize the library, if the library is not found, add the library path as argument
	pykinect.initialize_libraries()

	# Modify camera configuration
	device_config = pykinect.default_configuration
	device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
	device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

	# Start device
	device = pykinect.start_device(config=device_config)

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
  		maxDepth = np.max(depth)
		print("-" * (int(maxDepth)//110))
  
		# if maxDepth > totalMaxDepth:
		# 	totalMaxDepth = maxDepth
		
		if not ret2:
			continue
		

		# Plot the image
		cv2.imshow('Depth Image', depth_image)

		# Press q key to stop
		if cv2.waitKey(1) == ord('q'):
			# print(np.shape(depth)) # 512 x 512
			# print("max number:" + str(totalMaxDepth))
			# 14000 mm max
			# 128 characters max
			break



# pip install pykinect_azure

# import socket
# import time
# import numpy as np
# import pandas as pd
# import cv2
# import pykinect_azure as pykinect

# # Initialize Kinect
# pykinect.initialize_libraries()
# device = pykinect.start_device()

# # Server setup
# HOST = ''        # Listen on all interfaces
# PORT = 12345
# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.bind((HOST, PORT))
# server_socket.listen(1)

# print("Waiting for REV hub to connect...")
# conn, addr = server_socket.accept()
# print(f"Connected by {addr}")

# while True:
#     capture = device.update()
#     ret, depth_image = capture.get_depth_image()

#     if not ret:
#         continue

#     depth_meters = depth_image.astype(np.float32) / 1000.0
#     df = pd.DataFrame(depth_meters)
#     min_distance = df[df > 0].min().min()

#     # Send data
#     message = f"{min_distance:.3f}\n"
#     conn.sendall(message.encode())

#     time.sleep(0.1)