import cv2
import pykinect_azure as pykinect
from utils import Open3dVisualizer
import numpy as np
import open3d as o3d

if __name__ == "__main__":

    pykinect.initialize_libraries()

    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

    device = pykinect.start_device(config=device_config)

    open3dVisualizer = Open3dVisualizer()

    # Global map point cloud
    global_map = o3d.geometry.PointCloud()

    cv2.namedWindow('Depth Image', cv2.WINDOW_NORMAL)

    while True:
        capture = device.update()
        ret_depth, depth_img = capture.get_colored_depth_image()
        ret_points, points = capture.get_pointcloud()

        if not ret_depth or not ret_points:
            continue
        
        # Create Open3D point cloud for current frame
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # TODO: If you have camera pose:
        # pcd.transform(T_cam_to_world)

        # Add to global map
        global_map += pcd

        # Downsample occasionally to keep memory reasonable
        if len(global_map.points) > 300000:
            global_map = global_map.voxel_down_sample(voxel_size=0.02)

        # Display the global map
        open3dVisualizer(np.asarray(global_map.points))

        cv2.imshow('Depth Image', depth_img)

        if cv2.waitKey(1) == ord('q'):
            break





# import cv2 # pip install opencv-python

# import pykinect_azure as pykinect # pip install pykinect-azure
# from utils import Open3dVisualizer # pip install open3d, only works on python up to 3.12
# import numpy as np # pip install numpy
# import open3d as o3d

# if __name__ == "__main__":

# 	# Initialize the library, if the library is not found, add the library path as argument
# 	pykinect.initialize_libraries()

# 	# Modify camera configuration
# 	device_config = pykinect.default_configuration
# 	device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
# 	device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

# 	# Start device
# 	device = pykinect.start_device(config=device_config)

# 	# Initialize the Open3d visualizer
# 	open3dVisualizer = Open3dVisualizer()

# 	cv2.namedWindow('Depth Image',cv2.WINDOW_NORMAL)
# 	while True:

# 		# Get capture
# 		capture = device.update()

# 		# Get the color depth image from the capture
# 		ret_depth, depth_image = capture.get_colored_depth_image()

# 		# Get the 3D point cloud
# 		ret_points, points = capture.get_pointcloud()

# 		if not ret_depth or not ret_points:
# 			continue

# 		open3dVisualizer(points)	

# 		# Plot the image
# 		cv2.imshow('Depth Image',depth_image)
		
# 		# Press q key to stop
# 		if cv2.waitKey(1) == ord('q'):  
# 			break