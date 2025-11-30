import open3d as o3d

# Load the PLY (your splat file)
pcd = o3d.io.read_point_cloud("frame_splat_color.ply")

print(pcd)
print("Loaded points:", len(pcd.points))

# Visualize
o3d.visualization.draw_geometries([pcd])
