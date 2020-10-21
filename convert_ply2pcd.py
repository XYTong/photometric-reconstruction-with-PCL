import open3d as o3d
pcd = o3d.io.read_point_cloud("out.ply")
o3d.io.write_point_cloud("pointcloud.pcd", pcd)
