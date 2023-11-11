
###########################################################################################################################################
import os
import open3d as o3d
import numpy as np

def calculate_surface_normal(orientation_reference, point):

  normal = orientation_reference - point
  normal = normal / np.linalg.norm(normal)

  return normal

if __name__ == "__main__":
    # Check the current working directory
    print("Current working directory:", os.getcwd())

    # Load the .ply file
    file_path = "/home/noemoji041/Desktop/point_cloud/output.ply"
    try:
        pcd_load = o3d.io.read_point_cloud(file_path)
#        pcd_load.compute_point_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,max_nn=3000))
#        pcd_load.orient_normals_to_align_with_direction()
        pcd_load.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2,max_nn=30))
    except Exception as e:
        print("Error:", e)

    # Visualize the point cloud if successfully loaded
    if pcd_load:
        print(np.asarray(pcd_load))
#	 for downsampling, remove the underlaying comment
#        pcd_load=pcd_load.voxel_down_sample(voxel_size=0.05)
        o3d.visualization.draw_geometries([pcd_load])
###################################################################################################################################################################


