import open3d as o3d
import os
import copy
import numpy as np
# import pandas as pd
# from PIL import Image
# PYTHON VERSION: 3.6.13

np.random.seed(42)

print(o3d.__version__)

mesh = o3d.io.read_triangle_mesh('./result.fbx', enable_post_processing=True)

# # BASIC VISUALIZE
draw_geoms_list = [mesh]
o3d.visualization.draw_geometries(draw_geoms_list)

# ONLY one o3d visualize works at a time so don't have multiple only the first one will work. 

# # VISUALIZING WITH NORMALS (ie looks like the image)
# mesh.compute_vertex_normals()
# draw_geoms_list = [mesh]
# o3d.visualization.draw_geometries(draw_geoms_list)

# # VISUALIZING WITH COORDINATES
# Creating a mesh of the XYZ axes Cartesian coordinates frame.
# This mesh will show the directions in which the X, Y & Z-axes point,
# and can be overlaid on the 3D mesh to visualize its orientation in
# the Euclidean space.
# X-axis : Red arrow
# Y-axis : Green arrow
# Z-axis : Blue arrow
mesh_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])

# # Visualizing the mesh with the coordinate frame to understand the orientation.
# draw_geoms_list = [mesh_coord_frame, mesh]
# o3d.visualization.draw_geometries(draw_geoms_list)

# Uniformly sampling 100,000 points from the mesh to convert it to a point cloud.
# n_pts = 100_000
# pcd = mesh.sample_points_uniformly(n_pts)

# # Visualizing the point cloud.
# draw_geoms_list = [mesh_coord_frame, pcd]
# o3d.visualization.draw_geometries(draw_geoms_list)