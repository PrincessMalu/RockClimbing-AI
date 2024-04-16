from matplotlib import pyplot as plt
import open3d as o3d
import os
import copy
import numpy as np
# import pandas as pd
# from PIL import Image

np.random.seed(42)

mesh = o3d.io.read_triangle_mesh('./alta_test2/result.fbx', enable_post_processing=True)

# # BASIC VISUALIZE
# draw_geoms_list = [mesh]
#o3d.visualization.draw_geometries(draw_geoms_list)

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
#mesh_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=5, origin=[0, 0, 0])

mesh.compute_vertex_normals()
mesh = mesh.subdivide_midpoint(number_of_iterations=1)
vert = np.asarray(mesh.vertices)
min_vert, max_vert = vert.min(axis=0), vert.max(axis=0)
for _ in range(30):
    cube = o3d.geometry.TriangleMesh.create_box()
    cube.scale(0.005, center=cube.get_center())
    cube.translate(
        (
            np.random.uniform(min_vert[0], max_vert[0]),
            np.random.uniform(min_vert[1], max_vert[1]),
            np.random.uniform(min_vert[2], max_vert[2]),
        ),
        relative=False,
    )
    mesh += cube
mesh.compute_vertex_normals()
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    triangle_clusters, cluster_n_triangles, cluster_area = (
        mesh.cluster_connected_triangles())
triangle_clusters = np.asarray(triangle_clusters)
cluster_n_triangles = np.asarray(cluster_n_triangles)
cluster_area = np.asarray(cluster_area)

mesh_0 = copy.deepcopy(mesh)
triangles_to_remove = cluster_n_triangles[triangle_clusters] < 100
mesh_0.remove_triangles_by_mask(triangles_to_remove)
# # Visualizing the mesh with the coordinate frame to understand the orientation.
# draw_geoms_list = [mesh_coord_frame, mesh]
# o3d.visualization.draw_geometries(draw_geoms_list)

# Uniformly sampling 500,000 points from the mesh to convert it to a point cloud.
n_pts = 500_000
pcd = mesh_0.sample_points_uniformly(n_pts)
# # Visualizing the point cloud.
#draw_geoms_list = [mesh_coord_frame, pcd]
#o3d.visualization.draw_geometries(draw_geoms_list)
# assert (pcd.has_normals())

oboxes = pcd.detect_planar_patches(
    normal_variance_threshold_deg=30,
    coplanarity_deg=75,
    outlier_ratio=0.75,
    min_plane_edge_length=0,
    min_num_points=0,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=500))

print("Detected {} patches".format(len(oboxes)))

geometries = []
for obox in oboxes:
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, 0.0001])
    mesh.paint_uniform_color(obox.color)
    geometries.append(mesh)
    geometries.append(obox)
geometries.append(pcd)

o3d.visualization.draw_geometries(geometries)