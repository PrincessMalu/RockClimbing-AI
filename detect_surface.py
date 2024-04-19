import copy
import open3d as o3d
import numpy as np
import random
import sys
import time
np.set_printoptions(threshold=sys.maxsize)

# add mesh to model
mesh = o3d.io.read_triangle_mesh('./alta_test2/result.fbx', enable_post_processing=True)

# remove smallest clusters from model
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

# sample 500,000 points from the smooth mesh
n_pts = 500_000
pcd = mesh_0.sample_points_uniformly(n_pts)

# get normals
downpcd = pcd.voxel_down_sample(voxel_size=0.05)
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

all_points = np.asarray(downpcd.points)
rand_point = random.choice(all_points)

# Ethan Code

"""

color_surfaces(pointcloud, color)
    examined_points = []
    queue = empty
    select arbitrary point in pointcloud
    while len(point cloud) is not equal to len(examined_points):
        for each xpoint in point cloud:
            if xpoint is adjacent/extremely close to point
            and xpoint is less than x degrees off of point
            and xpoint is not in queue
            and xpoint is not in examined_points:
                color xpoint to surface color
                enqueue(xpoint)
        add point to examined_points
        while queue is not empty:
            run this function again
"""