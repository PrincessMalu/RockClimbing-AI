import copy
import open3d as o3d
import numpy as np

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

# plane segmentation
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=500, num_iterations=1000, probability=0.9999)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud = inlier_cloud.paint_uniform_color([0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)

# # outlier plane segmentation
plane_model2, inliers2 = outlier_cloud.segment_plane(distance_threshold=0.06, ransac_n=5, num_iterations=5000, probability=0.9999)
[e, f, g, h] = plane_model2
print(f"Plane equation 2: {e:.2f}x + {f:.2f}y + {g:.2f}z + {h:.2f} = 0")
inlier_cloud2 = outlier_cloud.select_by_index(inliers2)
inlier_cloud2 = inlier_cloud2.paint_uniform_color([0, 1, 0])
outlier_cloud2 = outlier_cloud.select_by_index(inliers2, invert=True)

# # outlier (again) plane segmentation
plane_model3, inliers3 = outlier_cloud2.segment_plane(distance_threshold=0.04, ransac_n=3, num_iterations=5000, probability=0.9999)
[i, j, k, l] = plane_model3
print(f"Plane equation 3: {i:.2f}x + {j:.2f}y + {k:.2f}z + {l:.2f} = 0")
inlier_cloud3 = outlier_cloud2.select_by_index(inliers3)
inlier_cloud3 = inlier_cloud3.paint_uniform_color([0, 0, 1])
outlier_cloud3 = outlier_cloud2.select_by_index(inliers3, invert=True)

# Walls found for alta_test2
# Plane equation: -0.20x + 0.98y + 0.06z + -0.27 = 0
# Plane equation 2: -0.48x + 0.88y + 0.02z + 1.61 = 0
# Plane equation 3: -0.49x + 0.87y + 0.00z + -3.06 = 0

# Ground plane equation: 0.00x + 0.00y + 1.00z + -14.55 = 0

# Wall Angle = 90 degrees - Angle(Ground, Plane)
# arccos((n1 * n2)/(|n1||n2|)) --> only z plane matters
wall_angle_1 = np.degrees(np.arccos(c/(np.sqrt(np.square(a)+np.square(b)+np.square(c)))))
wall_angle_2 = np.degrees(np.arccos(g/(np.sqrt(np.square(e)+np.square(f)+np.square(g)))))
wall_angle_3 = np.degrees(np.arccos(k/(np.sqrt(np.square(h)+np.square(i)+np.square(k)))))

# output wall angles
print(np.floor(90-wall_angle_1))
print(np.floor(90-wall_angle_2))
print(np.floor(90-wall_angle_3))

# numpy array of points for walls
wall_indicies = []
wall_indicies.append(np.asarray(inlier_cloud.points))
wall_indicies.append(np.asarray(inlier_cloud2.points))
wall_indicies.append(np.asarray(inlier_cloud3.points))

# draw model

print(np.asarray(inlier_cloud.points))


o3d.visualization.draw_geometries([inlier_cloud, inlier_cloud2, inlier_cloud3, outlier_cloud3])
# o3d.visualization.draw_geometries([inlier_cloud, inlier_cloud2, inlier_cloud3, outlier_cloud3])