import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import math


def PCA(data):
    # get mean data
    mean = np.mean(data, axis=0)

    # find the covariance matrix
    cov = np.cov(data.T)

    # find the eigenvalues and eigenvectors
    eig_val, eig_vec = np.linalg.eig(cov)

    # plot the eigenvalues and eigenvectors
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(data[:, 0], data[:, 1], data[:, 2], c='b', marker='o')
    ax.quiver(mean[0], mean[1], mean[2], eig_vec[0, 0], eig_vec[1, 0], eig_vec[2, 0], color='r', length=0.5)
    ax.quiver(mean[0], mean[1], mean[2], eig_vec[0, 1], eig_vec[1, 1], eig_vec[2, 1], color='g', length=0.5)
    ax.quiver(mean[0], mean[1], mean[2], eig_vec[0, 2], eig_vec[1, 2], eig_vec[2, 2], color='b', length=0.5)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

    return eig_vec, mean

# origin plane
plane_ori = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])

# Load the point cloud
pcd = o3d.io.read_point_cloud(r"Pointcloud\C5_New.ply")

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd, plane_ori])

# downsampling the point cloud
downpcd = pcd.voxel_down_sample(voxel_size=0.02)

# Visualize the downsampled point cloud
# o3d.visualization.draw_geometries([downpcd])

# remove outliers
cl, ind = downpcd.remove_statistical_outlier(nb_neighbors=10,std_ratio=4)
downpcd = downpcd.select_by_index(ind)
# o3d.visualization.draw_geometries([downpcd])

# cluster by plane and keep the original plane coordinate
plane_model, inliers = downpcd.segment_plane(distance_threshold=0.005,ransac_n=3,num_iterations=1000)
inlier_cloud = downpcd.select_by_index(inliers)

vector, mean = PCA(np.asarray(inlier_cloud.points))

# get mean of point cloud
mean = np.mean(np.asarray(downpcd.points), axis=0)

# move pcd to the origin
pcd = pcd.translate(-mean)

c, b, a, d = plane_model

# Compute the normal vector of the plane
n = np.array([a, b, c])

# Compute two orthogonal vectors that lie in the plane
v1 = np.array([1, -a/b, 0])
v2 = np.cross(n, v1)

# Normalize the vectors
n /= np.linalg.norm(n)
v1 /= np.linalg.norm(v1)
v2 /= np.linalg.norm(v2)

# Construct a rotation matrix that aligns the plane with the z-axis
R = np.array([v1, v2, n]).T

print(a, b, c, d)
print (R)

# Apply the rotation matrix
pcd = pcd.rotate(R, center=[0, 0, 0])

# Visualize the transformed point cloud
o3d.visualization.draw_geometries([pcd, plane_ori])

# ----------------------------------------------------second loop--------------------------------------------------------

downpcd = pcd.voxel_down_sample(voxel_size=0.02)
# Visualize the downsampled point cloud
# o3d.visualization.draw_geometries([downpcd])

# remove outliers
cl, ind = downpcd.remove_statistical_outlier(nb_neighbors=10,std_ratio=4)
downpcd = downpcd.select_by_index(ind)
# o3d.visualization.draw_geometries([downpcd])

# cluster by plane and keep the original plane coordinate
plane_model, inliers = downpcd.segment_plane(distance_threshold=0.005,ransac_n=3,num_iterations=1000)
inlier_cloud = downpcd.select_by_index(inliers)
# pca analysis
vector, mean = PCA(np.asarray(inlier_cloud.points))

c, b, a, d = plane_model

# Compute the normal vector of the plane
n = np.array([a, b, c])

# construct a rotation matrix that aligns the plane with the z-axis
R = np.array([vector[0], vector[1], vector[2]]).T

# Apply the rotation matrix
pcd = pcd.rotate(R, center=[0, 0, 0])

# flip along x axis
# pcd = pcd.rotate(np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]), center=[0, 0, 0])

# # rotate 90 degree along x axis
# pcd = pcd.rotate(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]), center=[0, 0, 0])
# pcd = pcd.rotate(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]), center=[0, 0, 0])

# # # flip along y axis
# pcd = pcd.rotate(np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]), center=[0, 0, 0])

# # rotate 90 degree along y axis
# pcd = pcd.rotate(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]), center=[0, 0, 0])
# pcd = pcd.rotate(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]), center=[0, 0, 0])


# # # flip along z axis
# pcd = pcd.rotate(np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]), center=[0, 0, 0])

# # rotate 90 degree along z axis
# pcd = pcd.rotate(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]), center=[0, 0, 0])

# Visualize the transformed point cloud
o3d.visualization.draw_geometries([pcd, plane_ori])



# --------------------------------------- after rotating point cloud ---------------------------------------

# downsample the pcd
downpcd = pcd.voxel_down_sample(voxel_size=0.03)

# remove outliers
cl, ind = downpcd.remove_statistical_outlier(nb_neighbors=100,std_ratio=2)
downpcd = downpcd.select_by_index(ind)
# o3d.visualization.draw_geometries([downpcd])

# find z min and max
xyz = np.asarray(downpcd.points)
z_min = xyz[:, 2].min()
z_max = xyz[:, 2].max()
z_mean = (z_min + z_max) / 2

print("z_min: ", z_min)
print("z_max: ", z_max)
print("z_mean: ", z_mean)
z_table = z_mean - 0.21
z_log = z_mean + 0.15
print("z_table: ", z_table)


# --------------------------------------- clean table ---------------------------------------

# table mask
mask_table = []
for i in range(len(pcd.points)):
    if pcd.points[i][2] > z_table:
        # add i to a list
        mask_table.append(i)

# visualize the mask
mask_table = pcd.select_by_index(mask_table)
# mask_table = downpcd.select_by_index(mask_table, invert=True)
o3d.visualization.draw_geometries([mask_table, plane_ori])


# downsample the pcd
downpcd_table = mask_table.voxel_down_sample(voxel_size=0.03)
o3d.visualization.draw_geometries([downpcd_table, plane_ori])


# high res mesh of pcd
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh_high, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        mask_table, depth=9)
print(mesh_high)
o3d.visualization.draw_geometries([mesh_high])

# save the mesh
o3d.io.write_triangle_mesh("C5_mesh_high.ply", mesh_high)

# save pcd
o3d.io.write_point_cloud("C5.ply", mask_table)

# --------------------------------------- find the log ---------------------------------------

mask_log = []
# add points to list if z value is greater than zmean
for i in range(len(downpcd.points)):
    if downpcd.points[i][2] > z_log:
        # add i to a list
        mask_log.append(i)

# visualize the mask
mask_log = downpcd.select_by_index(mask_log)
o3d.visualization.draw_geometries([mask_log])

# low res mesh of pcd
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh_low, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        mask_log, depth=9)
o3d.visualization.draw_geometries([mesh_low, plane_ori])

# Save pcd
o3d.io.write_point_cloud("C5_log_pcd.ply", mask_log)
# save mesh
o3d.io.write_triangle_mesh("C5_log_mesh.ply", mesh_low)

# -------------------------------------------------------------------------------End---------------------------------------
