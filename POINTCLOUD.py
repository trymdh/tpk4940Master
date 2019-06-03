import os
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from UtilityFunctions import getUname, getNpyPaths
from PointArrayFunctions import unhomogenify, homogenify
from RansacFunctions import ransacPlane
from PlaneFunctions import plotPlane
import open3d as o3d
from open3d import estimate_normals,KDTreeSearchParamHybrid

uname = getUname()
wd = os.getcwd()

os.chdir(wd + "\snaps1\Snaps")
scan_fname = getNpyPaths()
os.chdir(wd + "\snaps1\SnapPoses")
scan_pose_fname = getNpyPaths()
os.chdir(wd)
X = np.load("X.npy")

#load all scans and scan poses into arrays
scans = []
scan_poses = []
for fname in scan_fname:
    scans.append(np.load(fname))
scans = np.asarray(scans)
for fname in scan_pose_fname:
    scan_poses.append(np.load(fname))
scan_poses = np.asarray(scan_poses)

for scan in scan_poses:
        scan[:3,3] = scan[:3,3]*1000

#gather all scans into a single pointcloud array
pointcloud = []
i = 0
for scan in scans:
    Y = np.dot(scan_poses[i],X)
    for point in scan:
        pointcloud.append(np.dot(Y,np.append(point,1)))
    i += 1 
points = unhomogenify(np.asarray(pointcloud))
#points = points[15*800:16*800]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
estimate_normals(pcd, search_param = KDTreeSearchParamHybrid(radius = 5, max_nn = 5))
o3d.io.write_point_cloud("sync.ply", pcd)

pcd_load = o3d.io.read_point_cloud("sync.ply")
#o3d.visualization.draw_geometries([pcd_load])

print("Downsample the point cloud with a voxel of 0.05")
downpcd = o3d.geometry.voxel_down_sample(pcd, voxel_size=0.05)

print("Recompute the normal of the downsampled point cloud")
o3d.geometry.estimate_normals(
    downpcd,
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10,
                                                          max_nn=30))
print(np.asarray(pcd.normals))
print("Print a normal vector of the 0th point")
print(downpcd.normals[0])
print("Print the normal vectors")
normals = np.asarray(downpcd.normals)
