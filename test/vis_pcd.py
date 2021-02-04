import numpy as np 
import os 
import open3d as o3d
import ipdb 
st = ipdb.set_trace

def get_bbox_lineset(x_max, x_min, y_max, y_min, z_max, z_min):
    points = [
        [x_min, y_min, z_min],
        [x_min, y_min, z_max],
        [x_min, y_max, z_min],
        [x_min, y_max, z_max],
        [x_max, y_min, z_min],
        [x_max, y_min, z_max],
        [x_max, y_max, z_min],
        [x_max, y_max, z_max]
    ]
    lines = [
            [0, 1],
            [0, 2],
            [1, 3],
            [2, 3],
            [4, 5],
            [4, 6],
            [5, 7],
            [6, 7],
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],
    ]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    return line_set

def make_pcd(pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts[:, :3])
    # if the dim is greater than 3 I expect the color
    if pts.shape[1] == 6:
        pcd.colors = o3d.utility.Vector3dVector(pts[:, 3:] / 255.\
            if pts[:, 3:].max() > 1. else pts[:, 3:])
    return pcd

pp = np.load("/Users/shamitlal/Desktop/temp/cvpr21/scene0002_01_vert.npy")
bbox = np.load("/Users/shamitlal/Desktop/temp/cvpr21/scene0002_01_bbox.npy")
# st()
o3dlist = []
for bb in bbox:
    cx,cy,cz,dx,dy,dz = bb[:6]
    xmin = cx - dx/2.
    xmax = cx + dx/2.
    ymin = cy - dy/2.
    ymax = cy + dy/2.
    zmin = cz - dz/2.
    zmax = cz + dz/2.
    lineset = get_bbox_lineset(xmin, xmax, ymin, ymax, zmin, zmax)
    o3dlist.append(lineset)



pcd = make_pcd(pp)
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=10.0, origin=[0, 0, 0])
o3dlist.append(mesh_frame)
o3dlist.append(pcd)
o3d.visualization.draw_geometries(o3dlist)