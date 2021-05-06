# Student name: YIMING CHEN
# Student number: 400230266
# Python Version: 3.8


import numpy as np
import open3d as o3d
if __name__ == "__main__":

    pcd = o3d.io.read_point_cloud("cheny466_2.xyz", format='xyz')
    
    numpoints = len(pcd.points)

    lines = []

    i = 0
    while (i < (numpoints/512)):
        lines.append([0+(512*i),511+(512*i)])
        for x in range(511):
            lines.append([x+(i*512),x+1+(i*512)])
        i += 1

    i = 1

    while (i < (numpoints/512)):
        lines.append([0+(512*(i-1)),0+(512*i)]) 
        lines.append([63+(512*(i-1)),63+(512*i)])
        lines.append([127+(512*(i-1)),127+(512*i)])
        lines.append([191+(512*(i-1)),191+(512*i)])
        lines.append([255+(512*(i-1)),255+(512*i)]) 
        lines.append([319+(512*(i-1)),319+(512*i)])
        lines.append([383+(512*(i-1)),383+(512*i)])
        lines.append([447+(512*(i-1)),447+(512*i)])
        i += 1

    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))

    o3d.visualization.draw_geometries([line_set])
