#!/usr/bin/env python3
"""Smooth a FREE-graph seed in XY while retaining its ordered height profile."""
import math
import numpy as np


def smooth_free_path(path, graph):
    path=np.asarray(path,dtype=float)
    if len(path)<3:return path.copy()
    lengths=np.linalg.norm(np.diff(path,axis=0),axis=1)
    keep=np.r_[True,lengths>1e-9];path=path[keep]
    if len(path)<3:return path.copy()
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    spacing=.5*graph.resolution
    sample=np.linspace(0.,arc[-1],max(3,int(math.ceil(arc[-1]/spacing))+1))
    original=np.column_stack([np.interp(sample,arc,path[:,i]) for i in range(3)])
    result=original.copy();resolution=graph.resolution

    def supported(point):
        key=tuple(np.floor(point[:2]/resolution).astype(int))
        # Reference Z has already recovered the demonstrated surface height;
        # allow its voxel quantization without changing Z or merging floors.
        return any(abs(graph.points[i,2]-point[2])<=resolution+1e-6
                   for i in graph.columns.get(key,()))

    def edge_free(a,b):
        count=max(1,int(math.ceil(np.linalg.norm(b-a)/(.25*resolution))))
        return all(supported(a+(b-a)*(i/count)) for i in range(count+1))

    if not all(edge_free(a,b) for a,b in zip(result[:-1],result[1:])):
        return path.copy()
    for _ in range(40):
        largest=0.
        for i in range(1,len(result)-1):
            candidate=result[i].copy()
            delta=.45*(.5*(result[i-1,:2]+result[i+1,:2])-result[i,:2])
            delta+=.06*(original[i,:2]-result[i,:2])
            candidate[:2]+=delta
            # Stay local to the graph chain; preserve the route's topology and
            # keep the demonstrated stair heights at the same path progress.
            if np.linalg.norm(candidate[:2]-original[i,:2])>resolution:continue
            if edge_free(result[i-1],candidate) and edge_free(candidate,result[i+1]):
                largest=max(largest,float(np.linalg.norm(delta)))
                result[i]=candidate
        if largest<1e-5:break
    return result
