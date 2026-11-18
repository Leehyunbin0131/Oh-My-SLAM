"""Geometry used by the deterministic course and independent acceptance observer."""
from __future__ import annotations
import math
import numpy as np


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def compose(a, b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return np.array([a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],angle_delta(a[2]+b[2],0)])


def inverse(a):
    c,s=math.cos(a[2]),math.sin(a[2])
    return np.array([-c*a[0]-s*a[1],s*a[0]-c*a[1],-a[2]])


def polygon(pose, footprint):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return np.asarray(footprint) @ np.array([[c,s],[-s,c]]) + pose[:2]


def rectangle(bounds):
    a,b,c,d=bounds
    return np.array([[a,c],[b,c],[b,d],[a,d]],dtype=float)


def intersects(a, b):
    for shape in (a,b):
        edge=np.roll(shape,-1,axis=0)-shape
        for axis in np.column_stack((-edge[:,1],edge[:,0])):
            pa,pb=a@axis,b@axis
            if pa.max()<pb.min()-1e-9 or pb.max()<pa.min()-1e-9:
                return False
    return True


def separation(a,b):
    if intersects(a,b): return 0.
    minimum=float('inf')
    for points,lines in ((a,b),(b,a)):
        for point in points:
            for start,end in zip(lines,np.roll(lines,-1,axis=0)):
                direction=end-start
                t=float(np.clip(np.dot(point-start,direction)/np.dot(direction,direction),0,1))
                minimum=min(minimum,float(np.linalg.norm(point-start-t*direction)))
    return minimum


def interpolate(a,b,step=.01):
    delta=np.asarray(b)-a; delta[2]=angle_delta(b[2],a[2])
    count=max(1,math.ceil(max(np.linalg.norm(delta[:2]),abs(delta[2]))/step))
    return [np.asarray(a)+delta*(index/count) for index in range(1,count+1)]


def object_bounds(definition, center=None):
    center=definition['center'] if center is None else center
    dx,dy=definition['size'][:2]
    return [center[0]-dx/2,center[0]+dx/2,center[1]-dy/2,center[1]+dy/2]
