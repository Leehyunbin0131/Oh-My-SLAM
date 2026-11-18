"""Display conversions. No command generation or ROS dependencies."""
import math
import numpy as np


def quaternion_rpy(q):
    x, y, z, w = (float(getattr(q, k)) for k in ('x', 'y', 'z', 'w'))
    if not all(math.isfinite(v) for v in (x, y, z, w)) or abs(x*x+y*y+z*z+w*w-1)>0.02:
        return None
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def grid_world(column, row, resolution, origin):
    x, y, yaw = origin
    a, b = column*resolution, row*resolution
    return x+math.cos(yaw)*a-math.sin(yaw)*b, y+math.sin(yaw)*a+math.cos(yaw)*b


def world_grid(x, y, resolution, origin):
    ox, oy, yaw = origin
    dx, dy = x-ox, y-oy
    return ((math.cos(yaw)*dx+math.sin(yaw)*dy)/resolution,
            (-math.sin(yaw)*dx+math.cos(yaw)*dy)/resolution)


def depth_array(message):
    """Respect encoding, endian and row stride. Invalid depths stay NaN."""
    if message.encoding not in ('32FC1', '16UC1'):
        raise ValueError('지원하지 않는 깊이 encoding: '+message.encoding)
    dtype = np.dtype(('>' if message.is_bigendian else '<') + ('f4' if message.encoding=='32FC1' else 'u2'))
    if message.step < message.width*dtype.itemsize or len(message.data)<message.step*message.height:
        raise ValueError('잘못된 깊이 영상 크기')
    array=np.ndarray((message.height,message.width), dtype=dtype, buffer=bytes(message.data),
                     strides=(message.step,dtype.itemsize)).astype(np.float32)
    if message.encoding=='16UC1': array*=.001
    array[(array<=0)|~np.isfinite(array)]=np.nan
    return array


def depth_rgb(array, max_range=5.):
    t=np.clip(np.nan_to_num(array, nan=0.)/max_range,0,1)
    rgb=np.stack([255*(1-t), 210*(1-np.abs(2*t-1)), 255*t],axis=-1).astype(np.uint8)
    rgb[~np.isfinite(array)]=[65,75,88]
    return np.ascontiguousarray(rgb)


def quaternion_product(a,b):
    x,y,z,w=a;u,v,t,s=b
    return (w*u+x*s+y*t-z*v,w*v-x*t+y*s+z*u,w*t+x*v-y*u+z*s,w*s-x*u-y*v-z*t)


def rotate_vector(q,v):
    x,y,z,w=q
    return quaternion_product(quaternion_product(q,(*v,0.)),(-x,-y,-z,w))[:3]
