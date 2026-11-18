from types import SimpleNamespace
import math
import numpy as np
from omslam_operator.geometry import depth_array, grid_world, world_grid, quaternion_rpy, quaternion_product, rotate_vector


def test_depth_units_stride_endian_and_invalid():
    msg=SimpleNamespace(encoding='16UC1',is_bigendian=True,width=2,height=2,step=6,
                        data=np.array([1000,0,65535,2500,500,65535],dtype='>u2').tobytes())
    got=depth_array(msg)
    assert got[0,0]==1 and np.isnan(got[0,1]) and got[1,0]==2.5 and got[1,1]==.5


def test_rotated_map_round_trip_and_invalid_imu():
    origin=(4.,-3.,math.pi/2)
    assert np.allclose(grid_world(2,3,.05,origin),(3.85,-2.9))
    assert np.allclose(world_grid(3.85,-2.9,.05,origin),(2,3))
    assert quaternion_rpy(SimpleNamespace(x=0,y=0,z=0,w=0)) is None


def test_mounting_rotation_and_body_orientation():
    # Sensor is mounted +90 degrees about Y: sensor Z is robot X.
    q=(0.,math.sin(math.pi/4),0.,math.cos(math.pi/4))
    assert np.allclose(rotate_vector(q,(0.,0.,1.)),(1.,0.,0.))
    body=quaternion_product(q,(-q[0],-q[1],-q[2],q[3]))
    assert np.allclose(body,(0.,0.,0.,1.))
