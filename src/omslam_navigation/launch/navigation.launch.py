"""Saved simulation map, AMCL and gated Nav2 navigation."""
from omslam_navigation.launch_support import make_launch_description


def generate_launch_description():
    return make_launch_description('amcl')
