import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv

from tpk4170.models import ColladaMesh, Grid, Axes, PointCloud, Plane, Line
from tpk4170.visualization import Viewer
from tpk4170.utils.transformations import quaternion_from_matrix
from tpk4170.rigid_body_motions.rotations import exp, vec

v = Viewer()
v.add(Grid())