import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from stl import Mesh
import os 

figure = plt.figure()
axes = mplot3d.Axes3D(figure)
R = np.array([[0,0,1],[0, -1, 0], [1, 0,0]])
# Load the STL files and add the vectors to the plot
os.chdir(os.getcwd() + "\TestPiece")
your_mesh = Mesh.from_file('Testpiece.stl')
your_mesh.rotate([1,0,0],np.deg2rad(90),point=[0,0,0])
your_mesh.translate([0,0,100])
axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))

# Auto scale to the mesh size
scale = your_mesh.points.flatten(-1)
axes.auto_scale_xyz(scale, scale, scale)

# Show the plot to the screen
plt.show()