import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from enum import Enum
# Change the figure size
plt.rcParams["figure.figsize"] = [12,12]


class Rotation(Enum):
    ROLL = 0
    PITCH = 1
    YAW = 2

class EulerRotation:
    def __init__(self,rotations):
        """
        rotations is a list of 2-element tuples
        where the first element is the rotation 
        kind and the second is angle in degrees"""

        self._rotations = rotations
        self._rotation_map = {Rotation.ROLL:self.roll,
                             Rotation.PITCH:self.pitch,
                             Rotation.YAW:self.yaw}
    
    def roll(self,phi):
        """Returns a rotation matrix along the roll axis"""
        return np.array([[1,0,0],
                        [0,np.cos(phi),-np.sin(phi)],
                        [0,np.sin(phi),np.cos(phi)]])
    
    def pitch(self, theta):
        """Returns the rotation matrix along the pitch axis"""
        return np.array([[np.cos(theta),0,np.sin(theta)],
                         [0, 1,0],
                         [-np.sin(theta), 0, np.cos(theta)]])

    def yaw(self, psi):
        """Returns the rotation matrix along the yaw axis"""
        return np.array([[np.cos(psi),-np.sin(psi), 0],
                        [np.sin(psi),np.cos(psi),  0],
                        [0,0,1]])

    def rotate(self):
        """Applies the rotations in sequential order"""
        t = np.eye(3)
        for rotation in self._rotations:
            kind = rotation[0]
            angles = np.deg2rad(rotation[1])
            t = np.dot(self._rotation_map[kind](angles),t)
        return t
    
    def plot_rotation(self,vector,rotation):
        # Calculate new vector
        rotated_vector = np.dot(R,vector)
        
        # Initialize the plot
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        
        # Axes shown in black
        ax.quiver(0,0,0,1.5,0,0,color='black',arrow_length_ratio=0.15)
        ax.quiver(0,0,0,0,1.5,0,color='black',arrow_length_ratio=0.15)
        ax.quiver(0,0,0,0,0,1.5,color='black',arrow_length_ratio=0.15)
        # Original vector
        ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], color='blue', arrow_length_ratio=0.15)
        # Rotated vector
        ax.quiver(0, 0, 0, rotated_vector[0],rotated_vector[1],rotated_vector[2],color='red',arrow_length_ratio=0.15)
        
        #Configure the plot
        ax.set_xlim3d(-1, 1)
        ax.set_ylim3d(1, -1)
        ax.set_zlim3d(1, -1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()




# Test your code by passing in some rotation values
rotations = [
    (Rotation.ROLL, 25),
    (Rotation.PITCH, 75),
    (Rotation.YAW, 90),
]

# unit vector along x-axis
v = np.array([1, 0, 0])
R = EulerRotation(rotations).rotate()
print('Rotation matrix ...')
print(R)

EulerRotation(rotations).plot_rotation(v,R)