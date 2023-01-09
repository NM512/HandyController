# Standard imports
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

class Visualizer3D:
    def __init__(self):
        self.fig = plt.figure(figsize = (5, 5))
        self.ax = self.fig.add_subplot(111, projection='3d')

    def render(self, joint_positions):
        for finger in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            x, y, z = [], [], []
            for id in ['first', 'second', 'third', 'fourth', 'wrist']:
                x.append(joint_positions[finger][id][0])
                y.append(joint_positions[finger][id][1])
                z.append(joint_positions[finger][id][2])
            self.ax.plot(x, y, z)
            self.ax.scatter(x, y, z, s = 10)
        self.ax.set_xlim(0.0, 1.0)
        self.ax.set_ylim(0.0, 1.0)
        self.ax.set_zlim(0.0, 1.0)
        plt.draw()
        plt.pause(0.001)
        self.ax.cla()

def map_3Dvec_on_plane(oa, ob, oc):
    """
    OA: place vector 1
    OB: place vector 2
    OC: target vector to be mapped
    oh: vector OH(perpendicular to OAB plane)
    """
    oa2 = sum([ele**2.0 for ele in oa])
    ob2 = sum([ele**2.0 for ele in ob])
    oaob = sum([ele1*ele2 for ele1, ele2 in zip(oa, ob)])
    co = [-ele for ele in oc]
    cooa = sum([ele1*ele2 for ele1, ele2 in zip(co, oa)])
    coob = sum([ele1*ele2 for ele1, ele2 in zip(co, ob)])

    b = (oa2*coob - cooa*oaob) / (oaob**2.0 - oa2*ob2)
    a = - (cooa + b*oaob) / oa2
    oh = [a*ele1 + b*ele2 for ele1, ele2 in zip(oa, ob)]

    return oh

def angle_between_2vectors(vec1, vec2):
    unit_vec1 = vec1 / np.linalg.norm(vec1)
    unit_vec2 = vec2 / np.linalg.norm(vec2)
    # rad is 0.0 ~ 3.141516
    rad = np.arccos(np.dot(unit_vec1, unit_vec2))
    return rad


if __name__ == '__main__':
    print(map_3Dvec_on_plane([0.5,0.5,0], [0.2,1.0,0], [0.2,1.0,0]))