import numpy as np
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D

def DHtransform(a, alpha, d, theta):
    m = np.matrix([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
        [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return m

def WAM(ltheta, xe):
    tmp = xe * DHtransform(0, -math.pi/2, 0, ltheta[0])
    tmp = tmp * DHtransform(0, math.pi/2, 0, ltheta[1])
    tmp = tmp * DHtransform(45, -math.pi/2, 550, ltheta[2])
    tmp = tmp * DHtransform(-45, math.pi/2, 0, ltheta[3])
    tmp = tmp * DHtransform(0, -math.pi/2, 300, ltheta[4])
    tmp = tmp * DHtransform(0, math.pi/2, 0, ltheta[5]) 
    tmp = tmp * DHtransform(0, 0, 60, ltheta[6])
    tmp = tmp * DHtransform(0, 0, 120, 0)
    return tmp

data = np.loadtxt('JointData.txt', delimiter = ' ')
p0 = np.eye(4)
trac = np.matrix([[0,0,0]])
for i in data:
    p = WAM(i, p0)
    trac = np.concatenate((trac, p.T[-1, 0:3]))
x = np.asarray(trac[1:,0]).squeeze()
y = np.asarray(trac[1:,1]).squeeze()
z = np.asarray(trac[1:,2]).squeeze()
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, z,color='b')
plt.savefig('p5.pdf')
