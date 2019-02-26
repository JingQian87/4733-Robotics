import numpy as np
import matplotlib.pyplot as plt
import math

#Define the transformation
def DHtransform(a, alpha, d, theta):
    m = np.matrix([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
        [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return m

def WAM(ltheta):
    T1 = DHtransform(0, 0, 0.333, ltheta[0])
    T2 = T1 * DHtransform(0, -math.pi/2, 0, ltheta[1]) 
    T3 = T2 * DHtransform(0, math.pi/2, 0.316, ltheta[2]) 
    T4 = T3 * DHtransform(0.0825, math.pi/2, 0, ltheta[3]) 
    T5 = T4 * DHtransform(-0.0825, -math.pi/2, 0.384, ltheta[4]) 
    T6 = T5 * DHtransform(0, math.pi/2, 0, ltheta[5]) 
    T7 = T6 * DHtransform(0.088, math.pi/2, 0, ltheta[6]) 
    #tmp = tmp * DHtransform(0, 0, 0.107, 0) 
    return [T1,T2,T3,T4,T5,T6,T7]

#calculate Jacobian (forward kinematics)
def Jacob(ltheta):
    Tmatrix = WAM(ltheta)
    Jo = np.array((0,0,1)).reshape(-1,1)
    for i in range(6):
        Jo = np.hstack((Jo, Tmatrix[i][0:3,2]))
        pe = T7[0:3,3]
        Jp = np.cross(Jo[:,0].T,pe.T).T
    for i in range(6):
        jpi = np.cross(Jo[:,i+1].T,(pe-Tmatrix[i][0:3,3]).T).T
        Jp = np.hstack((Jp,jpi))
    J = np.vstack((Jp,Jo))
    T = Tmatrix[-1]
    return (J,T)

#Sanity check for question a
Jacob(ltheta=[0.1,0.2,0.3,-0.4,0.5,0.6,0.7])[0]

# (b) simple Jacobian transpose method
nlimit = 3000
eth = 0.01
error = float('Inf')
n = 0
Te = np.matrix([[ 0.51525409,  0.48131384,  0.70911932, -0.081512  ],
        [ 0.4758376 , -0.8488226 ,  0.2303883 , -0.04093309],
        [ 0.71280558,  0.21871712, -0.66638654,  0.39938295],
        [ 0.        ,  0.        ,  0.        ,  1.        ]])
Td = np.matrix([[-0.781, -0.474, 0.407, -0.206],
               [-0.220, 0.818, 0.531, -0.147],
               [-0.585, 0.325, -0.743, 0.62],
               [0, 0, 0, 1]])
pd = Td[0:3,3]
q = np.array((0.1,0.2,0.3,-0.4,0.5,0.6,0.7)).reshape(-1,1)
qs = [np.squeeze(np.asarray(q))]#configuration record
errors = []#error record
pes = [np.squeeze(np.asarray(Te[0:3,3]))]#end position record
while error > eth:
    n += 1
    if n > nlimit: break
    J, Te = Jacob(q)
    pk = Te[0:3,3]
    ep = pd-pk
    eo = np.array((0,0,0))
    for i in range(3):
        eo = eo +np.cross(Td[0:3,i].T,Te[0:3,i].T)
    e = np.vstack((ep, eo.T))
    error = np.linalg.norm(e)
    errors.append(error)
    JJe = J*J.T*e
    alpha = np.dot(JJe.T, e)/np.dot(JJe.T, JJe)
    dq = alpha[0,0]*(J.T*e)
    q = q + dq
    qs.append(np.squeeze(np.asarray(q)))
    pes.append(np.squeeze(np.asarray(Te[0:3,3])))

np.savetxt('qs_simple.txt', qs, delimiter=',')
import matplotlib.pyplot as plt
plt.plot(errors)
plt.xlabel('iteration times')
plt.ylabel('error (magnitude)')
plt.savefig('error_simple.pdf')
plt.show()
from mpl_toolkits.mplot3d import Axes3D
x,y,z=[],[],[]

for i in range(len(pes)):
    x.append(pes[i][0])
    y.append(pes[i][1])
    z.append(pes[i][2])
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, z,color='b')
plt.savefig('postion_simple.pdf')
plt.show()

# (c) pseudoinverse transpose method
nlimit = 3000
eth = 0.01
error = float('Inf')
n = 0
Te = np.matrix([[ 0.51525409,  0.48131384,  0.70911932, -0.081512  ],
        [ 0.4758376 , -0.8488226 ,  0.2303883 , -0.04093309],
        [ 0.71280558,  0.21871712, -0.66638654,  0.39938295],
        [ 0.        ,  0.        ,  0.        ,  1.        ]])
Td = np.matrix([[-0.781, -0.474, 0.407, -0.206],
               [-0.220, 0.818, 0.531, -0.147],
               [-0.585, 0.325, -0.743, 0.62],
               [0, 0, 0, 1]])
pd = Td[0:3,3]
q = np.array((0.1,0.2,0.3,-0.4,0.5,0.6,0.7)).reshape(-1,1)
qs = [np.squeeze(np.asarray(q))]#configuration record
errors = []#error record
pes = [np.squeeze(np.asarray(Te[0:3,3]))]#end position record
while error > eth:
    n += 1
    if n > nlimit: break
    J, Te = Jacob(q)
    pk = Te[0:3,3]
    ep = pd-pk
    eo = np.array((0,0,0))
    for i in range(3):
        eo = eo +np.cross(Td[0:3,i].T,Te[0:3,i].T)
    e = np.vstack((ep, eo.T))
    error = np.linalg.norm(e)
    errors.append(error)
    Jr = J.T*np.linalg.inv(J*J.T) #right
    dq = Jr*e
    q = q + dq
    qs.append(np.squeeze(np.asarray(q)))
    pes.append(np.squeeze(np.asarray(Te[0:3,3])))
plt.plot(errors)
plt.xlabel('iteration times')
plt.ylabel('error (magnitude)')
plt.savefig('error_pseudoinverse.pdf')
plt.show()
from mpl_toolkits.mplot3d import Axes3D
x,y,z=[],[],[]

for i in range(len(pes)):
    x.append(pes[i][0])
    y.append(pes[i][1])
    z.append(pes[i][2])
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, z,color='b')
plt.savefig('postion_pseudoinverse.pdf')
plt.show()

# (c) damped
nlimit = 3000
eth = 0.01
k = 1
Kp = np.eye(3)*1
Ko = np.eye(3)*0.5
error = float('Inf')
n = 0
Te = np.matrix([[ 0.51525409,  0.48131384,  0.70911932, -0.081512  ],
        [ 0.4758376 , -0.8488226 ,  0.2303883 , -0.04093309],
        [ 0.71280558,  0.21871712, -0.66638654,  0.39938295],
        [ 0.        ,  0.        ,  0.        ,  1.        ]])
Td = np.matrix([[-0.781, -0.474, 0.407, -0.206],
               [-0.220, 0.818, 0.531, -0.147],
               [-0.585, 0.325, -0.743, 0.62],
               [0, 0, 0, 1]])
pd = Td[0:3,3]
q = np.array((0.1,0.2,0.3,-0.4,0.5,0.6,0.7)).reshape(-1,1)
qs = [np.squeeze(np.asarray(q))]#configuration record
errors = []#error record
pes = [np.squeeze(np.asarray(Te[0:3,3]))]#end position record
while error > eth:
    n += 1
    if n > nlimit: break
    J, Te = Jacob(q)
    pk = Te[0:3,3]
    ep = pd-pk
    eo = np.array((0,0,0))
    for i in range(3):
        eo = eo +np.cross(Td[0:3,i].T,Te[0:3,i].T)
    e = np.vstack((ep, eo.T))
    error = np.linalg.norm(e)
    errors.append(error)
    
    estar = np.vstack((Kp*ep, Ko*np.asmatrix(eo.T)))
    Jstar = J.T*np.linalg.inv(J*J.T+np.eye(6)*k) #right
    dq = Jstar*estar
    q = q + dq
    qs.append(np.squeeze(np.asarray(q)))
    pes.append(np.squeeze(np.asarray(Te[0:3,3])))

plt.plot(errors)
plt.xlabel('iteration times')
plt.ylabel('error (magnitude)')
plt.savefig('error_damp.pdf')
plt.show()
from mpl_toolkits.mplot3d import Axes3D
x,y,z=[],[],[]

for i in range(len(pes)):
    x.append(pes[i][0])
    y.append(pes[i][1])
    z.append(pes[i][2])
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, z,color='b')
plt.savefig('postion_damp.pdf')
plt.show()