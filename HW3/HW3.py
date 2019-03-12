import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def IntFunc(y, t, c):
  x0, y0, theta0, phi0 = y
  dphidt = c*np.sin(t)
  dthetadt = np.tan(phi0)
  dxdt = np.cos(theta0)
  dydt = np.sin(theta0)
  return [dxdt, dydt, dthetadt, dphidt]

fig = plt.figure(figsize=(5.6, 5))
plt.subplots_adjust(left = 0.17, bottom = 0.15, right = 0.95, top = 0.95)
ratio=1
timestep = 0.01
ninterval = 2001
t = np.linspace(0,20,ninterval)
 # xs, ys, thetas, phis = 
result = odeint(IntFunc,[0,0,0,0],t, args=(ratio,))
plt.plot(result.T[0], result.T[1])
plt.xlabel('x (t)', fontsize=16)
plt.ylabel('y (t)', fontsize=16)
plt.text(0.1, 0.5, r'$\dot{\phi}(t)=%s \sin(t),\Delta t=%s s$'%(ratio,timestep), fontsize=16)
#plt.title(r'$\dot{\phi}(t)=%s * \sin(t),\Delta t=%s s$'%(ratio,timestep))
plt.savefig('xy%s.png' %ratio)