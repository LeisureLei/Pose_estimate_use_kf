from __future__ import division
import matplotlib.pyplot as plt
from numpy.linalg import inv
from numpy import *
import numpy as np
import math

def compSteer(x,wp_noise,iWp,G,dt):
	maxG  = 60*math.pi/180
	rateG = 30*math.pi/180
	minD  = 0.5
	cwp   = wp_noise[:,iWp-1]
	d2    = (cwp[0]-x[0])**2 + (cwp[1]-x[1])**2
	if d2 < minD**2:
		iWp += 1
		if iWp > len(wp[0]):
			iWp = 0
			return G,iWp
		cwp = wp_noise[:,iWp-1]
	deltaG = piTopi(math.atan2((cwp[1]-x[1]),(cwp[0]-x[0]))-x[2]-G)
	maxDelta = rateG*dt
	if abs(deltaG) > maxDelta:
		deltaG = sign(deltaG)*maxDelta
	G = G+deltaG
	if abs(G) > maxG:
		G = sign(G)*maxG
	return G,iWp

def compound(Xwa,Xab):
    Xwb = array(zeros([3,3]))
    Xab2 = Xab[0:2,:]
    Xwb2 = Xwb[0:2,:]
    rot = array([[math.cos(Xwa[2]),-math.sin(Xwa[2])],[math.sin(Xwa[2]),math.cos(Xwa[2])]])
    Xwb2 = dot(rot ,Xab2) + tile(Xwa[0:2], (1, len(Xab[0,:])))
    if len(Xab[:,0]) == 3:
        Xwb[2] = piTopi(Xwa[2] + Xab[2])
    Xwb[0] = Xwb2[0]
    Xwb[1] = Xwb2[1]
    return Xwb

def piTopi(angle):
	twopi = 2*math.pi
	angle = angle - twopi*fix(angle/twopi)
	for i in nditer(angle,op_flags = ['writeonly']):
		if i[...] >= math.pi:
			i[...]= i[...] - twopi
		if i[...] < -math.pi:
			i[...] = i[...] + twopi
	return angle

def kf_predict(wp, P , A , Q , B , U ):
    wp = dot(A , wp) + dot(B, U)
    P = dot(A, dot(P, A.T)) + Q
    return (wp ,P)

def kf_update(wp, P ,Y ,H ,R):
    IS = R + dot(H, dot(P, H.T))
    K = dot(P,dot(H.T ,inv(IS)))
    wp = wp + dot(K, (Y - dot(H ,wp)))
    P = P - dot(K, dot(H, P))
    return (wp, P, K )

steering  = 0
velocity  = 4.0
wheelbase = 1
dt        = 0.05
ipos      = 0
iWp       = 1
rob       = array([[0,-wheelbase,-wheelbase],[0,-0.5,0.5]])
pos       = zeros([3,1])
path      = zeros([3,3390])

#The real pursuiting points
wp       = array([[10,10,30,40,60],[20,40,50,35,40]])
wpcopy   = array([[10,10,30,40,60],[20,40,50,35,40]])
#Plot the intial pose of robot
RP       = array([[0,-wheelbase,-wheelbase,0],[0,-4,4,0]])
ax       = plt.axes(xlim=(0,70),ylim=(0,60))
robplot, = ax.plot(RP[0],RP[1],'b')
#Measurements W~N(0,2)
wp_noise=wp.astype(float)
for i in nditer(wp_noise,op_flags = ['writeonly']):
    i[...] += 2*random.randn()
#store the predicted points
wphat     = zeros([wp.shape[0],wp.shape[1]])
#store the corrected points
X1        = zeros([wp.shape[0],wp.shape[1]])
X1[:,0:1] = array([[0.0],[0.0]])
# Initialization of state matrices
P = diag((0.01,0.01))
A = array([[1,0],[0,1]])
Q = eye(wp.shape[0],)
B = eye(wp.shape[0],)
U = zeros((wp.shape[0],1))
# Measurement matrices
H = array([[1,0],[0,1]])
R = eye(wp.shape[0])

iWp1=0
while iWp != 0:
    if iWp1!=iWp:
# Applying the Kalman Filter
        Y = wp_noise[:,iWp-1:iWp]   #measurements
        (wp[:,iWp-1:iWp],P)   = kf_predict(wp[:,iWp-1:iWp], P, A, Q, B, U)
        wphat[:,iWp-1:iWp]    = wp[:,iWp-1:iWp]  #predicted points
        (wp[:,iWp-1:iWp],P,K) = kf_update(wp[:,iWp-1:iWp], P ,Y ,H ,R)
        X1[:,iWp-1:iWp] = wp[:,iWp-1:iWp]     #corrected points
        iWp1=iWp
#Plot the point, red '*' represents the real points, yellow '.' represents measurements, blue '+' represents estimated points.
    ax.plot(X1[0,iWp-1:iWp],X1[1,iWp-1:iWp],'b+',wp_noise[0,iWp-1:iWp],wp_noise[1,iWp-1:iWp],'y.',wpcopy[0,iWp-1:iWp],wpcopy[1,iWp-1:iWp],'r*')
#Plot the trajectory of robot
    steering,iWp = compSteer(pos, X1, iWp, steering, dt)
    pos[0] = pos[0] + velocity * dt * cos(steering + pos[2,:])
    pos[1] = pos[1] + velocity * dt * sin(steering + pos[2,:])
    pos[2] = pos[2] + velocity * dt * sin(steering) / wheelbase
    pos[2] = piTopi(pos[2])
    robPos = compound(pos,rob)
    path[:,ipos+1] = pos[:,0]
    ipos+=1
    ax.plot(path[0,0:ipos],path[1,0:ipos],'r')
    b=robPos[0:2,0:1]
    a=robPos[0:2]
    c=hstack((a,b))
    robplot.set_data(c[0],c[1])
    plt.pause(0.00001)
plt.show()
