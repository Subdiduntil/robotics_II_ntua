import numpy as np
import matplotlib.pyplot as plt
plt.close('all')
# Read data files
with open('joint1.txt') as f:
    joint1 = f.readlines()
with open('joint2.txt') as f:
    joint2 = f.readlines()
with open('joint3.txt') as f:
    joint3 = f.readlines()
with open('joint4.txt') as f:
    joint4 = f.readlines()
with open('joint5.txt') as f:
    joint5 = f.readlines()
with open('joint6.txt') as f:
    joint6 = f.readlines()
with open('joint7.txt') as f:
    joint7 = f.readlines()

with open('vjoint1.txt') as f:
    vjoint1 = f.readlines()
with open('vjoint2.txt') as f:
    vjoint2 = f.readlines()
with open('vjoint3.txt') as f:
    vjoint3 = f.readlines()
with open('vjoint4.txt') as f:
    vjoint4 = f.readlines()
with open('vjoint5.txt') as f:
    vjoint5 = f.readlines()
with open('vjoint6.txt') as f:
    vjoint6 = f.readlines()
with open('vjoint7.txt') as f:
    vjoint7 = f.readlines()

with open('px.txt') as f:
    px = f.readlines()
with open('py.txt') as f:
    py = f.readlines()
with open('pz.txt') as f:
    pz = f.readlines()
    
with open('vx.txt') as f:
    vx = f.readlines()
with open('vy.txt') as f:
    vy = f.readlines()
with open('vz.txt') as f:
    vz = f.readlines()
    
with open('error.txt') as f:
    error = f.readlines()
with open('V1.txt') as f:
    V1 = f.readlines()
with open('V2.txt') as f:
    V2 = f.readlines()

with open('green_q1.txt') as f:
    greenq1 = f.readlines()
with open('red_q1.txt') as f:
    redq1 = f.readlines()
with open('green_q2.txt') as f:
    greenq2 = f.readlines()
with open('red_q2.txt') as f:
    redq2 = f.readlines()

# Preprocess Data
px[0] = px[0].replace('[', '').replace(']', '')
py[0] = py[0].replace('[', '').replace(']', '')
pz[0] = pz[0].replace('[', '').replace(']', '')
vx[0] = vx[0].replace('[', '').replace(']', '')
vy[0] = vy[0].replace('[', '').replace(']', '')
vz[0] = vz[0].replace('[', '').replace(']', '')
V1[0] = V1[0].replace('[', '').replace(']', '')
V2[0] = V2[0].replace('[', '').replace(']', '')
error[0] = error[0].replace('[', '').replace(']', '')
greenq1[0] = greenq1[0].replace('[', '').replace(']', '')
redq1[0] = redq1[0].replace('[', '').replace(']', '')
greenq2[0] = greenq2[0].replace('[', '').replace(']', '')
redq2[0] = redq2[0].replace('[', '').replace(']', '')

joint1 = (joint1[0].split())
joint2 = (joint2[0].split())
joint3 = (joint3[0].split())
joint4 = (joint4[0].split())
joint5 = (joint5[0].split())
joint6 = (joint6[0].split())
joint7 = (joint7[0].split())
vjoint1 = (vjoint1[0].split())
vjoint2 = (vjoint2[0].split())
vjoint3 = (vjoint3[0].split())
vjoint4 = (vjoint4[0].split())
vjoint5 = (vjoint5[0].split())
vjoint6 = (vjoint6[0].split())
vjoint7 = (vjoint7[0].split())
px = (px[0].split())
py = (py[0].split())
pz = (pz[0].split())
vx = (vx[0].split())
vy = (vy[0].split())
vz = (vz[0].split())
error = (error[0].split())
V1 = (V1[0].split())
V2 = (V2[0].split())
greenq1 = (greenq1[0].split())
redq1 = (redq1[0].split())
greenq2 = (greenq2[0].split())
redq2 = (redq2[0].split())


joint1 = np.array([float(i) for i in joint1])
joint2 = np.array([float(i) for i in joint2])
joint3 = np.array([float(i) for i in joint3])
joint4 = np.array([float(i) for i in joint4])
joint5 = np.array([float(i) for i in joint5])
joint6 = np.array([float(i) for i in joint6])
joint7 = np.array([float(i) for i in joint7])
vjoint1 = np.array([float(i) for i in vjoint1])
vjoint2 = np.array([float(i) for i in vjoint2])
vjoint3 = np.array([float(i) for i in vjoint3])
vjoint4 = np.array([float(i) for i in vjoint4])
vjoint5 = np.array([float(i) for i in vjoint5])
vjoint6 = np.array([float(i) for i in vjoint6])
vjoint7 = np.array([float(i) for i in vjoint7])
px = np.array([float(i) for i in px])
py = np.array([float(i) for i in py])
pz = np.array([float(i) for i in pz])
vx = np.array([float(i) for i in vx])
vy = np.array([float(i) for i in vy])
vz = np.array([float(i) for i in vz])
V1 = np.array([float(i) for i in V2])
V2 = np.array([float(i) for i in V2])
error = np.array([float(i) for i in error])
greenq1 = np.array([float(i) for i in greenq1])
redq1 = np.array([float(i) for i in redq1])
greenq2 = np.array([float(i) for i in greenq2])
redq2 = np.array([float(i) for i in redq2])

# Plot Data
# Time Intervals of Simulation
T = 2.5
Tmax = int(600*T + 1)
t = [(i/600) for i in range(int(Tmax))] # time vector

# Joint Angles
fig1, axs = plt.subplots(2,4)
fig1.suptitle('Joint Angles as Functions of Time')
axs[0][0].plot(t,joint1)
axs[0, 0].legend(['Joint 1'])
axs[0][1].plot(t,joint2)
axs[0, 1].legend(['Joint 2'])
axs[0][2].plot(t,joint3)
axs[0, 2].legend(['Joint 3'])
axs[0][3].plot(t,joint4)
axs[0, 3].legend(['Joint 4'])
axs[1][0].plot(t,joint5)
axs[1, 0].legend(['Joint 5'])
axs[1][1].plot(t,joint6)
axs[1, 1].legend(['Joint 6'])
axs[1][2].plot(t,joint7)
axs[1, 2].legend(['Joint 7'])
fig1.delaxes(axs[1][3])

# Joint Velocities
fig10, axs = plt.subplots(2,4)
fig10.suptitle('Joint Velocities as Functions of Time')
axs[0][0].plot(t,vjoint1)
axs[0, 0].legend(['Joint 1'])
axs[0][1].plot(t,vjoint2)
axs[0, 1].legend(['Joint 2'])
axs[0][2].plot(t,vjoint3)
axs[0, 2].legend(['Joint 3'])
axs[0][3].plot(t,vjoint4)
axs[0, 3].legend(['Joint 4'])
axs[1][0].plot(t,vjoint5)
axs[1, 0].legend(['Joint 5'])
axs[1][1].plot(t,vjoint6)
axs[1, 1].legend(['Joint 6'])
axs[1][2].plot(t,vjoint7)
axs[1, 2].legend(['Joint 7'])
fig10.delaxes(axs[1][3])

# End - Effector Position
fig2, axs = plt.subplots(1,3)
fig2.suptitle('Position of End-Effector')
axs[0].plot(t,px)
axs[0].set_xlabel('Time [sec]')
axs[0].set_ylabel('Coordinates [m]')
axs[0].legend(['Px'])
axs[1].plot(t,py)
axs[1].set_xlabel('Time [sec]')
axs[1].legend(['Py'])
axs[2].plot(t,pz)
axs[2].set_xlabel('Time [sec]')
axs[2].legend(['Pz'])

# End Effector Desired Velocity
fig2, axs = plt.subplots(1,3)
fig2.suptitle('Desired Velocity of End-Effector')
axs[0].plot(t,vx)
axs[0].set_xlabel('Time [sec]')
axs[0].set_ylabel('Linear Velocity [m/sec]')
axs[0].legend(['Vx'])
axs[1].plot(t,vy)
axs[1].set_xlabel('Time [sec]')
axs[1].legend(['Vy'])
axs[2].plot(t,vz)
axs[2].set_xlabel('Time [sec]')
axs[2].legend(['Vz'])

# Positional Error on y-axis
fig3 = plt.figure()
plt.plot(t,error)
plt.xlabel('Time [sec]')
plt.ylabel('Error [m]')
plt.title('Positional Error on y-axis')

# Obstacle Avoidance Criteria
fig4 = plt.figure()
plt.plot(t,V1, label = 'V1')
plt.plot(t,V2, label = 'V2')
plt.legend()
plt.xlabel('Time [sec]')
plt.ylabel('Criteria [m^2]')
plt.title('Obstacle Avoidance Criteria')

# Joint q3 Distance from Obstacles
fig5, axs = plt.subplots(1,2)
axs[0].plot(t,np.abs(greenq1),'g', label = 'Distance from Green Obstacle')
axs[0].plot(t,np.abs(redq1),'r', label = 'Distance from Red Obstacle')
axs[0].legend()
axs[0].set_xlabel('Time [sec]')
axs[0].set_ylabel('Distance [m]')
axs[0].set_title('Joint q3 - Distance from Obstacles')

# Joint q4 Distance from Obstacles
axs[1].plot(t,np.abs(greenq2),'g', label = 'Distance from Green Obstacle')
axs[1].plot(t,np.abs(redq2),'r', label = 'Distance from Red Obstacle')
axs[1].legend()
axs[1].set_xlabel('Time [sec]')
axs[1].set_ylabel('Distance [m]')
axs[1].set_title('Joint q4 - Distance from Obstacles')