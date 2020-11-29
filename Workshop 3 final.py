#!/usr/bin/env python
# coding: utf-8

# In[4]:


import matplotlib.pyplot as plt

circle1 = plt.Circle((-1.775, 2.1), 0.5, color='r')
circle2 = plt.Circle((-5.8, -.975), 0.5, color='r')
circle3 = plt.Circle((7.275, -7.75), 0.5, color='r')
circle4 = plt.Circle((2.425, 4.075), 0.5, color='r')
circle5 = plt.Circle((4.4, .875), 0.5, color='r')
circle6 = plt.Circle((2.825, -4.85), 0.5, color='r')
circle7 = plt.Circle((-.525, 6.3), 0.5, color='r')
circle8 = plt.Circle((-5.125, 3.425), 0.5, color='r')
goal = plt.Circle((-5.125, 3.425), 0.5, color='r')

rectangle11=plt.Circle((-5.15, -6.2), 0.665, color='b')
rectangle12=plt.Circle((-4.485, -6.865), 0.665, color='b')
rectangle13=plt.Circle((-5.815, -5.535), 0.665, color='b')

rectangle21=plt.Circle((2.725, -1.5), 0.665, color='b')
rectangle22=plt.Circle((3.415, -2.165), 0.665, color='b')
rectangle23=plt.Circle((2.085, -.835), 0.665, color='b')

rectangle31=plt.Circle((7.4, 4.725), 0.665, color='b')
rectangle32=plt.Circle((6.735, 4.725), 0.665, color='b')
rectangle33=plt.Circle((8.065, 4.725), 0.665, color='b')

rectangle41=plt.Circle((-6.7, 6.4), 0.665, color='b')
rectangle42=plt.Circle((-6.035, 6.4), 0.665, color='b')
rectangle43=plt.Circle((-7.365, 6.4), 0.665, color='b')

rectangle51=plt.Circle((-1.025, -1.5), 0.665, color='b')
rectangle52=plt.Circle((-1.69, -1.5), 0.665, color='b')
rectangle53=plt.Circle((-.36, -1.5), 0.665, color='b')

goal = plt.Circle((-7.425, -7.925), 0.4, color='g')


fig, ax = plt.subplots()

ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)


ax.add_artist(circle1)
ax.add_artist(circle2)
ax.add_artist(circle3)
ax.add_artist(circle4)
ax.add_artist(circle5)
ax.add_artist(circle6)
ax.add_artist(circle7)
ax.add_artist(circle8)

ax.add_artist(rectangle11)
ax.add_artist(rectangle12)
ax.add_artist(rectangle13)

ax.add_artist(rectangle21)
ax.add_artist(rectangle22)
ax.add_artist(rectangle23)

ax.add_artist(rectangle31)
ax.add_artist(rectangle32)
ax.add_artist(rectangle33)

ax.add_artist(rectangle41)
ax.add_artist(rectangle42)
ax.add_artist(rectangle43)

ax.add_artist(rectangle51)
ax.add_artist(rectangle52)
ax.add_artist(rectangle53)

ax.add_artist(goal)


fig.savefig('workshop3_diagram.png')


# In[5]:


get_ipython().run_line_magic('pylab', 'inline')
plt.style.use('dark_background')
pylab.rcParams['figure.figsize'] = (10.0, 6.0)
matplotlib.rcParams['animation.embed_limit'] = 2**128

# Potential functions
def force_goal(x, y, goal):
    goal_x, goal_y, k_goal = goal
    Fx_goal, Fy_goal = k_goal * (goal_x - x), k_goal *(goal_y - y)
    return Fx_goal, Fy_goal

def force_obstacle(x, y, obstacle):
    (obs_x, obs_y, k_obs) = obstacle
    dist_x, dist_y = x - obs_x, y - obs_y
    dist_obs = np.hypot(dist_x, dist_y)
    Fx_obs = (dist_x / dist_obs) * k_obs / dist_obs ** 3
    Fy_obs = (dist_y / dist_obs) * k_obs / dist_obs ** 3

    return Fx_obs, Fy_obs 

def total_force(x, y, goal, obstacles):
    Fx, Fy = force_goal(x, y, goal)

    for obs in obstacles:
        Fo_x, Fo_y = force_obstacle(x, y, obs)
        Fx += Fo_x
        Fy += Fo_y
    return Fx, Fy   
    
def simulate(q, goal, obstacles, num_steps=200, delta_time=1.9):
    fmax = .5
    X, Y = np.meshgrid(np.arange(-10, 10, .5), np.arange(-10, 10, .5))

    # Vector field of the forces
    Fx, Fy = total_force(X, Y, goal, obstacles)
   
    # For visualization
    F_m = np.hypot(Fx, Fy)
    Fx[F_m > fmax], Fy[F_m > fmax] = None, None
   
    fig, ax= plt.subplots(figsize=(10,10))
    ax.quiver(X,Y,Fx,Fy,F_m,color='0.1')

    trajectory = []
    for i in range(num_steps):
        force = total_force(q[0], q[1], goal, obstacles)
        # Robot velocity follows the force vector
        vel = np.array(force)
        # Integrate
        q += vel * delta_time
        trajectory.append(np.copy(q))

   
    return np.array(trajectory)


import matplotlib.pyplot as plt
import numpy as np

# creating goal and robot position
goal = [-7.425, -7.925, 0.02]
r_pos = np.array((7.625, 8.55))

#creating obstacles
obstacles =[ [-1.775, 2.1, 0.5],
[-5.8, -.975, 0.5],
[7.275, -7.75, 0.5],
[2.425, 4.075, 0.5],
[4.4, .875, 0.5],
[2.825, -4.85, 0.5],
[-.525, 6.3, 0.5],
[-5.125, 3.425, 0.5],
[-5.15, -6.2, .665],
[-4.485, -6.865, .665],
[-5.815, -5.535, .665],
[2.725, -1.5, .665],
[3.415, -2.165, .665],
[2.085, -.835, .665],
[7.4, 4.725, .665],
[6.735, 4.725, .665],
[8.065, 4.725, .665],
[-6.7, 6.4, .665],
[-6.035, 6.4, .665],
[-7.365, 6.4, .665],
[-1.025, -1.5, .665],
[-1.69, -1.5, .665],
[-.36, -1.5, .665],
]


#changing default axes limits
trajectory = simulate(r_pos, goal, obstacles)
plot(trajectory[:,0], trajectory[:,1], 'r--')
plot(goal[0], goal[1], 'xy')


# In[6]:


print(trajectory)


# In[ ]:




