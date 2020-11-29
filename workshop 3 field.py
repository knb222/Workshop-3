#!/usr/bin/env python
# coding: utf-8

# In[7]:


get_ipython().run_line_magic('pylab', 'inline')
import sim
import numpy


# In[8]:


sim.simxFinish(-1)  # Close opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim

if clientID != -1:
    print('Connected')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)

    print('Simulation time in milliseconds: ', sim.simxGetLastCmdTime(clientID))
    
    # Get Object position
    name = 'Omnirob'
    err_code, cuboid = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_blocking)
    res, position = sim.simxGetObjectPosition(clientID, cuboid, -1, sim.simx_opmode_blocking)        
    print('Omnirob is at [x,y,z]=', position)
    
    # Now close the connection to CoppeliaSim:
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
    print('Disconnected')
else:
    print('Failed connecting to remote API server')


# In[9]:


class robot():
    
    def __init__(self, frame_name, motor_names=[], client_id=0):  
        # If there is an existing connection
        if client_id:
                self.client_id = client_id
        else:
            self.client_id = self.open_connection()
            
        self.motors = self._get_handlers(motor_names) 
        
        # Robot frame
        self.frame =  self._get_handler(frame_name)
            
        
    def open_connection(self):
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to CoppeliaSim 
        
        if clientID != -1:
            print('Robot connected')
        else:
            print('Connection failed')
        return clientID
        
    def close_connection(self):    
        sim.simxGetPingTime(self.client_id)  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        sim.simxFinish(self.client_id)  # Now close the connection to CoppeliaSim:
        print('Connection closed')
    
    def isConnected(self):
        c,result = sim.simxGetPingTime(self.client_id)
        # Return true if the robot is connected
        return result > 0         
        
    def _get_handler(self, name):
        err_code, handler = sim.simxGetObjectHandle(self.client_id, name, sim.simx_opmode_blocking)
        return handler
    
    def _get_handlers(self, names):
        handlers = []
        for name in names:
            handler = self._get_handler(name)
            handlers.append(handler)
        
        return handlers

    def send_motor_velocities(self, vels):
        for motor, vel in zip(self.motors, vels):
            err_code = sim.simxSetJointTargetVelocity(self.client_id, 
                                                      motor, vel, sim.simx_opmode_streaming)      
            
    def set_position(self, position, relative_object=-1):
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)        
        sim.simxSetObjectPosition(clientID, self.frame, relative_object, position, sim.simx_opmode_oneshot)
        
    def simtime(self):
        return sim.simxGetLastCmdTime(self.client_id)
    
    def get_position(self, relative_object=-1):
        # Get position relative to an object, -1 for global frame
        if relative_object != -1:
            relative_object = self._get_handler(relative_object)
        res, position = sim.simxGetObjectPosition(self.client_id, self.frame, relative_object, sim.simx_opmode_blocking)        
        return array(position)
    
    def get_object_position(self, object_name):
        # Get Object position in the world frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, -1, sim.simx_opmode_blocking)
        return array(position)
    
    def get_object_relative_position(self, object_name):        
        # Get Object position in the robot frame
        err_code, object_h = sim.simxGetObjectHandle(self.client_id, object_name, sim.simx_opmode_blocking)
        res, position = sim.simxGetObjectPosition(self.client_id, object_h, self.frame, sim.simx_opmode_blocking)
        return array(position)


# In[10]:


def traject(rp, dp):
    tf = 2.
    tstep = 0.1
    a0 = rp
    a1 = 0
    a2 = (3*dp-3*a0)/tf**2
    a3 = (2*a0-2*dp)/tf**3

    # Time interval
    time_steps = linspace(0, tf, int(tf/tstep))

    for t in time_steps:
        # Compute the trajectory
        traj_point = double(a0+a1*t+a2*(t**2)+a3*(t**3))
        vel_traj = double(3*a3*(t**2)+2*a2*t+a1)

        # Location sesing
        robot_position = r.get_position()
    
        # drive the robot using the trajectory tracker
        u = 5*(traj_point - robot_position) + vel_traj
    
    
        vx, vy, vz = u
        r.send_motor_velocities([-vy - vx, vy - vx, vy + vx, -vy + vx])
        time.sleep(tstep)


    
    r.send_motor_velocities([0, 0, 0, 0])


# In[11]:


####wheel robot
motor_names = ['Omnirob_FLwheel_motor', 'Omnirob_FRwheel_motor', 'Omnirob_RRwheel_motor', 'Omnirob_RLwheel_motor']
r = robot('Omnirob', motor_names)  # Create an instance of our robot


robot_position = r.get_position()
desired_position = numpy.array([7, 8, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([6.5, 7.5, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([6, 7.1, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([5.4, 6.7, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([4.8, 6.3, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([4.25, 5.88, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([3.76, 5.5, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([3.2, 4.9, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([3.2, 5, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([3.15, 4.9, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([3.15, 5, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([3.08, 5, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([3.03, 5, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([2.9, 5.12, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([2.8, 5.2, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([2.57, 5.28, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([2.26, 5.33, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([1.857, 5.32, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([1.39, 5.17, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([0.9, 4.87, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([0.5, 4.45, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([0.16, 3.97, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.15, 3.53, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.4, 3.15, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.55, 2.86, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.57, 2.65, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.49, 2.46, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.4, 2.23, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.33, 1.94, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.32, 1.63, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.36, 1.31, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.46, 1.01, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.60, 0.78, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.76, 0.62, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-0.94, .52, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-1.15, 0.47, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-1.39, 0.42, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-1.68, 0.37, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-2.02, 0.31, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-2.43, 0.23, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-2.85, 0.11, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-3.25, -0.05, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-3.86, -0.46, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.05, -0.67, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.17, -0.89, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.23, -1.13, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.23, -1.13, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.23, -1.13, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.23, -1.13, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.26, -1.39, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.3, -1.69, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.37, -2.01, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.46, -2.33, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.58, -2.62, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.70, -2.88, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.81, -3.1, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-4.92, -3.25, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-5.02, -3.37, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-5.11, -3.46, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-5.21, -3.5, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-5.3, -3.54, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-5.39, -3., 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-5.48, -3.55, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-5.58, -3.55, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-5.69, -3.55, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-5.93, -3.55, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-6.06, -3.55, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-6.2, -3.55, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-6.35, -3.57, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-6.5, -3.59, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-6.67, -3.62, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-6.82, -3.65, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-6.98, -3.70, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-7.14, -3.76, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-7.29, -3.8, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-7.44, -3.89, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-7.58, -3.97, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-7.72, -4.05, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-7.84, -4.14, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-7.96, -4.23, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.07, -4.33, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.17, -4.42, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.27, -4.53, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.35, -4.63, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.44, -4.73, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.52, -4.84, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.58, -4.94, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.64, -5.04, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.71, -5.14, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.76, -5.24, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.8, -5.34, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.85, -5.44, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.89, -5.54, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.92, -5.64, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.96, -5.73, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.98, -5.82, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.01, -5.91, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.03, -6.00, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.06, -6.08, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.08, -6.17, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.09, -6.25, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.1, -6.33, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.11, -6.40, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.13, -6.48, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.13, -6.56, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.14, -6.62, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.14, -6.7, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.15, -6.77, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.15, -6.83, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.15, -6.9, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.15, -6.95, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.15, -7.01, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.14, -7.08, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.14, -7.13, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.13, -7.18, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.13, -7.24, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.13, -7.29, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.12, -7.34, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.11, -7.39, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.11, -7.44, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.10, -7.48, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.10, -7.52, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.10, -7.57, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.10, -7.61, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.07, -7.65, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.06, -7.68, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.055, -7.72, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.047, -7.75, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.038, -7.8, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.03, -7.82, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.02, -7.86, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.01, -7.89, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.00, -7.92, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-9.00, -7.95, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.98, -7.98, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.97, -8.01, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.96, -8.03, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.95, -8.06, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.94, -8.09, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.94, -8.11, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.93, -8.14, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.92, -8.16, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.91, -8.18, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.90, -8.2, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.89, -8.22, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.89, -8.24, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.88, -8.26, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.87, -8.28, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.86, -8.30, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.85, -8.32, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.85, -8.34, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.84, -8.35, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.83, -8.37, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.82, -8.38, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.82, -8.40, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.81, -8.41, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.80, -8.43, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.79, -8.44, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.79, -8.45, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.78, -8.47, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.77, -8.48, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.77, -8.48, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.76, -8.49, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.75, -8.5, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.75, -8.51, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.74, -8.52, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.74, -8.54, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.74, -8.55, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.73, -8.56, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.73, -8.56, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.72, -8.57, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.71, -8.58, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.71, -8.59, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.70, -8.60, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.70, -8.61, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.69, -8.62, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.69, -8.62, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.68, -8.63, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.68, -8.65, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.68, -8.65, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.67, -8.66, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.67, -8.67, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.66, -8.67, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.66, -8.68, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.65, -8.69, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.65, -8.70, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.65, -8.70, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.64, -8.71, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.64, -8.71, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.64, -8.72, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.63, -8.72, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.62, -8.72, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.62, -8.73, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.62, -8.73, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.61, -8.74, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.61, -8.74, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.61, -8.75, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.60, -8.75, 0])
traject(robot_position, desired_position)

robot_position = r.get_position()
desired_position = numpy.array([-8.60, -8.75, 0])
traject(robot_position, desired_position)


# In[ ]:





# In[ ]:




