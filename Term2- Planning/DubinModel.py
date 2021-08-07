import numpy as np 
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = 12,12

# Define Parameters
STEERING_ANGLE_MAX = np.deg2rad(30) # This is the maximum steering angle in radians

def sample_steering_angle():
    """
    This function returns a random steering angle in the
    range of -/+ STEERING_ANGLE_MAX
    """
    return np.random.uniform(-STEERING_ANGLE_MAX,STEERING_ANGLE_MAX)

def simulate(state,angle,v,dt):
    """
    Simulate one step the Dubin's Car model 

    args:
        state - array-like with 3 elements corresponding to x,y and theta 
        angle - float, the steering angle input
        v - float, the velocity input
        dt - float, the time step 
    returns:
        array_like with 3 elements of the new state
    """
    new_x = state[0] + v*np.cos(state[2])*dt
    new_y = state[1] + v*np.sin(state[2])*dt
    new_theta = state[2] + v*np.tan(angle)*dt
    return [new_x,new_y,new_theta]


# Define the simulations variables
v = 5
dt = 0.05 
total_time = 10

# Initial state
states = [[0,0,0]]

# Simulate the model
for _ in np.arange(0,total_time,dt):
    angle = sample_steering_angle()
    state= simulate(states[-1],angle,v,dt)
    states.append(state)

# Visualize the results
states = np.array(states)
plt.plot(states[:,0],states[:,1],color='blue')
plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()