import numpy as np
import time
import matplotlib.pyplot as plt
from ruckig import InputParameter, Ruckig, Trajectory

class PositionController():
    def __init__(self, current_pos, target_pos):

        # Define the parameters for the trajectory
        num_dof = 1  # Number of degrees of freedom
        ruckig = Ruckig(num_dof)  # Create a Ruckig instance

        inp = InputParameter(num_dof)

        # Initial conditions
        inp.current_position = [current_pos]
        inp.current_velocity = [0.0]
        inp.current_acceleration = [0.0]

        # Target conditions
        inp.target_position = [target_pos]
        inp.target_velocity = [0.0]
        inp.target_acceleration = [0.0]

        # Limits
        inp.max_velocity = [3000.0]
        inp.max_acceleration = [5000.0]
        inp.max_jerk = [1500.0]

        # Create a trajectory output
        self.trajectory = Trajectory(num_dof)

        # Calculate the trajectory
        ruckig.calculate(inp, self.trajectory)

        self.start_pos = current_pos
        self.target_pos = target_pos
        self.start_time = time.time()

    def get_pos(self):
        duration = time.time()-self.start_time
        if duration >= self.trajectory.duration:
            position = self.target_pos
        else:
            new_position, new_velocity, new_acceleration = self.trajectory.at_time(duration)
            position = new_position[0]

        return position
    
    def target_reached(self):
        duration = time.time()-self.start_time
        if duration >= self.trajectory.duration:
            return True
        else:
            return False

# # Extract trajectory data
# dt = 0.01
# time_array = np.arange(0, trajectory.duration, dt)
# position_array = []
# velocity_array = []
# acceleration_array = []
# jerk_array = []

# for t in time_array:
#     new_position, new_velocity, new_acceleration = trajectory.at_time(t)
#     position_array.append(new_position[0])
#     velocity_array.append(new_velocity[0])
#     acceleration_array.append(new_acceleration[0])
# jerk_array = omputed_jerk = np.gradient(acceleration_array, dt)

# Plot the trajectory
# plt.figure(figsize=(12, 8))

# plt.subplot(4, 1, 1)
# plt.plot(time_array, position_array)
# plt.title('Position')
# plt.ylabel('Position (m)')

# plt.subplot(4, 1, 2)
# plt.plot(time_array, velocity_array)
# plt.title('Velocity')
# plt.ylabel('Velocity (m/s)')

# plt.subplot(4, 1, 3)
# plt.plot(time_array, acceleration_array)
# plt.title('Acceleration')
# plt.ylabel('Acceleration (m/s^2)')

# plt.subplot(4, 1, 4)
# plt.plot(time_array, jerk_array)
# plt.title('Jerk')
# plt.ylabel('Jerk (m/s^3)')
# plt.xlabel('Time (s)')

# plt.tight_layout()
# plt.show()
