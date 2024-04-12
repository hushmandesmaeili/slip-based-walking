import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class COMTrajectoryPlotter:
    def __init__(self):
        pass

    def plot_3d(self, traj_x, traj_y, traj_z, x_cop_0, y_cop_0, x_cop_T, y_cop_T):
        # Create a new 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the trajectory
        ax.plot(traj_x, traj_y, traj_z, label='COM Trajectory')

        # Plot the CoP positions
        ax.scatter(x_cop_0, y_cop_0, 0, label='Initial CoP', color='r')
        ax.scatter(x_cop_T, y_cop_T, 0, label='Final CoP', color='g')

        # Labeling
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.set_title('3D COM Trajectory')
        ax.legend()

        # Show the plot
        plt.show()