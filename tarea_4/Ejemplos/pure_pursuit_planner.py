import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation


class DifferentialRobot:

    def __init__(self,
                 local_planner,
                 radius=1.,
                 min_linear_vel =-1.,
                 max_linear_vel = 1.,
                 min_angular_vel=-1,
                 max_angular_vel= 1):

        self._radius = radius
        self._max_linear_vel  = max_linear_vel
        self._min_linear_vel  = min_linear_vel
        self._max_angular_vel = max_angular_vel
        self._min_angular_vel = min_angular_vel

        self._position = None
        self._orientation = None

        self._local_planner = local_planner
        local_planner.set_vel_limits(self._min_linear_vel,
                                     self._max_linear_vel,
                                     self._min_angular_vel,
                                     self._max_angular_vel)


    def set_pose(self, position, orientation=0):

        self._position = position
        self._orientation = orientation
        self._local_planner.set_pose(position, orientation)


    def set_map(self, input_map):
        self._map = input_map
        self._map_height, self._map_width = input_map.shape


    def step(self, action, dt=0.1):

        v_x, v_theta = action
        x, y = self._position
        theta = self._orientation

        new_x = x + v_x * np.cos(theta) * dt
        new_y = y + v_x * np.sin(theta) * dt
        new_theta = theta + v_theta * dt

        position = [new_x, new_y]
        orientation = new_theta

        if self.check_for_collision(position):
            return
        else:
            self.set_pose(position, orientation)


    def check_for_collision(self, position):

        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if self._map[np.clip(int(position[1]) + j, 0,
                                     self._map_height - 1),
                             np.clip(int(position[0]) + i, 0,
                                     self._map_width - 1)] == 1:

                    return True

        return False


    def update_frame(self, frame):

        if self._local_planner._success:
            self._diff_robot_sim.event_source.stop()

        cmd = self._local_planner.get_ctrl_cmd()

        self.step(cmd)

        self._robot_viz.center = (self._position[0],
                                  self._position[1])

        self._robot_heading_viz.set_data(x=self._position[0],
                                         y=self._position[1],
                                         dx=2.0*np.cos(self._orientation),
                                         dy=2.0*np.sin(self._orientation))

        self._robot_trail.append(self._position.copy())
        self._trail_scatter.set_xdata(np.array(self._robot_trail)[:, 0])
        self._trail_scatter.set_ydata(np.array(self._robot_trail)[:, 1])


    def visualize(self):

        fig, ax = plt.subplots()
        ax.imshow(self._map, cmap='binary')
        ax.set_xlim((0, self._map_width))
        ax.set_ylim((0, self._map_height))

        self._robot_trail = []
        self._robot_trail.append(self._position.copy())

        self._robot_heading_viz = mpatches.FancyArrow(self._position[0],
                                                      self._position[1],
                                                      2.0*np.cos(self._orientation),
                                                      2.0*np.sin(self._orientation),
                                                      width=0.4,
                                                      zorder=1)

        self._robot_viz = mpatches.Circle((self._position[0],
                                           self._position[1]),
                                           self._radius,
                                           edgecolor='black', zorder=2)

        ax.plot(self._local_planner._plan[:, 0],
                self._local_planner._plan[:, 1], color='C1', zorder=0)

        self._trail_scatter = ax.plot(np.array(self._robot_trail)[:, 0],
                                      np.array(self._robot_trail)[:, 1])[0]

        ax.add_artist(self._robot_viz)
        ax.add_artist(self._robot_heading_viz)

        ax.set_aspect('equal')

        self._diff_robot_sim = animation.FuncAnimation(fig=fig,
                                                 func=self.update_frame,
                                                 frames=1000,
                                                 interval=5)
        plt.show()


class PurePursuitPlanner:

    def __init__(self,
                 kx=1.0,
                 look_ahead_dist=5.0):

        self._kx = kx
        self._look_ahead_dist = look_ahead_dist

        self._plan_idx = 0

        self._current_position = None
        self._current_orientation = None

        self._success = False
        self._dist_thresh = 1.0


    def set_plan(self, plan):
        self._plan = plan
        self._plan_idx = 0


    def set_vel_limits(self, min_linear_vel, max_linear_vel,
                             min_angular_vel, max_angular_vel):

        self._max_linear_vel  =  max_linear_vel
        self._min_linear_vel  =  min_linear_vel
        self._max_angular_vel =  max_angular_vel
        self._min_angular_vel =  min_angular_vel


    def set_pose(self, position, orientation=0):
        self._current_position = position
        self._current_orientation = orientation


    def get_local_pose(self, waypoint):

        dx = waypoint[0] - self._current_position[0]
        dy = waypoint[1] - self._current_position[1]

        theta = -self._current_orientation
        x_local = dx * np.cos(theta) - dy * np.sin(theta)
        y_local = dx * np.sin(theta) + dy * np.cos(theta)

        return [x_local, y_local]


    def get_waypoint(self):
        for i in range(self._plan_idx, len(self._plan)):
            dist = np.linalg.norm(np.array(self._current_position) - np.array(self._plan[i]))
            if dist <= self._look_ahead_dist:
                if i + 1 < len(self._plan):
                    self._plan_idx = i + 1
                break

        if self._plan_idx == len(self._plan) - 1:
            if np.linalg.norm(np.array(self._current_position) - np.array(self._plan[self._plan_idx])) <= self._dist_thresh:
                self._success = True

        return self._plan[self._plan_idx]


    def get_ctrl_cmd(self):
        waypoint = self.get_waypoint()
        local_position = self.get_local_pose(waypoint)

        x = local_position[0]
        y = local_position[1]

        linear_vel = np.clip(self._kx * x, self._min_linear_vel, self._max_linear_vel)
        angular_vel = np.clip(2 * y / (self._look_ahead_dist ** 2), self._min_angular_vel, self._max_angular_vel)

        return [linear_vel, angular_vel]


