#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import time
import os
from collections import deque
from tf_transformations import euler_from_quaternion

class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')

        plt.style.use('seaborn')
        self.fig, self.ax1 = plt.subplots(figsize=(9, 9))
        self.fig.suptitle('EKF vs Ground Truth - Trayectoria y Orientación', fontsize=14)
        self.ax1.set_title('Trayectoria')
        self.ax1.set_xlabel('Posición X (m)')
        self.ax1.set_ylabel('Posición Y (m)')
        self.ax1.grid(True)
        self.ax1.set_aspect('equal')

        self.pose_pairs = deque(maxlen=500)  # [(ekf_x, ekf_y, ekf_theta, gt_x, gt_y, gt_theta, t_rel)]

        self.latest_ekf = None
        self.latest_gt = None
        self.counter = 0
        self.downsample_rate = 5  # guarda 1 cada 5 lecturas

        self.start_time = time.time()

        self.ekf_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/ekf_pose',
            self.ekf_callback,
            10)
        self.gt_sub = self.create_subscription(
            PoseStamped,
            '/ground_truth_pose',
            self.gt_callback,
            10)

        plt.ion()
        plt.show(block=False)
        self.timer = self.create_timer(0.2, self.update_plot)

        self.get_logger().info("Visualización iniciada")

    def ekf_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_ekf = (x, y, yaw)

        self.try_sync_and_store()

    def gt_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_gt = (x, y, yaw)

        self.try_sync_and_store()

    def try_sync_and_store(self):
        """Empareja las poses si ambas están disponibles"""
        if self.latest_ekf and self.latest_gt:
            self.counter += 1
            if self.counter % self.downsample_rate == 0:
                t_rel = time.time() - self.start_time
                ekf = self.latest_ekf
                gt = self.latest_gt
                self.pose_pairs.append((*ekf, *gt, t_rel))
                # Reset para evitar usar la misma pose muchas veces
                self.latest_ekf = None
                self.latest_gt = None

    def update_plot(self):
        self.fig.clf()
        gs = self.fig.add_gridspec(1, 3, width_ratios=[1, 0.03, 0.03], wspace=0.2)

        ax_main = self.fig.add_subplot(gs[0])
        ax_cb_ekf = self.fig.add_subplot(gs[1])
        ax_cb_gt = self.fig.add_subplot(gs[2])

        ax_main.set_title('Trayectoria con Orientación Temporal')
        ax_main.set_xlabel('Posición X (m)')
        ax_main.set_ylabel('Posición Y (m)')
        ax_main.grid(True)
        ax_main.set_aspect('equal')

        if len(self.pose_pairs) == 0:
            return

        from matplotlib import cm
        import matplotlib as mpl

        times = [p[6] for p in self.pose_pairs]
        t_min, t_max = min(times), max(times)
        t_norm = [(t - t_min) / (t_max - t_min + 1e-5) for t in times]

        ekf_x = [p[0] for p in self.pose_pairs]
        ekf_y = [p[1] for p in self.pose_pairs]
        gt_x = [p[3] for p in self.pose_pairs]
        gt_y = [p[4] for p in self.pose_pairs]

        ax_main.plot(gt_x, gt_y, color='blue', linestyle='--', linewidth=1.4, label='GT trayectoria')
        ax_main.plot(ekf_x, ekf_y, color='red', linestyle='--', linewidth=1.4, label='EKF trayectoria')

        for i, p in enumerate(self.pose_pairs):
            ex, ey, etheta, gx, gy, gtheta, _ = p
            color_ekf = cm.viridis(t_norm[i])
            color_gt = cm.plasma(t_norm[i])

            ax_main.arrow(ex, ey, 0.3*np.cos(etheta), 0.3*np.sin(etheta),
                          head_width=0.2, head_length=0.3, fc=color_ekf, ec=color_ekf, alpha=0.9)

            ax_main.arrow(gx, gy, 0.3*np.cos(gtheta), 0.3*np.sin(gtheta),
                          head_width=0.2, head_length=0.3, fc=color_gt, ec=color_gt, alpha=0.9)

        ax_main.legend(loc='upper right')

        # Ajuste dinámico
        all_x = ekf_x + gt_x
        all_y = ekf_y + gt_y
        x_margin = max(1.0, (max(all_x) - min(all_x)) * 0.2)
        y_margin = max(1.0, (max(all_y) - min(all_y)) * 0.2)
        ax_main.set_xlim(min(all_x) - x_margin, max(all_x) + x_margin)
        ax_main.set_ylim(min(all_y) - y_margin, max(all_y) + y_margin)

        # Colorbars
        norm = mpl.colors.Normalize(vmin=t_min, vmax=t_max)
        cb1 = mpl.colorbar.ColorbarBase(ax_cb_ekf, cmap=cm.viridis, norm=norm, orientation='vertical')
        cb1.set_label('Tiempo (s) EKF', fontsize=9)

        cb2 = mpl.colorbar.ColorbarBase(ax_cb_gt, cmap=cm.plasma, norm=norm, orientation='vertical')
        cb2.set_label('Tiempo (s) GT', fontsize=9)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = PosePlotter()
    try:
        while rclpy.ok() and plt.fignum_exists(node.fig.number):
            rclpy.spin_once(node, timeout_sec=0.1)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()