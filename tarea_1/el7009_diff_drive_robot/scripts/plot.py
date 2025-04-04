#!/usr/bin/env python3
import matplotlib.pyplot as plt

def plot_trajectories(x_r, y_r, x_o, y_o, markers_r=None, markers_o=None):

    plt.figure(figsize=(10, 6))
    plt.plot(x_r, y_r, 'r-', label='Trayectoria Real', alpha=0.85)
    plt.plot(x_o, y_o, 'b-', label='Trayectoria Odom', alpha=0.7)

    if markers_r:
        x_markers = [x_r[i] for i in markers_r]
        y_markers = [y_r[i] for i in markers_r]
        plt.plot(x_markers, y_markers, 'ro', label='Puntos Real', markersize=8)
    
    if markers_o:
        x_markers = [x_o[i] for i in markers_o]
        y_markers = [y_o[i] for i in markers_o]
        plt.plot(x_markers, y_markers, 'bo', label='Puntos Odom', markersize=8)

    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Comparación de Trayectorias\n10 Vueltas a la caja')
    plt.grid(True)
    plt.legend()
    plt.axis('equal') 
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    x_r = [0, 0, -2.9, -2.91, -0.01, 0.06, -2.84, -2.91, -0.01, 0.1, -2.78, -2.93, -0.03, 0.14, -2.76, -2.94, -0.04, 0.23, -2.66, -3.07, -0.19, 0.22, -2.66, -3.12, -0.24, 0.22, -2.66, -3.14, -0.26, 0.21, -2.67, -3.19, -0.31, 0.19, -2.68, -3.23, -0.36, 0.18, -2.69, -3.3, -0.44]
    y_r = [0, 3.76, 3.76, 0.01, 0, 3.75, 3.8, 0.05, -0.01, 3.73, 3.89, 0.09, -0.02, 3.72, 3.86, 0.11, -0.04, 3.71, 3.92, 0.19, -0.12, 3.61, 3.93, 0.2, -0.15, 3.57, 3.92, 0.19, -0.17, 3.55, 3.91, 0.2, -0.21, 3.51, 3.9, 0.18, -0.24, 3.47, 3.89, 0.19, -0.29]
    x_o = [0, 0.12, -2.75, -3.15, -0.3, 0.36, -2.44, -3.4, -0.66, 0.59, -2.09, -3.6, -1.02, 0.73, -1.75, -3.72, -1.32, 0.85, -1.4, -3.81, -1.69, 0.94, -1.02, -3.86, -2.07, 0.93, -0.69, -3.83, -2.39, 0.91, -0.34, -3.75, -2.7, 0.83, -0.04, -3.65, -2.95, 0.72, 0.24, -3.46, -3.22]
    y_o = [0, 3.73, 3.96, 0.25, -0.16, 3.51, 4.15, 0.53, -0.32, 3.21, 4.28, 0.86, -0.42, 2.88, 4.32, 1.14, -0.46, 2.57, 4.35, 1.5, -0.45, 2.21, 4.32, 1.88, -0.39, 1.85, 4.22, 2.22, -0.28, 1.49, 4.09, 2.57, -0.12, 1.14, 3.69, 2.86, 0.07, 0.83, 3.68, 3.2, 0.33]
    
    markers_r = list(range(41))
    markers_o = list(range(41)) 
    
    # Generar gráfico
    plot_trajectories(x_r, y_r, x_o, y_o, markers_r, markers_o)