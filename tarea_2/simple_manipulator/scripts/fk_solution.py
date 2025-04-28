import sympy as sym
import numpy as np

def fk_solution():
    sym.init_printing(use_unicode=True)
    
    # Símbolos para las articulaciones
    theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = sym.symbols("theta_1 theta_2 theta_3 theta_4 theta_5 theta_6")

    # Dimensiones del brazo
    L_base = 0.1
    L_shoulder_y = 0.15
    L_shoulder_z = 0.2
    L_arm_y = -0.1
    L_arm_z = 0.8
    L_forearm_y = 0.1
    L_forearm_z = 0.8
    L_wrist1_x = -0.1
    L_wrist1_z = 0.3
    L_wrist2_z = 0.2
    L_wrist3_z = 0.1
    
    # Rotación alrededor de Z (shoulder_yaw)
    t_01 = sym.Matrix([
        [sym.cos(theta_1), -sym.sin(theta_1), 0, 0],
        [sym.sin(theta_1), sym.cos(theta_1), 0, 0],
        [0, 0, 1, L_base],
        [0, 0, 0, 1]
    ])
    
    # Rotación alrededor de Y (shoulder_pitch)
    t_12 = sym.Matrix([
        [sym.cos(theta_2), 0, sym.sin(theta_2), L_shoulder_y],
        [0, 1, 0, 0],
        [-sym.sin(theta_2), 0, sym.cos(theta_2), L_shoulder_z],
        [0, 0, 0, 1]
    ])
    
    # Rotación alrededor de Y (elbow)
    t_23 = sym.Matrix([
        [sym.cos(theta_3), 0, sym.sin(theta_3), L_arm_y],
        [0, 1, 0, 0],
        [-sym.sin(theta_3), 0, sym.cos(theta_3), L_arm_z],
        [0, 0, 0, 1]
    ])
    
    # Rotación alrededor de X (wrist_1)
    t_34 = sym.Matrix([
        [1, 0, 0, L_forearm_y],
        [0, sym.cos(theta_4), -sym.sin(theta_4), 0],
        [0, sym.sin(theta_4), sym.cos(theta_4), L_forearm_z],
        [0, 0, 0, 1]
    ])
    
    # Rotación alrededor de Y (wrist_2)
    t_45 = sym.Matrix([
        [sym.cos(theta_5), 0, sym.sin(theta_5), L_wrist1_x],
        [0, 1, 0, 0],
        [-sym.sin(theta_5), 0, sym.cos(theta_5), L_wrist1_z],
        [0, 0, 0, 1]
    ])
    
    # Rotación alrededor de X (wrist_3)
    t_56 = sym.Matrix([
        [1, 0, 0, 0],
        [0, sym.cos(theta_6), -sym.sin(theta_6), 0],
        [0, sym.sin(theta_6), sym.cos(theta_6), L_wrist2_z],
        [0, 0, 0, 1]
    ])
    
    # Transformación fija al efector final
    t_6eef = sym.Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, L_wrist3_z],
        [0, 0, 0, 1]
    ])
    
    t_0eef = t_01 * t_12 * t_23 * t_34 * t_45 * t_56 * t_6eef
    
    pos_vector = sym.Matrix([0, 0, 0, 1])
    
    tf_pos = t_0eef * pos_vector
    tf_pos = tf_pos[:3, 0]
    
    params = (theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)
    pos_jacobian = tf_pos.jacobian(params)
    
    fk_function = sym.lambdify(params, tf_pos)
    jac_function = sym.lambdify(params, pos_jacobian)
    
    return jac_function, fk_function

if __name__=='__main__':
    fk_solution()