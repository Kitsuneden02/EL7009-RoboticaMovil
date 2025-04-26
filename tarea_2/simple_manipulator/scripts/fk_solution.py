import sympy as sym

# TODO: use simpy to re-write your numerical forward kinematics solution, and
# get its jacobian with respect to only the end-effector position

def fk_solution():

    sym.init_printing(use_unicode=True)
    
    # TODO: use these symbols for your solution
    theta_1, theta_2, theta_3, \
            theta_4, theta_5, theta_6 = sym.symbols("theta_1 theta_2 theta_3 theta_4 theta_5 theta_6")

    # TODO: write down you forward kinematics solution here using sympy.
    # NOTE: you should look up the documentation in sympy for:
    #   - symbols
    #   - Matrix
    #   - jacobian
    #   - lambdify

    t_0eef = None

    # TODO: Multiply the homogeneous transformation matrix representing the manipulator's
    # forward kinematics by [0, 0, 0, 1]^T and get the jacobian of the result with respect to
    # our variables of interest 
    # TODO: get a callable ethod for the forward kinematics for the position of the end effector
    # and for evaluating its jacobian
    # NOTE: you should look up the documentation in sympy for:
    #   - jacobian
    #   - lambdify
    # TODO: this method should return both callable functions

    tf_pos = None
    pos_jacobian = None

    # TODO: at this point you may want to ommit the last row of the results

    params = theta_1, theta_2, theta_3, theta_4, theta_5, theta_6

    """fk_function = sym.lambdify(?)
    jac_function = sym.lambdify(?)"""

    #return jac_function, fk_function
    return None, None


if __name__=='__main__':
    fk_solution()

