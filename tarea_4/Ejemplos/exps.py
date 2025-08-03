import numpy as np
import time

from pure_pursuit_planner import DifferentialRobot
from pure_pursuit_planner import PurePursuitPlanner
from rrt_planner import RRTPlanner

# For RRT and integration tests
def add_map_obstacles_1(input_map):
    # DO NOT MODIFY
    assert input_map.shape == (64, 64)
    input_map[20:22, 10:50] = 1
    input_map[22:42, 48:50] = 1
    input_map[40:42, 10:50] = 1

def add_map_obstacles_2(input_map):
    # DO NOT MODIFY
    assert input_map.shape == (64, 64)
    input_map[10:12, 14:50] = 1
    input_map[20:22, 14:50] = 1
    input_map[40:42, 14:50] = 1
    input_map[50:52, 14:50] = 1
    input_map[5:59 ,  7:9 ] = 1
    input_map[5:59 , 55:57] = 1


def add_map_obstacles_3(input_map):
    # DO NOT MODIFY
    assert input_map.shape == (64, 64)
    input_map[10:12, 0:60] = 1
    input_map[20:22, 5:64] = 1
    input_map[30:32, 0:60] = 1
    input_map[40:42, 5:64] = 1
    input_map[50:52, 0:60] = 1


def rrt_exps(exp:str):

    ''' Ejemplo!!
    max_attemts = 1

    input_map = np.zeros((64, 64))
    add_map_obstacles_2(input_map)

    rrt_planner = RRTPlanner(input_map)
    rrt_planner.set_init_position([2, 60])
    rrt_planner.set_target_position([2, 2])

    plan = rrt_planner.generate_rrt()

    if plan is None:
        counter = 1
        while plan is None:
            rrt_planner.set_random_seed(counter)
            plan = rrt_planner.generate_rrt()
            counter+=1
            if counter > max_attemts:
                break

    rrt_planner.plot_rrt()
    '''
    max_attemps = 1 if exp in ["1a", "1b", "1c"] else 1

    if exp == "1a":
        # 1.a) Mapa vacío 64x64, taverse_distance = 2.0
        for iterations in [10, 100, 500]:
            print(f"\n[Mapa vacío 64x64] Iteraciones: {iterations}, traverse_distance = 2.0")
            input_map = np.zeros((64, 64))
            rrt = RRTPlanner(input_map, nb_iterations=iterations, traverse_distance=2.0)
            rrt.set_init_position([5, 5])
            rrt.set_target_position([58, 58])
            start_time = time.time()
            plan = rrt.generate_rrt()

            if plan is None:
                counter = 1
                while plan is None:
                    rrt.set_random_seed(counter)
                    plan = rrt.generate_rrt()
                    counter+=1
                    if counter > max_attemps:
                        break

            print(f"Plan: {'Encontrado' if plan is not None else 'No encontrado'}. Tiempo de ejecución: {time.time() - start_time} segundos.")

            rrt.plot_rrt()

    if exp == "1b":
        # 1.b) Mapa vacío 64x64, nb_iterations = 200
        for dist in [1.0, 2.0, 4.0]:
            print(f"\n[Mapa vacío 64x64] nb_iterations = 200, traverse_distance = {dist}")
            input_map = np.zeros((64, 64))
            rrt = RRTPlanner(input_map, nb_iterations=200, traverse_distance=dist)
            rrt.set_init_position([5, 5])
            rrt.set_target_position([58, 58])
            start_time = time.time()
            plan = rrt.generate_rrt()

            if plan is None:
                counter = 1
                while plan is None:
                    rrt.set_random_seed(counter)
                    plan = rrt.generate_rrt()
                    counter+=1
                    if counter > max_attemps:
                        break

            print(f"Plan: {'Encontrado' if plan is not None else 'No encontrado'}. Tiempo de ejecución: {time.time() - start_time} segundos.")
            rrt.plot_rrt()
    
    if exp == "1c":
        # 1.c) Se repite a) y b) pero con un mapa vacío de 64x128
        for iterations in [10, 100, 500]:
            print(f"\n[Mapa vacío 64x128] Iteraciones: {iterations}, traverse_distance = 2.0")
            input_map = np.zeros((64, 128))
            rrt = RRTPlanner(input_map, nb_iterations=iterations, traverse_distance=2.0)
            rrt.set_init_position([5, 5])
            rrt.set_target_position([120, 58])
            start_time = time.time()
            plan = rrt.generate_rrt()

            if plan is None:
                counter = 1
                while plan is None:
                    rrt.set_random_seed(counter)
                    plan = rrt.generate_rrt()
                    counter+=1
                    if counter > max_attemps:
                        break

            print(f"Plan: {'Encontrado' if plan is not None else 'No encontrado'}. Tiempo de ejecución: {time.time() - start_time} segundos.")

            rrt.plot_rrt()

        for dist in [1.0, 2.0, 4.0]:
            print(f"\n[Mapa vacío 64x128] nb_iterations = 200, traverse_distance = {dist}")
            input_map = np.zeros((64, 128))
            rrt = RRTPlanner(input_map, nb_iterations=200, traverse_distance=dist)
            rrt.set_init_position([5, 5])
            rrt.set_target_position([120, 58])
            start_time = time.time()
            plan = rrt.generate_rrt()

            if plan is None:
                counter = 1
                while plan is None:
                    rrt.set_random_seed(counter)
                    plan = rrt.generate_rrt()
                    counter+=1
                    if counter > max_attemps:
                        break

            print(f"Plan: {'Encontrado' if plan is not None else 'No encontrado'}. Tiempo de ejecución: {time.time() - start_time} segundos.")
            rrt.plot_rrt()

    if exp == "1d":
        # 1.d) Mapa vacío 64x64, nb_iterations = 200
        for iterations in [500, 1000, 2000, 4000]:
            print(f"\n[Mapa vacío 64x64] Iteraciones: {iterations}, traverse_distance = 2.0")
            input_map = np.zeros((64, 64))
            rrt = RRTPlanner(input_map, nb_iterations=iterations, traverse_distance=2.0)
            rrt.set_init_position([5, 5])
            rrt.set_target_position([55, 32])
            add_map_obstacles_1(input_map=input_map)
            start_time = time.time()
            plan = rrt.generate_rrt()

            if plan is None:
                counter = 1
                while plan is None:
                    rrt.set_random_seed(counter)
                    plan = rrt.generate_rrt()
                    counter+=1
                    if counter > max_attemps:
                        break

            print(f"Plan: {'Encontrado' if plan is not None else 'No encontrado'}. Tiempo de ejecución: {time.time() - start_time} segundos.")
            rrt.plot_rrt()

    if exp == "1e":
        # 1.e) Mapa vacío 64x64, nb_iterations = 200
        for iterations in [500, 1000, 2000, 4000]:
            print(f"\n[Mapa vacío 64x64] Iteraciones: {iterations}, traverse_distance = 2.0")
            input_map = np.zeros((64, 64))
            rrt = RRTPlanner(input_map, nb_iterations=iterations, traverse_distance=2.0)
            rrt.set_init_position([2, 60])
            rrt.set_target_position([60, 12])
            add_map_obstacles_2(input_map=input_map)
            start_time = time.time()
            plan = rrt.generate_rrt()

            if plan is None:
                counter = 1
                while plan is None:
                    rrt.set_random_seed(counter)
                    plan = rrt.generate_rrt()
                    counter+=1
                    if counter > max_attemps:
                        break

            print(f"Plan: {'Encontrado' if plan is not None else 'No encontrado'}. Tiempo de ejecución: {time.time() - start_time} segundos.")
            rrt.plot_rrt()

    if exp == "1f":
        # 1.f) Mapa vacío 64x64, nb_iterations = 200
        for iterations in [5000, 10000, 20000]:
            print(f"\n[Mapa vacío 64x64] Iteraciones: {iterations}, traverse_distance = 2.0")
            input_map = np.zeros((64, 64))
            rrt = RRTPlanner(input_map, nb_iterations=iterations, traverse_distance=2.0)
            rrt.set_init_position([2, 60])
            rrt.set_target_position([2, 2])
            add_map_obstacles_3(input_map=input_map)
            start_time = time.time()
            plan = rrt.generate_rrt()

            if plan is None:
                counter = 1
                while plan is None:
                    rrt.set_random_seed(counter)
                    plan = rrt.generate_rrt()
                    counter+=1
                    if counter > max_attemps:
                        break

            print(f"Plan: {'Encontrado' if plan is not None else 'No encontrado'}. Tiempo de ejecución: {time.time() - start_time} segundos.")
            rrt.plot_rrt()


def pure_pursuit_exps(exp:str, ang_vel_lim:bool):

    input_map = np.zeros((64, 64))

    # Parametric plan generation (do not modify the plan)
    time = np.linspace(-np.pi, np.pi, 200)

    x_position_plan = np.sin(time)**3
    y_position_plan = (13 * np.cos(time) - (5 * np.cos(2 * time)) - \
                      ( 2 * np.cos(3 * time)) - (np.cos(4 * time))) / 16.

    y_position_plan -= min(y_position_plan) + 1.0

    plan = np.zeros((len(time), 2))
    plan[:, 0] = x_position_plan
    plan[:, 1] = y_position_plan

    # Plan scaling
    plan = plan * 30 + 32

    kx = 1
    look_ahead_distance = 5.0 if exp == "2a" else 1.0 if exp == "2b" else 10.0
    initial_pose_list = [[30, 10], [20, 30]]

    for initial_pose in initial_pose_list:
        local_planner=PurePursuitPlanner(kx=kx, look_ahead_dist=look_ahead_distance)

        diff_robot = DifferentialRobot(local_planner=local_planner, min_angular_vel=-0.1, max_angular_vel=0.1) if ang_vel_lim \
                else DifferentialRobot(local_planner=local_planner)
        
        diff_robot.set_map(input_map)

        diff_robot.set_pose(initial_pose)

        diff_robot._local_planner.set_plan(plan)

        diff_robot.visualize()


def integration_exps():

    obstacle_funcs = [add_map_obstacles_1, add_map_obstacles_2, add_map_obstacles_3]
    lookahead_values = [5.0, 1.0, 10.0]

    for lookahead in lookahead_values:
        print(f"Pruebas con look_ahead_distance = {lookahead}")
        for idx, add_obstacles in enumerate(obstacle_funcs):
            print(f"\nMapa con obstáculos {idx+1}")

            input_map = np.zeros((64, 64))
            add_obstacles(input_map)

            diff_robot = DifferentialRobot(local_planner=PurePursuitPlanner(kx=1, look_ahead_dist=lookahead))
            rrt_planner = RRTPlanner(input_map, nb_iterations=2000, traverse_distance=2.0)
            diff_robot.set_map(input_map)

            if idx == 0:
                diff_robot.set_pose([32, 32])
                rrt_planner.set_init_position([32, 32])
                rrt_planner.set_target_position([55, 32])
            elif idx == 1:
                diff_robot.set_pose([2, 60])
                rrt_planner.set_init_position([2, 60])
                rrt_planner.set_target_position([60, 12])
            else:
                diff_robot.set_pose([2, 60])
                rrt_planner.set_init_position([2, 60])
                rrt_planner.set_target_position([2, 2])

            plan = rrt_planner.generate_rrt()

            max_attemts = 10
            if plan is None:
                counter = 1
                while plan is None:
                    rrt_planner.set_random_seed(counter)
                    plan = rrt_planner.generate_rrt()
                    counter+=1
                    if counter > max_attemts:
                        break
            
            if plan is None:
                print("No hubo plan.")
            else:
                print("Plan encontrado.")

            diff_robot._local_planner.set_plan(plan)

            try:
                rrt_planner.plot_rrt()
                diff_robot.visualize()
            except:
                pass

def integration_exps_particular():
    input_map = np.zeros((64, 64))
    add_map_obstacles_3(input_map)

    diff_robot = DifferentialRobot(local_planner=PurePursuitPlanner(kx=1, look_ahead_dist=1.0))
    rrt_planner = RRTPlanner(input_map, nb_iterations=20000, traverse_distance=2.0)
    diff_robot.set_map(input_map)


    #diff_robot.set_pose([2, 62])
    #rrt_planner.set_init_position([2, 62])
    #rrt_planner.set_target_position([60, 12])

    diff_robot.set_pose([2, 60])
    rrt_planner.set_init_position([2, 60])
    rrt_planner.set_target_position([2, 2])

    plan = rrt_planner.generate_rrt()

    max_attemts = 10
    if plan is None:
        counter = 1
        while plan is None:
            rrt_planner.set_random_seed(counter)
            plan = rrt_planner.generate_rrt()
            counter+=1
            if counter > max_attemts:
                break
    
    if plan is None:
        print("No hubo plan.")
    else:
        print("Plan encontrado.")

    diff_robot._local_planner.set_plan(plan)

    try:
        rrt_planner.plot_rrt()
        diff_robot.visualize()
    except:
        pass

if __name__ == '__main__':

    ppp_exps = ["2a", "2b", "2c"]
    #rrt_exps("1e")
    #for exp in ppp_exps:
    #    pure_pursuit_exps(exp, True)
    integration_exps_particular()
