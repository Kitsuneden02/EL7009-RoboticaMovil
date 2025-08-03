import numpy as np
import matplotlib.pyplot as plt


class Node:

    def __init__(self, x, y):
        self.position = (x,y)
        self.parent = []


class RRTPlanner:

    def __init__(self,
                 input_map,
                 init_position=None,
                 target_position=None,
                 nb_iterations=2000,
                 traverse_distance=2.0,
                 random_seed=0):

        # get map and its dimensions
        self._map  = input_map

        self._map_height, self._map_width = input_map.shape

        self._nb_iterations = nb_iterations
        self._traverse_distance = traverse_distance

        self._init_position = init_position

        if self._init_position is None:
            self._init_position = [self._map_width // 2, self._map_height // 2]

        self._target_position = target_position

        # Initialize tree
        self._tree = []
        self._tree.append(Node(self._init_position[0], self._init_position[1]))

        self._plan = None

        np.random.seed(random_seed)


    def set_random_seed(self, random_seed):
        np.random.seed(random_seed)
        self.reset_tree()


    def reset_tree(self):
        self._tree = [Node(self._init_position[0], self._init_position[1])]
        pass


    def set_init_position(self, init_position):
        self._init_position = init_position
        self.reset_tree()


    def set_target_position(self, target_position):
        self._target_position = target_position


    def sample_random_position(self):

        sampled_x = np.random.uniform(0, self._map_width)
        sampled_y = np.random.uniform(0, self._map_height)

        return (sampled_x, sampled_y)


    def get_nearest_neighbour(self, position):

        min_distance = float('inf')
        min_index = -1

        for idx, node in enumerate(self._tree):
            node_x, node_y = node.position
            distance = np.linalg.norm(np.array(position) - np.array([node_x, node_y]))
            if distance < min_distance:
                min_distance = distance
                min_index = idx

        return min_index, self._tree[min_index]


    def get_new_position(self, random_position, nearest_position):

        dx = random_position[0] - nearest_position[0]
        dy = random_position[1] - nearest_position[1]

        angle = np.arctan2(dy, dx)

        new_x = nearest_position[0] + self._traverse_distance * np.cos(angle)
        new_y = nearest_position[1] + self._traverse_distance * np.sin(angle)

        new_x = np.clip(new_x, 0, self._map_width - 1)
        new_y = np.clip(new_y, 0, self._map_height - 1)

        return new_x, new_y


    def recover_plan(self):

        plan = []

        current_node = self._tree[-1]
        while current_node.parent != []:
            plan.append(list(current_node.position))
            current_node = current_node.parent[1]

        plan.append(list(self._init_position))  # agrega el nodo inicial
        plan.reverse()
        return np.array(plan)

    def check_for_collision(self, new_position):

        for i in [-2, 0, 2]:
            for j in [-2, 0, 2]:
                x = int(np.clip(new_position[0] + i, 0, self._map_width - 1))
                y = int(np.clip(new_position[1] + j, 0, self._map_height - 1))
                if self._map[y, x] == 1:
                    return True
        return False


    def generate_rrt(self):

        for _ in range(self._nb_iterations):
            random_position = self.sample_random_position()
            nearest_idx, nearest_node = self.get_nearest_neighbour(random_position)
            nearest_position = nearest_node.position
            new_position = self.get_new_position(random_position, nearest_position)

            if self.check_for_collision(new_position):
                continue

            new_node = Node(new_position[0], new_position[1])
            new_node.parent = [nearest_idx, nearest_node]
            self._tree.append(new_node)

            if self._target_position is not None:
                distance_to_target = np.linalg.norm(np.array(new_position) - np.array(self._target_position))
                if distance_to_target <= self._traverse_distance:
                    target_node = Node(self._target_position[0], self._target_position[1])
                    target_node.parent = [len(self._tree) - 1, new_node]
                    self._tree.append(target_node)
                    self._plan = self.recover_plan()
                    break

        return self._plan


    def plot_rrt(self):

        positions = []
        edges = []

        for idx, node in enumerate(self._tree):
            positions.append(list(node.position))
            if node.parent != []:
                edges.append([node.parent[0], idx])

        positions = np.array(positions)
        edges = np.array(edges)

        x_pos = positions[:, 0]
        y_pos = positions[:, 1]

        # Plotting
        fig, ax = plt.subplots()

        ax.imshow(self._map, cmap='binary')
        ax.set_xlim((0, self._map_width))
        ax.set_ylim((0, self._map_height))

        ax.plot(x_pos[edges.T], y_pos[edges.T], color='C0')
        ax.scatter(positions[:,0], positions[:,1], s=20)

        if self._target_position is not None:
            ax.scatter(self._target_position[0], self._target_position[1], s=50)

        if self._plan is not None:
            ax.scatter(self._plan[:,0], self._plan[:,1], s=20)
            ax.plot(self._plan[:,0], self._plan[:,1], color='red')

        ax.scatter(self._init_position[0], self._init_position[1], s=50)

        ax.set_aspect('equal')
        plt.show()
