__author__ = 'Murray Tannock'
import sys

import ellipse
from probabilistic_search import *


def step():
    """
    One step of constricted RRT* generation, see paper for more details
    """
    x_rand = sample()
    x_nearest = new_nearest_neighbour(x_rand)
    x_new = steer(x_nearest, x_rand)
    if obstacle_free(x_nearest, x_new):
        X_near = new_neighbourhood(x_new)
        x_min = x_nearest
        c_min = x_nearest.cost + x_nearest.dist_to(x_new)
        for x_near in X_near:
            if obstacle_free(x_near, x_new) and (x_near.cost + x_near.dist_to(x_new) < c_min):
                x_min = x_near
                c_min = (x_near.cost + x_near.dist_to(x_new) < c_min)
        x_new_node = add_node(x_new, x_min, True)
        for x_near in X_near:
            if obstacle_free(x_near, x_new) and (x_new_node.cost + x_near.dist_to(x_new) < x_near.cost):
                x_near.change_parent(x_new_node)
    # Here I check for goal paths and draw the circle
    updated = False
    if shared.root_path:
        updated = goal_path_resolve(shared.root_path[0])
    updated = updated or goal_path_resolve(shared.nodes[-1])
    if updated:
        diameter = shared.root_path_length
        center = [0] * shared.dimensions
        for i in range(shared.dimensions):
            center[i] = shared.root_path[0].coords[i] + shared.root_path[-1].coords[i] / 2
        if shared.region:
            shared.region.remove_from_batch()
        shared.region = ellipse.Ellipse(center[0], center[1], diameter)
        shared.region.add_to_batch()


def sample():
    """
    sampling method for constricted RRT* paper gives more details.
    """
    if shared.root_path_length < sys.maxsize:
        if shared.dimensions == 2:
            # We make a circle
            center = ((shared.root_path[0].coords[0] + shared.root_path[-1].coords[0]) / 2,
                      (shared.root_path[0].coords[1] + shared.root_path[-1].coords[1]) / 2)
            r = shared.root_path_length / 2
            while True:
                coords = sample_unit_ball()
                coords[0] *= r
                coords[1] *= r
                coords[0] += center[0]
                coords[1] += center[1]
                if shared.x_domain[1] > coords[0] > shared.x_domain[0] and shared.y_domain[1] > coords[1] > shared.y_domain[0]:
                    return coords
        elif shared.dimensions == 3:
            # We make a sphere
            center = ((shared.root_path[0].coords[0] + shared.root_path[-1].coords[0]) / 2,
                      (shared.root_path[0].coords[1] + shared.root_path[-1].coords[1]) / 2,
                      (shared.root_path[0].coords[2] + shared.root_path[-1].coords[2]) / 2)
            r = shared.root_path_length / 2
            while True:
                coords = sample_unit_ball()
                coords[0] *= r
                coords[1] *= r
                coords[2] *= r
                coords[0] += center[0]
                coords[1] += center[1]
                coords[2] += center[2]
                if shared.x_domain[1] > coords[0] > shared.x_domain[0] and shared.x_domain[1] > coords[1] > shared.x_domain[0] and shared.x_domain[1] > coords[2] > shared.x_domain[0]:
                    return coords
    else:
        return sample_free()


step.__name__ = "cRRT*"