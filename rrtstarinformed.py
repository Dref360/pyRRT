__author__ = 'Murray Tannock'
import sys

import ellipse
from probabilistic_search import *


def step():
    """
    One step of informed RRT* generation, see paper for more details
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

        # Here I check for goal regions and draw the ellipse
        updated = False
        if shared.root_path:
            updated = goal_path_resolve(shared.root_path[0])
        updated = updated or goal_path_resolve(shared.nodes[-1])
        if updated:
            major = shared.root_path_length
            minor = math.sqrt(
                major ** 2 - shared.root_path[0].dist_to(shared.root_path[-1].coords) ** 2)
            angle = math.atan2(shared.root_path[0].coords[1] - shared.root_path[-1].coords[1],
                               shared.root_path[0].coords[0] - shared.root_path[-1].coords[0])
            center = (shared.root_path[0].coords[0] + shared.root_path[-1].coords[0] / 2,
                      shared.root_path[0].coords[1] + shared.root_path[-1].coords[1] / 2)
            if shared.region:
                shared.region.remove_from_batch()
            shared.region = ellipse.Ellipse(center[0], center[1], major, minor, angle)
            shared.region.add_to_batch()


def sample():
    """
    sampling method for constricted RRT* paper gives more details.
    """
    if shared.root_path_length < sys.maxsize:
        if shared.dimensions == 2:
            # We make a circle
            major = shared.root_path_length
            minor = math.sqrt(
                major ** 2 - shared.root_path[0].dist_to(shared.root_path[-1].coords) ** 2)
            angle = math.atan2(shared.root_path[0].coords[1] - shared.root_path[-1].coords[1],
                               shared.root_path[0].coords[0] - shared.root_path[-1].coords[0])

            center = ((shared.root_path[0].coords[0] + shared.root_path[-1].coords[0]) / 2,
                      (shared.root_path[0].coords[1] + shared.root_path[-1].coords[1]) / 2)
            r = shared.root_path_length / 2
            while True:

                [x, y] = sample_unit_ball()
                x *= major / 2
                y *= minor / 2
                x, y = x * math.cos(angle) - y * math.sin(angle), \
                       x * math.sin(angle) + y * math.cos(angle)
                x += center[0]
                y += center[1]
                if shared.x_domain[1] > x > shared.x_domain[0] and shared.y_domain[1] > y > shared.y_domain[0]:
                    return [x, y]
        else:
            # We make a sphere
            major = shared.root_path_length
            minor = math.sqrt(major ** 2 - shared.root_path[0].dist_to(shared.root_path[-1].coords)**2)

            angle = math.atan2(shared.root_path[0].coords[1] - shared.root_path[-1].coords[1],
                               shared.root_path[0].coords[0] - shared.root_path[-1].coords[0])

            center = ((shared.root_path[0].coords[0] + shared.root_path[-1].coords[0]) / 2,
                      (shared.root_path[0].coords[1] + shared.root_path[-1].coords[1]) / 2,
                      (shared.root_path[0].coords[2] + shared.root_path[-1].coords[2]) / 2)
            r = shared.root_path_length / 2
            while True:
                coords = sample_unit_ball()
                coords[0] *= major / 2
                coords[1] *= minor / 2
                coords[2] *= minor / 2
                x, y = coords[0] * math.cos(angle) - coords[1] * math.sin(angle), \
                       coords[0] * math.sin(angle) + coords[1] * math.cos(angle)
                z = coords[0] * math.cos(angle) - coords[1] * math.sin(angle)
                x += center[0]
                y += center[1]
                z += center[2]
                if shared.x_domain[1] > x > shared.x_domain[0] and shared.y_domain[1] > y > shared.y_domain[0] and shared.x_domain[1] > z > shared.x_domain[0]:
                    return [x, y, z]
    else:
        return sample_free()


step.__name__ = "iRRT*"