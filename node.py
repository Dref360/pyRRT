__author__ = 'Murray Tannock'

import math

from pyglet import gl

import shared


class Node(object):
    """
    Node class for use in graphics manipulation, and cost calculations
    """
    def __init__(self, coords, parent=None, node_type="normal"):
        self.coords = coords
        self.parent = parent
        self.children = []
        self.type = node_type
        if self.parent is None:
            self.cost = 0
        else:
            self.cost = self.parent.cost + self.dist_to(self.parent.coords)

        if self.parent is not None:
            self.parent.children.append(self)
            if shared.dimensions == 2:
                self.line = shared.batch.add(2, gl.GL_LINES, None,
                                             ('v2f', (self.parent.coords[0], self.parent.coords[1],
                                                      self.coords[0], self.coords[1])))

        if shared.dimensions == 2:
            if node_type == "normal":
                self.node = shared.batch.add(4, gl.GL_QUADS, None,
                                             ('v2f', (self.coords[0] - 1, self.coords[1] + 1,
                                                      self.coords[0] + 1, self.coords[1] + 1,
                                                      self.coords[0] + 1, self.coords[1] - 1,
                                                      self.coords[0] - 1, self.coords[1] - 1)))
            if node_type == "root":
                self.node = shared.batch.add(4, gl.GL_QUADS, None,
                                             ('v2f', (self.coords[0] - 5, self.coords[1] + 5,
                                                      self.coords[0] + 5, self.coords[1] + 5,
                                                      self.coords[0] + 5, self.coords[1] - 5,
                                                      self.coords[0] - 5, self.coords[1] - 5)),
                                             ('c3B', (255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0)))
            if node_type == "goal":
                self.node = shared.batch.add(4, gl.GL_QUADS, None,
                                             ('v2f', (self.coords[0] - 5, self.coords[1] + 5,
                                                      self.coords[0] + 5, self.coords[1] + 5,
                                                      self.coords[0] + 5, self.coords[1] - 5,
                                                      self.coords[0] - 5, self.coords[1] - 5)),
                                             ('c3B', (0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255, 0)))

    def __str__(self):
        s = "("
        for x in self.coords:
            s += str(x) + ", "
        return s[0:-2] + ")"

    def __repr__(self):
        s = "("
        for x in self.coords:
            s += str(x) + ", "
        return s[0:-2] + ")"

    def dist_to(self, other):
        """
        calculates distance between this node and another point
        :param other: other point
        :return: float distance between points
        """
        dist = 0
        for i, x in enumerate(self.coords):
            dist += (x - other[i])**2
        return math.sqrt(dist)

    def root_path_color(self):
        """
        Colours the path to the root, used in goal highlighting
        :return:
        """
        if shared.dimensions == 2:
            self.line.delete()
            self.line = shared.batch.add(2, gl.GL_LINES, None,
                                         ('v2f', (self.parent.coords[0], self.parent.coords[1],
                                                  self.coords[0], self.coords[1])),
                                         ('c3B', (0, 255, 255,
                                                  0, 255, 255)))
            if self.type == "normal":
                self.node.delete()
                self.node = shared.batch.add(4, gl.GL_QUADS, None,
                                             ('v2f', (self.coords[0] - 2.5, self.coords[1] + 2.5,
                                                      self.coords[0] + 2.5, self.coords[1] + 2.5,
                                                      self.coords[0] + 2.5, self.coords[1] - 2.5,
                                                      self.coords[0] - 2.5, self.coords[1] - 2.5)),
                                             ('c3B', (0, 255, 255,
                                                      0, 255, 255,
                                                      0, 255, 255,
                                                      0, 255, 255)))
                self.line.delete()
                self.line = shared.batch.add(2, gl.GL_LINES, None,
                                             ('v2f', (self.parent.coords[0], self.parent.coords[1],
                                                      self.coords[0], self.coords[1])),
                                             ('c3B', (0, 255, 255,
                                                      0, 255, 255)))

    def deroot_path_color(self):
        """
        Removes colour from the root path, used when this is no longer the best path
        :return: None
        """
        if shared.dimensions == 2:
            if self.type != "root":
                self.line.delete()
                self.line = shared.batch.add(2, gl.GL_LINES, None,
                                             ('v2f', (self.parent.coords[0], self.parent.coords[1],
                                                      self.coords[0], self.coords[1])))
            if self.type == "normal":
                self.node.delete()
                self.node = shared.batch.add(4, gl.GL_QUADS, None,
                                             ('v2f', (self.coords[0] - 1, self.coords[1] + 1,
                                                      self.coords[0] + 1, self.coords[1] + 1,
                                                      self.coords[0] + 1, self.coords[1] - 1,
                                                      self.coords[0] - 1, self.coords[1] - 1)))

    def update_cost(self):
        """
        updates cost of reaching node after parent changes
        :return:
        """
        self.cost = self.parent.cost + self.dist_to(self.parent.coords)
        for child in self.children:
            child.update_cost()

    def change_parent(self, new_parent):
        """
        Changes the parent of node and updates graphics and costs
        :param new_parent:
        :return:
        """
        if self.parent:
            #self.line.delete()
            self.parent.children.remove(self)

        new_parent.children.append(self)
        self.parent = new_parent
        self.update_cost()
        if shared.dimensions == 2:
            self.line = shared.batch.add(2, gl.GL_LINES, None,
                                         ('v2f', (self.parent.coords[0], self.parent.coords[1],
                                                  self.coords[0], self.coords[1])))