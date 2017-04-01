from pyglet import gl

import shared


class Obstacle(object):
    def __init__(self, coords, dims):
        # right now we restrict obstacles to rectangles, eventually extend to polygons using triangle fans
        self.coords = coords
        self.dims = dims
        self.shape = None

    #Only used when in 2D
    def add_to_default_batch(self):
        if shared.dimensions == 2:
            if self.coords[1] < 50:
                self.dims[1] += self.coords[1] - 50
                self.coords[1] = 50
            if self.dims[0] < shared.STEP_SIZE or self.dims[1] < shared.STEP_SIZE:
                self.delete()
                return
            self.shape = shared.batch.add(4, gl.GL_QUADS, None,
                                          ('v2f', (self.coords[0], self.coords[1],
                                                   self.coords[0] + self.dims[0], self.coords[1],
                                                   self.coords[0] + self.dims[0], self.coords[1] + self.dims[1],
                                                   self.coords[0], self.coords[1] + self.dims[1])))

    def collides_with(self, point):
        """
        :param point: List of coordinates for 2-4 dimensions
        :return: Boolean True if a collision occurs
        """
        collide = True
        for i, x in enumerate(self.coords):
            collide = collide and x + self.dims[i] > point[i] > x
        return  collide

    def delete(self):
        """
        Deletes the obstacle
        Deletes the graphics object if it exists then remove from the obstacles list
        :return: None
        """
        if self.shape is not None:
            self.shape.delete()
        if self in shared.obstacles:
            shared.obstacles.remove(self)