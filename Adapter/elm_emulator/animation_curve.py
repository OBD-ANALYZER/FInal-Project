import numpy as np


class AnimationCurve:
    def __init__(self, x_range, y_range):
        self.points = []
        self.tangents = {}
        self.x_range = x_range
        self.y_range = y_range

    def duration(self):
        points = sorted(self.points)
        return points[-1][0]

    def add_point(self, x, y):
        self.points.append((x, y))
        self.tangents[(x, y)] = ((self.x_range // 25) + 1, 0)

    def move_point(self, index, x, y):
        if index is not None and 0 <= index < len(self.points):
            old_point = self.points[index]
            new_point = (x, y)
            self.points[index] = new_point
            self.tangents[new_point] = self.tangents.pop(old_point)

    def adjust_tangent(self, index, tx, ty):
        if index is not None and 0 <= index < len(self.points):
            px, py = self.points[index]
            tangent = (tx - px, ty - py)
            self.tangents[(px, py)] = tangent

    def remove_point(self, index):
        if index is not None and 0 <= index < len(self.points):
            point = self.points[index]
            del self.points[index]
            del self.tangents[point]

    def evaluate(self, x):
        points = sorted(self.points)

        for i in range(len(points) - 1):
            p0, p1 = points[i], points[i + 1]
            t0, t1 = self.tangents[p0], self.tangents[p1]

            if p0[0] <= x <= p1[0]:
                t = (x - p0[0]) / (p1[0] - p0[0])

                hermite_basis = np.array([
                    [2, -2, 1, 1],
                    [-3, 3, -2, -1],
                    [0, 0, 1, 0],
                    [1, 0, 0, 0]
                ])
                parameters = np.array([t**3, t**2, t, 1])

                control = np.array([p0[1], p1[1], t0[1], t1[1]])
                basis_with_params = np.matmul(parameters, hermite_basis)
                hermite_blend = np.dot(basis_with_params, control)

                return hermite_blend

        return 0.0
