from tkinter import *

import numpy as np

from elm_emulator.animation_curve import AnimationCurve


class AnimationCurveEditor(Frame):
    def __init__(self, parent):
        #Frame.__init__(self, parent)

        self.canvas = Canvas(parent, bg="white", width=800, height=400)
        self.canvas.grid(sticky=(N, W, E, S))

        self.x_range = (0, 10000)
        self.y_range = (0, 120)
        self.curve = AnimationCurve((self.x_range[1] - self.x_range[0]), (self.y_range[1] - self.y_range[0]))
        self.selected_point = None
        self.zoom_level = 1.0
        self.padding = {
            'top': 25,
            'right': 25,
            'bottom': 50,
            'left': 75
        }

        self.canvas.bind("<ButtonPress-1>", self.select_point_for_move)
        self.canvas.bind("<B1-Motion>", self.move_point)
        self.canvas.bind("<Shift-B1-Motion>", self.adjust_tangent)
        self.canvas.bind("<ButtonRelease-1>", self.deselect_point)

        self.canvas.bind("<ButtonPress-3>", self.select_point_for_removal)
        self.canvas.bind("<ButtonRelease-3>", self.remove_point)

        self.canvas.bind("<MouseWheel>", self.zoom)

        self.draw_curve()

    def world_to_canvas(self, x, y):
        canvas_width = self.canvas.winfo_width() - self.padding['left'] - self.padding['right']
        canvas_height = self.canvas.winfo_height() - self.padding['top'] - self.padding['bottom']
        canvas_x = self.padding['left'] + (x - self.x_range[0]) / (self.x_range[1] - self.x_range[0]) * canvas_width * self.zoom_level
        canvas_y = self.padding['top'] + canvas_height - (y - self.y_range[0]) / (self.y_range[1] - self.y_range[0]) * canvas_height * self.zoom_level
        return canvas_x, canvas_y

    def canvas_to_world(self, x, y): # receives 'event' coordinates (0,0 is canvas' top-left, 600,300 is canvas' bottom-right)
        canvas_width = self.canvas.winfo_width() - self.padding['left'] - self.padding['right']
        canvas_height = self.canvas.winfo_height() - self.padding['top'] - self.padding['bottom']
        world_x = ((x - self.padding['left']) / (canvas_width * self.zoom_level)) * (self.x_range[1] - self.x_range[0]) + self.x_range[0]
        world_y = ((canvas_height - (y - self.padding['top'])) / (canvas_height * self.zoom_level)) * (self.y_range[1] - self.y_range[0]) + self.y_range[0]
        return world_x, world_y # returns "world", or more like 'graph' coordinates (0,0 is graph origo, 1000,100 is 1000ms, 100kmh)

    def select_point_for_move(self, event):
        x, y = self.canvas_to_world(event.x, event.y)
        # print(f"ex = {event.x}, ey = {event.y}, wx = {x}, wy = {y}")
        for i, (px, py) in enumerate(self.curve.points):
            if abs(px - x) < ((self.x_range[1] - self.x_range[0]) // 100 + 1) and abs(py - y) < ((self.y_range[1] - self.y_range[0]) // 100) + 1:
                self.selected_point = i
                return
        self.curve.add_point(x, y)
        self.draw_curve()

    def move_point(self, event):
        if self.selected_point is not None:
            x, y = self.canvas_to_world(event.x, event.y)
            self.curve.move_point(self.selected_point, x, y)
            self.draw_curve()

    def adjust_tangent(self, event):
        if self.selected_point is not None:
            x, y = self.canvas_to_world(event.x, event.y)
            self.curve.adjust_tangent(self.selected_point, x, y)
            self.draw_curve()

    def deselect_point(self, event):
        self.selected_point = None

    def select_point_for_removal(self, event):
        x, y = self.canvas_to_world(event.x, event.y)
        for i, (px, py) in enumerate(self.curve.points):
            if abs(px - x) < ((self.x_range[1] - self.x_range[0]) // 100 + 1) and abs(py - y) < ((self.y_range[1] - self.y_range[0]) // 100) + 1:
                self.selected_point = i
                return

    def remove_point(self, event):
        if self.selected_point is not None:
            self.curve.remove_point(self.selected_point)
            self.selected_point = None
            self.draw_curve()

    def draw_curve(self):
        self.canvas.delete("all")
        self.draw_axes()
        self.draw_points()
        self.draw_bezier_curve()

    def draw_axes(self):
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()

        for x in np.linspace(self.x_range[0], self.x_range[1], num=11):
            canvas_x, _ = self.world_to_canvas(x, 0)
            self.canvas.create_line(canvas_x, self.padding['top'], canvas_x, height - self.padding['bottom'], fill="grey", dash=(4, 2))
            if self.padding['left'] <= canvas_x <= width - self.padding['right']:
                self.canvas.create_text(canvas_x, height - self.padding['bottom'] + 15, text=f"{x/1000.0:.1f} s", fill="grey")

        for y in np.linspace(self.y_range[0], self.y_range[1], num=13):
            _, canvas_y = self.world_to_canvas(0, y)
            self.canvas.create_line(self.padding['left'], canvas_y, width - self.padding['right'], canvas_y, fill="grey", dash=(4, 2))
            if self.padding['top'] <= canvas_y <= height - self.padding['bottom']:
                self.canvas.create_text(self.padding['left'] - 32, canvas_y, text=f"{y:.0f} km/h", fill="grey")

    def draw_points(self):
        for x, y in self.curve.points:
            canvas_x, canvas_y = self.world_to_canvas(x, y)
            self.canvas.create_oval(canvas_x - 5, canvas_y - 5, canvas_x + 5, canvas_y + 5, fill="red")
            tangent = self.curve.tangents[(x, y)]
            t1_x, t1_y = self.world_to_canvas(x + tangent[0], y + tangent[1])
            t2_x, t2_y = self.world_to_canvas(x - tangent[0], y - tangent[1])
            self.canvas.create_line(canvas_x, canvas_y, t1_x, t1_y, fill="blue")
            self.canvas.create_line(canvas_x, canvas_y, t2_x, t2_y, fill="blue")

    def draw_bezier_curve(self):
        if len(self.curve.points) < 2:
            return

        points = sorted(self.curve.points)
        curve_points = []

        for i in range(len(points) - 1):
            p0, p1 = points[i], points[i + 1]
            t0, t1 = self.curve.tangents[p0], self.curve.tangents[p1]

            for t in np.linspace(0, 1, 100):
                hermite_basis = np.array([
                    [2, -2, 1, 1],
                    [-3, 3, -2, -1],
                    [0, 0, 1, 0],
                    [1, 0, 0, 0]
                ])
                parameters = np.array([t**3, t**2, t, 1])

                controls = np.array([p0, p1, t0, t1])
                basis_with_params = np.matmul(parameters, hermite_basis)
                x, y = np.dot(basis_with_params, controls[:, 0]), np.dot(basis_with_params, controls[:, 1])

                curve_points.append((x, y))

        for i in range(len(curve_points) - 1):
            x1, y1 = self.world_to_canvas(*curve_points[i])
            x2, y2 = self.world_to_canvas(*curve_points[i + 1])
            self.canvas.create_line(x1, y1, x2, y2, fill="black", tags="curve")

    def zoom(self, event):
        if event.delta > 0:
            self.zoom_level *= 1.1
        elif event.delta < 0:
            self.zoom_level /= 1.1
        self.draw_curve()
