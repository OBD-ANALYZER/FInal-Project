import json
import logging
import os
import time

from tkinter import *
from tkinter import ttk

from elm_emulator.animation_curve_editor import AnimationCurveEditor
from elm_emulator.car_emulator import Car


class CarEmulatorGUI:
    def __init__(self, emulator):
        self.emulator = emulator
        self.car = emulator.car  # Use the Car instance from the emulator
        self.lock = False
        self.presets = [x for x in os.listdir(os.curdir) if x.endswith("json")]

    def start(self):
        root = Tk()
        root.title("Engine Control")
        mainframe = ttk.Frame(root, padding="3 3 3 3")
        mainframe.grid(column=0, row=0, sticky=(N, S, E, W))
        mainframe.config(relief="sunken", borderwidth=2)

        # Configure window layout
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        mainframe.columnconfigure([0, 1, 2, 3], weight=1)
        mainframe.rowconfigure([0, 1, 2, 3, 4, 5, 6, 7, 8], weight=1)

        # Labels
        engine_temp_label = ttk.Label(mainframe, text="Engine Temp: 40°C")
        engine_temp_label.grid(column=0, columnspan=4,
                               row=1, padx=5, pady=5, sticky="w")

        rpm_label = ttk.Label(mainframe, text="Current RPM: 0")
        rpm_label.grid(column=0, columnspan=4, row=2,
                       padx=5, pady=5, sticky="w")

        speed_label = ttk.Label(mainframe, text="Current Speed: 0.0 km/h")
        speed_label.grid(column=0, columnspan=4, row=3,
                         padx=5, pady=5, sticky="w")
        
        # Adding labels for gear and gear position*******
        gear_label = ttk.Label(mainframe, text="Gear: 1")
        gear_label.grid(column=0, columnspan=4, row=4,
                        padx=5, pady=5, sticky="w")

        gear_position_label = ttk.Label(mainframe, text="Gear Position: N")
        gear_position_label.grid(column=0, columnspan=4, row=5,
                                 padx=5, pady=5, sticky="w")

        # Throttle & Brake Sliders
        def update_sliders(_):
            if not self.lock:
                throttle_value = throttle_slider.get()
                brake_value = brake_slider.get()
                self.car.update(throttle_value / 100, brake_value / 100)
                throttle_label.config(text=f"Throttle\n{throttle_value:.0f} %")
                brake_label.config(text=f"Brake\n{brake_value:.0f} %")
                anim_scale_value = anim_speed_slider.get()
                anim_speed_label.config(
                    text=f"Animation Speed Multiplier: {anim_scale_value:.1f}x")

                # Update gear and gear position labels''***************
                gear_label.config(text=f"Gear: {self.car.gear}")
                gear_position_label.config(text=f"Gear Position: {self.car.gear_position}")

        throttle_slider = ttk.Scale(
            mainframe, from_=100, to=0, orient="vertical", command=update_sliders)
        throttle_slider.grid(column=1, row=0, padx=5, pady=5, sticky=(N, S))
        throttle_label = ttk.Label(mainframe, text="Throttle\n0 %")
        throttle_label.grid(column=0, row=0, padx=5, pady=5, sticky="e")

        brake_slider = ttk.Scale(
            mainframe, from_=100, to=0, orient="vertical", command=update_sliders)
        brake_slider.grid(column=2, row=0, padx=5, pady=5, sticky=(N, S))
        brake_label = ttk.Label(mainframe, text="Brake\n0 %")
        brake_label.grid(column=3, row=0, padx=5, pady=5, sticky="w")



        #*********************************'

        def update_values():
            """
            Update the car's state (RPM, speed, engine temperature, gear, and gear position).
            """
            if not self.lock:
                self.car.update(self.car.throttle_position, self.car.brake_position)
                rpm_label.config(text=f"Current RPM: {self.car.rpm:.0f}")
                speed_label.config(text=f"Current Speed: {self.car.speed:.1f} km/h")
                temp_label.config(text=f"Engine Temperature: {self.car.engine_temp:.1f} °C")
                gear_label.config(text=f"Gear: {self.car.gear}")  # Update gear label
                gear_position_label.config(text=f"Gear Position: {self.car.gear_position}")  # Update gear position label

            root.after(10, update_values)