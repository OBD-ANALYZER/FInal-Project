import time
import numpy as np

class Car:
    RPM = (800, 6000)
    SPEED = (0, 250)  # Max speed in km/h
    BRAKE_FORCE = 5000  # Maximum braking force in N
    DRAG_COEFFICIENT = 0.32
    FRONTAL_AREA = 2.2  # in square meters
    AIR_DENSITY = 1.225  # kg/m^3
    CAR_MASS = 1500  # kg
    GEAR_RATIOS = [3.5, 2.8, 2.0, 1.5, 1.0, 0.8]  # 6 gears
    FINAL_DRIVE_RATIO = 3.42
    WHEEL_RADIUS = 0.3  # in meters
    FUEL_TANK_CAPACITY = 50  # Fuel tank capacity in liters
    BASE_FUEL_CONSUMPTION_RATE = 0.1
    
    # Constants for gear position stability
    SPEED_THRESHOLD = 2.0  # km/h - Speed threshold for gear changes
    BRAKE_THRESHOLD = 0.5  # Brake threshold for gear position changes
    THROTTLE_THRESHOLD = 0.05  # Lower threshold for better stability
    POSITION_HOLD_TIME = 1.0  # Increased hold time for more stability
    GEAR_HOLD_TIME = 0.5  # seconds - Minimum time between gear changes
    
    # Additional stability parameters
    POSITION_STABILITY_COUNT = 5  # Number of consistent readings needed for position change
    SPEED_STABILITY_THRESHOLD = 0.5  # km/h - Speed must be stable within this range
    MIN_DRIVE_SPEED = 1.0  # Minimum speed to maintain Drive
    
    # Gear shift thresholds
    UPSHIFT_RPM = 3000  # RPM threshold for upshift
    DOWNSHIFT_RPM = 1500  # RPM threshold for downshift
    
    # Speed thresholds for each gear (in km/h)
    GEAR_SPEED_THRESHOLDS = [0, 20, 40, 60, 80, 100]

    def __init__(self):
        # Initialize basic vehicle state
        self.speed = 0.0
        self.rpm = self.RPM[0]  # Start at idle RPM
        self.gear = 1
        self.gear_position = "P"  # P, R, N, D
        self.throttle_position = 0.0
        self.brake_position = 0.0
        self.engine_temp = 40.0  # Start at 40°C
        self.fuel_level = self.FUEL_TANK_CAPACITY  # Start with full tank
        
        # Initialize stability tracking variables
        self._last_gear_change = 0
        self._last_position_change = 0
        self._gear_change_pending = False
        self._position_change_pending = False
        self._last_gear = 1
        self._last_position = "P"
        self._last_speed = 0
        self._stable_speed_timer = 0
        
        # Initialize database for data storage
        self.database = {
            "rpm": self.rpm,
            "speed": self.speed,
            "gear": self.gear,
            "gear_position": self.gear_position,
            "throttle_position": self.throttle_position,
            "brake_position": self.brake_position,
            "engine_temp": self.engine_temp,
            "fuel_level": self.fuel_level
        }

    def get_real_time_data(self):
        """Get current vehicle data."""
        return {
            "rpm": self.rpm,
            "speed": self.speed,
            "gear": self.gear,
            "gear_position": self.gear_position,
            "throttle_position": self.throttle_position,
            "brake_position": self.brake_position,
            "engine_temp": self.engine_temp,
            "fuel_level": self.get_fuel_level_percentage(),
            "fuel_consumption_rate": self.get_fuel_consumption_rate()
        }

    def update(self, throttle_position, brake_position):
        """Update vehicle state based on inputs."""
        self.throttle_position = max(0, min(throttle_position, 1))
        self.brake_position = max(0, min(brake_position, 1))
        
        self.update_speed()
        self.update_rpm()
        self.update_engine_temp()
        self.update_fuel_consumption()
        
        # Update database
        self.database.update(self.get_real_time_data())

    def update_fuel_consumption(self):
        """Update fuel consumption based on throttle and speed."""
        # Base consumption rate increases with throttle position
        consumption_rate = self.BASE_FUEL_CONSUMPTION_RATE
        
        # Add throttle influence (exponential relationship)
        consumption_rate += (self.throttle_position ** 2) * 0.3
        
        # Add speed influence (quadratic relationship)
        speed_factor = (self.speed / 100) ** 2
        consumption_rate += speed_factor * 0.2
        
        # Add gear efficiency factor (higher gears are more efficient)
        if self.gear_position == "D":
            gear_efficiency = 1.0 - ((self.gear - 1) * 0.1)  # Each higher gear is 10% more efficient
            consumption_rate *= max(0.5, gear_efficiency)  # Cap efficiency improvement at 50%
        
        # Reduce fuel level based on consumption (scaled by time step)
        time_step = 0.01  # 10ms update interval
        fuel_used = consumption_rate * time_step
        self.fuel_level = max(0, self.fuel_level - fuel_used)
        
        # Update database with new fuel level
        self.database["fuel_level"] = self.get_fuel_level_percentage()

    def get_fuel_consumption_rate(self):
        """Calculate current fuel consumption rate in L/100km."""
        if self.speed < 1:  # Avoid division by zero
            return self.BASE_FUEL_CONSUMPTION_RATE * (1 + self.throttle_position)
        
        # Base rate that increases with throttle (exponential)
        base_rate = 5 + (self.throttle_position ** 2) * 15
        
        # Add speed influence (U-shaped curve)
        optimal_speed = 80  # km/h
        speed_factor = abs(self.speed - optimal_speed) / 40  # Deviation from optimal speed
        consumption_rate = base_rate + (speed_factor ** 2) * 3
        
        # Apply gear efficiency
        if self.gear_position == "D":
            gear_efficiency = 1.0 - ((self.gear - 1) * 0.1)
            consumption_rate *= max(0.5, gear_efficiency)
        
        return consumption_rate

    def refuel(self):
        """Refill the fuel tank to capacity."""
        self.fuel_level = self.FUEL_TANK_CAPACITY
        self.database["fuel_level"] = self.fuel_level

    def get_fuel_level_percentage(self):
        """Return the fuel level as a percentage of the tank capacity."""
        return (self.fuel_level / self.FUEL_TANK_CAPACITY) * 100

    def set_fuel_level(self, level):
        """Set the fuel level to a specific value (in liters)."""
        self.fuel_level = max(0, min(level, self.FUEL_TANK_CAPACITY))
        self.database["fuel_level"] = self.fuel_level
        self.update_gear_position()  # Update gear position based on speed and throttle

    def update_engine_temp(self):
        """Simulate engine temperature changes based on throttle and brake inputs."""
        # Increase temperature with throttle input
        if self.throttle_position > 0:
            self.engine_temp += self.throttle_position * 0.1

        # Decrease temperature with brake input
        if self.brake_position > 0:
            self.engine_temp -= self.brake_position * 0.05

        # Clamp temperature to realistic bounds (e.g., 40°C to 120°C)
        self.engine_temp = max(40, min(self.engine_temp, 120))

    def get_engine_temp(self):
        return self.engine_temp

    def update_rpm(self):
        """Update RPM based on throttle, speed, and gear."""
        # Base idle RPM
        idle_rpm = self.RPM[0]
        
        if self.speed == 0:
            # At standstill, RPM depends on throttle
            target_rpm = idle_rpm + (self.throttle_position * 1000)
        else:
            # Calculate wheel RPM from speed
            wheel_rpm = (self.speed * 1000) / (60 * 2 * np.pi * self.WHEEL_RADIUS * 3.6)
            
            # Calculate engine RPM through drivetrain
            target_rpm = wheel_rpm * self.GEAR_RATIOS[self.gear - 1] * self.FINAL_DRIVE_RATIO
            
            # Add throttle influence
            target_rpm += self.throttle_position * 1000
            
        # Apply brake influence
        if self.brake_position > 0:
            target_rpm = max(target_rpm * (1 - self.brake_position), idle_rpm)
            
        # Smoothly interpolate to target RPM
        rpm_diff = target_rpm - self.rpm
        self.rpm += rpm_diff * 0.1  # Smooth factor
        
        # Clamp RPM to valid range
        self.rpm = min(max(self.rpm, self.RPM[0]), self.RPM[1])

    def update_speed(self):
        # Calculate the engine power based on RPM and throttle position
        max_power = 200  # Max power in kW
        power = max_power * (self.rpm / self.RPM[1]) * self.throttle_position  # Power in kW

        # Convert power to force (assuming 100% efficiency for simplicity)
        force = (power * 1000) / (self.speed / 3.6 + 0.1)  # Force in N

        # Calculate drag force
        drag_force = 0.5 * self.DRAG_COEFFICIENT * self.FRONTAL_AREA * \
            self.AIR_DENSITY * (self.speed / 3.6) ** 2

        # Calculate braking force
        brake_force = self.brake_position * self.BRAKE_FORCE

        # Calculate acceleration
        net_force = force - drag_force - brake_force
        acceleration = net_force / self.CAR_MASS

        # Update speed
        self.speed += acceleration * 3.6  # Convert m/s^2 to km/h
        self.speed = min(max(self.speed, self.SPEED[0]), self.SPEED[1])

        # Update gear based on speed
        self.update_gear()

    def update_gear(self):
        """Update gear based on speed, RPM and gear position with improved stability."""
        current_time = time.time()
        
        # Don't change gears if not in Drive
        if self.gear_position != "D":
            self.gear = 1
            return
        
        # Don't change gears if speed is 0
        if self.speed < 1:
            self.gear = 1
            return
            
        # Don't change gears too frequently
        if current_time - self._last_gear_change < self.GEAR_HOLD_TIME:
            return
            
        # Calculate target gear based on speed
        target_gear = 1
        for i, threshold in enumerate(self.GEAR_SPEED_THRESHOLDS[1:], 1):
            if self.speed >= threshold:
                target_gear = i + 1
                
        # Consider RPM for gear changes
        if self.rpm > self.UPSHIFT_RPM and self.gear < 6:
            target_gear = min(self.gear + 1, 6)
        elif self.rpm < self.DOWNSHIFT_RPM and self.gear > 1:
            target_gear = max(self.gear - 1, 1)
            
        # Apply gear change with hysteresis
        if target_gear != self.gear:
            if not self._gear_change_pending:
                self._gear_change_pending = True
                self._last_gear = self.gear
            elif current_time - self._last_gear_change > self.GEAR_HOLD_TIME * 2:
                self.gear = target_gear
                self._gear_change_pending = False
                self._last_gear_change = current_time
                self.rpm = max(self.rpm * 0.8, self.RPM[0])  # Smooth RPM transition
        else:
            self._gear_change_pending = False

    def update_gear_position(self):
        """Update gear position with improved stability and realism."""
        current_time = time.time()
        
        # Don't change position too frequently
        if current_time - self._last_position_change < self.POSITION_HOLD_TIME:
            return
            
        # Get current state
        current_position = self.gear_position
        
        # Update position based on speed and inputs
        if self.speed < self.SPEED_THRESHOLD:
            # Vehicle is nearly stopped
            if self.brake_position > self.BRAKE_THRESHOLD:
                # Brake is pressed significantly
                if current_position == "D":
                    self.gear_position = "P"
                    self._last_position_change = current_time
            elif self.throttle_position > self.THROTTLE_THRESHOLD:
                # Gas is pressed
                if current_position == "P":
                    self.gear_position = "D"
                    self._last_position_change = current_time
        else:
            # Vehicle is moving
            if current_position == "P":
                self.gear_position = "D"
                self._last_position_change = current_time
        
        # Update gear when position changes
        if self.gear_position != current_position:
            if self.gear_position == "P":
                self.gear = 1
            elif self.gear_position == "D":
                # Calculate appropriate starting gear based on speed
                for i, threshold in enumerate(self.GEAR_SPEED_THRESHOLDS[1:], 1):
                    if self.speed >= threshold:
                        self.gear = i + 1
                        break
                else:
                    self.gear = 1
            
            # Update database
            self.database.update({
                "gear": self.gear,
                "gear_position": self.gear_position
            })

    def get_gear_position(self):
        """Return current gear position."""
        return self.gear_position

    def get_gear(self):
        """Return current gear number."""
        return self.gear 