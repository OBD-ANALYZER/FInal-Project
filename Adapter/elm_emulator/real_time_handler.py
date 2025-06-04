import logging
from enum import Enum
import threading
from typing import Dict, Optional
from dataclasses import dataclass
import time

class PID:
    """OBD-II Parameter IDs"""
    ENGINE_RPM = "0C"
    VEHICLE_SPEED = "0D"
    ENGINE_TEMP = "05"
    THROTTLE_POS = "11"
    FUEL_LEVEL = "2F"
    FUEL_RATE = "5E"

@dataclass
class VehicleData:
    """Real-time vehicle data structure"""
    rpm: float = 0.0
    speed: float = 0.0
    engine_temp: float = 70.0
    throttle_position: float = 0.0
    fuel_level: float = 100.0
    fuel_consumption_rate: float = 0.0
    gear: int = 1
    gear_position: str = "N"
    brake_position: float = 0.0

class DataUpdateMode(Enum):
    REAL = "real"      # Real sensor data
    MANUAL = "manual"  # Manual input
    OFF = "off"       # No updates

class RealTimeHandler:
    def __init__(self):
        self._data = VehicleData()
        self._lock = threading.Lock()
        self._update_thread: Optional[threading.Thread] = None
        self._running = False
        self._mode = DataUpdateMode.MANUAL
        self._update_interval = 0.1  # 100ms update interval

    def start(self):
        """Start the real-time data handler"""
        if not self._running:
            self._running = True
            self._update_thread = threading.Thread(target=self._update_loop)
            self._update_thread.daemon = True
            self._update_thread.start()

    def stop(self):
        """Stop the real-time data handler"""
        self._running = False
        if self._update_thread:
            self._update_thread.join()
            self._update_thread = None

    def set_mode(self, mode: DataUpdateMode):
        """Set the data update mode"""
        self._mode = mode
        logging.info(f"Data update mode set to: {mode.value}")

    def update_data(self, data: Dict[str, float]):
        """Update vehicle data with new values"""
        with self._lock:
            try:
                if "rpm" in data:
                    self._data.rpm = float(data["rpm"])
                if "speed" in data:
                    self._data.speed = float(data["speed"])
                if "engine_temp" in data:
                    self._data.engine_temp = float(data["engine_temp"])
                if "throttle_position" in data:
                    self._data.throttle_position = float(data["throttle_position"])
                if "fuel_level" in data:
                    self._data.fuel_level = float(data["fuel_level"])
                if "fuel_consumption_rate" in data:
                    self._data.fuel_consumption_rate = float(data["fuel_consumption_rate"])
                if "gear" in data:
                    self._data.gear = int(data["gear"])
                if "gear_position" in data:
                    self._data.gear_position = str(data["gear_position"])
                if "brake_position" in data:
                    self._data.brake_position = float(data["brake_position"])
            except Exception as e:
                logging.error(f"Error updating vehicle data: {str(e)}")

    def get_pid_value(self, pid: str) -> Optional[float]:
        """Get the current value for a specific PID"""
        with self._lock:
            pid = pid.split(" ")[1]  # Extract PID from "01 XX" format
            
            if pid == PID.ENGINE_RPM:
                return self._data.rpm
            elif pid == PID.VEHICLE_SPEED:
                return self._data.speed
            elif pid == PID.ENGINE_TEMP:
                return self._data.engine_temp
            elif pid == PID.THROTTLE_POS:
                return self._data.throttle_position
            elif pid == PID.FUEL_LEVEL:
                return self._data.fuel_level
            elif pid == PID.FUEL_RATE:
                return self._data.fuel_consumption_rate
            return None

    def _update_loop(self):
        """Main update loop for real-time data"""
        while self._running:
            if self._mode == DataUpdateMode.REAL:
                # TODO: Implement real sensor data reading
                pass
            elif self._mode == DataUpdateMode.MANUAL:
                # Manual mode - data is updated via update_data()
                pass
            time.sleep(self._update_interval) 