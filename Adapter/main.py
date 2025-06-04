import logging
import threading
import time
from elm_emulator import Elm, RealTimeHandler, DataUpdateMode, VehicleData, CarEmulatorGUI
from elm_emulator.firebase_handler import FirebaseHandler
import socket

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Constants for network stability
UPDATE_INTERVAL = 0.1  # 100ms update interval
MAX_RETRIES = 3
RETRY_DELAY = 1.0  # seconds

def update_vehicle_data(handler: RealTimeHandler, car, firebase_handler: FirebaseHandler):
    """Update vehicle data periodically with improved stability"""
    retry_count = 0
    last_successful_update = time.time()
    
    while True:
        try:
            # Get current data from car
            data = car.get_real_time_data()
            
            # Ensure minimum time between updates
            current_time = time.time()
            time_since_last = current_time - last_successful_update
            if time_since_last < UPDATE_INTERVAL:
                time.sleep(UPDATE_INTERVAL - time_since_last)
            
            # Update the handler with real car data
            new_data = {
                "rpm": data["rpm"],
                "speed": data["speed"],
                "engine_temp": data["engine_temp"],
                "throttle_percentage": data["throttle_position"],
                "fuel_level": data["fuel_level"],
                "fuel_consumption_rate": data["fuel_consumption_rate"],
                "gear": data["gear"],
                "gear_position": data["gear_position"]
            }
            
            # Update the handler with new data
            handler.update_data(new_data)
            
            # Save to Firebase
            if firebase_handler:
                firebase_handler.save_data(new_data)
            
            # Reset retry count on successful update
            retry_count = 0
            last_successful_update = time.time()
            
        except Exception as e:
            logging.error(f"Error updating vehicle data: {e}")
            retry_count += 1
            
            if retry_count >= MAX_RETRIES:
                logging.error("Maximum retry attempts reached. Resetting connection...")
                retry_count = 0
                time.sleep(RETRY_DELAY * 2)  # Longer delay before reset
            else:
                time.sleep(RETRY_DELAY)

def main():
    # Configure socket server with improved stability
    socket_config = {
        'net_port': 3000,
        'tcp_nodelay': True,  # Disable Nagle's algorithm for faster transmission
        'keepalive': True,    # Enable TCP keepalive
        'timeout': 5.0        # Socket timeout in seconds
    }
    
    try:
        # Initialize Firebase handler
        firebase_handler = FirebaseHandler()
        
        with Elm(net_port=socket_config['net_port']) as emulator:
            # Create and configure socket
            emulator.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            emulator.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            emulator.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            emulator.socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            emulator.socket.settimeout(socket_config['timeout'])
            
            # Bind and listen
            emulator.socket.bind(('', socket_config['net_port']))
            emulator.socket.listen(1)
            logging.info(f"Socket server started on port {socket_config['net_port']}")
            
            # Set data update mode to manual for controlled updates
            emulator.real_time_handler.set_mode(DataUpdateMode.MANUAL)
            
            # Create and start the GUI
            gui = CarEmulatorGUI(emulator)
            gui_thread = threading.Thread(target=gui.start)
            gui_thread.daemon = True
            gui_thread.start()
            
            # Start data update thread using the car instance from GUI
            update_thread = threading.Thread(
                target=update_vehicle_data,
                args=(emulator.real_time_handler, gui.car, firebase_handler)
            )
            update_thread.daemon = True
            update_thread.start()
            
            try:
                emulator.run()
            except KeyboardInterrupt:
                logging.info("Shutting down emulator...")
            except Exception as e:
                logging.error(f"Emulator error: {e}")
            finally:
                emulator.stop()
                if emulator.socket:
                    emulator.socket.close()
                logging.info("Emulator stopped")
    
    except Exception as e:
        logging.error(f"Failed to initialize emulator: {e}")
        return

if __name__ == "__main__":
    main() 