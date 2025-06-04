import logging

from elm_emulator import Elm, CarEmulatorGUI

logging.basicConfig(level=logging.DEBUG)

with Elm(net_port=3000) as emulator:
    # Create and start the GUI
    gui = CarEmulatorGUI(emulator)
    gui.start()
