## How to start:

### Set up hardware

To run this app, you need a Raspberry Pi, running an Android Automotive image, version 11 or newer. This app is developed with Raspberry Pi 4 (4 GB variant) and Audroid Automotive version 14, other Raspberry Pi's or Android versions are not tested.

Connect Raspberry Pi to HDMI screen (optional), and power Raspberry Pi from your PC's USB port (Raspberry Pi 4's power port is also debug port).

### Set up software

Open app project in Android Studio. Open Device Manager in Android Studio and ensure that your Raspberry Pi shows up.

Insert PC's IP address to app/automotive/src/main/res/values/strings.xml as `server_host`.

Click the green 'play'-button in Android Studio to install and run app.

### How to use this app

Connect Raspberry Pi to same network with the PC that runs the car emulator GUI. Wifi is known to work, ethernet has had issues.

Launch the car emulator GUI. If TCP connection to GUI is not available in 20 retries, Android app stops reconnecting. Connect devices to same network, check IP settings, and relaunch Android app.

When connected to server, this app shows some live data from vehicle via network connection. Network traffic is done in OBD-compliant data format.
Speed data is analysed, and app gives feedback to user about his/her driving style.