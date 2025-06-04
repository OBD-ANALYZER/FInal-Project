# OBD Project Documentation

This documentation provides a comprehensive guide on how to set up, configure, and use the OBD project, which consists of an Android Automotive app and an OBD adapter emulator.

## Table of Contents
1. [Project Overview](#project-overview)
2. [System Requirements](#system-requirements)
3. [Setting Up the Development Environment](#setting-up-the-development-environment)
4. [Adapter Setup](#adapter-setup)
5. [App Configuration](#app-configuration)
6. [Cloud Deployment](#cloud-deployment)
7. [Troubleshooting](#troubleshooting)

## Project Overview

The project consists of two main components:
- **Android Automotive App**: A native Android Automotive OS application for displaying OBD data
- **OBD Adapter**: A Python-based OBD emulator that can run locally or on a cloud server

## System Requirements

### For Adapter Development
- Python 3.7 or higher
- Required Python packages (listed in adapter/requirements.txt)
- Network connectivity for cloud deployment

### For App Development
- Android Studio Arctic Fox or newer
- JDK 11 or higher
- Android Automotive OS SDK
- Google Play Services (for production builds)

## Setting Up the Development Environment

1. Clone the repository:
```bash
git clone [repository-url]
cd OBD
```

2. Set up the adapter environment:
```bash
cd adapter
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

3. Set up the Android app:
- Open the `app` folder in Android Studio
- Sync Gradle files
- Install required SDK components when prompted

## Adapter Setup

### Local Development
1. Navigate to the adapter directory
2. Run the adapter:
```bash
python main.py
```

The adapter will start on port 3000 by default.

### Configuration Options
The adapter can be configured through the following methods:

1. **Environment Variables**:
   - `OBD_PORT`: Port number (default: 3000)
   - `OBD_HOST`: Host address (default: '')
   - `UPDATE_INTERVAL`: Data update interval in seconds (default: 0.1)

2. **Direct Code Configuration**:
   Edit `adapter/main.py` to modify the socket configuration:
```python
socket_config = {
    'net_port': 3000,  # Change port number here
    'tcp_nodelay': True,
    'keepalive': True,
    'timeout': 5.0
}
```

## App Configuration

### Changing Server IP and Port

1. Open the app's network configuration file
2. Modify the connection settings:
```kotlin
// Update SERVER_IP and SERVER_PORT in the network configuration
const val SERVER_IP = "your_server_ip"
const val SERVER_PORT = 3000
```

### Building the App
1. Open the project in Android Studio
2. Select your build variant (debug/release)
3. Click "Build > Make Project"
4. Deploy to your Android Automotive device or emulator

## Cloud Deployment

### Deploying the Adapter to Cloud Server

1. **Prepare the Server**:
   ```bash
   # Install required packages
   sudo apt-get update
   sudo apt-get install python3 python3-pip
   
   # Clone the repository
   git clone [repository-url]
   cd OBD/adapter
   
   # Install dependencies
   pip3 install -r requirements.txt
   ```

2. **Configure Firewall**:
   ```bash
   # Allow traffic on adapter port
   sudo ufw allow 3000
   ```

3. **Run as a Service**:
   Create a systemd service file `/etc/systemd/system/obd-adapter.service`:
   ```ini
   [Unit]
   Description=OBD Adapter Service
   After=network.target
   
   [Service]
   User=ubuntu
   WorkingDirectory=/path/to/adapter
   ExecStart=/usr/bin/python3 main.py
   Restart=always
   
   [Install]
   WantedBy=multi-user.target
   ```

4. **Start the Service**:
   ```bash
   sudo systemctl enable obd-adapter
   sudo systemctl start obd-adapter
   ```

### Connecting the App to Cloud Server

1. Update the app's network configuration with your cloud server's IP address
2. Ensure your cloud server's security group/firewall allows incoming connections on port 3000
3. Test the connection using the app's built-in connection test feature

## Troubleshooting

### Common Issues

1. **Connection Timeout**
   - Check if the server IP and port are correct
   - Verify firewall settings
   - Ensure the adapter service is running

2. **Data Not Updating**
   - Check the UPDATE_INTERVAL setting
   - Verify network stability
   - Check server logs for errors

3. **App Crashes**
   - Check Android Studio's logcat for error messages
   - Verify app permissions
   - Ensure all required dependencies are installed

### Logging

- Adapter logs are available in the console and can be redirected to a file
- App logs can be viewed through Android Studio's logcat
- For cloud deployment, check system logs:
  ```bash
  sudo journalctl -u obd-adapter.service
  ```

## Additional Resources

- [Project README](README/README.md)
- [Android Automotive Documentation](https://source.android.com/docs/automotive/start/what_automotive)
- [OBD-II PIDs Reference](https://en.wikipedia.org/wiki/OBD-II_PIDs) 
