# OBD project

## Pipeline of OBD-II

### universal approach:

 ```mermaid
sequenceDiagram
    participant User
    participant App
    participant CustomAdapter
    participant VehicleECU

    User->>+App: Select OBD-II Service
    App->>+CustomAdapter: Send Request
    CustomAdapter->>+VehicleECU: Establish Connection
    VehicleECU->>+CustomAdapter: Identify Protocol
    CustomAdapter->>+VehicleECU: Send Data Request
    VehicleECU-->>-CustomAdapter: Receive Data
    CustomAdapter->>CustomAdapter: Format Data
    CustomAdapter->>CustomAdapter: Error Handling
    CustomAdapter-->>-App: Transmit Data via Bluetooth/Wi-Fi
    App->>App: Interpret and Process Data
    App-->>-User: Display Data on UI
 ```

### VHAL approach
*(may be not fit for production)*

How to test in VHAL: [link](https://cs.android.com/android/platform/superproject/main/+/main:packages/services/Car/tools/emulator/README.md)

And use VHAL flavor code:

```kotlin
// Legacy code
val car = Car.createCar(this)
val carPropertyManager = car.getCarManager(Car.PROPERTY_SERVICE) as CarPropertyManager?

val property = carPropertyManager?.getProperty<Any>(VehiclePropertyIds.OBD2_LIVE_FRAME, 0);
// or ...
val property = carPropertyManager?.getProperty<Any>(VehiclePropertyIds.OBD2_FREEZE_FRAME, 0);

// Latest code
val car = Car.createCar(this)
val carDiagnosticManager = car.getCarManager(Car.PROPERTY_SERVICE) as CarDiagnosticManager?
val liveFrame = carDiagnosticManager.getLatestLiveFrame();
// or ...
val timestamps = carDiagnosticManager.getFreezeFrameTimestamps();
if (null != timestamps) {
  for (timestamp in timestamps) {
    val freezeFrame = carDiagnosticManager.getFreezeFrame(timestamp);
  }
}
```

## Components of OBD-II

The OBD-II system consists of a computer and sensors integrated throughout the
vehicle that monitor functions such as engine temperature, fuel system, vehicle
speed, and emissions control systems.

### ISO standard

<details>
  <summary>ISO 9141: Road vehicles – Diagnostic systems. International Organization for Standardization, 1989.</summary>
  Part 1: Requirements for interchange of digital information<br>
  Part 2: CARB requirements for interchange of digital information<br>
  Part 3: Verification of the communication between vehicle and OBD II scan tool<br>
</details>
<details>
  <summary>ISO 11898: Road vehicles – Controller area network (CAN). International Organization for Standardization, 2003.</summary>
  Part 1: Data link layer and physical signalling<br>
  Part 2: High-speed medium access unit<br>
  Part 3: Low-speed, fault-tolerant, medium-dependent interface<br>
  Part 4: Time-triggered communication<br>
</details>
<details>
  <summary>ISO 14230: Road vehicles – Diagnostic systems – Keyword Protocol 2000, International Organization for Standardization, 1999.</summary>
  Part 1: Physical layer<br>
  Part 2: Data link layer<br>
  Part 3: Application layer<br>
  Part 4: Requirements for emission-related systems<br>
</details>
<details>
  <summary>ISO 15031: Communication between vehicle and external equipment for emissions-related diagnostics, International Organization for Standardization, 2010.</summary>
  Part 1: General information and use case definition<br>
  Part 2: Guidance on terms, definitions, abbreviations and acronyms<br>
  Part 3: Diagnostic connector and related electrical circuits, specification and use<br>
  Part 4: External test equipment<br>
  Part 5: Emissions-related diagnostic services<br>
  Part 6: Diagnostic trouble code definitions<br>
  Part 7: Data link security<br>
</details>
<details>
  <summary>ISO 15765: Road vehicles – Diagnostics on Controller Area Networks (CAN). International Organization for Standardization, 2004.</summary>
  Part 1: General information<br>
  Part 2: Network layer services ISO 15765-2<br>
  Part 3: Implementation of unified diagnostic services (UDS on CAN)<br>
  Part 4: Requirements for emissions-related systems<br>
</details>

## Connector of OBD-II

source: https://en.wikipedia.org/wiki/On-board_diagnostics#OBD-II_diagnostic_connector

## Protocols of OBD-II

source: https://en.wikipedia.org/wiki/On-board_diagnostics#OBD-II_signal_protocols

### Service / Mode

| Service (hex) | Description                                       |
|:-------------:|---------------------------------------------------|
|     0x01      | Show current data                                 |
|     0x02      | Show freeze frame data                            |
|      ..       | ...                                               |
|     0x09      | Request vehicle information                       |

source: https://en.wikipedia.org/wiki/OBD-II_PIDs#Services_/_Modes

The feature and data calculation can be found from
https://en.wikipedia.org/wiki/OBD-II_PIDs#Standard_PIDs

#### Service 0x001

Here listed few sample PID and relative infos

| PIDs (hex) | Bytes returned | Description                                                                                                                                                            |
|------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0x00       | 4              | A bitmask for display supported PIDS under range [01 - 20]                                                                                                             |
| 0x01       | 4              | Monitor status since DTCs cleared, under [rule](https://en.wikipedia.org/wiki/OBD-II_PIDs#Service_01_PID_01_-_Monitor_status_since_DTCs_cleared)                       |
| 0x02       | 2              | DTC that caused freeze frame to be stored, under [rule](https://en.wikipedia.org/wiki/OBD-II_PIDs#Service_03_(no_PID_required)_-_Show_stored_Diagnostic_Trouble_Codes) |
| 0x03       | 2              | Status of fuel system, value $\frac{A}{2.55}$                                                                                                                          |     
| 0x04       | 1              | Calculated engine load, value $A - 40$                                                                                                                                 |    
| 0x05       | 1              | Engine coolant temperature, Value $\frac{A}{1.28} -100$                                                                                                                |  
| ..         | ..             | ...                                                                                                                                                                    | 

### CAN bus
Here's how data moves:

 - 1.The sensor collects data (e.g., temperature, speed).
 - 2.Node A, which might be responsible for engine-related functions, receives data from the sensor.
 - 3.Node A packages this data into a CAN message and sends it onto the CAN bus.
 - 4.The message travels along the CAN bus to Node B, which could be responsible for transmission-related functions.
 - 5.Node B receives the message, processes it if necessary, and then forwards it to the OBD port.
 - 6.The data is made available through the OBD port for diagnostic or monitoring purposes.


link to wiki page:
- [What CAN bus is and how data moves in it](https://en.wikipedia.org/wiki/CAN_bus)

# Android Automotive (Android Automotive OS / AAOS)

Variation of google's Android opertaing system, tailored for its use in vechile dashboards. Android Automive is opensource project and can be use freely, but GAS (Google Automotive Services) is 
behind paywall and it is payed per vechile. GAS includes google play services.

Realted link:
- [What is automotive](https://source.android.com/docs/automotive/start/what_automotive)
- [Jetpack compose android.car.app](https://developer.android.com/codelabs/car-app-library-fundamentals)

## OBD II on Android Automotive

Small test on android.car.obd2 package https://github.com/Sieluna/OBDViewer
...

## Usage

| year | Manufacturer                                |
|------|---------------------------------------------|
| 2018 | Polestar, Renault-Nissan-Mitsubishi Allince |
| 2019 | Genereal Motors                             |
| 2021 | Ford, Lucid Motors, Honda                   |
| 2022 | BMW                                         |
| 2023 | VolksWagen, Porche                          |
| ..   | ..                                          |

Related link to wikipedia:
https://en.wikipedia.org/wiki/Android_Automotive

## Relative Works

<details>
  <summary>
    <a href="https://github.com/Pbartek/pyobd-pi/blob/master/README.md">
    Raspberry Pi Displaying Car Diagnostics (OBD-II) Data
    </a>
  </summary>
  This work use <code>ELM327 Bluetooth Adapter</code> or <code>ELM327 USB Cable</code> to connect with
  <code>OBD Port</code>, use <code>pyOBD</code> ref: "https://github.com/peterh/pyobd" library to handle
  protocol.
</details>
<details>
  <summary>
    <a href="https://github.com/amund7/CANBUS-Analyzer">
    CANBUS-Analyzer For Tesla
  </summary>
  The work produced a visulization tool for anaylse the CAN Bus data from Tesla.
</details>
<details>
  <summary>
    <a href="https://github.com/HiMinds/himinds-iot-project-android-automotive-OBD-simulator">
    Mock few useful OBD data and pipe to android automotive VHAL
  </summary>
  The work wrapper OBD data into protobuf in the HAL. Could use when we mock vehicle OBD data
</details>
<details>
  <summary>
    <a href="https://github.com/limiter121/esp32-obd2-emulator">
    ESP32 OBD-II Emulator
    </a>
  </summary>
  This work use <code>ESP32 + CAN tranceiver IC </code> via <code>WiFi throught web UI or API</code>. Project has
  good <code>README</code> so chek it out.
</details>
<details>
  <summary>
    <a href="https://github.com/Ircama/ELM327-emulator">
    ELM327-OBD2 scanner emulator. Good example of an emulator that we need to emulate OBD2 data. 
    </a>
  </summary>
  This project emulates ELM327-OBD2 scanner. It's good starting point to figure out what kind of emulator we need in our project.
</details>
