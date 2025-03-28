# NTRIP Client Operation Manual

## Introduction
The NTRIP (Networked Transport of RTCM via Internet Protocol) client is designed to receive RTCM correction data from an NTRIP caster, provide it to GNSS receivers for improved positioning accuracy, and send position information of the GNSS receiver to a MQTT broker. 

## Requirements
- A device running the NTRIP client firmware (e.g., ATOM Lite).
- Access to an NTRIP caster with valid credentials (host, port, username, password, and mountpoint).
- Access to an MQTT broker with valid credentials (host, port, username, password, and topic).
- A stable internet connection (Wi-Fi or other supported methods).
- A GNSS receiver capable of processing RTCM correction data.

## Setup Instructions

### 1. Configure Wi-Fi
1. When the NTRIP client boots up, it tries to connect to a previously saved WiFi network.
2. If no known network is found, it creates a WiFi Access Point (AP) with:
   - **SSID**: "NTRIPClient_Config"
   - **Password**: none
3. Connect your phone or computer to this network.
4. Open a browser and go to 192.168.4.1.
5. Select your WiFi network, enter the password, and save the settings.

When the NTRIP client is connected to a WiFi network:
1. the config menu can be invoked by pressing the button on the NTRIP client for more than 3 seconds. 
2. Connect your computer to the same network as the NTRIP client. 
3. Open a browser and go to the IP-address of the NTRIP client. 
4. Select a WiFi network, enter the password, and save the settings.

### 2. Configure NTRIP Caster Details
1. Provide the following details for the NTRIP caster:
   - **Host**: The hostname or IP address of the NTRIP Caster (e.g., `caster.example.com`)
   - **Port**: The port number for the NTRIP caster (e.g., `2101`)
   - **Mountpoint**: Name of the moint point where the RTCM data will be retrieved (e.g., `RTCM3`)
   - **Username and password**: Credentials for authenticating with the NTRIP Caster (if required)
2. Save the configuration.

### 3. Start the NTRIP Client
1. Initiate the connection to the NTRIP caster.
2. Verify that the device successfully connects and starts receiving RTCM data.

### 4. Connect to GNSS Receiver
1. Ensure the GNSS receiver is properly connected to the device (e.g., via UART or other supported interfaces).
2. Verify that the GNSS receiver is receiving RTCM correction data.

### 5. Configure MQTT Client
1. Access the device's configuration interface.
2. Provide the following MQTT broker details:
   - **Broker Address**: The hostname or IP address of the MQTT broker (e.g., `mqtt.example.com`).
   - **Port**: The port number for the MQTT broker (e.g., `1883`).
   - **Username and Password**: Credentials for authenticating with the MQTT broker (if required).
   - **Topic**: The MQTT topic where the device will publish data (e.g., `ntrip/status`).
3. Save the configuration.

### Purpose of MQTT Client
The MQTT client allows the NTRIP client to publish status updates and diagnostic information to an MQTT broker. This enables remote monitoring and integration with IoT platforms or dashboards.

### JSON Structure of Published Data
The NTRIP client publishes data in JSON format to the configured MQTT topic. Below is the structure of the JSON payload:

```json
{
  "timestamp": "2023-01-01T12:00:00Z", // ISO 8601 timestamp of the message
  "status": "connected",              // Current connection status (e.g., "connected", "disconnected")
  "ntrip": {
    "host": "caster.example.com",     // NTRIP caster hostname
    "port": 2101,                     // NTRIP caster port
    "mountpoint": "RTCM3"             // NTRIP mountpoint in use
  },
  "gnss": {
    "rtcm_received": true,            // Whether RTCM data is being received
    "last_rtcm_time": "2023-01-01T11:59:50Z" // Timestamp of the last RTCM message received
  }
}
```

### Example Use Case
- Use the MQTT data to monitor the connection status of the NTRIP client.
- Integrate the JSON payload into a dashboard to visualize real-time data.
- Trigger alerts if the client disconnects or stops receiving RTCM data.

## Troubleshooting
- **No Internet Connection**: Check Wi-Fi credentials and signal strength.
- **Failed to Connect to NTRIP Caster**: Verify the host, port, and credentials.
- **No RTCM Data on GNSS Receiver**: Check the connection between the device and the GNSS receiver.

## Additional Notes
- Ensure the device's firmware is up-to-date for optimal performance.
- Contact your NTRIP caster provider for assistance with credentials or mountpoint details.

