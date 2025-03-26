# Configuration file config.json

The `config.json` file is used to store the settings required for the Ntrip client to function properly. It contains three main sections: `wifi`, `ntrip`, and `mqtt`. Below is a detailed explanation of each section and its name-value pairs:

## Configuration Details

### WiFi Configuration (`wifi`)
- **ssid**: The SSID (name) of the WiFi network to connect to.
- **password**: The password for the WiFi network.

### NTRIP Configuration (`ntrip`)
- **host**: The hostname or IP address of the NTRIP caster.
- **port**: The port number of the NTRIP caster (default is usually 2101).
- **mntpnt**: The mountpoint to connect to on the NTRIP caster.
- **user**: The username for NTRIP authentication.
- **passwd**: The password for NTRIP authentication.

### MQTT Configuration (`mqtt`)
- **broker**: The *IP address* of the MQTT broker. *The hostname of the broker cannot be used here!*
- **port**: The port number of the MQTT broker (default is usually 1883).
- **topic**: The MQTT topic to publish or subscribe to.
- **user**: The username for MQTT authentication.
- **passwd**: The password for MQTT authentication.

## Instructions

### Step 1: Prepare the Configuration File
1. Copy the directory `data_template` and its contents to a new directory named `data`.
2. Open `data/config.json` and update the values for `wifi`, `ntrip`, and `mqtt` with the required credentials and settings for your environment.

### Step 2: Remove Unnecessary Files
- Delete the file `data/readme.md` to ensure it is not uploaded to the SPIFFS filesystem.

### Step 3: Upload the Configuration to the ESP32
1. Open a terminal in PlatformIO.
2. Run the following command to compile and upload the SPIFFS filesystem to your ESP32:
   ```bash
   pio run --target uploadfs
   ```
3. Once the upload is complete, the ESP32 will use the settings from `config.json` to connect to the WiFi network, NTRIP caster, and MQTT broker.

### Notes
- Ensure that the ESP32 has sufficient permissions and connectivity to access the specified WiFi network, NTRIP caster, and MQTT broker.
- Double-check the JSON syntax in `config.json` to avoid errors during runtime.

This configuration file allows the Ntrip client to operate seamlessly by providing all necessary credentials and connection details in one place.
