# Protocols

## Condiguration file

## MQTT message

The NTRIP Client send position information and metadata to the broker. This information is typcally shown below. A description of the name-value pairs is presented after this. : 

```json
{
   "num": 1357720,
   "daytime": "2025-03-28 10:27:06.200",
   "lat": 5212.688959,
   "lon": 559.0198035,
   "alt": 21.394,
   "fix_type": 5,
   "speed": 0,
   "dir": 334.2,
   "sats": 31,
   "hdop": 0.48
}
```
### Field specification

 - **num**: Number (Integer), Incrementing number from the start of the NTRIP Client
 - **daytime**: String, Actual date and time in ms of the psoition and vehicle.
 - **lat**: Number (Float), Latitude of the coordinate of the location in Degrees, decimal-degrees "DD.DDDDDDD"
 - **lon**: Number (Float), Longitude of the coordinate of the location in Degrees, decimal-degrees "DD.DDDDDDD"
 - **alt**: Number (Float), Altitude in meters ASL
 - **fix_type**: uint8_t, Fix type, See *NMEA 0183 - GGA Sentence Fix Types* below.
 - **speed**: Number (Float), Vehicle speed in m/s
 - **dir**: Number (Float), Direction of the vehicle reletive to true North, in deg. 
 - **sats**: Number (Integer), Number of satellites in fix
 - **hdop**: Number (Float), Number (Integer)
 

#### Date-time format
Date time format is following the **ISO 8601** standard for databases, log files, and time-sensitive applications:
```
YYYY-MM-DD HH:mm:ss.sss
```
The format **YYYY-MM-DD HH:mm:ss.sss** represents a **timestamp** with the following components:  

- **YYYY** → 4-digit year (e.g., 2025)  
- **MM** → 2-digit month (01-12)  
- **DD** → 2-digit day of the month (01-31)  
- **HH** → 2-digit hour in 24-hour format (00-23)  
- **mm** → 2-digit minutes (00-59)  
- **ss** → 2-digit seconds (00-59)  
- **sss** → 3-digit milliseconds (000-999)  

#### Latitude longitude format

Latitude and longitude is written in **degrees decimal degrees (DDD)** who are geographic coordinates used to specify locations on Earth. They are expressed as:  

### **Format:**  
```
Latitude:   ±DD.DDDDDD
Longitude: ±DDD.DDDDDD
```

- **Latitude (Lat)**: Ranges from **-90.000000** to **+90.000000**  
  - **Positive (+)** → North of the Equator  
  - **Negative (-)** → South of the Equator  
- **Longitude (Lon)**: Ranges from **-180.000000** to **+180.000000**  
  - **Positive (+)** → East of the Prime Meridian  
  - **Negative (-)** → West of the Prime Meridian  

### **Example:**  
- **Latitude:** 37.774929 → **37.774929° N**  
- **Longitude:** -122.419416 → **122.419416° W**  

#### NMEA 0183 - GGA Sentence Fix Types
The GGA sentence contains a field indicating the GPS fix type:

| **Fix Type Code** | **Description** |
|------------------|---------------|
| **0** | No fix (invalid) |
| **1** | GPS fix (standard) |
| **2** | DGPS (Differential GPS) fix |
| **3** | PPS (Precise Positioning Service) fix |
| **4** | RTK (Real-Time Kinematic) fixed integer |
| **5** | RTK float |
| **6** | Estimated (dead reckoning) fix |
| **7** | Manual input mode |
| **8** | Simulation mode |

## Time message to telemetry unit
The message format is as follows:

 - **Start-byte**: Control-A (0x01)
 - **Timestamp**: (string) formatted as Date-time format (as decribed above)
 - **CRC-16**: (2 bytes, big-endian) over Timestamp (Start-byte excluded)
 - **Stop-byte**: Control-X (0x18)

```
   1                n               2      1    <- bytes in payload
[0x01][YYYY-MM-DD HH:MM:SS.SSSS][CRC-16][0x18]
```

For a **24-byte string**, **CRC-16** is often the better choice than **CRC-32**, unless you require extra-strong error detection.  

**Comparison of CRC-16 vs. CRC-32 for a 15-byte string**  

| Feature      | **CRC-16** | **CRC-32** |
|-------------|-----------|-----------|
| **Checksum Size** | 16 bits (2 bytes) | 32 bits (4 bytes) |
| **Error Detection** | Good for small data, detects single-bit and burst errors | Stronger, but overkill for small data |
| **Collision Probability** | Lower risk for short data | Even lower, but unnecessary for 15 bytes |
| **Processing Speed** | Faster (less computational overhead) | Slightly slower due to larger checksum |

**Why CRC-16 is Better in this application?**:
1. **Sufficient Error Detection**:  
   - For small payloads like 24 bytes, **CRC-16 is strong enough** to catch most transmission errors.  
   - CRC-32 provides better protection but is **not needed** unless extremely high reliability is required.  
2. **Lower Overhead**:  
   - CRC-16 adds only **2 bytes**, while CRC-32 adds **4 bytes** to the message.  
   - This makes CRC-16 more efficient in bandwidth-limited or low-memory environments.  
3. **Faster Computation**:  
   - CRC-16 is **lighter** in processing, making it ideal for embedded systems or low-power devices.  

**Conclusion**: For a **24-byte string**, **CRC-16 is the better choice** in most cases due to its efficiency and sufficient error detection.
