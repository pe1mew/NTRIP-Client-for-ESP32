# Protocols

## Condiguration file

## MQTT message

The NTRIP Client send position information and metadata to the broker. This information is typcally shown below: 

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
