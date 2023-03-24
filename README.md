# Xsens_DOT_PC_Reader
use bleak to scan, connect, configure, identify(LED Blink), change device tag, read data, save to csv for DOT

Since this code is using bleak, it is possible to use cross-platforms like Windows/ubuntu, etc. But the author only tested on Windows 11.

##Windows 11 Setup
Before using this code, firstly you need to go to your Bluetooth & devices in Windows Settings, and "Add device", you only need to pair once of the sensors, next time you don't need to do this again.
![Alt text](add_bluetooth_device.jpg)

##Library Dependency
You will need to use pip install the relative libraries in this code.

currently 3 payloads type are created:
orientationEuler: timestamp + roll/pitch/yaw
orientationQuaternion: timestamp + quaternion(q0, q1, q2, q3)
customMode1: timestamp + euler angle(roll, pitch, yaw) + free acceleration(x,y,z) + angular velocity(x,y,z)

##How to run this code:
open your CMD(Command Prompt)
cd to your folder
```
python main.py
```


To Scan the Xsens DOT sensors:
```
devices = await blesensor.Scanner.scan_and_filter_xsens_dot(10)
```

To create BleSensor objects:
```
sensors = []
    for d in devices:
        sensors.append(blesensor.BleSensor(d.address))
```

To Connect sensors:
```
for s in sensors:
    await s.connect()
```        

To identify the sensor:
```
for s in sensors:
    await s.identifySensor()
```



To get battery info:
```
for i in range(len(sensors)):
    batteryInfo = await sensors[i].getBatteryInfo()
    print(batteryInfo)
```


To change the device tag(name), also update the BleSensor object's name:
```
for i in range(len(sensors)):
    sensors[i].name = "dot" + str(i+1)
    await sensors[i].setDeviceTag(sensors[i].name)
```

To disable the loggging:
```
for s in sensors:
    s.recordFlag = False
```

To Power off the sensors:
```
for s in sensors:
    await s.poweroffSensor()
```

Heading reset was done before start recording, during the very first few seconds, make sure the sensors are in the charger, so that they share the same physical heading.

Change the timeToRunInMinute to the time that you want to measure.



