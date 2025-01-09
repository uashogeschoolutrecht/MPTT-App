import asyncio
import time
from bleak import BleakScanner, BleakClient
import sys
import os


# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from movella_dot_py.core.sensor import MovellaDOTSensor
from movella_dot_py.models.data_structures import SensorConfiguration
from movella_dot_py.models.enums import OutputRate, FilterProfile, PayloadMode

async def main():
    # Scan for sensors
    print("Scanning for Movella DOT sensors (5 seconds)...")
    devices = await BleakScanner.discover(timeout=5.0)
    dot_devices = [d for d in devices if d.name and "Movella DOT" in d.name]
    
    if not dot_devices:
        print("No Movella DOT sensors found")
        return
        
    # Limit to maximum 5 sensors
    max_sensors = 5
    dot_devices = dot_devices[:max_sensors]
    print(f"Found {len(dot_devices)} Movella DOT sensors")
    
    # Create sensor instances
    sensors = []
    # Change to your desired values.
    config = SensorConfiguration(
        output_rate=OutputRate.RATE_1,
        filter_profile=FilterProfile.DYNAMIC,
        payload_mode=PayloadMode.CUSTOM_MODE_5
    )
    
    # Connect and configure all sensors
    for device in dot_devices:
        try:
            sensor = MovellaDOTSensor(config)
            sensor.client = BleakClient(device.address)
            print(f"\nConnecting to {device.name} ({device.address})...")
            await sensor.client.connect()
            sensor.is_connected = True
            sensor._device_address = device.address
            sensor._device_name = device.name
            
            # Get and print device info
            print("\nReading device information...")
            device_info = await sensor.get_device_info()
            sensor._device_tag = device_info.device_tag
            print(f"MAC Address: {device_info.mac_address}")
            print(f"Firmware Version: {device_info.firmware_version}")
            print(f"Serial Number: {device_info.serial_number}")
            print(f"Product Code: {device_info.product_code}")
            print(f"Device Tag: {device_info.device_tag}")
            print(f"Current Output Rate: {device_info.output_rate} Hz")
            print(f"Current Filter Profile: {device_info.filter_profile.name}")
            
            # Identify the sensor
            print("\nIdentifying sensor...")
            await sensor.identify_sensor()
            await asyncio.sleep(2)  # Wait for LED blinking
            
            # Configure sensor
            await sensor.configure_sensor()
            sensors.append(sensor)
            print(f"Successfully connected and configured {device.name}")
            
        except Exception as e:
            print(f"Failed to connect to {device.name}: {str(e)}")
    
    if not sensors:
        print("No sensors were successfully connected")
        return
    
    try:
        # Start measurement on all sensors
        print("\nStarting measurements on all sensors...")
        await asyncio.gather(*(sensor.start_measurement() for sensor in sensors))

        print("\nCollecting data for 5 seconds...")
        await asyncio.sleep(5)
        
        # Stop measurement on all sensors
        print("\nStopping measurements...")
        await asyncio.gather(*(sensor.stop_measurement() for sensor in sensors))
        
        await asyncio.sleep(1)        
        # Start recording on all sensors
        print("\nStarting recording on all sensors...")
        await asyncio.gather(*(sensor.start_recording(duration_seconds=10) for sensor in sensors))
        
        
        # Stop recording on all sensors
        print("\nStopping recordings...")
        await asyncio.gather(*(sensor.stop_recording() for sensor in sensors))
        
        # Print collected data summary for each sensor
        for i, sensor in enumerate(sensors):
            print(f"\nSensor Data Summary:")
            data = sensor.get_collected_data()
            if data:
                print(f"Device: {data['device_tag']}")
                print(f"MAC Address: {data['mac_address']}")
                timestamps = data['timestamps']
                euler_angles = data['euler_angles']
                
                if len(timestamps) > 0:
                    print(f"Collected {len(timestamps)} samples")
                    print(f"Time span: {(timestamps[-1] - timestamps[0])/1e6:.2f} seconds")
                    if len(euler_angles) > 0:
                        print(f"First euler angles: {euler_angles[0]}")
                        print(f"Last euler angles: {euler_angles[-1]}")
                else:
                    print("No data was collected")
                    
    except Exception as e:
        print(f"Error during measurement: {str(e)}")
    finally:
        # Disconnect all sensors
        print("\nDisconnecting all sensors...")
        await asyncio.gather(*(sensor.disconnect() for sensor in sensors))

if __name__ == "__main__":
    asyncio.run(main())
 