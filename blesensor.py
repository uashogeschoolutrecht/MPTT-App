import numpy as np
import struct
import bleak
from enum import Enum
import asyncio
import datetime
import csv
import os


#global variables
DOT_Configuration_ServiceUUID = "15171000-4947-11E9-8646-D663BD873D93"
DOT_Configuration_Control_CharacteristicUUID = "15171002-4947-11E9-8646-D663BD873D93"
DOT_Measure_ServiceUUID = "15172000-4947-11E9-8646-D663BD873D93"
DOT_Control_CharacteristicUUID = "15172001-4947-11E9-8646-D663BD873D93"
Heading_Reset_Control_CharacteristicUUID = "15172006-4947-11E9-8646-D663BD873D93"
DOT_ShortPayload_CharacteristicUUID = "15172004-4947-11E9-8646-D663BD873D93"
DOT_MediumPayload_CharacteristicUUID = "15172003-4947-11E9-8646-D663BD873D93"
DOT_LongPayload_CharacteristicUUID = "15172002-4947-11E9-8646-D663BD873D93"

Select_Orientation_Euler = bytes([1, 1, 4])
Deselect_Orientation_Euler = bytes([1, 0, 4])
Select_Orientation_Quaternion = bytes([1, 1, 5])
Deselect_Orientation_Quaternion = bytes([1, 0, 5])
Select_CustomMode1 = bytes([1, 1, 22])
Deselect_CustomMode1 = bytes([1, 0, 22])
Heading_Reset_Buffer = bytes([1, 0])



class PayloadMode(Enum):
    orientationEuler = 0
    orientationQuaternion=1
    customMode1 = 2


class DotData:
    def __init__(self):
        self.name = []
        self.address = []
        self.timestamp = []
        self.quaternion = np.array([1, 0, 0, 0])
        self.eulerAngle = np.array([0, 0, 0])
        self.freeAcc = np.array([0, 0, 0])
        self.acc = np.array([0, 0, 0])
        self.angularVelocity = np.array([0, 0, 0])
        self.magneticField = np.array([0, 0, 0])


class Scanner:
    def __init__(self):
        self._client = None

    async def scan_and_filter_xsens_dot(self, timeout=10):
        print(f"Scanning BLE devices for {timeout} seconds.....")
        async with bleak.BleakScanner() as scanner:
            bledevices = await scanner.discover(timeout=timeout)

        if not bledevices:
            print("No BLE devices")

        xsens_dot_devices = []
        for i, d in enumerate(bledevices):
            if d.name and "xsens dot" in d.name.lower():
                xsens_dot_devices.append(d)

        if not xsens_dot_devices:
            print("No Xsens Dot Devices")
            return

        numOfDevices = len(xsens_dot_devices)
        print(f"{numOfDevices} of Xsens DOT devices found:")
        for i, d in enumerate(xsens_dot_devices):
            print(f"{i+1}#: {d.name}, {d.address}")

        return xsens_dot_devices

class BleSensor:
    def __init__(self, address):
        self._client = None
        self.dotdata = []
        self.name = []
        self.address = address
        self.recordFlag = False
        self.fileName = []
        self.payloadType = PayloadMode.orientationEuler


    async def connect(self):
        print(f"connecting to {self.address}")
        self._client = bleak.BleakClient(self.address)
        await self._client.connect()

    async def get_services(self):
        services = self._client.services
        # for service in services:
        #     print(f"Service: {service}")
        #     for characteristic in service.characteristics:
        #         print(f"  Characteristic: {characteristic}")
        await asyncio.sleep(0.1)
        #assign the device tag
        self.name = await self.getDeviceTag()
        await asyncio.sleep(0.1)
        print(f"device tag:{self.name}")

    ##############################################################################
    ########           Start/Stop Measurement, heading reset              ########
    ##############################################################################

    async def startMeasurement(self):
        if self.payloadType == PayloadMode.orientationEuler:
            print("PayloadMode = orientationEuler")
            await self.enable_sensor(DOT_Control_CharacteristicUUID, Select_Orientation_Euler)
            await self._client.start_notify(DOT_ShortPayload_CharacteristicUUID,
                                           self.orientationEuler_notification_handler)
        elif self.payloadType == PayloadMode.orientationQuaternion:
            print("PayloadMode = orientationQuaternion")
            await self.enable_sensor(DOT_Control_CharacteristicUUID, Select_Orientation_Quaternion)
            await self._client.start_notify(DOT_ShortPayload_CharacteristicUUID,
                                           self.orientationQuaternion_notification_handler)
        elif self.payloadType == PayloadMode.customMode1:
            print("PayloadMode = customMode1")
            await self.enable_sensor(DOT_Control_CharacteristicUUID, Select_CustomMode1)
            await self._client.start_notify(DOT_MediumPayload_CharacteristicUUID,
                                           self.customMode1_notification_handler)


    async def stopMeasurement(self):
        if self.payloadType == PayloadMode.orientationEuler:
            print("Stop Measurement PayloadMode = orientationEuler")
            try:
                await self.disable_sensor(DOT_Control_CharacteristicUUID, Deselect_Orientation_Euler)
                print("disable sensor successful")
            except bleak.exc.BleakError as e:
                print(f"Error disable sensor: {e}")

            try:
                await self._client.stop_notify(DOT_ShortPayload_CharacteristicUUID)
            except bleak.exc.BleakError as e:
                print(f"Error stopping notification: {e}")

        elif self.payloadType == PayloadMode.orientationQuaternion:
            print("Stop Measurement PayloadMode = orientationQuaternion")
            try:
                await self.disable_sensor(DOT_Control_CharacteristicUUID, Deselect_Orientation_Quaternion)
                print("disable sensor successful")
            except bleak.exc.BleakError as e:
                print(f"Error disable sensor: {e}")
            try:
                await self._client.stop_notify(DOT_ShortPayload_CharacteristicUUID)
            except bleak.exc.BleakError as e:
                print(f"Error stopping notification: {e}")

        elif self.payloadType == PayloadMode.customMode1:
            print("Stop Measurement PayloadMode = customMode1")
            try:
                await self.disable_sensor(DOT_Control_CharacteristicUUID, Deselect_CustomMode1)
                print("disable sensor successful")
            except bleak.exc.BleakError as e:
                print(f"Error disable sensor: {e}")
            try:
                await self._client.stop_notify(DOT_MediumPayload_CharacteristicUUID)
            except bleak.exc.BleakError as e:
                print(f"Error stopping notification: {e}")

    async def enable_sensor(self, control_uuid, payload):
        await self._client.write_gatt_char(control_uuid, payload)
        await asyncio.sleep(0.1)  # wait for response

    async def disable_sensor(self, control_uuid, payload):
        await self._client.write_gatt_char(control_uuid, payload)



    ##############################################################################
    ########                Information, Heading Reset                    ########
    ##############################################################################

    async def identifySensor(self):
        try:
            bArr = bytearray(32)
            bArr[0] = 1
            bArr[1] = 1
            await self._client.write_gatt_char( DOT_Configuration_Control_CharacteristicUUID, bArr)
            print("identify sensor successful")
        except bleak.exc.BleakError as e:
            print(f"Error identify sensor: {e}")

    async def poweroffSensor(self):
        try:
            bArr = bytearray(32)
            bArr[0] = 2
            bArr[1] = 0
            bArr[2] = 1
            await self._client.write_gatt_char( DOT_Configuration_Control_CharacteristicUUID, bArr)
            print(f"power off sensor {self.address} successful")
        except bleak.exc.BleakError as e:
            print(f"power off sensor  {self.address} : {e}")

    async def doHeadingReset(self):
        await self._client.write_gatt_char(Heading_Reset_Control_CharacteristicUUID, Heading_Reset_Buffer)
        await asyncio.sleep(0.1)  # wait for response

    async def getDeviceTag(self):
        await asyncio.sleep(0.2)  # wait for response
        # Read the device tag
        read_bytes = await self._client.read_gatt_char(DOT_Configuration_Control_CharacteristicUUID)
        device_tag_bytes = read_bytes[8:24]
        #remove the blank bytes
        device_tag_bytes = device_tag_bytes.replace(b'\x00', b'')
        #decode to ascii name
        device_tag = device_tag_bytes.decode()
        print(f"read device tag: {device_tag}.")
        return device_tag

    async def setDeviceTag(self, tag: str) -> bool:
        if not tag or len(tag) > 16:
            return False

        allowedTag = " !;_()-+=.%@$={}[]^#~0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
        for i, char in enumerate(tag):
            if char not in allowedTag:
                print(f"Tag name is not allowed at {char}")
                return False

        bArr = bytearray(32)
        bArr[0] = 9
        bArr[1] = 1
        bArr[7] = len(tag)
        if len(tag) > 0:
            #converts the tag string to bytes
            bytes = tag.strip().encode()
            #copies the bytes into the bArr array starting at index 8.
            bArr[8:8 + len(bytes)] = bytes
        try:
            await self._client.write_gatt_char(DOT_Configuration_Control_CharacteristicUUID, bArr)
            print(f"changed the tag name to {tag} for  {self.address}  successful.")
        except bleak.exc.BleakError as e:
            print(f"change the tage name failed for  {self.address} : {e}")
        return True

    ##############################################################################
    ########                Data Parsing                                  ########
    ##############################################################################

    def orientationEuler_notification_handler(self, sender, data):
        # print(f"Received data: {data.hex()}")
        #d893decb0a3509c146500842ea43853e00000000
        #20 bytes of data, only the first 16 bytes valid
        hexData = data.hex()
        time = self.timeStampConvert(hexData)
        euler = self.eulerConvert(hexData)
        stringToPrint = f"Name: {self.name}, " + f"Address: {self.address}, " + "Time: {:.0f}, Roll: {:.1f}, Pitch: {:.1f}, Yaw: {:.1f}".format(time, euler[0], euler[1], euler[2])
        print(stringToPrint)
        if self.recordFlag == True:
            dotdata = DotData()
            dotdata.name = self.name
            dotdata.address = self.address
            dotdata.timestamp = time
            dotdata.eulerAngle = euler
            self.record_data(dotdata)

    def orientationQuaternion_notification_handler(self, sender, data):
        # print(f"Received data: {data.hex()}")
        #20 bytes of data
        hexData = data.hex()
        time = self.timeStampConvert(hexData)
        quat = self.quaternionConvert(hexData)
        stringToPrint = f"Address: {self.address}, " + "Time: {:.0f}, q0: {:.1f}, q1: {:.1f}, q2: {:.1f}, q3: {:.1f}".format(time, quat[0], quat[1], quat[2], quat[3])
        print(stringToPrint)
        if self.recordFlag == True:
            dotdata = DotData()
            dotdata.name = self.name
            dotdata.address = self.address
            dotdata.timestamp = time
            dotdata.quaternion = quat
            self.record_data(dotdata)

    def customMode1_notification_handler(self, sender, data):
        # print(f"Received data: {data.hex()}")
        # 526bea567cc32c4128fdca41f87912be62528c40e301e9bf9b199ebfb5c4bdbe8a5f7cbe6725de3c
        # #40 bytes of data
        hexData = data.hex()
        time = self.timeStampConvert(hexData)
        euler = self.eulerConvert(hexData)
        freeAccX = struct.unpack('<f', bytes.fromhex(hexData[32:40]))[0]
        freeAccY = struct.unpack('<f', bytes.fromhex(hexData[40:48]))[0]
        freeAccZ = struct.unpack('<f', bytes.fromhex(hexData[48:56]))[0]
        freeAcc = np.array([freeAccX, freeAccY, freeAccZ])
        gyroX = struct.unpack('<f', bytes.fromhex(hexData[56:64]))[0]
        gyroY = struct.unpack('<f', bytes.fromhex(hexData[64:72]))[0]
        gyroZ = struct.unpack('<f', bytes.fromhex(hexData[72:80]))[0]
        gyro = np.array([gyroX, gyroY, gyroZ])
        stringToPrint = f"Address: {self.address}, " + "Time: {:.0f}, Roll: {:.1f}, Pitch: {:.1f}, Yaw: {:.1f}".format(time, euler[0], euler[1],
                                                                                        euler[2])
        stringToPrint += ", freeAccX: {:.1f}, freeAccY: {:.1f}, freeAccZ: {:.1f}".format(freeAccX, freeAccY,
                                                                                        freeAccZ)
        stringToPrint += ", gyroX: {:.1f}, gyroY: {:.1f}, gyroZ: {:.1f}".format(gyroX, gyroY,
                                                                                         gyroZ)
        stringToPrint += " "
        print(stringToPrint)
        if self.recordFlag == True:
            dotdata = DotData()
            dotdata.name = self.name
            dotdata.address = self.address
            dotdata.timestamp = time
            dotdata.eulerAngle = euler
            dotdata.freeAcc = freeAcc
            dotdata.angularVelocity = gyro
            self.record_data(dotdata)

    def timeStampConvert(self, data):
        t = bytes.fromhex(data[:8])
        return struct.unpack('<I', t)[0]

    def eulerConvert(self, data):
        roll = struct.unpack('<f', bytes.fromhex(data[8:16]))[0]
        pitch = struct.unpack('<f', bytes.fromhex(data[16:24]))[0]
        yaw = struct.unpack('<f', bytes.fromhex(data[24:32]))[0]
        euler = np.array([roll, pitch, yaw])
        return euler

    def quaternionConvert(self, data):
        #unpacks the 4-byte data from the 8th to the 16th hex digit, convert to a float
        w = struct.unpack('<f', bytes.fromhex(data[8:16]))[0]
        x = struct.unpack('<f', bytes.fromhex(data[16:24]))[0]
        y = struct.unpack('<f', bytes.fromhex(data[24:32]))[0]
        z = struct.unpack('<f', bytes.fromhex(data[32:40]))[0]
        quat = np.array([w, x, y, z])
        return quat

    ##############################################################################
    ########                Data Saving                                   ########
    ##############################################################################
    def create_csvfile(self):
        # Extract necessary information from the data
        m_address = self.address.replace(':', '_')
        # Create the folder path
        folder_path = os.path.join(os.getcwd(), 'data_logging')
        # Create the folder if it does not exist
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        # Create the file name based on the information extracted
        now = datetime.datetime.now()
        file_name = f"{self.name}_{m_address}_{now.strftime('%y%m%d_%H%M%S')}.csv"
        # Create the file path
        file_path = os.path.join(folder_path, file_name)
        # Write the title row and the data to the csv file
        with open(file_path, mode='w', newline='') as csv_file:
            print(f"file created: {file_name}")
            writer = csv.writer(csv_file)
            if self.payloadType == PayloadMode.orientationEuler:
                writer.writerow(['name', 'address', 'timestamp', 'roll', 'pitch', 'yaw'])
            elif self.payloadType == PayloadMode.orientationQuaternion:
                writer.writerow(['name', 'address', 'timestamp', 'q0', 'q1', 'q2', 'q3'])
            elif self.payloadType == PayloadMode.customMode1:
                writer.writerow(['name', 'address', 'timestamp', 'roll', 'pitch', 'yaw','freeAccX','freeAccY','freeAccZ', 'gyroX', 'gyroY', 'gyroZ'])

        csv_file.close()
        return file_path

    def record_data(self, data):
        # Write the new data to the existing csv file
        with open(self.fileName, mode='a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            if self.payloadType == PayloadMode.orientationEuler:
                writer.writerow([data.name, data.address, data.timestamp, data.eulerAngle[0], data.eulerAngle[1], data.eulerAngle[2]])
            elif self.payloadType == PayloadMode.orientationQuaternion:
                writer.writerow([data.name, data.address, data.timestamp, data.quaternion[0], data.quaternion[1], data.quaternion[2], data.quaternion[3]])
            elif self.payloadType == PayloadMode.customMode1:
                writer.writerow([data.name, data.address, data.timestamp, data.eulerAngle[0], data.eulerAngle[1], data.eulerAngle[2], data.freeAcc[0],data.freeAcc[1],data.freeAcc[2], data.angularVelocity[0], data.angularVelocity[1], data.angularVelocity[2]])

