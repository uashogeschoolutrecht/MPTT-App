import asyncio
import blesensor
import time
import sys

###############################################################################################
####### Notice: 2023-03-24                                                          ###########
####### before running this program                                                 ###########
####### please firstly go to Windows Bluetooth, Add device, and connect that DOT!!! ###########
###############################################################################################

async def main():
    #scan sensors
    devices = await blesensor.Scanner.scan_and_filter_xsens_dot(10)
    if not devices:
        print("No Xsens DOT")
        return

    sensors = []
    for d in devices:
        sensors.append(blesensor.BleSensor(d.address))

    print("start to connect:")
    for s in sensors:
        await s.connect()
    print("connect finish.")

    print("\nServices and characteristics:")
    for s in sensors:
        await s.get_services()

    # await asyncio.sleep(2)  # wait for response
    # print("Identify Sensors")
    # for s in sensors:
    #     await s.identifySensor()
    # await asyncio.sleep(2)  # wait for response

    # await asyncio.sleep(2)  # wait for response
    # print("Change Device Tag")
    # for s in sensors:
    #     await s.setDeviceTag("myDot")
    # await asyncio.sleep(2)  # wait for response

    #Start Measurement in specific mode, cutomMode1 or orientationEuler or orientationQuaternion
    for s in sensors:
        s.payloadType = blesensor.PayloadMode.customMode1
        await s.startMeasurement()
    print("\nNotifications enabled. Waiting for data...")

    #heading reset
    await asyncio.sleep(0.2)  # wait for response
    print("do heading reset")
    for s in sensors:
        await s.doHeadingReset()
        #default is no recording, here to enable the recording after heading reset
        s.fileName = s.create_csvfile()
        s.recordFlag = True


    # Run within the timeToRunInMinute time range.
    startTimeSeconds = int(round(time.time()))
    timeToRunInMinute = 0.2
    print(f"run the test for {timeToRunInMinute} minutes")
    while int(round(time.time())) - startTimeSeconds < timeToRunInMinute*60:
        await asyncio.sleep(0.1)

    #stop measurement
    for s in sensors:
        await s.stopMeasurement()
    await asyncio.sleep(0.5)

    # print("Power Off Sensors")
    # for s in sensors:
    #     await s.poweroffSensor()
    # await asyncio.sleep(2)  # wait for response

    print("exit program")
    await sys.exit(0)


if __name__ == "__main__":
    asyncio.run(main())
