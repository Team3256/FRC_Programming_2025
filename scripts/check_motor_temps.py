import requests
import json
import time
import os

RIO_IP_ADDR = "127.0.0.1"

def get_devices():
    url = f"http://{RIO_IP_ADDR}:1250/?action=getdevices"
    resp = requests.get(url)
    if resp.status_code != 200:
        print(f"Error: {resp.status_code}")
        return None
    
    if resp.text == "":
        print("No response from RIO.")
        return None
    
    resp = resp.json()
    if resp["GeneralReturn"]["Error"] != 0:
        print(f"Error: {resp['GeneralReturn']['ErrorMessage']}")
        return None
    return resp["DeviceArray"]

def get_temp_signal_id(deviceModel, deviceID, deviceCANbus):
    url = f"http://{RIO_IP_ADDR}:1250/?action=getsignals&model={deviceModel}&id={deviceID}&canbus={deviceCANbus}"

    resp = requests.get(url)
    if resp.status_code != 200:
        print(f"Error: {resp.status_code}")
        return None
    if resp.text == "":
        print("No response from RIO.")
        return None
    resp = resp.json()
    if resp["GeneralReturn"]["Error"] != 0:
        print(f"Error: {resp['GeneralReturn']['ErrorMessage']}")
        return None
    
    for signal in resp["Signals"]:
        if signal["Name"] == "DeviceTemp":
            return signal["Id"]

    print(f"Error: DeviceTemp signal not found for {deviceModel} {deviceID} {deviceCANbus}")
    return -1

def get_temp_plot(deviceModel, deviceID, deviceCANbus, signalID):
    url = f"http://{RIO_IP_ADDR}:1250/?action=plotpro&model={deviceModel}&id={deviceID}&canbus={deviceCANbus}&signals={signalID}&resolution=40"

    resp = requests.get(url)
    if resp.status_code != 200:
         print(f"Error: {resp.status_code}")
         return None
    if resp.text == "":
        print("No response from RIO.")
        return None
    resp = resp.json()
    if resp["GeneralReturn"]["Error"] != 0:
        print(f"Error: {resp['GeneralReturn']['ErrorMessage']}")
        return None
    return resp["Points"]
    
def ensure_device_reporting_signals(deviceModel, deviceID, deviceCANbus, signalID):
    url = f"http://{RIO_IP_ADDR}:1250/?action=plotpro&model={deviceModel}&id={deviceID}&canbus={deviceCANbus}&signals={signalID}&resolution=40"

    resp = requests.get(url)
    if resp.status_code != 200:
         print(f"Error: {resp.status_code}")
         return False
    if resp.text == "":
        print("No response from RIO.")
        return False
    resp = resp.json()
    if resp["GeneralReturn"]["Error"] != 0:
        print(f"Error: {resp['GeneralReturn']['ErrorMessage']}")
        return False
    return resp["Points"] != [] or resp["Points"] != None or resp["Signals"] != None or resp["Signals"] != []

def main():
    print("Getting devices...")
    devices = get_devices()
    if devices is None:
        exit(1)
    print("Devices:")
    
    print([device["Name"] for device in devices if 'Talon FX' in device["Model"]])
    motor_devices = [device for device in devices if 'Talon FX' in device["Model"]]
    invalid_devices = []

    if len(motor_devices) == 0:
        print("No Talon FX devices found.")
        exit(1)
    print("Getting DeviceTemp signal IDs...")
    for device in motor_devices:
        signal_id = get_temp_signal_id(device["Model"], device["ID"], device["CANbus"])
        if signal_id == -1:
            print(f"Error: DeviceTemp signal not found for {device['Model']} {device['ID']} {device['CANbus']}")
            motor_devices.remove(device)
            invalid_devices.append(device)
            continue
        if signal_id != 2038:
            print(f"Error: DeviceTemp signal ID is not 2038 for {device['Model']} {device['ID']} {device['CANbus']}")
            motor_devices.remove(device)
            invalid_devices.append(device)
            continue

        print(f"Device: {device['Name']}, Signal ID: {signal_id}")

    print("Getting DeviceTemp plots...")
    retry_devices = []
    for device in motor_devices:
        retry_devices.append(device)
    final_data = []
    retry_count = {}
    while len(retry_devices) > 0:
        device = retry_devices.pop(0)
        plot = get_temp_plot(device["Model"], device["ID"], device["CANbus"], 2038)
        if plot is None or len(plot) == 0:
            print(f"Error: DeviceTemp plot not found for {device['Model']} {device['ID']} {device['CANbus']}")
            print(f"Retrying DeviceTemp plot for {device['Model']} {device['ID']} {device['CANbus']}")
            if device["ID"] not in retry_count:
                retry_count[device["ID"]] = 0
            retry_count[device["ID"]] += 1
            if retry_count[device["ID"]] > 5:
                print(f"Error: DeviceTemp plot not found for {device['Model']} {device['ID']} {device['CANbus']}")
                print(f"Giving up on DeviceTemp plot for {device['Model']} {device['ID']} {device['CANbus']}")
                invalid_devices.append(device)
                continue
            retry_devices.append(device)
            continue
        #print(f"Device: {device['Name']}, Plot: {plot}")
        # plot is a value like
        # {"ordinal": 0, "timestamp": "", "signals": {"2038": "<Actual Data>"}}
        # so we want to get two things:
        # 1. the latest data
        # 2. the average of the last data points
        # get the latest data
        if "2038" not in plot[-1]["Signals"]:
            print(f"Error: No data points found for {device['Name']} and signal 2038")
            retry_devices.append(device)
            continue
        latest_data = plot[-1]["Signals"]["2038"]
        # get the average of the last 10 data points
        print(f"Device: {device['Name']}, Latest Data: {latest_data}")
        final_data.append({
            "Device": device["Name"],
            "Temp": latest_data,
            "Flag": latest_data > 70
           # "Average Data": sum([point["Signals"]["2038"] for point in plot[-10:]]) / 10
        })
        time.sleep(0.1)
    return final_data, invalid_devices
if __name__ == "__main__":
    final_data, invalid_devices = main()
    if len(final_data) > 0:
        print("Final Data:")
        for data in final_data:
            print(data)
    if len(invalid_devices) > 0:
        print("Invalid Devices:")
        for device in invalid_devices:
            print(device["Name"])
    if len(final_data) == 0 and len(invalid_devices) == 0:
        print("No devices found.")
    exit(0)