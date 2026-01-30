import asyncio
import time
import struct
# Import the HTTP library in order to push to Thingysboard
import requests  
# Import scanner 
from bleak import BleakScanner

# CONFIGURATION
TARGET_NAME = {"Lab4-Adv", "LabGroup1"}
COMPANY_ID = 0x0059 

# ThingsBoard Config
TB_URL = "https://demo.thingsboard.io/api/v1"
TB_ACCESS_TOKEN = "yyg96elwr9hjg19hfgot"  # <--- PASTE TOKEN HERE

# State variables for throttling
last_sent_time = 0
UPLOAD_INTERVAL = 2.0 # Send to cloud every 1 second (even if BLE is faster)

def push_to_cloud(temperature, grp_id, grp_rssi):
    """Sends JSON data to ThingsBoard via HTTP"""
    url = f"{TB_URL}/{TB_ACCESS_TOKEN}/telemetry"
    
    json_key_temp = f"Temperature_{grp_id}"
    json_key_rssi = f"RSSI_{grp_id}"    
    
    # Payload format follows this JSON schema: https://thingsboard.io/docs/reference/http-api/
    payload = {
        json_key_temp: temperature, 
        json_key_rssi: grp_rssi
    }
    
    try:
        response = requests.post(url, json=payload, timeout=2)
        if response.status_code == 200:
            print(f" -> Cloud Upload Success: {payload}")
        elif response.status_code == 400:
            print(f"Invalid URL, request parameters of body")
        elif response.status_code == 404:
            print(f"Invalid ACCESS_TOKEN used")
        else:
            print(f" -> Cloud Error: {response.status_code}")
    except Exception as e:
        print(f" -> Cloud Connection Failed: {e}")

def detection_callback(device, advertisement_data):
    global last_sent_time
    
    if device.name and device.name in TARGET_NAME:
        if COMPANY_ID in advertisement_data.manufacturer_data:
            
            # This allows you to debug and observe the raw data from your BLE packet
            raw_packet = advertisement_data.manufacturer_data
            print(raw_packet)
            
            # We only want the data after the company ID part 
            raw_bytes = advertisement_data.manufacturer_data[COMPANY_ID]
            print(f"Actual data payload in HEX is: {raw_bytes.hex(' ')}")
            
            try:
                # 1. Decode BLE: refer to https://docs.python.org/3/library/struct.html, Section: Format Characters
                unpacked = struct.unpack("<hB", raw_bytes)

                temperature_c = unpacked[0] / 100.0 # Convert to float
                group_id = unpacked[1]
                current_rssi = advertisement_data.rssi
                
                if group_id < 0:
                    print(f"Error: Group ID {group_id} is out of bounds from device {device.name}")
                    return
                
                # Print real-time to console
                print(f"[{device.address}] BLE Rx: {temperature_c:.2f} °C from Group {group_id:d} with RSSI {current_rssi:d}")     
                
                # 2. Upload to Cloud (Throttled)
                current_time = time.time()
                if (current_time - last_sent_time) >= UPLOAD_INTERVAL:
                    push_to_cloud(temperature_c, group_id, current_rssi)
                    last_sent_time = current_time
                    
            except Exception as e:
                print(f"Error: {e}")
        else:
            print(f"Warning: Company ID mismatch...")

async def main():
    print(f"Starting Gateway for {TARGET_NAME}...")
    scanner = BleakScanner(detection_callback=detection_callback, scanning_mode='active')
    await scanner.start()
    await asyncio.Event().wait()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopping Gateway...")