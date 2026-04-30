"""
import serial
import serial.tools.list_ports

def find_esp32():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'USB' in port.description or 'CP210' in port.description or 'CH340' in port.description:
            return port.device
    return None

port = find_esp32()
if port:
    ser = serial.Serial(port, 115200, timeout=1)
    print(f"Connected to ESP32 on {port}")
else:
    print("ESP32 not found!")
    exit()

threshold = 1860

try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if line:
                # Check if line has comma
                if ',' in line:
                    adc_value, touch = line.split(',')
                    print(f"ADC: {adc_value} | Touch: {touch}")
                else:
                    # If no comma, assume it's just ADC value
                    try:
                        adc_value = int(line)
                        touch = 1 if adc_value <= threshold else 0
                        print(f"ADC: {adc_value} | Touch: {touch}")
                    except:
                        print(f"Unknown format: {line}")
                        
except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
"""
import time
import serial
import serial.tools.list_ports

def find_esp32():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").upper()
        if "USB" in desc or "CP210" in desc or "CH340" in desc:
            return port.device
    return None

port = find_esp32()
if port:
    ser = serial.Serial(port, 115200, timeout=0.02)
    print(f"Connected to ESP32 on {port}")
else:
    print("ESP32 not found!")
    raise SystemExit

touch_min = 1810
touch_max = 1870

def is_touch(adc_value):
    return touch_min <= adc_value <= touch_max

try:
    while True:
        if ser.in_waiting:
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if "," in line:
                    adc_str, _touch_str = line.split(",", 1)
                    adc_value = int(adc_str.strip())
                    touch = 1 if is_touch(adc_value) else 0
                    print(f"ADC: {adc_value} | Touch: {touch}")
                else:
                    adc_value = int(line)
                    touch = 1 if is_touch(adc_value) else 0
                    print(f"ADC: {adc_value} | Touch: {touch}")

            except ValueError:
                print(f"Unknown format: {line}")

        time.sleep(0.005)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()