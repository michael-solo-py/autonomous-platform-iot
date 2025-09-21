# ESP32 Code
# Saves sensor data to SD card and later transfers to Raspberry Pi via UART
try:
    import machine
except ImportError:
    print("The 'machine' module is not available. Ensure you are running this code on an ESP32 with MicroPython.")
    raise
import time
try:
    try:
        import dht
    except ImportError:
        print("The 'dht' module is not available. Mocking the module for testing purposes.")
        class MockDHT22:
            def __init__(self, pin):
                pass
            def measure(self):
                pass
            def temperature(self):
                return 25.0  # Mock temperature
            def humidity(self):
                return 50.0  # Mock humidity
        dht = type('dht', (), {'DHT22': MockDHT22})
except ImportError:
    print("The 'dht' module is not available. Ensure you are running this code on an ESP32 with MicroPython.")
    raise
import json
import os
try:
    import sdcard
except ImportError:
    print("The 'sdcard' module is not available. Mocking the module for testing purposes.")
    class MockSDCard:
        def __init__(self, spi, cs):
            pass
        def readblocks(self, block_num, buf):
            pass
        def writeblocks(self, block_num, buf):
            pass
        def ioctl(self, op, arg):
            return 0
    sdcard = type('sdcard', (), {'SDCard': MockSDCard})

# Initialize UART communication
uart = machine.UART(1, baudrate=115200, tx=17, rx=16)  # TX=GPIO17, RX=GPIO16
sensor = dht.DHT22(machine.Pin(4))  # Temperature & Humidity Sensor

# Initialize SD Card
spi = machine.SPI(1, baudrate=1000000, sck=machine.Pin(18), mosi=machine.Pin(23), miso=machine.Pin(19))
cs = machine.Pin(5, machine.Pin.OUT)
sd = sdcard.SDCard(spi, cs)
fs = os.VfsFat(sd)
os.mount(fs, '/sd')

file_path = '/sd/sensor_data.txt'

while True:
    try:
        sensor.measure()
        temperature = sensor.temperature()
        humidity = sensor.humidity()
        data = {"temperature": temperature, "humidity": humidity, "timestamp": time.time()}
        json_data = json.dumps(data)
        
        # Save data to SD Card
        with open(file_path, 'a') as file:
            file.write(json_data + "\n")
        
        time.sleep(2)
    except Exception as e:
        print("Error:", e)
