# main.py บน Raspberry Pi Pico (ปรับปรุง)
from umqtt.simple import MQTTClient
import time
from machine import Pin, ADC, Pin, PWM # สำหรับ Raspberry Pi Pico
import network # สำหรับ Wi-Fi (ถ้า Pico มี Wi-Fi หรือใช้ ESP32/ESP8266)
import json # สำหรับส่งข้อมูลสถานะเป็น JSON
from time import sleep
from machine import Pin, PWM
import time


BUZZER_PIN = 6 # Piezo buzzer + is connected to GP6, - is connected to the GND right beside GP6
buzzer = PWM(Pin(BUZZER_PIN, Pin.OUT))

def playNote(frequency, duration, pause) :
    global buzzer
    buzzer.duty_u16(5000)  # adjust loudness: smaller number is quieter.
    buzzer.freq(frequency)
    time.sleep(duration)
    buzzer.duty_u16(0) # loudness set to 0 = sound off
    time.sleep(pause)
    

def Success():
    #notes = [440, 494, 523, 587, 659, 698, 784]
    notes = [440, 523, 659, 784]
    for note in notes :
        playNote(note, 0.1, 0.01)

def Start():
    notes = [440, 494, 523, 587, 659, 698, 784]
    for note in notes :
        playNote(note, 0.1, 0.01)
def Error():
    notes = [65000, 1000, 65000, 65000, 1000]
    for note in notes :
        playNote(note, 0.1, 0.01)
        
def Click():
    notes = [1500,1200]
    for note in notes :
        playNote(note, 0.1, 0.01)
  


# --- MQTT Configuration ---
MQTT_BROKER = "34.124.162.209"
MQTT_PORT = 1883
DEVICE_ID = "vending_machine_001" # ID เฉพาะสำหรับเครื่องขายของนี้

# --- Wi-Fi Configuration (ถ้าใช้ ESP32/ESP8266 หรือ Pico W) ---
WIFI_SSID = 'SuperTrue_2.4G'
WIFI_PASSWORD = 'Ae789789'
DEBUG = True
# Complete project details at https://RandomNerdTutorials.com/raspberry-pi-pico-pwm-micropython/


# --- Relay Configuration ---
RELAY_PINS = [
    Pin(21, Pin.OUT), # Relay 1 (สินค้า A)
    Pin(20, Pin.OUT), # Relay 2 (สินค้า B)
    Pin(19, Pin.OUT), # Relay 3 (สินค้า C)
    Pin(18, Pin.OUT), # Relay 4 (สินค้า D)
    Pin(17, Pin.OUT), # Relay 5 (สินค้า E)
    Pin(16, Pin.OUT), # Relay 6 (สินค้า F)
    Pin(15, Pin.OUT), # Relay 7 (ไฟตู้)
    Pin(14, Pin.OUT)  # Relay 8 (ไฟเครื่องเครื่อง หยอดเหรียญ)
]
# Buttons (Inputs) - Using internal PULL_UP resistors
BUTTON_PINS = [
    Pin(8, Pin.IN, Pin.PULL_UP), # Button 1
    Pin(9, Pin.IN, Pin.PULL_UP), # Button 2
    Pin(10, Pin.IN, Pin.PULL_UP), # Button 3
    Pin(11, Pin.IN, Pin.PULL_UP), # Button 4
    Pin(12, Pin.IN, Pin.PULL_UP)  # Button 5
]

for relay in RELAY_PINS:
    relay.value(0) # ตั้งค่าเริ่มต้นให้ Relay ปิด (ขึ้นอยู่กับวงจร)


# --- Coin Acceptor Configuration ---
COIN_ACCEPTOR_PIN = Pin(28, Pin.IN, Pin.PULL_UP) # สมมติ Active LOW
last_coin_state = COIN_ACCEPTOR_PIN.value()
MOTOR_SENSOR_PIN = Pin(27, Pin.IN, Pin.PULL_DOWN) # ใช้ PULL_DOWN เพื่อให้สถานะเป็น LOW เมื่อไม่มีสัญญาณ

# --- Global Variables for Button State (for simple debounce) ---
last_button_state = [1] * len(BUTTON_PINS)

# --- Global Variable for Motor Sensor State ---
last_motor_sensor_state = MOTOR_SENSOR_PIN.value() # เก็บสถานะเริ่มต้นของเซ็นเซอร์

# --- Relay Control Functions ---
def set_relay_state(relay_index, state):
    """
    Sets the state of a specific relay.
    relay_index: 0-indexed (0 for CH1, 1 for CH2, etc.)
    state: True for ON, False for OFF
    """
    if 0 <= relay_index < len(RELAY_PINS):
        RELAY_PINS[relay_index].value(0 if state else 1) # Active Low
        print(f"Relay CH{relay_index + 1} set to {'ON' if state else 'OFF'}")
    else:
        print(f"Invalid relay index: {relay_index}")

def turn_all_relays_off():
    """Turns off all relays."""
    for i in range(len(RELAY_PINS)):
        RELAY_PINS[i].value(0) # Turn off (Active Low)
    #print("All relays turned OFF.")


# --- Vending Machine State Variables ---
# สถานะภายในเครื่องขายของ
vending_state = {
    "device_id": DEVICE_ID,
    "uptime_seconds": 0,
    "last_product_dispensed": None,
    "total_dispensed_count": 0,
    "current_credit": 0.0, # ยอดเงินในเครื่อง
    "relay_status": {str(i): "OFF" for i in range(1, 9)}, # สถานะ Relay ปัจจุบัน 1-8
    "is_online": True # สถานะการเชื่อมต่อ
}

# กำหนดราคา/สินค้า (ตัวอย่าง)
PRODUCT_PRICES = {
    "A": 10.0, # สินค้า A ราคา 10 บาท
    "B": 15.0, # สินค้า B ราคา 15 บาท
    "C": 20.0,
    "D": 25.0,
    "E": 30.0,
    "F": 35.0,
    "G": 40.0,
    "H": 45.0
}

# กำหนดว่าสินค้าแต่ละตัวใช้ Relay ตัวไหน
PRODUCT_RELAY_MAPPING = {
    "A": 1,
    "B": 2,
    "C": 3,
    "D": 4,
    "E": 5,
    "F": 6,
    "G": 7,
    "H": 8
}

# --- MQTT Callbacks ---
def sub_cb(topic, msg):
    """Callback เมื่อได้รับข้อความจาก MQTT Broker (คำสั่ง Relay, Product, Credit)"""
    topic_str = topic.decode('utf-8')
    msg_str = msg.decode('utf-8')
    print(f"Received from {topic_str}: {msg_str}")
    try:
        parts = topic_str.split('/')
        if len(parts) >= 5 and parts[0] == "devices" and parts[1] == DEVICE_ID and parts[2] == "commands":
            command_type = parts[3]
            target_id = parts[4]
            
            if command_type == "relay":
                try:
                    relay_index = int(target_id) - 1
                    if 0 <= relay_index < len(RELAY_PINS):
                        if msg_str == "ON":
                            RELAY_PINS[relay_index].value(1)
                            vending_state["relay_status"][str(relay_index + 1)] = "ON"
                            print(f"Relay {relay_index + 1} ON")
                            Click()
                        elif msg_str == "OFF":
                            RELAY_PINS[relay_index].value(0)
                            vending_state["relay_status"][str(relay_index + 1)] = "OFF"
                            print(f"Relay {relay_index + 1} OFF")
                        else:
                            print(f"Unknown relay command: {msg_str}")
                    else:
                        print(f"Invalid relay index: {target_id}")
                except ValueError:
                    print(f"Invalid relay ID format: {target_id}")

            elif command_type == "product":
                if msg_str == "DISPENSE" and target_id in PRODUCT_PRICES:
                    price = PRODUCT_PRICES[target_id]
                    if vending_state["current_credit"] >= price:
                        relay_num = PRODUCT_RELAY_MAPPING.get(target_id)
                        if relay_num:
                            relay_index = relay_num - 1
                            if 0 <= relay_index < len(RELAY_PINS):
                                print(f"Dispensing Product {target_id} via Relay {relay_num}...")
                                RELAY_PINS[relay_index].value(1) # เปิด Relay
                                time.sleep(0.5) # เปิดค้างไว้สักครู่
                                RELAY_PINS[relay_index].value(0) # ปิด Relay

                                vending_state["current_credit"] -= price
                                vending_state["last_product_dispensed"] = target_id
                                vending_state["total_dispensed_count"] += 1
                                print(f"Product {target_id} dispensed. Remaining credit: {vending_state['current_credit']:.2f}")
                                publish_vending_status() # อัปเดตสถานะหลังจำหน่าย
                            else:
                                print(f"Relay mapping error for Product {target_id}")
                        else:
                            print(f"No relay mapped for Product {target_id}")
                    else:
                        print(f"Insufficient credit for Product {target_id}. Need {price:.2f}, have {vending_state['current_credit']:.2f}")
                        client.publish(f"devices/{DEVICE_ID}/status/dispense_fail", f"Insufficient credit for {target_id}")
                else:
                    print(f"Product {target_id} not recognized or invalid action.")

            elif command_type == "credit":
                if target_id == "add":
                    try:
                        amount = float(msg_str)
                        Click()
                        if amount > 0:
                            vending_state["current_credit"] += amount
                            print(f"Credit added: {amount:.2f}. New total: {vending_state['current_credit']:.2f}")
                            publish_vending_status() # อัปเดตสถานะหลังเพิ่มเงิน
                        else:
                            print("Credit amount must be positive.")
                    except ValueError:
                        print(f"Invalid credit amount: {msg_str}")
                else:
                    print(f"Unknown credit command: {target_id}")
            else:
                print(f"Unknown command type: {command_type}")

    except Exception as e:
        Error()
        print(f"Error processing MQTT command: {e}")

# --- MQTT Publish Function ---
def publish_vending_status():
    """Publish สถานะปัจจุบันของเครื่องขายของไปยัง MQTT Broker"""
    vending_state["uptime_seconds"] = int(time.time()) # อัปเดต uptime
    try:
        json_status = json.dumps(vending_state)
        client.publish(f"devices/{DEVICE_ID}/status/full_status", json_status.encode('utf-8'))
        #print(f"Published full status: {json_status}")
    except Exception as e:
        print(f"Error publishing status: {e}")

# --- Wi-Fi Connection Function ---
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to network...')
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        max_attempts = 20
        while not wlan.isconnected() and max_attempts > 0:
            print('.', end='')
            time.sleep(1)
            max_attempts -= 1
        if wlan.isconnected():
            print('\nNetwork config:', wlan.ifconfig())
            vending_state["is_online"] = True
        else:
            Error()
            print('\nFailed to connect to Wi-Fi.')
            vending_state["is_online"] = False
            raise Exception("Wi-Fi connection failed")
    else:
        print('Already connected to network:', wlan.ifconfig())



# --- Main Program ---
if __name__ == '__main__':
    try:
        print("Initializing Vending Machine System with Motor Sensor...")
        turn_all_relays_off()
        time.sleep(1)
        print(COIN_ACCEPTOR_PIN.value(),MOTOR_SENSOR_PIN.value())
        # ตั้งค่าเริ่มต้นสำหรับไฟตู้และไฟหยอดเหรียญ
        set_relay_state(6, 0) # CH6 - เปิดไฟตู้เครื่องขายของ
        set_relay_state(7, 0) # CH7 - เปิดไฟเครื่องหยอดเหรียญ
        
        print("System ready. Waiting for button presses and motor sensor data...")
        connect_wifi()
        client = MQTTClient(DEVICE_ID, MQTT_BROKER, port=MQTT_PORT)
        client.set_callback(sub_cb)
        client.connect()
        print(f"Connected to MQTT broker as {DEVICE_ID}")
        Success()
        # Subscribe เพื่อรับคำสั่ง
        client.subscribe(f"devices/{DEVICE_ID}/commands/relay/#")
        client.subscribe(f"devices/{DEVICE_ID}/commands/product/#")
        client.subscribe(f"devices/{DEVICE_ID}/commands/credit/#") # รับคำสั่งเพิ่ม/จัดการเครดิต
        print(f"Subscribed to commands topics for {DEVICE_ID}")

        last_publish_time = time.time()
        publish_interval = 10 # ส่งสถานะทุก 10 วินาที

        while True:
            # ตรวจสอบสถานะช่องรับเหรียญ
            current_coin_state = COIN_ACCEPTOR_PIN.value()
            if current_coin_state != last_coin_state:
                time.sleep_ms(50) # Debounce
                current_coin_state = COIN_ACCEPTOR_PIN.value()
                if current_coin_state != last_coin_state:
                    if current_coin_state == 0: # สมมติ Active LOW เมื่อมีเหรียญ
                        # เราไม่ได้นับจำนวนเหรียญเฉพาะ แต่จะเพิ่ม credit ตามที่ตั้งค่าไว้
                        # สมมติ 1 เหรียญ = 10 บาท (ปรับตามเหรียญจริงที่ใช้)
                        coin_value = 10.0 # ค่าของเหรียญที่ใส่เข้ามา
                        vending_state["current_credit"] += coin_value
                        print(f"Coin detected! Added {coin_value:.2f}. New credit: {vending_state['current_credit']:.2f}")
                        publish_vending_status() # อัปเดตสถานะทันทีเมื่อมีเหรียญ
                    last_coin_state = current_coin_state

            # ตรวจสอบข้อความ MQTT ที่เข้ามา
            client.check_msg()

            # ส่งสถานะเต็มรูปแบบเป็นระยะ
            if time.time() - last_publish_time >= publish_interval:
                publish_vending_status()
                last_publish_time = time.time()

            time.sleep_ms(100) # หน่วงเวลาเล็กน้อย

    except OSError as e:
        #print(f"Connection or MQTT error: {e}. Setting offline status and retrying...")
        vending_state["is_online"] = False # ตั้งค่าสถานะเป็นออฟไลน์
        time.sleep(5)
    except Exception as e:
        #print(f"An unexpected error occurred: {e}")
        vending_state["is_online"] = False
        time.sleep(5)
