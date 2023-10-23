import serial

# Thiết lập kết nối Serial với ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200)  # Điều chỉnh cổng COMx cho ESP32

# Đọc và in thông tin khoảng cách
while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').rstrip()
        print(data)
