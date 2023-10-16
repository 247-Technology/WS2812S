import serial

# Thiết lập kết nối Serial với ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200)  # Điều chỉnh cổng COMx cho ESP32

# Điều khiển LED
while True:
    command = input("Nhập lệnh (0: Tắt LED, 1: Bật LED): ")
    ser.write(command.encode())
