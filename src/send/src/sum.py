import serial
import threading
import tkinter as tk

def read_distance():
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip()
            distance_label.config(text=f'Distance: {data} cm')
            # root.after(3000, clear_distance)

def clear_distance():
    distance_label.config(text='Distance: N/A cm')

def send_led_command(event):
    command = led_entry.get()
    ser.write(command.encode())
    led_entry.delete(0, 'end')
    root.after(1000, clear_led_command)

def clear_led_command():
    led_entry.delete(0, 'end')

ser = serial.Serial('/dev/ttyUSB0', 115200)

root = tk.Tk()
root.title("Control Panel")

distance_label = tk.Label(root, text='Distance: N/A cm')
distance_label.pack()

input_frame = tk.Frame(root)
input_frame.pack()

led_label = tk.Label(input_frame, text="LED Control:")
led_label.grid(row=0, column=0)

led_entry = tk.Entry(input_frame, width=5)
led_entry.grid(row=0, column=1)
led_entry.bind('<Return>', send_led_command)

distance_thread = threading.Thread(target=read_distance)
distance_thread.start()

root.mainloop()
