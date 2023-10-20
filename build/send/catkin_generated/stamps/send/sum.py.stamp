#!/usr/bin/env python
import serial
import threading
import tkinter as tk
import rospy
from std_msgs.msg import String

def read_distance():
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip()
            sensor_data = data.split('\t')  
#create a for loop to automatically create topics that match the data after the \t sign
            for i, reading in enumerate(sensor_data):
                topic_name = f'sensor_{i}' 
                distance_pub = rospy.Publisher(topic_name, String, queue_size=10)
                distance_pub.publish(reading)

def send_led_command(event):
    command = led_entry.get()
    led_control_pub.publish(command)  
    ser.write(command.encode()) 
    led_entry.delete(0, 'end')

def clear_led_command():
    led_entry.delete(0, 'end')

ser = serial.Serial('/dev/ttyUSB0', 115200)

rospy.init_node('serial_to_ros', anonymous=True)
led_control_pub = rospy.Publisher('led_control', String, queue_size=10)
#Design the viewing interface basic
root = tk.Tk()
root.title("Khanh-247")

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
