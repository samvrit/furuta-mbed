import tkinter as tk
from tkinter import messagebox
import serial
import serial.tools.list_ports as ports
import time
import struct
from threading import *

close_request = False
currently_streaming = False

def serial_connect_btn_callback():
	serial_com_port = serial_port_ent.get()
	serial_baud_rate = int(serial_port_baud_rate_ent.get())
	ser.baudrate = serial_baud_rate
	ser.bytesize = 8
	ser.parity = serial.PARITY_NONE
	ser.stopbits = 1
	ser.port = serial_com_port
	ser.rtscts = 0
	try:
		ser.open()
		textbox = f"Connected to {serial_com_port} with baudrate {serial_baud_rate}"
	except Exception as ex:
		textbox = f"Connection failed, error message: {ex}"
	command_ack_label.config(text=textbox)

def serial_close_button_callback():
	serial_com_port = ser.port
	try:
		ser.close()
		textbox = f"Closed connection to {serial_com_port}"
	except Exception as ex:
		textbox = f"Command failed, error message: {ex}"
	command_ack_label.config(text=textbox)

def calibrate_btn_callback():
	ser.write(b'c')
	command_ack_label.config(text="Calibrated")
	
def currently_streaming_btn_callback():
	
	global currently_streaming
	
	if (currently_streaming == False):
		currently_streaming = True
		ser.flushInput()
		ser.write(b's')
		currently_streaming_btn.config(text="Stop Streaming")
		command_ack_label.config(text="Sent command to start streaming")
	elif (currently_streaming == True):
		currently_streaming = False;
		ser.write(b't')
		currently_streaming_btn.config(text="Start Streaming")
		command_ack_label.config(text="Sent command to stop streaming")

def on_closing():
	close_request = True
	if messagebox.askokcancel("Quit", "Do you want to quit?"):
		ser.close()
		print('Closed serial')
		window.destroy()
		print('Closed window')

def update_info():
	while close_request == False:
		time.sleep(0.001)
		
		if(ser.is_open):
		
			id = ser.read()
			
			if (id == b'a'):
				data = ser.read(4)
				data_float = struct.unpack('f', data)
				x1_lbl.config(text="X1: {:.4f}".format(data_float[0]))
			elif (id == b'b'):
				data = ser.read(4)
				data_float = struct.unpack('f', data)
				x2_lbl.config(text="X2: {:.4f}".format(data_float[0]))
			elif (id == b'c'):
				data = ser.read(4)
				data_float = struct.unpack('f', data)
				x3_lbl.config(text="X3: {:.4f}".format(data_float[0]))
			elif (id == b'd'):
				data = ser.read(4)
				data_float = struct.unpack('f', data)
				x4_lbl.config(text="X4: {:.4f}".format(data_float[0]))
			elif (id == b'e'):
				data = ser.read(4)
				data_float = struct.unpack('f', data)
				x5_lbl.config(text="X5: {:.4f}".format(data_float[0]))
			elif (id == b'f'):
				data = ser.read(4)
				data_float = struct.unpack('f', data)
				x6_lbl.config(text="X6: {:.4f}".format(data_float[0]))
			elif (id == b'g'):
				data = ser.read(4)
				data_float = struct.unpack('f', data)
				meas1_lbl.config(text="Theta1: {:.4f}".format(data_float[0]))
			elif (id == b'h'):
				data = ser.read(4)
				data_float = struct.unpack('f', data)
				meas2_lbl.config(text="Theta2: {:.4f}".format(data_float[0]))
			elif (id == b'i'):
				data = ser.read(4)
				data_float = struct.unpack('f', data)
				meas3_lbl.config(text="Theta3: {:.4f}".format(data_float[0]))
			elif (id == b'j'):
				data = ser.read()
				rls_fault_lbl.config(text=str(data))
			elif (id == b'k'):
				data = ser.read()
				motor_fault_lbl.config(text=str(data))

ser = serial.Serial()

available_com_ports = serial.tools.list_ports.comports()
first_available_com_port = str(available_com_ports[0]).split(" ")[0]

window = tk.Tk()
window.protocol("WM_DELETE_WINDOW", on_closing)

serial_port_frame = tk.Frame(master=window, relief=tk.RAISED, borderwidth=1)
info_display_frame = tk.Frame(master=window, relief=tk.RAISED, borderwidth=1)
commands_frame = tk.Frame(master=window, relief=tk.RAISED, borderwidth=1)
command_ack_frame = tk.Frame(master=window, relief=tk.FLAT, borderwidth=1)

serial_port_ent_label = tk.Label(master=serial_port_frame, text="Serial COM Port")
serial_port_ent = tk.Entry(master=serial_port_frame)
serial_port_ent.insert(0,first_available_com_port)
serial_port_baud_ent_label = tk.Label(master=serial_port_frame, text="Serial Port Baud Rate")
serial_port_baud_rate_ent = tk.Entry(master=serial_port_frame)
serial_port_baud_rate_ent.insert(0, "115200")
serial_port_connect_btn = tk.Button(master=serial_port_frame, text="Connect", command=serial_connect_btn_callback)
serial_port_close_btn = tk.Button(master=serial_port_frame, text="Close", command=serial_close_button_callback)
serial_port_ent_label.grid(row=0)
serial_port_ent.grid(row=0,column=1)
serial_port_baud_ent_label.grid(row=1)
serial_port_baud_rate_ent.grid(row=1,column=1)
serial_port_connect_btn.grid(row=2)
serial_port_close_btn.grid(row=2,column=1)

x1_lbl = tk.Label(master=info_display_frame, text="X1: 0")
x2_lbl = tk.Label(master=info_display_frame, text="X2: 0")
x3_lbl = tk.Label(master=info_display_frame, text="X3: 0")
x4_lbl = tk.Label(master=info_display_frame, text="X4: 0")
x5_lbl = tk.Label(master=info_display_frame, text="X5: 0")
x6_lbl = tk.Label(master=info_display_frame, text="X6: 0")
meas1_lbl = tk.Label(master=info_display_frame, text="Theta1: 0")
meas2_lbl = tk.Label(master=info_display_frame, text="Theta2: 0")
meas3_lbl = tk.Label(master=info_display_frame, text="Theta3: 0")
rls_fault_lbl = tk.Label(master=info_display_frame, text="0")
motor_fault_lbl = tk.Label(master=info_display_frame, text="0")
currently_streaming_btn = tk.Button(master=info_display_frame, text="Start Streaming", command=currently_streaming_btn_callback)
x1_lbl.pack(expand=True, fill=tk.X)
x2_lbl.pack(expand=True, fill=tk.X)
x3_lbl.pack(expand=True, fill=tk.X)
x4_lbl.pack(expand=True, fill=tk.X)
x5_lbl.pack(expand=True, fill=tk.X)
x6_lbl.pack(expand=True, fill=tk.X)
meas1_lbl.pack(expand=True, fill=tk.X)
meas2_lbl.pack(expand=True, fill=tk.X)
meas3_lbl.pack(expand=True, fill=tk.X)
rls_fault_lbl.pack(expand=True, fill=tk.X)
motor_fault_lbl.pack(expand=True, fill=tk.X)

currently_streaming_btn.pack()
window.update()

calibrate_btn = tk.Button(master=commands_frame, text="Calibrate", command=calibrate_btn_callback)
calibrate_btn.pack()

command_ack_label = tk.Label(master=command_ack_frame, text="")
command_ack_label.pack()

serial_port_frame.grid(row=0,column=0)
info_display_frame.grid(row=1,column=0)
commands_frame.grid(row=0,column=1)
command_ack_frame.grid(row=2,column=0)

t1=Thread(target=update_info)
t1.start()

window.mainloop()
