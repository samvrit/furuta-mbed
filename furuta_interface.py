import tkinter as tk
from tkinter import messagebox
import serial
import serial.tools.list_ports as ports
import time
from threading import *

close_request = False

def serial_connect_btn_callback():
	serial_com_port = serial_port_ent.get()
	serial_baud_rate = int(serial_port_baud_rate_ent.get())
	ser.baudrate = serial_baud_rate
	ser.port = serial_com_port
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
	command_ack_label.config(text="Calibrated")

def on_closing():
	close_request = True
	if messagebox.askokcancel("Quit", "Do you want to quit?"):
		ser.close()
		print('Closed serial')
		window.destroy()
		print('Closed window')

def update_time():
	while close_request == False:
		time.sleep(0.001)
		time_text=time.strftime("%d/%m/%Y %A %H:%M:%S")
		time_lbl.config(text=time_text)

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

time_lbl = tk.Label(master=info_display_frame, text=time.strftime( "%d/%m/%Y %A %H:%M:%S"), font=(21), padx=10, pady=5, bg='#d9d8d7')
time_lbl.pack(expand=True)
window.update()

calibrate_btn = tk.Button(master=commands_frame, text="Calibrate", command=calibrate_btn_callback)
calibrate_btn.pack()

command_ack_label = tk.Label(master=command_ack_frame, text="")
command_ack_label.pack()

serial_port_frame.grid(row=0,column=0)
info_display_frame.grid(row=1,column=0)
commands_frame.grid(row=0,column=1)
command_ack_frame.grid(row=2,column=0)

t1=Thread(target=update_time)
t1.start()

window.mainloop()
