import serial
import time
port = serial.Serial("/dev/rfcomm0", baudrate=9600)
port.reset_input_buffer()

for i in range(50):
	port.write(bytes(str(i),'utf-8'))
	inVar = port.read_until().decode()
	print(f"OUT: {i}, IN: {inVar}")

port.close()