#!/usr/bin/python

# A sample application to read bluetooth data over serial port
# Usage: read_serial device_name baudrate outfile numbytes
# (e.g. read_serial /dev/tty.USB0 115200 out.txt 1000)

import sys
import serial
import struct
import math

MAX_CMD_LENGTH = 6

def open_device(device, rate):
	print "Opening Serial device", device
	btserial = serial.Serial(device,rate)
	print btserial
	return btserial

def read_device(device,length):
	buffer = []
	device.flushInput()
	buffer = device.read(length)
	return buffer

def write_device(device, buffer):
	if(len(buffer) < MAX_CMD_LENGTH):
		device.write(buffer)
		device.flushOutput()
	else:
		print "Error: Was trying to write too long command"

def parse_pkt(buffer):
	pkt_info = ()
	payload = []

	(start_code, pkt_num1, pkt_num2, payload_length) = struct.unpack("!BBBB",buffer[0:4])

	if (start_code != 0xAA):
		print "Error: Failed to detect header at packet start"
		valid = False
	else :
		valid = True
		(step_count, checksum) = struct.unpack("!HH",buffer[60:64])
		pkt_info = (pkt_num1, pkt_num2, step_count, checksum)
		payload = struct.unpack("!ffffffffffffff",buffer[4:60])
	return valid, pkt_info, payload

def create_ack(pkt_num1, pkt_num2):
	ack = []
	ack.append(1)
	ack.append(pkt_num1)
	ack.append(pkt_num2)
	ack.append((1 + pkt_num1 + pkt_num2) / 256)
	ack.append((1 + pkt_num1 + pkt_num2) % 256)
	return ack

def calc_disp(sensor_data, theta):
	d0 = sensor_data[0]
	d1 = sensor_data[1]
	d2 = sensor_data[2]
	d3 = sensor_data[3]

	sin_theta = math.sin(theta)
	cos_theta = math.cos(theta)

	dx = d0 * cos_theta + d1 * sin_theta
	dy = d0 * sin_theta + d1 * cos_theta
	dz = d2
	dp = d3

	disp = math.sqrt(dx * dx + dy * dy + dz * dz)
	return dx,dy,dz,dp,disp

def calc_angle(x,y):
	if (y !=0) and (x != 0 ):
		angle = math.atan(x/y)
		angle = math.degrees(angle)
	else:
		if (x ==0): angle = 0
		else: angle = 90
	return angle

def calc_dist(x,y,z):
	r = x*x + y*y + z*z
	return math.sqrt(r)


if len(sys.argv) < 5 :
	print "Usage: read_serial.py serial_dev_name baudrate outfile numbytes"
else :
	device = sys.argv[1]
	baudrate = int(sys.argv[2])
	outfile = sys.argv[3]
	numbytes = int(sys.argv[4])
	print device, baudrate, outfile, numbytes
	data = []
	btserial = open_device(device,baudrate)

	#write command to start dead step reckoning
	cmd = [0x14,0x0,0x14]
	write_device(btserial,cmd)

	count = 0
	pkt_len = 64
	num_pkts = 0
	curr_pkt = 0
	prev_pkt = -1

	xpos = 0.0   # x-coord in user's reference frame
	ypos = 0.0   # y-coord in user's reference frame
	zpos = 0.0   # z-coord in user's reference frame
	phi  = 0.0   # Angular position around Z-axis in user's reference frame

	while (count < numbytes):
		buffer = read_device(btserial, pkt_len)
		valid, packet_info, payload = parse_pkt(buffer)

		if (valid == False): continue

		curr_pkt = packet_info[0] * 256 + packet_info[1]
		print "Read packet %d...Sending Ack" % curr_pkt
		ack = create_ack(packet_info[0],packet_info[1])
		write_device(btserial,ack)

		if (curr_pkt == prev_pkt): continue

		dx,dy,dz,dp,disp = calc_disp(payload,phi)

		xpos += dx
		ypos += dy
		zpos += dz
		phi  += dp

		radial_dist = calc_dist(xpos,ypos,0) #for now do not factor in zpos

		num_pkts += 1
		count += pkt_len
		prev_pkt = curr_pkt

		data.append(xpos)
		data.append(ypos)
		data.append(zpos)
		data.append(phi)
		data.append(disp)

	#command to stop processing
	cmd = [0x27,0x0,0x27]
	write_device(btserial,cmd)

	#command to stop all outputs
	cmd = [0x21,0x0,0x21]
	write_device(btserial,cmd)
	out = open(outfile, "wb")
	for i in range(0, num_pkts*5, 5):
		out.write(', '.join(str(x) for x in data[i:i+5]))
		out.write("\n")
	out.flush()
	print "Written into file"
	out.close()
