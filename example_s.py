import serial
import time
import sys



# Define the serial port and baud rate
serial_port = '/dev/ttyUSB0'  # Change this to your specific serial port
baud_rate = 38400

ser = serial.Serial(serial_port, baud_rate, timeout=1)

# import serial
# import time
# # Configure the serial port
# ser= serial.Serial('/dev/ttyS0', 38400,timeout=1)  # Replace 'COM1' with your actual serial port and 9600 with your baud rate


# Create a serial object







#/dev/ttyS0, /dev/ttyS1
if len(sys.argv) < 3:
    print("Usage: python your_script.py yaw pitch")
    sys.exit(1)

arg1 = sys.argv[1]
arg2 = sys.argv[2]

print("yaw:", arg1)
print("pitch:", arg2)


data_to_send = arg1+','+arg2+'\n'
# Send data
#serial_port.write(data_to_send.encode('utf-8'))
ser.write(data_to_send.encode())
#time.sleep(.1)
#serial_port.flushInput()
#serial_port.flushOutput()
#serial_port.close()

