import serial
from serial import Serial
from time import sleep

# Connect to the arduino over a serial connection
ser = serial.Serial('/dev/ttyACM0', 9600)
sleep(2)
print('connected and ready to send commands')



while(True):
    inp = input("Press w to move forward: ")
    print(inp)
    if inp is not None:
        inp = ord(inp) 
        print('Sending {:}'.format(inp))
        ser.write(int(inp))
		
