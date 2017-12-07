import serial 
ser = serial.Serial(port='/dev/serial0', baudrate=9600, bytesize=8, parity='N', stopbits=1) 
print(ser.name)
message = 'A'

while 1:
    ser.write('A'.encode())
    ser.write('B'.encode())
    #message += 1
    
ser.close()
