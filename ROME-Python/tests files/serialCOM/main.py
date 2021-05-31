import serial
import serial.tools.list_ports
import time

comlist = serial.tools.list_ports.comports()
connected = []
for element in comlist:
    connected.append(element.device)
print("Connected COM ports: " + str(connected))

serialcom = serial.Serial('COM3', 115200)
serialcom.timeout = 1

while True:
    i = input("input: ").strip()
    if i == "done":
        break

    serialcom.write(i.encode())
    print(serialcom.readline().decode('ascii'))

serialcom.flush()
serialcom.close()
