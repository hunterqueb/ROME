import pyfirmata
import time

board = pyfirmata.ArduinoMega('COM3')

i = 0

board.digital[5].mode = pyfirmata.INPUT
# used to protect the user from data being buffered at the serial port causing overflow. this happens when the incoming data is not utilized and matlab likely does this
it = pyfirmata.util.Iterator(board)
it.start()

print("starting")
start = time.time()
while((time.time() - start) < 10):
    sw = board.digital[5].read()
    if sw is True:
        print("on")
    else:
        print("off")
    i = i + 1


print(i/10)
# ~8670.4hz ?????
# this doesnt seem right????
