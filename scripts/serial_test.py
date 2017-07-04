#!/usr/bin/env python3

import serial
from threading import Thread
from time import sleep

ser = serial.Serial('/dev/ttyACM0',115200)
sleep(2)
inputAvailable = False
entry = ""

def run_test():
  ser.write('m=1\n'.encode())
  for s in [0, 1000, 1600, 1200, 1700, 1300, 1800, 1400, 1900, 0]:
    sstr = 's=' + str(s) + '\n'
    for t in [0, 1200, 0, 1200, 1600, 0, 1700, 1800, 1300, 0, 1300, 0]:
      tstr = 't=' + str(t) + '\n'
      ser.write(sstr.encode())
      ser.write(tstr.encode())
      #Arduino should continue to send out these pulses
      sleep(1)
 

def output_function():
  bCont1 = True
  while bCont1:
    try:
      read_serial=ser.readline()
      print(read_serial)
      #sleep(.02)
    except KeyboardInterrupt:
      bCont1 = False
      raise

thread = Thread(target = output_function)
thread.start()
bCont2 = True
while bCont2:
  try:
    entry = input("Print value to send: ");
    if len(entry):
      if entry == 'r30':
        ser.write('l=90\n'.encode())
        ser.write('r=30\n'.encode())
      elif entry == 'l30':
        ser.write('r=90\n'.encode())
        ser.write('l=30\n'.encode())
      elif entry == 'r60':
        ser.write('l=90\n'.encode())
        ser.write('r=0\n'.encode())
      elif entry == 'l60':
        ser.write('r=90\n'.encode())
        ser.write('l=0\n'.encode())
      elif entry == 'r90':
        ser.write('l=90\n'.encode())
        ser.write('r=-90\n'.encode())
      elif entry == 'l90':
        ser.write('r=90\n'.encode())
        ser.write('l=-90\n'.encode())
      elif entry == 'test':
        run_test()
      else:
        ser.write(entry.encode())
      entry = ""

  except KeyboardInterrupt:
    bCont2 = False
    ser.write('th=0'.encode())
    raise

thread.join()
print('Done')
