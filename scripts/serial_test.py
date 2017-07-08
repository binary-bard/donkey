#!/usr/bin/env python3

import serial
from threading import Thread
from time import sleep
import argparse, logging

parser = argparse.ArgumentParser()
parser.add_argument('-d', '--device', default='/dev/serial0', help="Device to use for serial connection")
parser.add_argument('-l', '--logfile', default=None, help="Log file to use")
args = parser.parse_args()
print("Args are", args.device, args.logfile)

try:
  #ser = serial.Serial('/dev/ttyACM0', 115200)
  #ser = serial.Serial('/dev/serial0', 115200)
  ser = serial.Serial(args.device, 115200)
  #ser = serial.Serial('/dev/ttyAMA0', 115200)
except:
  print("Failed to open serial port", args.device)
  quit()

if args.logfile is not None:
  logging.basicConfig(filename=args.logfile, level=logging.DEBUG)
  #logging.basicConfig(filename=args.logfile, level=logging.DEBUG, format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
  logging.Formatter(fmt='%(asctime)s.%(msecs)03d %(message)s', datefmt='%H:%M:%S')

if ser.is_open:
  print(ser.name, "is open")

sleep(2)
inputAvailable = False
entry = ""
bCont = True

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
  #bCont1 = True
  #while bCont1:
  global bCont
  while bCont:
    try:
      read_serial=ser.readline()
      if len(read_serial):
        if args.logfile is not None:
          logging.info(read_serial)
        else:
          print(read_serial)

      #sleep(.02)
    except serial.SerialException:
      print("Exception happened")
      pass
    except KeyboardInterrupt:
      #bCont1 = False
      bCont = False
      #raise

thread = Thread(target = output_function)
thread.start()
#bCont2 = True
#while bCont2:
while bCont:
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
    #bCont2 = False
    bCont = False
    ser.write('th=0'.encode())
    #raise

thread.join()
ser.close()
print('Done')
