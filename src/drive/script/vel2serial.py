#!/usr/bin/env python

import time
import serial
import math

control = serial.Serial('/dev/ttyACM0', 9600)
encoder = serial.Serial('/dev/ttyACM1', 115200)

def main():
  i = 0
  while i < 100:
    time.sleep(0.1)
    
    msg_x = 0.2
    msg_y = 0.0
    msg_z = 0.0

    cmd = str(-round(msg_y, 4) * 1000.0) + ',' + str(round(msg_x, 4) * 1000.0) + ',' + str(round(msg_z, 4)) + '\n'
    control.write( cmd.encode("utf-8") )
    
    read = encoder.readline().decode("ascii")
    split = read.split()

    control.flush()
    encoder.flush() 
    if len(split) != 3:
      print("FA\t" + read)
      continue 
  
    en = list( map(float, split) )

    print("EN\t" + str(en))
    

  control.write( '0.0,0.0,0.0\r\n'.encode('utf-8') )

if __name__ == '__main__':
  main()
