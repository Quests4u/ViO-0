# isp_nano_AN0+LCD+CMD+EP+X2+DIF2

Arduino sorta BIOS to handle Sensor-data (here BMP 280[180]) 

1. Sensor  
  2.  PWM(parallel)      <put> constant analog sensor-stream -> through external 3xConverter  for SPS-conform 0-10V(2-10V) signals

3. Connex
  4.  USB(serial)        <put> regular measured and csv formatted data
  4.   -"-               <get>,<parse> command to configur inner parameter  ("debug=[on,off]; Tpwm=[min,max]; EPsave;.. aso")

5. Settings
  5.  USB-commands       soft settable 
  5  Jumper             onboard hard(overrule) setupable
 
6. Memory
  6.  EP[rom]            save,load,[std]  on initialisation/via USB-commands  
                         
7.) hallali what a crap
  
