# isp_nano_AN0+LCD+CMD+EP+X2+DIF2

Arduino sorta BIOS to handle Sensor-data (here BMP 280[180]) 

1. Sensor  
  11.  PWM(parallel)      <put> constant analog sensor-stream -> through external 3xConverter  for SPS-conform 0-10V(2-10V) signals

2. Connex
  2.  USB(serial)        <put> regular measured and csv formatted data
  2.   -"-               <get>,<parse> command to configur inner parameter  ("debug=[on,off]; Tpwm=[min,max]; EPsave;.. aso")

3. Settings
  3.  USB-commands       soft settable 
  3  Jumper             onboard hard(overrule) setupable
 
4. Memory
  4.  EP[rom]            save,load,[std]  on initialisation/via USB-commands  
                          
  
