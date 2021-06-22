# isp_nano_AN0+LCD+CMD+EP+X2+DIF2

Arduino sorta BIOS to handle Sensor-data (here BMP 280[180]) 

1. Sensor  
  1.  PWM(parallel)      <put> constant analog sensor-stream -> through external 3xConverter  for SPS-conform 0-10V(2-10V) signals

1. Connex
  1.  USB(serial)        <put> regular measured and csv formatted data
  1.   -"-               <get>,<parse> command to configur inner parameter  ("debug=[on,off]; Tpwm=[min,max]; EPsave;.. aso")

1. Settings
  1.  USB-commands       soft settable 
  1.  Jumper             onboard hard(overrule) setupable
 
1. Memory
  1.  EP[rom]            save,load,[std]  on initialisation/via USB-commands  
                          
  
