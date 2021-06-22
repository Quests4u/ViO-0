# isp_nano_AN0+LCD+CMD+EP+X2+DIF2
<pre>
Arduino sorta BIOS to handle Sensor-data (here BMP 280[180]) 

1. Sensor  
  a.)  PWM(parallel)      <put> constant analog sensor-stream -> through external 3xConverter  for SPS-conform 0-10V(2-10V) signals

2. Connex
  a.)  USB(serial)        <put> regular measured and csv formatted data
  b.)   -"-               <get>,<parse> command to configur inner parameter  ("debug=[on,off]; Tpwm=[min,max]; EPsave;.. aso")

3. Settings
  a.)  USB-commands       soft settable 
  b.)  Jumper             onboard hard(overrule) setupable
 
4. Memory
  a.)  EP[rom]            save,load,[std]  on initialisation/via USB-commands  
                        
</pre>  
