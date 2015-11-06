 
Befor launching this file it is necessary the following procedures:

--> plug the usb cable to the ps3 remote controller
  $ sixpair 
--> remove the usb cable from the ps3 controller
  $ sixad -s
--> press the PS button until the connection is established
  $ roslaunch teleop_ps3 fsr_hunter.launch
  
  
  Notes:
  
  if there are problems with the bluetooth adapter run:
   
   $ hciconfig hicX  X E [0,1...] try the first two 
   
  if there is no answer then run:
   
   $ rfkill unblock bluetooth
   
   try the first command again
  
  
  else if there is  answer
    
    look if the adpater is "running" and "up"
    
    if not then reun
    
    $ sudo hciconfig hciX up 
    
    
    
    
  
    
   
   
   
   