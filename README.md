# IMU and magnetic encoder sensor node
This repositry stores PCB files and firmware for HKUST Cybathlon sensor node.  
Repositry created and maintained by Alex Wong.  
  
## Hardware function
Each sensor node is intended to contain either a ICM-20602 IMU or MA730 magnetic rotary encoder. It is also possible for one node to contain both sensors.  
The sensor nodes are designed to be mounted on various points on the exoskeleton to sense limb attitude or joint angle.  
  
## Communication
Sensor nodes communicate through a 1Mbaud CAN bus network, each node broadcasts sensor readings to the network.   
All sensor nodes should be daisy-chained with JST GH 8-pin cables for power and data.  
Termination connectors must be connected to both ends of the daisy chain to ensure signal integrity.  
Data from all the sensors can be read out at any one of the sensor nodes, through the on-board USB-C port and USB to serial converter.  
It is also possible for sensor nodes to log sensor or CAN bus data to the on-board SD-card slot.  
  
## Power  
Sensor nodes can either be powered through the two JST GH 8-pin connectors or through the USB type-C connector.  
Nominal voltage input range: 4.5v to 6v.  
All sensor nodes are protected from over voltage and reversed voltage, surviving from -64v to 64v (limited by testing method).  
Each sensor node is rated at 200mA in normal operation, and will draw at most 350mA at short circuit.  

