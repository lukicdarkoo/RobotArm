RobotArm
========

Arduino library for Robot Arm control. It provides features like:
- easy way to set engle of every crank,
- set arm at specific coordinates,
- smooth movements...


REQUIREMENTS
------------

Arduino 1.0 or higher


INSTALATION
-----------

Just copy all files in '[Arduino path]/libraries'


EXAMPLE
-------

    #include <RobotArm.h>
    
    RobotArm robotArm;
    
    void setup() {
      robotArm.attach();
      
      robotArm.setCoordinates(10, 10, 0);
    }
    
    void loop() {
      robotArm.update();
    }
