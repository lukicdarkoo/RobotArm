RobotArm
========

Arduino library for Robot Arm control. It provides features like:
- Set angle of each crank easy,
- Set arm at specific coordinates,
- Smooth movements,
- Auto keep grabber parallel to surface...


REQUIREMENTS
------------

Arduino 1.0 or higher


INSTALATION
-----------

Just copy all files in **[Arduino path]/libraries**


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
