#include <RobotArm.h>


RobotArm robotArm;
int x = 0, y = 0, z = 0;



void setup() {
  robotArm.servoAngleFixes[ROBOTARM_SECOND_CRANK] = -67;
  robotArm.servoDirectionFixes[ROBOTARM_SECOND_CRANK] = true;
  robotArm.servoDirectionFixes[ROBOTARM_FIRST_CRANK] = true;
  robotArm.servoAngleFixes[ROBOTARM_FIRST_CRANK] = -37;
  robotArm.servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND] = 19;
  robotArm.servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD] = 10;
  
  Serial.begin(115200);
  Serial.setTimeout(3);
  
  robotArm.attach();
}

void loop() {
  robotArm.update();
  
  if (Serial.available() > 0) {
    // x10y10z0
    // x5y5z5
    
    switch(Serial.read()) {
      case 's':
        robotArm.setAngle(ROBOTARM_SECOND_CRANK, Serial.parseInt());
        break;
        
      case 'f':
        robotArm.setAngle(ROBOTARM_FIRST_CRANK, Serial.parseInt());
        break;
        
       case 'r':
        robotArm.setAngle(ROBOTARM_ROTATION_CRANK, Serial.parseInt());
        break;
        
       case 't':
        robotArm.setAngle(ROBOTARM_THIRD_CRANK, Serial.parseInt());
        break;
      
      case 'x':
        x = Serial.parseInt();
        break;
        
      case 'y':
        y = Serial.parseInt();
        break;
        
      case 'z':
        z = Serial.parseInt();
        robotArm.setCoordinates(x, y, z);
        break;
    }
  }
}
