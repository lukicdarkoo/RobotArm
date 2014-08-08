/*
 RobotArm.cpp - Arduino library for easy Robot Arm control.
 Copyright (c) 2014 Darko Lukic (lukicdarkoo@gmail.com).  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
EXPLANATION

                   * (Third crank)
                  / \
                 /   * (Grabber)
(Second crank)  *    /\
                |
                |
                ** (Rotation & First crank)
           ------------
	
	
Servo name - Angle name
-----------------------
First 	   - Alpha
Rotation   - Beta
Second 	   - Gama
Third 	   - Delta
*/

#include "RobotArm.h"

RobotArm::RobotArm() {
	setDefaults();
}

void RobotArm::attach() {
	for (uint8_t i = 0; i < 5; i++) {
		servos[i].attach(servosPins[i]);
		setAngle(i, 90);
	}
}

void RobotArm::setDefaults() {
	smoothMovementsFinished = false;
	smoothMovements = true;
	keepCrankParalel = true;
	smoothly = 100;
	lastRefresh = 0;
	
	// Set default PINS (first, second, third, rotate & grabber)
	// Set default ANGLE FIXES
	for (uint8_t i = 0; i < 5; i++) {
		servosPins[i] = i + 4;
		servoAngleFixes[i] = 0;
		servoDirectionFixes[i] = false;
	}

	// Set default DISTANCE BETWEEN SERVOS
	servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND] = 10;
	servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD] = 10;
	servoDistances[ROBOTARM_DISTANCE_THIRD_GRABBER] = 0;
}

// Set angle of crank
void RobotArm::setAngle(uint8_t crank, uint8_t angle) {
	if (smoothMovements) {
		startAngles[crank] = readAngle(crank);
		finalAngles[crank] = angle;
		lastServoSpeed[crank] = smoothly;
		
		if (keepCrankParalel) {
			startAngles[ROBOTARM_THIRD_CRANK] = readAngle(ROBOTARM_THIRD_CRANK);
			finalAngles[ROBOTARM_THIRD_CRANK] = abs(readFinalAngle(ROBOTARM_FIRST_CRANK) - readFinalAngle(ROBOTARM_SECOND_CRANK));
			lastServoSpeed[ROBOTARM_THIRD_CRANK] = smoothly;
		}
	}
	else {
		setFixedAngle(crank, angle);
		
		if (keepCrankParalel)
			setFixedAngle(ROBOTARM_THIRD_CRANK, abs(readFinalAngle(ROBOTARM_FIRST_CRANK) - readFinalAngle(ROBOTARM_SECOND_CRANK)));
	}
}

// Read servo angle with this function
uint8_t RobotArm::readAngle(uint8_t crank) {
	// Apply direction fix
	uint8_t fixedAngle = servoDirectionFixes[crank] ? 180 - servos[crank].read() : servos[crank].read();
	
	// Apply angle fix
	fixedAngle += servoAngleFixes[crank];

	return fixedAngle;
}

// For smooth mode it returns final angle
uint8_t RobotArm::readFinalAngle(uint8_t crank) {
	if (smoothMovements)
		return finalAngles[crank];
	else return readAngle(crank);
}

// Direct access to each servo
void RobotArm::setFixedAngle(uint8_t crank, uint8_t angle) {
	// Apply direction fix
	uint8_t fixedAngle = servoDirectionFixes[crank] ? 180 - angle : angle;
	
	// Apply angle fix
	fixedAngle += servoAngleFixes[crank];
	
	servos[crank].write(fixedAngle);
}

// Set arm on coordinates
// Hard part for someone who hates Mathematics
// Check picture for more information
void RobotArm::setCoordinates(short x, short y, short z) {
	// Calculate angle for Rotation servo
	uint8_t beta = atan2(z, x) * 180 / PI;
	setAngle(ROBOTARM_ROTATION_CRANK, beta);
	
	// Calculate angle for First and Second servo
	short l = servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND];
	short d = servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD];

	// (m, n) are coordinates of Second crank
	// p is x axis in extra coordinate system
	// m^2+n^2=l^2 & (m - p)^2+(n-y)^2=d^2
	float p = sqrt(x*x + z*z);
    float m = (1.0/(2*(p*p + y*y)))*((-d)*d*p - sqrt((-y)*y*(d*d*d*d - 2*d*d*l*l - 2*d*d*p*p - 2*d*d*y*y + l*l*l*l - 2*l*l*p*p - 2*l*l*y*y + p*p*p*p + 2*p*p*y*y + y*y*y*y)) + l*l*p + p*p*p + p*y*y);
	
    float n = (1.0/(2*y*(p*p + y*y)))*(-d*d*y*y + p*sqrt(-y*y*(d*d*d*d - 2*d*d*l*l - 2*d*d*p*p - 2*d*d*y*y + l*l*l*l - 2*l*l*p*p - 2*l*l*y*y + p*p*p*p + 2*p*p*y*y + y*y*y*y)) + 1.0*l*l*y*y + p*p*y*y + y*y*y*y);


	// s = sqrt(x*x + y*y + z*z), it's along from (0, 0, 0) to (x, y, z)
	uint8_t alpha = atan2(n, m) * 180 / PI;
	setAngle(ROBOTARM_FIRST_CRANK, alpha);
	
	uint8_t gama = atan2(y - n, p - m) * 180 / PI + 180 - alpha;
	setAngle(ROBOTARM_SECOND_CRANK, gama);
}

// Check if arm can reach coordinates
bool RobotArm::isCoordinateReachable (short x, short y, short z) {
	if (sqrt(x*x + y*y + z*z) > servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND] + servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD])
		return false;
		
	if (servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND] > servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD] && 
		sqrt(x*x + y*y + z*z) < servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND] - servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD])
		return false;
		
	return true;
}

// Let's see how it works
void RobotArm::printAngles() {
	Serial.print("Alpha: ");
	Serial.println(readFinalAngle(ROBOTARM_FIRST_CRANK));
	
	Serial.print("Beta: ");
	Serial.println(readFinalAngle(ROBOTARM_ROTATION_CRANK));
	
	Serial.print("Gama: ");
	Serial.println(readFinalAngle(ROBOTARM_SECOND_CRANK));
	
	Serial.print("Delta: ");
	Serial.println(readFinalAngle(ROBOTARM_THIRD_CRANK));
	
	Serial.println("");
}

// Call this function at least once every 50ms
void RobotArm::update() {

	// Smooth movements are a little bit complicated
	if (smoothMovements && !smoothMovementsFinished) {

		if (millis() > lastRefresh + 10) {
			lastRefresh = millis();

			smoothMovementsFinished = true;
			
			for (uint8_t crank = 0; crank < 5; crank++) {
				if (readAngle(crank) != finalAngles[crank]) {
				
					if (abs(readAngle(crank) - finalAngles[crank]) <= 2) {
						setFixedAngle(crank, finalAngles[crank]);
						break;
					}
				
					smoothMovementsFinished = false;
				
					int8_t direction = (finalAngles[crank] > readAngle(crank)) ? 1 : -1;
					
					if ((finalAngles[crank] + startAngles[crank]) / 2 < readAngle(crank))
						lastServoSpeed[crank] -= (10.0 / smoothly) * direction;
					else lastServoSpeed[crank] += (10.0 / smoothly) * direction;
					
					setFixedAngle(crank, readAngle(crank) + lastServoSpeed[crank] * direction);
				}
			}
		}
	}
}