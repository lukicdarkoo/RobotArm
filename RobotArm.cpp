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
                 /   * (Gripper)
(Second crank)  *    /\
                |
                |
                ** (First crank & Rotate crank)
           ------------  
	
	
Servo name - Angle name
-----------------------
First 	   - Alpha
Second 	   - Beta
Third 	   - Delta
Rotate	   - Gama
*/

#include "RobotArm.h"

RobotArm::RobotArm() {
	smoothMovementsFinished = false;
	smoothMovements = false;
	keepCrankParalel = false;
	lastRefresh = 0;
	
	// Set default PINS
	// Set default ANGLE FIXES
	for (uint8_t i = 0; i < 5; i++) {
		servosPins[i] = i + 4;
		servoAngleFixes[i] = 0;
		servoDirectionFixes[i] = false;
		defaultAngles[i] = 90;
	}

	// Set default DISTANCE BETWEEN SERVOS
	servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND] = 10;
	servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD] = 10;
	servoDistances[ROBOTARM_DISTANCE_THIRD_GRIPPER] = 0;
	
	// PID
	for (uint8_t i = 0; i < 5; i++) {
		pids[i] = new PID(&(currentAngles[i]), &(nextAngles[i]), &(finalAngles[i]), 0.2, 4, 0, DIRECT);
	}
}

void RobotArm::attach() {
	for (uint8_t i = 0; i < 5; i++) {
		// Set default angles for smooth mode
		finalAngles[i] = defaultAngles[i];
		currentAngles[i] = defaultAngles[i];
		nextAngles[i] = defaultAngles[i];
		
		// Turn on PIDs
		pids[i]->SetMode(AUTOMATIC);
		
		// Attach servos
		servos[i].attach(servosPins[i]);
		setFixedAngle(i, defaultAngles[i]);
	}
}

// Set angle of crank
void RobotArm::setAngle(uint8_t crank, uint8_t angle) {
	if (smoothMovements == true) {

		finalAngles[crank] = angle;
		if (keepCrankParalel == true) {
			finalAngles[ROBOTARM_THIRD_CRANK] = 360 - readFinalAngle(ROBOTARM_FIRST_CRANK) - readFinalAngle(ROBOTARM_SECOND_CRANK);
		}
	}
	else {
		setFixedAngle(crank, angle);
		
		if (keepCrankParalel == true) {
			setFixedAngle(ROBOTARM_THIRD_CRANK, 360 - readFinalAngle(ROBOTARM_FIRST_CRANK) - readFinalAngle(ROBOTARM_SECOND_CRANK));
		}
	}
}



// For smooth mode it returns final angle
uint8_t RobotArm::readFinalAngle(uint8_t crank) {
	if (smoothMovements == true) {
		return finalAngles[crank];
	}
	else {
		return readAngle(crank);
	}
}

// Direct access to each servo
void RobotArm::setFixedAngle(uint8_t crank, uint8_t angle) {
	// Apply direction fix
	uint8_t fixedAngle = servoDirectionFixes[crank] ? 180 - angle : angle;
	
	// Apply angle fix
	fixedAngle += servoAngleFixes[crank];
	
	servos[crank].write(fixedAngle);
	currentAngles[crank] = angle;
}

// Read servo angle with this function
uint8_t RobotArm::readAngle(uint8_t crank) {
	uint8_t fixedAngle = servos[crank].read();

	// Apply angle fix
	fixedAngle -= servoAngleFixes[crank];
	
	// Apply direction fix
	fixedAngle = servoDirectionFixes[crank] ? 180 - fixedAngle : fixedAngle;
	
	return fixedAngle;
}

// Set arm on coordinates
void RobotArm::setCoordinates(float x, float y, float z) {
	Serial.println(x);
	Serial.println(y);
	Serial.println(z);
	
	if (isCoordinateReachable(x, y, z, true) == false) {
		return;
	}
	
	// Cordinates of Second crank
	float p, m, n;
	getCoordinateOfSecondCrank(x, y, z, p, m, n);
	
	// Set angles
	uint8_t alpha = atan2(n, m) * 180 / PI;
	setAngle(ROBOTARM_FIRST_CRANK, alpha);
	
	uint8_t beta = atan2(y - n, p - m) * 180 / PI + 180 - alpha;
	setAngle(ROBOTARM_SECOND_CRANK, beta);
	
	uint8_t gama = atan2(z, x) * 180 / PI;
	setAngle(ROBOTARM_ROTATION_CRANK, gama);
}

void RobotArm::getCoordinateOfSecondCrank(float x, float y, float z, float &p, float &m, float &n) {
	// Calculate angle for First and Second servo
	float l = (float)servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND];
	float d = (float)servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD];
	
	p = sqrt(x*x + z*z);

	// Huh, some crazy Math :(
	// (m, n) are coordinates of Second crank
	// a = z, b = y
	// http://www.wolframalpha.com/input/?i=m%5E2%2Bn%5E2%3Dl1%2C+%28m-a%29%5E2%2B%28n-b%29%5E2%3Dl2
	m = (1.0/(2*(z*z + y*y)))*((-d)*d*z - sqrt((-y)*y*(d*d*d*d - 2*d*d*l*l - 2*d*d*z*z - 2*d*d*y*y + l*l*l*l - 2*l*l*z*z - 2*l*l*y*y + z*z*z*z + 2*z*z*y*y + y*y*y*y)) + l*l*z + z*z*z + z*y*y);

    n = (1.0/(2*y*(z*z + y*y)))*(-d*d*y*y + z*sqrt(-y*y*(d*d*d*d - 2*d*d*l*l - 2*d*d*z*z - 2*d*d*y*y + l*l*l*l - 2*l*l*z*z - 2*l*l*y*y + z*z*z*z + 2*z*z*y*y + y*y*y*y)) + 1.0*l*l*y*y + z*z*y*y + y*y*y*y);
}

// Check if arm can reach coordinates
bool RobotArm::isCoordinateReachable (float x, float y, float z, bool deepCheck) {
	if (sqrt(y*y + z*z) > servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND] + servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD]) {
		return false;
	}
		
	if (servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND] > servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD] && 
		sqrt(y*y + z*z) < servoDistances[ROBOTARM_DISTANCE_FIRST_SECOND] - servoDistances[ROBOTARM_DISTANCE_SECOND_THIRD]) {
		return false;
	}
	
	if (deepCheck == true) {
		float p, m, n;
		getCoordinateOfSecondCrank(x, y, z, p, m, n);
		if (m != m || n != n) {
			return false;
		}
	}
		
	return true;
}

// Call this function at least once every 50ms
void RobotArm::update() {
	// Simulate smoothMovements with PID
	if (!smoothMovementsFinished && smoothMovements) {
		for (uint8_t i = 0; i < 5; i++) {
			if (pids[i]->Compute() == true)  {
				setFixedAngle(i, nextAngles[i]);
			}
		}
	}
}

// Let's see how it works
void RobotArm::printAngles() {
	Serial.print("Alpha: ");
	Serial.println(readFinalAngle(ROBOTARM_FIRST_CRANK));
	
	Serial.print("Beta: ");
	Serial.println(readFinalAngle(ROBOTARM_SECOND_CRANK));
	
	Serial.print("Delta: ");
	Serial.println(readFinalAngle(ROBOTARM_THIRD_CRANK));
	
	Serial.println("");
}