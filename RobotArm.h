/*
 RobotArm.h - Arduino library for easy Robot Arm control.
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

#ifndef RobotArm_h
#define RobotArm_h

	#include <../Servo/Servo.h>
	#include <Arduino.h>
	#include <inttypes.h>
	#include <math.h>
	
	#define PI 3.14159265

	// Servos
	#define ROBOTARM_FIRST_CRANK 0
	#define ROBOTARM_SECOND_CRANK 1
	#define ROBOTARM_THIRD_CRANK 2
	#define ROBOTARM_ROTATION_CRANK 3
	#define ROBOTARM_GRABBER 4
	
	// Distances between servos
	#define ROBOTARM_DISTANCE_FIRST_SECOND 0
	#define ROBOTARM_DISTANCE_SECOND_THIRD 1
	#define ROBOTARM_DISTANCE_THIRD_GRABBER 2
	
	// Errors
	#define ROBOTARM_ERROR_OUTOFRANGE 1
	#define ROBOTARM_ERROR_NOSOLUTION 2

	class RobotArm {
		public:
			RobotArm();
			Servo servos[5];	// This should be private, but who knows... :)
			
			void attach(); 	// Call this in setup()
			void update();	// Call this function at least once every 50ms
			void setCoordinates(float, float, float);	// Set arm to specific coordinates
			void setAngle(uint8_t, uint8_t);			// Set angle of arm
			uint8_t readAngle(uint8_t);					// Read angle of crank
			bool isCoordinateReachable(float, float, float);	// Check if coordinates are reachable
			float smoothly;
			
			// For easy debugging
			bool debug;
			void printAngles(uint8_t, uint8_t, uint8_t);
			
			// Define these parameters before setCoordinates() or setAngle()
			int8_t servoAngleFixes[5]; 		// Change start angle with this
			bool servoDirectionFixes[5];	// Fix direction of each servo
			uint8_t servosPins[5];			// Pins for each servo
			float servoDistances[3];		// Distance between each servo. Unit doesn't matter, try with mm
			bool smoothMovements;
			
			uint8_t smoothAnglePart;
		private:
			void setFixedAngle(uint8_t, uint8_t);
			void setDefaults();
			
			// Temp values
			float lastServoSpeed[5];
			uint8_t finalAngles[5];
			uint8_t startAngles[5];
	};
#endif