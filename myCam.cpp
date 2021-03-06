/*
 *  myCam.cpp
 *  asteroids
 *
 *  Created by Ritesh Lala on 4/15/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "myCam.h"
#include <iostream.h>

float fiveDegrees = 5.0f;				// angle increments
float unitDistance = 0.2f;				// distance increments
int i;

void myCam::resetCam (){
	_cameraAngleX = 35.0f;
	_cameraAngleY = 310.0f;
	_cameraAngleZ = 0.0f;
	_cameraDistX = 0.0f;
	_cameraDistY = 0.0f;
	_cameraDistZ = -2.0f;
	_cameraFovy = 45.0f;
};

void myCam::updateCam (){
	for (i=0; i < 12; i++){
		if (_updateCamera[i] ) 
			{
				
				switch (i) {
					case 0:
						_cameraAngleX -= _cameraInc[i] * 7.0f;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 1:
						_cameraAngleX += _cameraInc[i] * 7.0f;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 2:
						_cameraAngleY -= _cameraInc[i] * 7.0f;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 3:
						_cameraAngleY += _cameraInc[i] * 7.0f;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 4:
						_cameraAngleZ -= _cameraInc[i] * 7.0f;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 5:
						_cameraAngleZ += _cameraInc[i] * 7.0f;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 6:
						_cameraDistZ  += _cameraInc[i]/2 * unitDistance;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 7:
						_cameraDistZ  -= _cameraInc[i]/2 * unitDistance;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 8:
						_cameraDistX  += _cameraInc[i]/2 * unitDistance;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 9:
						_cameraDistX  -= _cameraInc[i]/2 * unitDistance;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 10:
						_cameraDistY  += _cameraInc[i]/2 * unitDistance;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					case 11:
						_cameraDistY  -= _cameraInc[i]/2 * unitDistance;
                        _updateCamera[i] = false;
//						_cameraInc[i] += 0.1f;
						break;
					default:
						break;
				}
			}
		else {
			_cameraInc[i] = 2.0f;
			_cameraIncVel[i] = 0.1f;
			_updateCamera[i] = false;
		}
	}
}

// Rotations

void myCam::rotateXup (){
	if (!_updateCamera[0]) _updateCamera[0] = true;
	
	if (_cameraAngleX > 360) {
		_cameraAngleX -= 360;
	}
};

void myCam::rotateXdown (){
	if (!_updateCamera[1]) _updateCamera[1] = true;

	if (_cameraAngleX < 0) {
		_cameraAngleX += 360;
	}
};

void myCam::rotateYup (){
	if (!_updateCamera[2]) _updateCamera[2] = true;
	
	if (_cameraAngleY > 360) {
		_cameraAngleY -= 360;
	}
};

void myCam::rotateYdown (){
	if (!_updateCamera[3]) _updateCamera[3] = true;

	if (_cameraAngleY < 0) {
		_cameraAngleY += 360;
	}
};

void myCam::rotateZup (){
	if (!_updateCamera[4]) _updateCamera[4] = true;
	
	if (_cameraAngleZ > 360) {
		_cameraAngleZ -= 360;
	}
};

void myCam::rotateZdown (){
	if (!_updateCamera[5]) _updateCamera[5] = true;
	
	if (_cameraAngleZ < 0) {
		_cameraAngleZ += 360;
	}
};

// Movements

void myCam::thrustForward (){
	if (!_updateCamera[6]) _updateCamera[6] = true;
};

void myCam::thrustBackward (){
	if (!_updateCamera[7]) _updateCamera[7] = true;
};

void myCam::thrustLeft (){
	if (!_updateCamera[8]) _updateCamera[8] = true;
};

void myCam::thrustRight (){
	if (!_updateCamera[9]) _updateCamera[9] = true;
};

void myCam::thrustUp (){
	if (!_updateCamera[10]) _updateCamera[10] = true;
};

void myCam::thrustDown (){
	if (!_updateCamera[11]) _updateCamera[11] = true;
};

void myCam::fovyUp (){
	_cameraFovy += unitDistance;
};

void myCam::fovyDown (){
	_cameraFovy -= unitDistance;
};