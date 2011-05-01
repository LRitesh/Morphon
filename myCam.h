/*
 *  myCam.h
 *  asteroids
 *
 *  Created by Ritesh Lala on 4/15/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

class myCam {
	
public:	
	float _cameraAngleX;
	float _cameraAngleY;
	float _cameraAngleZ;
	float _cameraDistX;			// X Distance
	float _cameraDistY;			// Y Distance
	float _cameraDistZ;			// Z Distance
	float _cameraFovy;			// fovy: field of view
	float _cameraInc[12];			// Integrator 
	float _cameraIncVel[12];			// Integrator velocity
	bool  _updateCamera[12];		// flag for integrator

public:
	void rotateXup ();			// Rotate around X axis
	void rotateXdown ();
	void rotateYup ();			// Rotate around Y axis
	void rotateYdown ();
	void rotateZup ();			// Rotate around Z axis
	void rotateZdown ();
	
	void thrustForward ();		// Thrust Forward (Zoom In)
	void thrustBackward ();		// Thrust Backward (Zoom Out)	
	void thrustLeft ();			// Strafe Left 
	void thrustRight ();		// Strafe Right
	void thrustUp ();			// Up
	void thrustDown ();			// Down
	
	void resetCam ();			// Reset the camera
	void updateCam ();			// Update the camera for integration smoothing1
	
	void fovyUp ();				// Increase Field of View
	void fovyDown ();			// Decrease Field of View
	
	myCam () {
		_cameraAngleX = 35.0f;
		_cameraAngleY = 310.0f;
		_cameraAngleZ = 0.0f;
		_cameraDistX  = 0.0f;
		_cameraDistY  = 0.0f;
		_cameraDistZ  = -4.0f;

		_cameraFovy   = 45.0f;
		for (int j=0; j < 12; j++) _cameraInc[j]    = 2.0f;
		for (int j=0; j < 12; j++) _cameraIncVel[j]    = 0.1f;
		for (int k=0; k < 12; k++) _updateCamera[k] = false;
	}
};