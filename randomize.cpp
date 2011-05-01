/*
 *  randomize.cpp
 *  chaos
 *
 *  Created by Ritesh Lala on 3/6/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "randomize.h"

#include <stdlib.h>
#include <OpenGL/OpenGL.h>

/*	Functions for Randomization	*/

float randomize() {
	return (float) rand () / (GLfloat) RAND_MAX;
	//	return 0.7;
}

float randomize20() {
	return ((float) rand () / (GLfloat) RAND_MAX) * 20.0;
}

float randomize360() {
	return ((float) rand () / (GLfloat) RAND_MAX) * 360.0;
}
