/*	
 Morphon : 3D Totalistic Cellular Automata 
 Author: Ritesh Lala
 Winter 2011, Media Arts and Technology
*/

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <pthread.h>

#include "myCam.h"
#include "portaudio.h"
#include "randomize.h"
#include "textfile.h"

#define setRule 10
#define IX2(i,j,k) (i + N*j + N*N*k)

//PortAudio definitions

#define MAX_SINES     (500)
#define MAX_USAGE     (0.8)
#define SAMPLE_RATE   (44100)
#define FREQ_TO_PHASE_INC(freq)   (freq/(float)SAMPLE_RATE)

#define MIN_PHASE_INC  FREQ_TO_PHASE_INC(200.0f)
#define MAX_PHASE_INC  (MIN_PHASE_INC * (1 << 5))

#define FRAMES_PER_BUFFER  (512)
#ifndef M_PI
#define M_PI  (3.14159265)
#endif
#define TWOPI (M_PI * 2.0)

#define TABLE_SIZE   (512)

// UDP variables

int sock;
int addr_len, bytes_read;
int recv_data[1]; 
struct sockaddr_in server_addr , client_addr;

#define UDP_PORT 5000 // listen to port 5000 to receive messages from iPhone interface

using namespace std;

/* CA variables */

static int N;
static float _angle, alpha = 1.0f;
static int alphaFlag;

static bool * states, * states_prev;    // Variable for storing current and previous states (ON|OFF) of the cells

static float ruleNumber;
static bool * rule, ruleFlag;
static int globalLoopCounter, neighborhood = 1;
static int stateSpeed = 1;	// the speed at which new states are updated
bool initStates = false;
int liveCells = 0;

/* OpenGL variables */

GLfloat LightAmbient[]=		{ 0.5f, 0.5f, 0.5f, 1.0f };
GLfloat LightDiffuse[]=		{ 1.0f, 1.0f, 1.0f, 1.0f };
GLfloat LightSpecular[] =	{ 1.0f, 1.0f, 1.0f , 1.0f};

GLfloat LightPosition[]=	{ -0.55f, -0.55f, 0.55f, 1.0f};
GLfloat LightPosition2[]=	{ 0.0f, 0.0f, 0.0f, 1.0f};

GLfloat LightC1[]=	{ 0.0f, 0.5098f, 0.0f, 1.0f};			
GLfloat LightC2[]=	{ 0.10f, 0.1764f, 0.7509f, 0.7f};			
GLfloat LP1[]=	{ 0.45f, 0.45f, 0.45f, 1.0f};
GLfloat LP2[]=	{ 1.0f, 1.0f, 1.0f, 1.0f};

float zdist = 0.0f;
bool stereo = false;                    // 3D Stereo projection

int key_flag;
int width = 680;						// width of scene
int height = 680;						// height of scene
static int win_id;
bool grid, bb;							// grid and billboarding flags

static int colorS = 1;
GLuint	texture;                        // Storage For One Texture 

// Camera
myCam my_cam;					// myCam Instance
static int prevX, prevY, prevZ;

/* macros */


/*	Function Declarations	*/

void drawScene();
void mouseMovement(int x, int y);
void mouseCameraMovement(int x, int y);
void handleResize(int w, int h);
void updateStates(int value);
static void mouseFunc ( int button, int state, int x, int y );
void initRendering();
void calcReverseRule (bool *);
static void clear_states ( void );

/* 
 ----------------------------------------------------------------------
 UDP Server Functions
 ----------------------------------------------------------------------
 */

int serverSetup (){
	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		perror("Socket");
		return(0);
	}
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(UDP_PORT);
	server_addr.sin_addr.s_addr = INADDR_ANY;
	bzero(&(server_addr.sin_zero),8);
	
	
	if (bind(sock,(struct sockaddr *)&server_addr,
			 sizeof(struct sockaddr)) == -1)
	{
		perror("Bind");
		return(0);
	}
	
	addr_len = sizeof(struct sockaddr);
	
	cout << endl << "UDPServer Waiting for client on port " << UDP_PORT << endl;
	fflush(stdout);	
	return 1;
}

void *recvUDP (void *){             // Receive messages from iPhone interface and interpret them to do stuff
	
	do{
		bytes_read = recvfrom(sock,recv_data,1,0,
							  (struct sockaddr *)&client_addr, (socklen_t*)&addr_len);
		
		recv_data[bytes_read] = '\0';
		for (int i=0 ; i<bytes_read ; i++ ) {
			cout << recv_data[i];
		}
		
		switch (recv_data[0]){
			case 0:
				my_cam.thrustForward();
				break;
			case 1:
				my_cam.thrustBackward();
				break;
			case 2:
				my_cam.rotateYup();
				break;
			case 3:
				my_cam.rotateYdown();
				break;
				
			case 4:
				ruleFlag = !ruleFlag;
				break;
				
			case 5:
				colorS = 1;
				break;
			case 6:
				colorS = 2;
				break;
			case 7:
				colorS = 3;
				break;
			case 8:
				colorS = 4;
				break;
			case 9:
				colorS = 5;
				break;
				
			case 10:
				clear_states();
				break;
			case 11:
				stateSpeed++;
				break;
			case 12:
				stateSpeed--;
				if(stateSpeed == 0) stateSpeed = 1;
				break;
		}
	}while(1);
	
		
	fflush(stdout);
	fflush(stdin);
}

//----------------------------------------------------------------------------------------------------------\\
//	PortAudio Stuff \\

typedef struct paTestData
{
    int numSines;
    float sine[TABLE_SIZE + 1]; /* add one for guard point for interpolation */
    float phases[MAX_SINES];
}
paTestData;

PaStream*           stream;
paTestData          data = {0};

/* Convert phase between and 1.0 to sine value
 * using linear interpolation.
 */
float LookupSine( paTestData *data, float phase );
float LookupSine( paTestData *data, float phase )
{
    float fIndex = phase*TABLE_SIZE;
    int   index = (int) fIndex;
    float fract = fIndex - index;
    float lo = data->sine[index];
    float hi = data->sine[index+1];
    float val = lo + fract*(hi-lo);
    return val;
}

/* This routine will be called by the PortAudio engine when audio is needed. */

static int patestCallback(const void*                     inputBuffer,
                          void*                           outputBuffer,
                          unsigned long                   framesPerBuffer,
                          const PaStreamCallbackTimeInfo* timeInfo,
                          PaStreamCallbackFlags           statusFlags,
                          void*                           userData )
{
    paTestData *data = (paTestData*)userData;
    float *out = (float*)outputBuffer;
    float outSample;
    float scaler;
    int numForScale;
    unsigned long i;
    int j;
    int finished = 0;
    (void) inputBuffer; /* Prevent unused argument warning. */
	
    /* Determine amplitude scaling factor */
    numForScale = data->numSines;
    if( numForScale < 8 ) numForScale = 8;  /* prevent pops at beginning */
    scaler = 1.0f / numForScale;
    
    for( i=0; i<framesPerBuffer; i++ )
    {
        float output = 0.0;
        float phaseInc = MIN_PHASE_INC;
        float phase;
        for( j=0; j<data->numSines; j++ )
        {
            /* Advance phase of next oscillator. */
            phase = data->phases[j];
            phase += phaseInc;
            if( phase >= 1.0 ) phase -= 1.0;
			
            output += LookupSine(data, phase); 
            data->phases[j] = phase;
            
            phaseInc *= 1.02f;
            if( phaseInc > MAX_PHASE_INC ) phaseInc = MIN_PHASE_INC;
        }
		
        outSample = (float) (output * scaler);
        *out++ = outSample; /* Left */
        *out++ = outSample; /* Right */
    }
    return finished;
}

//----------------------------------------------------------------------------------------------------------\\
// Loading the Texture \\

GLuint loadTexture( const char * filename, int width, int height, int wrap )
{
    GLuint texture;
    unsigned char * data;
    FILE * file;
	
    //The following code will read in the RAW file
    file = fopen( filename, "rb" );
    if ( file == NULL ) return 0;
    data = (unsigned char *)malloc( width * height * 3 );
    fread( data, width * height * 3, 1, file );
    fclose( file );
    glGenTextures( 1, &texture ); //generate the texture with the loaded data
	
    glBindTexture( GL_TEXTURE_2D, texture ); //bind the texture to it’s array
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE ); //set texture environment parameters
  
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST  );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR );
  
	//	Here we are setting the parameter to repeat the texture instead of clamping the texture
    //	to the edge of our shape. 
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap ? GL_REPEAT : GL_CLAMP );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap ? GL_REPEAT : GL_CLAMP );
	
	//	Generate the texture, here's where the texture is actually generated/loaded from "data"
	gluBuild2DMipmaps( GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, data );
    
    free( data ); //free the texture
	
    return texture; //return whether it was successfull
}

void freeTexture( GLuint texture )
{
	glDeleteTextures( 1, &texture ); 
}

//----------------------------------------------------------------------------------------------------------\\
// Loading the Shaders \\

GLuint v,f,f2,p;

void setShaders() {
	
	char *vs = NULL,*fs = NULL;
	
	v = glCreateShader(GL_VERTEX_SHADER);
	f = glCreateShader(GL_FRAGMENT_SHADER);
	
	vs = textFileRead("sss2.vert");
	fs = textFileRead("sss2.frag");
	
	const char * ff = fs;
	const char * vv = vs;
	
	glShaderSource(v, 1, &vv,NULL);
	glShaderSource(f, 1, &ff,NULL);
	
	free(vs);free(fs);
	
	glCompileShader(v);
	glCompileShader(f);
	
	p = glCreateProgram();
	glAttachShader(p,f);
	glAttachShader(p,v);
	
	glLinkProgram(p);
	glUseProgram(p);
}

/*
 ----------------------------------------------------------------------
 free/clear/allocate simulation data
 ----------------------------------------------------------------------
 */


static void free_data ( void )
{
	if ( states ) free ( states );
	if ( states_prev ) free ( states_prev );
	if ( rule ) free ( rule );
}

static void clear_states ( void )
{
	int i, size=N*N*N;
	
	for ( i=0 ; i<=(size-1) ; i++ ) {
		
		states[i] = states_prev[i] = false;
		
	}
	
	states[IX2((N-1)/2,(N-1)/2,(N-1)/2)] = true;
	data.numSines = 2;
	Pa_Sleep(200);
}


static int allocate_data ( void )
{
	int size = N*N*N;
	
	states		= (bool *) malloc ( size*sizeof(bool) );	
	states_prev	= (bool *) malloc ( size*sizeof(bool) );
	rule		= (bool *) malloc ( 128*sizeof(bool) );	
	
	if ( !states || !states_prev || !rule) {
		cout << "cannot allocate data";
		return ( 0 );
	}
	
	return ( 1 );
}

/*	Calculate the Rule	*/
// This creates the rule bits. you access the rule bit as a hash table where the key is obtained by calculating the 
// number accroding to the live cells in the local neighborhood of thisCell, and then use the value in rule[128] to
// get the value for the nextState


void calcRule (float ruleN)
{
	float temp = ruleN;
	
	for (int i = 0; i < 128; i++)
	{
		rule[i] = int (fmod(temp, 2));
		temp /= 2.0f;
	}
}

void calcReverseRule (bool * rule)
{	
	bool ruleBits[128];
	
	for (int i=0 ; i<128 ; i++ ) {
		ruleBits[i] = false;
	}


	float ruleNumberFromRuleBits = 0;
	
	for (int i = 0; i < 128; i++)
	{
		if (ruleBits[i]){
			ruleNumberFromRuleBits += pow (2,i);
		}
	}
}

void *calcNextState (void * )
{
	int i, j, k, totalCellsAround = 0;
	int size = N*N*N;
	int newSines;

	bool nextState = false;
	
	bool * s = states;
	
	initStates = false;
	
	int temp = N-1;
	
	// First setup the boundary conditions
	for (int a=0 ; a<=(N-1) ; a++ ) {
		for (int b=0 ; b<=(N-1) ; b++ ) {
			
			s[IX2(0  , a  , b  )] = initStates;
			s[IX2(0  , 0  , a  )] = initStates;
			
			s[IX2(a  , 0  , b  )] = initStates;
			s[IX2(a  , 0  , 0  )] = initStates;
			
			s[IX2(a  , b  , 0  )] = initStates;
			s[IX2(0  , a  , 0  )] = initStates;

			
			s[IX2(temp, a  , b  )] = initStates;
			s[IX2(a  , temp, b  )] = initStates;
			s[IX2(a  , b  , temp)]  = initStates;

			s[IX2(a  , temp, temp)] = initStates;
			s[IX2(temp, a  , temp)] = initStates;
			s[IX2(temp, temp, a  )] = initStates;				
			
			
			s[IX2(0  , a  , temp)] = initStates;
			s[IX2(temp, a  , 0)] = initStates;
			
			s[IX2(a  , 0  , temp)] = initStates;
			s[IX2(a  , temp, 0  )] = initStates;
			
			s[IX2(0  , temp, a  )] = initStates;
			s[IX2(temp, 0  , a  )] = initStates;
			
		}
	}
	
	s[IX2(0  , 0  , 0  )] = initStates; //(s[IX2(1,0,0)] && s[IX2(0,1,0)] && s[IX2(0,0,1)]);
	s[IX2(0  , 0  , temp  )] = initStates; //(s[IX2(1,0,0)] && s[IX2(0,1,0)] && s[IX2(0,0,temp)]);
	s[IX2(0  , temp  , 0  )] = initStates; //(s[IX2(1,0,0)] && s[IX2(0,temp,0)] && s[IX2(0,0,1)]);
	s[IX2(0  , temp  , temp  )] = initStates; //(s[IX2(1,0,0)] && s[IX2(0,temp,0)] && s[IX2(0,0,temp)]);
	s[IX2(temp  , 0  , 0  )] = initStates; // (s[IX2(temp,0,0)] && s[IX2(0,1,0)] && s[IX2(0,0,1)]);
	s[IX2(temp  , 0  , temp  )] = initStates; //(s[IX2(temp,0,0)] && s[IX2(0,1,0)] && s[IX2(0,0,temp)]);
	s[IX2(temp  , temp  , 0  )] = initStates; //(s[IX2(temp,0,0)] && s[IX2(0,temp,0)] && s[IX2(0,0,1)]);
	s[IX2(temp  , temp  , temp  )] = initStates; //(s[IX2(temp,0,0)] && s[IX2(0,temp,0)] && s[IX2(0,0,temp)]);
	
	for ( i=0 ; i<=(size-1) ; i++ ) states_prev[i] = states[i];	
	
	if ((globalLoopCounter % stateSpeed  == 0) && (ruleFlag)) {		// check for ruleFlag and calculate next states
		liveCells = 0;
			for ( i=1 ; i<=N-2 ; i++ ) {
				for ( j=1 ; j<=N-2; j++ ) {
					for ( k=1; k<=N-2 ; k++ ) {
						
						for (int tempi = -neighborhood; tempi <= neighborhood; tempi++){
							for (int tempj = -neighborhood; tempj <= neighborhood; tempj++){
								for (int tempk = -neighborhood; tempk <= neighborhood; tempk++){
									
									int iNeighbor = i+tempi;
									int jNeighbor = j+tempj;
									int kNeighbor = k+tempk;
									
									if (s[IX2(iNeighbor, jNeighbor, kNeighbor)]) ++totalCellsAround;
								}
							}
						}
					
						if (stateSpeed < 8){
							if (totalCellsAround == 1 || totalCellsAround == 12 ) {
								nextState = true;
								liveCells++;
							}
							
							else if (totalCellsAround == 1 || totalCellsAround == 12 ) {
								nextState = true;
								liveCells++;
							}
							
							else nextState = false;
						}
						
						else {
							if (totalCellsAround == 3 ) {
								nextState = true;
								liveCells++;
							}
							else nextState = false;
						
						}
						
						states_prev[IX2(i,j,k)] = nextState;
											
						totalCellsAround = 0;
					}
				}
			}
			for ( i=0 ; i<=(size-1) ; i++ ) states[i] = states_prev[i];			
	}
	globalLoopCounter++;
	
	if (liveCells == 0) clear_states();
	newSines = 2 + round((((float) liveCells) * MAX_SINES) / size);//((N-6)*(N-6)*(N-6)));

	/* Play sine tones equal to the number of living cells */
	if (newSines < MAX_SINES && ruleFlag) 
		data.numSines = newSines;
	
	Pa_Sleep(200);
	
	pthread_exit(NULL);
}

/*
 ----------------------------------------------------------------------
 GLUT callback routines
 ----------------------------------------------------------------------
 */

//Called when a Special key (Up,Down Arrow etc) is pressed
void handleSpecialKeypress(int key, int x, int y) {
	switch (key) {
			
		case GLUT_KEY_UP:						 //Rotate the camera down (Scene goes up)
			my_cam.rotateXup();
			cout << my_cam._cameraAngleX << endl;
			break;
			
		case GLUT_KEY_DOWN:						 //Rotate the camera up (Scene goes down)
			my_cam.rotateXdown();
			cout << my_cam._cameraAngleX << endl;
			break;
			
		case GLUT_KEY_LEFT:						//Rotate the camera right (Scene goes Left)
			key_flag = glutGetModifiers();
			if (key_flag == GLUT_ACTIVE_SHIFT) {my_cam.rotateZup();}
			else {
				my_cam.rotateYup();
				cout << my_cam._cameraAngleY << endl;
			}
			break;
			
		case GLUT_KEY_RIGHT:					//Rotate the camera left (Scene goes right)
			key_flag = glutGetModifiers();
			if (key_flag == GLUT_ACTIVE_SHIFT) {my_cam.rotateZdown();}
			else {
				my_cam.rotateYdown();
				cout << my_cam._cameraAngleY << endl;
			}			
			break;
	}
}

// Called when a Special key (Up,Down Arrow etc) is released

void handleSpecialUp(int key, int x, int y){
    for (int cameraIncCount = 0; cameraIncCount < 12; cameraIncCount++){
        cout << "up Now" << endl;   
        my_cam._updateCamera[cameraIncCount] = false;
    }
}

//Called when a key is pressed
void handleKeypress(unsigned char key, int x, int y) {
	
	switch (key) {
			
		case (102 | 70):	// "f" or "F" to enter FullScreen Mode
			glutFullScreen();
			break;
			
		case 27:	// Escape key
			if (glutGameModeGet(GLUT_GAME_MODE_ACTIVE)){
				glutLeaveGameMode();
			}
			free_data ();
			exit(0);
			break;
						
//--------------------------------------Begin Controlling Camera Movement-----------------------------------\\
			
		case 32:	// Space Bar
			cout << my_cam._cameraDistZ << endl;
			key_flag = glutGetModifiers();
			if (key_flag == GLUT_ACTIVE_ALT) { my_cam.thrustForward();}
			else {
				my_cam.thrustBackward();
			}
			break;
			
		case (99 | 67):	// "c" or "C"
			key_flag = glutGetModifiers();
			if (key_flag == GLUT_ACTIVE_ALT) { my_cam.thrustLeft();}
			else {
				my_cam.thrustRight();
			}
			break;
			
		case (120 | 88):	// "X" or "x"
			key_flag = glutGetModifiers();
			if (key_flag == GLUT_ACTIVE_ALT) {my_cam.thrustUp();}
			else {
				my_cam.thrustDown();
			}
			break;
			
		case (65 | 97):	// 'a' or 'A'
			my_cam.fovyUp();
			break;
			
		case (90 | 122):	// 'z' or 'Z'
			my_cam.fovyDown();
			break;
			
		case (82 | 114):	// 'r' or 'R'
			my_cam.resetCam();
			break;
			
		case '/':			// Increase stateSpeed
			stateSpeed++;
			cout << stateSpeed << endl;
			break;	
			
		case '.':			// Decrease stateSpeed
			stateSpeed--;
			if(stateSpeed == 0) stateSpeed = 1;
			cout << stateSpeed << endl;
			break;	
			
		case 'n':			// reset cell states
		case 'N':
			clear_states();
			break;
			
		case 's':			// Stop / Resume updating rules
		case 'S':
			ruleFlag = !ruleFlag;
			break;	
			
		case 'g':			// Show / Hide Grid
		case 'G':
			grid = !grid;
			break;	
			
		case 'd':			// Stop / Start Billboarding
		case 'D':
			bb = !bb;
			break;	
			
		case 'v':			// increment rule
		case 'V':
			ruleNumber+=1;
			calcRule(ruleNumber);
			cout << "rule: " << ruleNumber << endl;
			break;
			
		case 'b':			// decrement rule
		case 'B':
			ruleNumber-=1;
			calcRule(ruleNumber);
			cout << "rule: " << ruleNumber << endl;
			break;
			
		case 't':			// decrement rule
		case 'T':
			stereo = !stereo;
			break;			
			
		case '1':			// Points
			colorS = 1;
			break;
			
		case '2':			// Quads
			colorS = 2;
			break;
			
		case '3':			// Spheres
			colorS = 3;
			break;
			
		case '4':			// Texture blend
			colorS = 4;
			break;
			
        case '5':			// Alpha Blend
			colorS = 5;
			break;
	}
 }

void mouseMovement(int x, int y) {      // glMotionFunc Callback

	int diffx=x-prevX; //check the difference between the current x and the last x position
    int diffy=y-prevY; //check the difference between the current y and the last y position
    prevX=x; //set lastx to the current x position
    prevY=y; //set lasty to the current y position
	prevZ=y; //set lasty to the current y position
	
	my_cam._cameraAngleX += (float) diffy; //set the xrot to xrot with the addition	of the difference in the y position
    my_cam._cameraAngleY += (float) diffx;    //set the xrot to yrot with the addition of the difference in the x position
	

}

//window resize callback

void handleResize(int w, int h) {
	width = w;
	height = h;
 	glViewport(0, 0, w/2, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(my_cam._cameraFovy, (double)w/2 / (double)h, 1.0, 200.0);
}

//--------------------------------------End Controlling Camera Movement-----------------------------------\\


/*
 ----------------------------------------------------------------------
 OpenGL specific drawing routines
 ----------------------------------------------------------------------
 */

//Initializes 3D rendering
void initRendering() {
	glClearDepth(1.0f);	
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glDepthFunc(GL_LEQUAL); 
	
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);		// Setup The Ambient Light
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);		// Setup The Diffuse Light
	glLightfv(GL_LIGHT1, GL_SPECULAR, LightSpecular);	// Setup The Diffuse Light
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);	// Position The Light
	
	glLightfv(GL_LIGHT2, GL_DIFFUSE, LightDiffuse);		// Setup The Diffuse Light
	glLightfv(GL_LIGHT2, GL_POSITION, LightPosition2);	// Position The Light
		
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT1);
	glColorMaterial ( GL_FRONT_AND_BACK, GL_SPECULAR  || GL_AMBIENT_AND_DIFFUSE ) ;

	glEnable(GL_COLOR_MATERIAL);                        //Enable color

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
	glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);					// Really Nice Point Smoothing

}

void glInit() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glMatrixMode(GL_MODELVIEW); //Switch to the drawing perspective
	glLoadIdentity(); //Reset the drawing perspective
	
	if (my_cam._updateCamera) my_cam.updateCam();
	
	gluLookAt(-my_cam._cameraDistX, -my_cam._cameraDistY, -my_cam._cameraDistZ, 
              -my_cam._cameraDistX, -my_cam._cameraDistY, -my_cam._cameraDistZ-2.0f, 
              0.0f, 1.0f, 0.0f);
	glRotatef(my_cam._cameraAngleX, 1.0f, 0.0f, 0.0f); //Rotate the camera
	glRotatef(my_cam._cameraAngleY, 0.0f, 1.0f, 0.0f); //Rotate the camera
	glRotatef(my_cam._cameraAngleZ, 0.0f, 0.0f, 1.0f); //Rotate the camera	
}

static void draw_cells ()
{
	
	int i, j, k;
	float x, y, z, h, newh, d000, d001, d010, d011, d100, d101, d110, d111;
	float rR, rG, rB, cR, cG, cB, mR, mG, mB;
	
	cR = 80.0/255.0;
	cG = 118.0/255.0;
	cB = 18.0/255.0;
	
	rR = randomize();
	rG = randomize();
	rB = randomize();
	
	h = 1.0f/N;
	newh =  h;
	
	if (zdist == h/4) zdist = -h/4;
	else zdist = h/4;
	
	glPointSize(15.0f);
	
	for ( i = 0 ; i <= N-1 ; i++ ) {
		x = i*h - h/2;
			for ( j = 0 ; j <= N-1 ; j++ ) {
				y = j*h - h/2;
					for ( k = 0 ; k <= N-1 ; k++ ) {
						z = k*h - h/2;
				
						mR =  0.247; //1.0;
						mG =  0.347; //0.388;
						mB =  fabs (z-0.5) * 2 * 0.147; // 0.294;
						
						
						if(states[IX2(i,j,k)]){
											
							d000 = d010 = d011 = d001 = d100 = d101 = d111 = d110 = alpha;
						
						switch (colorS){
								
							case 1:
								glEnable(GL_DEPTH_TEST);
								glDepthFunc(GL_LEQUAL); 
								glEnable(GL_LIGHTING);
								glEnable(GL_LIGHT1);
								glEnable(GL_LIGHT2);
								
								glDisable (GL_BLEND);
								
								glPushMatrix();
								
								glTranslatef(x, y, z);
								
								if (i < N/2) glColor4f (0.6f, 0.24f, 0.1843f, 0.7f);
								else glColor4f (0.0f, 0.1764f, 0.2509f, 0.7f);
								
								glutSolidSphere(h, 20, 20);
                                glPopMatrix();
								
								break;
								
							case 2:				
								
								glDisable ( GL_TEXTURE_2D );

								
								glDisable(GL_DEPTH_TEST);
								glDisable(GL_LIGHTING);
								
								glEnable (GL_BLEND);
								glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

								glBegin ( GL_QUADS );
								
								glNormal3f(1.0f, 0.0f, 0.0f);
								glColor4f ( mR, mG, mB, d000 ); glVertex3f ( x, y, z );
								glColor4f ( mR, mG, mB, d001 ); glVertex3f ( x, y, z+newh );
								glColor4f ( mR, mG, mB, d011 ); glVertex3f ( x, y+newh, z+newh);
								glColor4f ( mR, mG, mB, d010 ); glVertex3f ( x, y+newh, z );
								
								glNormal3f(0.0f, 1.0f, 0.0f);
								glColor4f ( mR, mG, mB, d010 ); glVertex3f ( x, y+newh, z );
								glColor4f ( mR, mG, mB, d010 ); glVertex3f ( x+newh, y+newh, z );
								glColor4f ( mR, mG, mB, d111 ); glVertex3f ( x+newh, y+newh, z+newh );
								glColor4f ( mR, mG, mB, d011 ); glVertex3f ( x, y+newh, z+newh );
								
								glNormal3f(0.0f, 0.0f, 1.0f);
								glColor4f ( mR, mG, mB, d011 ); glVertex3f ( x, y+newh, z+newh );
								glColor4f ( mR, mG, mB, d001 ); glVertex3f ( x, y, z+newh );
								glColor4f ( mR, mG, mB, d101 ); glVertex3f ( x+newh, y, z+newh );
								glColor4f ( mR, mG, mB, d111 ); glVertex3f ( x+newh, y+newh, z+newh );
								
								glNormal3f(-1.0f, 0.0f, 0.0f);
								glColor4f ( mR, mG, mB, d111 ); glVertex3f ( x+newh, y+newh, z+newh );
								glColor4f ( mR, mG, mB, d110 ); glVertex3f ( x+newh, y+newh, z );
								glColor4f ( mR, mG, mB, d100 ); glVertex3f ( x+newh, y, z);
								glColor4f ( mR, mG, mB, d101 ); glVertex3f ( x+newh, y, z+newh );
								
								glNormal3f(0.0f, -1.0f, 0.0f);
								glColor4f ( mR, mG, mB, d101 ); glVertex3f ( x+newh, y, z+newh );
								glColor4f ( mR, mG, mB, d100 ); glVertex3f ( x+newh, y, z );
								glColor4f ( mR, mG, mB, d000 ); glVertex3f ( x, y, z );
								glColor4f ( mR, mG, mB, d001 ); glVertex3f ( x, y, z+newh );
								
								glNormal3f(0.0f, 0.0f, -1.0f);
								glColor4f ( mR, mG, mB, d000 ); glVertex3f ( x, y, z );
								glColor4f ( mR, mG, mB, d100 ); glVertex3f ( x+newh, y, z );
								glColor4f ( mR, mG, mB, d110 ); glVertex3f ( x+newh, y+newh, z);
								glColor4f ( mR, mG, mB, d010 ); glVertex3f ( x, y+newh, z );		

								glEnd();
								
								break;
								
							case 3:
								
								glEnable(GL_DEPTH_TEST);
								glDepthFunc(GL_LEQUAL); 
								glEnable(GL_LIGHTING);
								glEnable(GL_LIGHT1);
								glEnable(GL_LIGHT2);
								glLightf (GL_LIGHT2, GL_SPOT_CUTOFF, 180.0f);

								glDisable (GL_BLEND);
								
								glPushMatrix();
								
								glTranslatef(x, y, z);
								
                                if (i < N/2) glColor4f (0.6f, 0.24f, 0.1843f, 0.7f);
								else glColor4f (0.0f, 0.1764f, 0.2509f, 0.7f);
								
								glutSolidSphere(3*h, 20, 20);
								glPopMatrix();
								
								break;
								

							case 4:
								
								glEnable ( GL_TEXTURE_2D );
								glBindTexture(GL_TEXTURE_2D, texture);
								
								glDisable(GL_DEPTH_TEST);
								
								glEnable (GL_BLEND);
								
								glBlendFunc(GL_ONE, GL_ONE);			// Enable Alpha Blending (disable alpha testing)
								
									glPushMatrix();
									glTranslatef(x, y, z);
									glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
									
									glBegin(GL_QUADS);
									
									glTexCoord2d(0,1); glVertex2f(0.0f, 2*h);
									glTexCoord2d(1,1); glVertex2f(2*h, 2*h);
									glTexCoord2d(1,0); glVertex2f(2*h, 0.0f);
									glTexCoord2d(0,0); glVertex2f(0.0f, 0.0f);
									
									glEnd();
									
									glPopMatrix();

								glDisable( GL_TEXTURE_2D );

								break;
								
							case 5:
								
								glDisable(GL_DEPTH_TEST);
								glDisable(GL_LIGHTING);
								
								glEnable (GL_BLEND);
								glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
								
								glPushMatrix();
								glTranslatef(x, y, z);
								
								glColor4f (0.17f, fabs (x-0.5) * 2 * 0.347f, 0.047f, alpha);
								glutSolidSphere(h, 20, 20);
								glPopMatrix();
								
								break;
								
						} 
						
				}
			
			}
		}
	}
	
}

//Draws the Scene!!!
void drawScene() {
		
	LightPosition[0] = sinf(_angle); 
	LightPosition[1] = sinf(_angle); 
	LightPosition[2] = sinf(_angle); 
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);	// Position The Light
	
	pthread_t ruleThread;
	int rc = pthread_create(&ruleThread, NULL, calcNextState, NULL);	// Create another thread for calculating rules

	if (rc){
		printf("ERROR; return code from pthread_create() is %d\n", rc);
		exit(-1);
	}
	
	glInit();
	
//	recvUDP();                                          // Enable to receive data from iPhone interface
	
	// Start Drawing Now

	glPushMatrix();
	
	//Move the camera to a nice point of view	
	glTranslatef(-0.5f, -0.5f, -0.5f);			
			
	if(stereo){                                         // Check for Stereo Flag
		glClear(GL_COLOR_BUFFER_BIT |  GL_DEPTH_BUFFER_BIT);
			glDrawBuffer(GL_BACK);
			glClear(GL_COLOR_BUFFER_BIT |  GL_DEPTH_BUFFER_BIT);
		
			glDrawBuffer(GL_BACK_LEFT);
						
			draw_cells ();
		
			glTranslatef(0.025f, 0.0f, 0.0f);
			glDrawBuffer(GL_BACK_RIGHT);
		
			glClear(GL_COLOR_BUFFER_BIT |  GL_DEPTH_BUFFER_BIT);
		
			draw_cells ();
	}
	
	else {
		
		glDrawBuffer(GL_BACK_LEFT);
		glClear(GL_COLOR_BUFFER_BIT |  GL_DEPTH_BUFFER_BIT);

		glViewport(0, 0, width/2, height);
		
		draw_cells ();
		
		glViewport(width/2, 0, width/2, height);
		glTranslatef(-0.050f, 0.0f, 0.0f);
		
		draw_cells ();

	}
	
    glPopMatrix();
	
	glFlush();
	glutSwapBuffers();

	rc = pthread_join(ruleThread, NULL);								// Join the thread back to the draw loop
	if (rc) {
		printf("ERROR; return code from pthread_join() is %d\n", rc);
		exit(-1);
	}
	
		
}

void updateStates(int argc) {
	
	_angle += 0.5f; // oscillate LIGHT1

	if (alpha <= 0.5f && alphaFlag) alpha += 0.05f;
	else if (alpha > 0.5f && alphaFlag) alphaFlag = 0; 
	
	if (alpha >= 0.0f && !alphaFlag) alpha -= 0.05f; 
	else if (alpha < 0.0f && !alphaFlag) alphaFlag = 1; 
	
	glutPostRedisplay(); //Tell GLUT that the display has changed
	
	glutTimerFunc(5, updateStates, 0);		 // last argument is passed to update (value) but unused here
}

int main(int argc, char** argv) {
	
	//Setup UDP Server to get data on a specific UDP_PORT
	if ( !serverSetup () ) exit (1);
	
	pthread_t udpThread;
	int rc2 = pthread_create(&udpThread, NULL, recvUDP, NULL);	// Create another thread for calculating rules
	
	if (rc2){
		printf("ERROR; return code from pthread_create() is %d\n", rc2);
		exit(-1);
	}
	
	
	/* Init PortAudio (todo: Make a separate fn later) */
	
	int                 i;
    PaStreamParameters  outputParameters;
    PaError             err;
	
    /* initialize sinusoidal wavetable */
    for( i=0; i<TABLE_SIZE; i++ )
    {
        data.sine[i] = (float) sin( ((double)i/(double)TABLE_SIZE) * M_PI * 2. );
    }
    data.sine[TABLE_SIZE] = data.sine[0]; /* set guard point */
	
    err = Pa_Initialize();
    if( err != paNoError )
        goto error;
    outputParameters.device                    = Pa_GetDefaultOutputDevice(); /* Default output device. */
    if (outputParameters.device == paNoDevice) {
		fprintf(stderr,"Error: No default output device.\n");
		goto error;
    }
    outputParameters.channelCount              = 2;                           /* Stereo output. */
    outputParameters.sampleFormat              = paFloat32;                   /* 32 bit floating point output. */
    outputParameters.hostApiSpecificStreamInfo = NULL;
    outputParameters.suggestedLatency          = Pa_GetDeviceInfo(outputParameters.device)
	->defaultHighOutputLatency;
    err = Pa_OpenStream(&stream,
                        NULL,               /* no input */
                        &outputParameters,
                        SAMPLE_RATE,
                        FRAMES_PER_BUFFER,
                        paClipOff,          /* No out of range samples should occur. */
                        patestCallback,
                        &data);
    if( err != paNoError )
        goto error;
	
    err = Pa_StartStream( stream );
    if( err != paNoError )
        goto error;
	

	/* Init GLUT and Start Drawing */
	
	glutInit(&argc, argv);
	
		N = 17;                             // N = 64 !
		ruleNumber = setRule;
	
	if ( !allocate_data () ) exit ( 1 );

	clear_states();
	
	if (stereo) glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_ALPHA | GLUT_STEREO);
	else glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_ALPHA);
	
	glutInitWindowSize(width, height);
	
	//Create the window
	win_id = glutCreateWindow(" Morphon : 3D Totalistic Cellular Automata ");
    
    cout << endl << "-----3D Cellular Automaton Engaged-----" << endl;
    
    cout << endl << "Controls: Press keyboard buttons"<< endl 
    << "'1' through '5' for different visuals" << endl
    << "'s' to Play/Pause Rule Calculation" << endl
    << "'.' and '/' to change Speed of Rule Calculation" << endl
    << "Keyboard arrows (or mouse click & drag) to rotate and Spacebar or Alt+Space to Zoom In or Out" << endl
    << "'n' to Reset Cells and 'r' to Reset View" << endl;
    
	initRendering();
	
	//Set handler functions
	glutDisplayFunc(drawScene);
	
	glutMotionFunc(mouseMovement);				//check for mouse motion (with a button pressed) for disrupting fluid flow
	glutKeyboardFunc(handleKeypress);
	glutSpecialFunc(handleSpecialKeypress);
//    glutSpecialUpFunc(handleSpecialUp);
	glutReshapeFunc(handleResize);
	
	//Load Texture
	if(!(texture = loadTexture( "../../glow.raw", 80, 80, 1))) cout << "error loading texture";
	
	glutTimerFunc(5 , updateStates, 0);			// Add a timer to update frames
	
	setShaders();                               // Turn on to use shaders
	
	glutMainLoop();
	
	freeTexture( texture );
	
	/*	Stop Audio Playback */
	
    Pa_Sleep(2000);     /* Stay for 2 seconds before stopping. */
	
    err = Pa_StopStream( stream );
    if( err != paNoError )
        goto error;
	
    err = Pa_CloseStream( stream );
    if( err != paNoError )
        goto error;
	
    Pa_Terminate();
    cout << "Automaton Successful." << endl;
    return err;
	
	error:
		Pa_Terminate();
		fprintf( stderr, "An error occured while using the portaudio stream\n" );
		fprintf( stderr, "Error number: %d\n", err );
		fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
	return err;
	
	rc2 = pthread_join(udpThread, NULL);								// Join the thread back to the draw loop
	if (rc2) {
		printf("ERROR; return code from pthread_join() is %d\n", rc2);
		exit(-1);
	}
	
	
	exit (EXIT_SUCCESS);
}