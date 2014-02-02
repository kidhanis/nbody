#include <nbody/Simulation.h>
#include "GlutWrapper.h"
#include "Shaders.h"

#include <glload/gl_3_0.h>
#include <glload/gll.hpp>
#include <GL/freeglut.h>

#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>
int evolutions;
int allSize;
int steps;
float dt;
namespace nBodyShaders {

  const std::string vertex1(
    "#version 130\n"
    "in vec4 position;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = position;\n"
    "}\n"
  );

  const std::string fragment1(
    "#version 130\n"
    "out vec4 outputColor;\n"
    "void main()\n"
    "{\n"
    "   outputColor = vec4(1.0f,1.0f,1.0f,1.0f);"// COLOR OF RING THAT TURNS
    "}\n"
  );
} // namespace shaders

class NBodyWindow : public GlutWrapper {
public:
	NBodyWindow( const std::string &title, 
               Mode debugMode = Mode::NDEBUG );
  ~NBodyWindow();

  void display();
  void reshape( int theWidth, int theHeight );
  void keyboard( unsigned char key, int x, int y );
};

NBodyWindow::NBodyWindow( const std::string &title, Mode debugMode ) : 
GlutWrapper{ title, debugMode } {
	_instance = this;
}

NBodyWindow::~NBodyWindow() {}

void NBodyWindow::reshape( int theWidth, int theHeight ) { // CHANGES THE SIZE OF THE RING, NOT OF THE WINDOW.
	glViewport( 0, 0, (GLsizei) theWidth, (GLsizei) theHeight );
}

void NBodyWindow::keyboard( unsigned char key, int /*x*/, int /*y*/ ) {
	const char ESCAPE_KEY = 27;
  if( key == ESCAPE_KEY ) {
    glutLeaveMainLoop();
  }
}

int theCount = 0;

void NBodyWindow::display() {
	// Adjust positions

	for( size_t i = 0; i < _bufSize / 4; ++i ) {// THESE ADD TO THE BUFFER THE COORDINATES, AND MAKE THE RING ROTATE

		_buf[4*i] = _all[(_bufSize* theCount + i * 4)%allSize]/.5f;
		_buf[4*i+1] = _all[(_bufSize*theCount+ i * 4+1)%allSize]/.5f;
		_buf[4 * i + 2] = _all[(_bufSize*theCount + i * 4 + 2) % allSize]/.5f;
		_buf[4*i+ 3] = 1.0f;
		/*_buf[4 * i] = cosf(2 * 3.1415f * float(i + theCount / 20000.0f) / float(_bufSize / 4));
		_buf[4 * i + 1] =  sinf(2 * 3.1415f * float(i - theCount / 20000.0f) / float(_bufSize / 4));
		_buf[4*i+2] = 0.0f;
		_buf[4*i+3] = 1.0f;*/
		theCount++;//MAKES THE RING TURN BY ADDING TO SINE AND COSINE
  }
	if (theCount == evolutions*steps){
		glutLeaveMainLoop();
	}
	glBindBuffer( GL_ARRAY_BUFFER, _positionBufferObject );
	glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof( float ) * _bufSize, _buf );
	glBindBuffer( GL_ARRAY_BUFFER, 0 );

	glClearColor( 0.0f, 0.0f, 0.0f, 0.0f ); //BACKGROUND COLOR OF RING
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );  // COLOR_BUFFER REFILLS COLOR AFTER RING PASSES.  CAN USE AFTER BODIES MOVE.

  glUseProgram( _program );
  glBindBuffer( GL_ARRAY_BUFFER, _positionBufferObject );
  glEnableVertexAttribArray( 0);
  glVertexAttribPointer( 0, 4, GL_FLOAT, GL_FALSE, 0, 0 ); // THE 4 CHANGES THE SOURCE OF THE LINES OF THE RING, THE STRIDE MAKES AN ADDITIONAL LINE 

  glDrawArrays( GL_POINTS, 0, (GLsizei) _bufSize );// TRIANGLE_STRIP MAKES THE LINES OF THE RING, THE GLSIZE IS THE SIZE OF THE BUFFER, OR UP TO HOW MANY LINES TO INPUT

  glDisableVertexAttribArray( 0);
  glUseProgram( 0 );
  glutSwapBuffers();
	glutPostRedisplay();// THIS STOPPED THE RING FROM MOVING WHEN COMMENTED OUT
}

int main(int argc, char **argv) {
	size_t N = 2;
	steps = 1e3;
	dt = 0.00001;
	evolutions = 70;
	size_t bufSize = 4 * N;
	allSize = bufSize * evolutions*steps;
	float *all = new float[allSize];
	try {
		std::ifstream input{ "resources/nbody/binary-system-simple.txt" };
		nbody::Simulation sim{ input };
		for (int i = 0; i < evolutions; ++i) {
			std::cout << "==EVOLUTION " << i + 1 << "\n";
			sim.saveRun();
			sim.evolveSystem(steps, dt, all, i);
		}
		sim.saveRun();
	}
	catch (const std::exception &e) {
		std::cerr << "Error: " << e.what() << "\n";
	}
  try {
		float *buf = new float[bufSize*evolutions*steps];

		for( size_t i = 0; i < N; ++i ) {
			buf[4*i] = cosf( 2 * 3.1415f * float( i ) / float( N ) );
			buf[4*i+1] = sinf( 2 * 3.1415f * float( i ) / float( N ) );
			buf[4*i+2] = 0.0f;
			buf[4*i+3] = 1.0f;
			std::cout << buf[4*i] << " " << buf[4*i+1];
      std::cout << " " << buf[4*i+2] << " " << buf[4*i+3] << "\n";
    }

		Shaders shaders;
		shaders.addToVertexList( nBodyShaders::vertex1 ); //FIRST STRING AT TOP
		shaders.addToFragmentList( nBodyShaders::fragment1 );//SECOND STRING AT TOP
    NBodyWindow window{ "N-Body Simulation", GlutWrapper::NDEBUG };
		window.init( argc, argv, 500, 500, &shaders,all, bufSize, buf );
		window.run();
		delete[] all;
    return 0;
  } catch( const std::exception &e ) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}

