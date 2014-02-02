#include <nbody/constants.h>
#include <nbody/System.h>
#include <nbody/Vector3.h>

#include <fstream>
#include <stdexcept>
#include <iostream>
#include <iomanip>

namespace nbody {

  inline void System::interactBodies( size_t i, size_t j, float softFactor, Vector3f &acc ) const {
    Vector3f r = _body[j].position() - _body[i].position();
    float distance = r.norm() + softFactor;
    float invDist = 1.0f / distance;
    float invDistCubed = cube( invDist );
    acc = acc + NEWTON_G * _body[j].mass() * invDistCubed * r;
  }

  void System::computeGravitation() {
    for( size_t i = 0; i < _nBodies; ++i ) {
      Vector3f acc{ 0.0f, 0.0f, 0.0f };
      for( size_t j = 0; j < _nBodies; ++j ) {
        if( i != j ) {
          interactBodies( i, j, _softFactor, acc );
        }
      }
      _body[i].force() = acc;
    }
  }

  void System::update(int nSteps, float dt, float * all, int evo, int step) {

	  if (_integrator == nullptr){
		  _integrator = new Integrator(_body, _nBodies, _softFactor, _dampingFactor, all, nSteps);
	  }
	  computeGravitation();
	  _integrator->integrateSystem(dt,	RUNGE_KUTTA, evo, step);
  }

  void System::readState( std::istream &input ) {
    input >> _nBodies;
    if( _nBodies > MAX_BODIES_RECOMMENDED ) {
      throw std::runtime_error( "Too many input bodies" );
    }
    _body = new Body[_nBodies];
	for (size_t i = 0; i < _nBodies; ++i) {
		input >> _body[i];
	}
  }

  void System::writeState( std::ostream &output ) const {
    output << _nBodies << "\r\n";
    for( size_t i = 0; i < _nBodies; ++i ) {
      output << _body[i] << "\r\n";
    }
  }

} // namespace nbody
