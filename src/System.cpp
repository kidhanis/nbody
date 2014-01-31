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

  void System::integrateSystem( float dt ) {
	  for (size_t f = 0; f < _nBodies; ++f){
		  Vector3f k1v, k2v, k3v, k4v, k1r, k2r, k3r, k4r, v, r;
		  v = _body[f].velocity();
		  r = _body[f].position();

		  k1r = dt*v;
		  k1v = dt*grav(r, f);
		  k2r = dt*(v + k1v / 2.0f);
		  k2v = dt*(grav(r + k1r / 2.0f, f));
		  k3r = dt*(v + k2v / 2.0f);
		  k3v = dt*(grav(r + k2r / 2.0f, f));
		  k4r = dt*(v + k3v);
		  k4v = dt*(grav(r + k3r, f));
		  v = v + (k1v + 2.0f * k2v + 2.0f * k3v + k4v) / 6.0f;
		  r = r + (k1r + 2.0f * k2r + 2.0f * k3r + k4r) / 6.0f;
		  _body[f].position() = r;
		  _body[f].velocity() = v;
	  }
  }
  Vector3f System::grav(const Vector3f x, size_t i){
	  Vector3f forces{ 0.0f, 0.0f, 0.0f };
	  for (size_t j = 0; j < _nBodies; ++j){
		  if (i != j){
			  Vector3f r = _body[j].position() - x;
			  float distance = r.norm() + _softFactor;
			  float invDist = 1.0f / distance;
			  float invDistCubed = cube(invDist);
			  forces = forces + NEWTON_G * _body[j].mass() * invDistCubed * r;
		  }
	  }
	  return forces;
  }
  void System::update(float dt) {
	  computeGravitation();
	  integrateSystem(dt);
  }

  void System::readState( std::istream &input ) {
    input >> _nBodies;
    if( _nBodies > MAX_BODIES_RECOMMENDED ) {
      throw std::runtime_error( "Too many input bodies" );
    }
    _body = new Body[_nBodies];
    for( size_t i = 0; i < _nBodies; ++i ) {
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
