#ifndef _NBODY_SYSTEM_H
#define _NBODY_SYSTEM_H

#include <nbody/Vector3.h>
#include <nbody/Body.h>
#include <nbody/Integrator.h>

#include <iosfwd>
#include <string>

namespace nbody {

  class System {
    Integrator *_integrator = nullptr;
    size_t _nBodies;
    Body *_body;
    float _softFactor = 1e-9f;
    float _dampingFactor = 1.0f;
    System() = delete;
    System( const System &sys ) = delete;
    System& operator=( const System &sys ) = delete;
  public:
    System( size_t N ) :  _nBodies{N}, _body{ new Body[N] } { initRandomState(); }
    System( std::istream &input ) :  _nBodies{}, _body{nullptr} { readState( input ); }
    System( std::string filename ) :  _nBodies{}, _body{nullptr} { readState( filename ); }
	~System() { delete[] _body; delete _integrator; }
    void interactBodies( size_t i, size_t j, float softFactor, Vector3f &acc ) const;
    void computeGravitation();
    void readState( std::istream &input );
    void readState( std::string filename );
    void writeState( std::ostream &output ) const;
    void writeState( std::string filename ) const;
    void initRandomState();
    void update( int nSteps, float dt, float*all , int evo, int step);
    void setSoftening( float soft ) { _softFactor = soft; }
    void setDamping( float damp ) { _dampingFactor = damp; }
  };

} // namespace nbody

#endif // _NBODY_SYSTEM_H
