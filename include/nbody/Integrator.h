#ifndef _NBODY_INTEGRATOR_H
#define _NBODY_INTEGRATOR_H

#include <nbody/Vector3.h>
#include <nbody/Body.h>

namespace nbody{

	class Integrator{
		Body* _body;
		size_t _nBodies;
		float _softFactor;
		Integrator(const Integrator &integ) = delete;
		Integrator& operator=(const Integrator &integ) = delete;

	public:
		Integrator(Body* & body, size_t nBodies, float softFactor) : _body{ body }, _nBodies{ nBodies }, _softFactor{ softFactor }{}
		~Integrator() {}
		void RKIntegration( float dt);
		Vector3f grav(const Vector3f x, size_t i);
	};



} //namespace nbody

#endif // _NBODY_INTEGRATOR_H
