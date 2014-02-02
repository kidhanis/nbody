#ifndef _NBODY_INTEGRATOR_H
#define _NBODY_INTEGRATOR_H

#include <nbody/Vector3.h>
#include <nbody/Body.h>

namespace nbody{
	enum Mode{ RUNGE_KUTTA, BASIC };
	class Integrator{
		Body* _body;
		size_t _nBodies;
		float _softFactor;
		float _dampingFactor;
		void RKIntegration(float dt);
		void basicIntegration(float dt);
		Integrator(const Integrator &integ) = delete;
		Integrator& operator=(const Integrator &integ) = delete;
	public:
		Integrator(Body* & body, size_t nBodies, float softFactor, float dampingFactor) : _body{ body }, 
			_nBodies{ nBodies }, _softFactor{ softFactor }, _dampingFactor{dampingFactor}{}
		~Integrator() { delete[] _body; }
		void integrateSystem(float dt, Mode mode);
		Vector3f grav(const Vector3f x, size_t i);
	};
} //namespace nbody

#endif // _NBODY_INTEGRATOR_H
