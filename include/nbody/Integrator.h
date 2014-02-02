#ifndef _NBODY_INTEGRATOR_H
#define _NBODY_INTEGRATOR_H

#include <nbody/Vector3.h>
#include <nbody/Body.h>
#include <iostream>

namespace nbody{
	enum Mode{ RUNGE_KUTTA, BASIC };
	class Integrator{
		Body* _body;
		size_t _nBodies;
		float _softFactor;
		float _dampingFactor;
		float* _all;
		int _steps;
		void RKIntegration(float dt, int evo, int turn);
		void basicIntegration(float dt, int evo, int turn);
		Integrator(const Integrator &integ) = delete;
		Integrator& operator=(const Integrator &integ) = delete;
	public:
		Integrator(Body* & body, size_t nBodies, float softFactor, float dampingFactor, float*all, int steps) : _body{ body }, 
			_nBodies{ nBodies }, _softFactor{ softFactor }, _dampingFactor{ dampingFactor }, _all{ all }, _steps{ steps }{
		}
		~Integrator() { delete[] _body; delete[] _all; }
		void integrateSystem(float dt, Mode mode, int evo, int turn);
		Vector3f grav(const Vector3f x, size_t i);
	};
} //namespace nbody

#endif // _NBODY_INTEGRATOR_H
