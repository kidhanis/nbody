#include <nbody/Integrator.h>
#include <nbody/Vector3.h>
#include <nbody/Body.h>
#include <nbody/constants.h>
#include <iostream>


namespace nbody{
	
	void Integrator::basicIntegration(float dt,int evo, int turn) {

		Vector3f r, v, a;
		for (size_t i = 0; i < _nBodies; ++i) {
			r = _body[i].position();
			v = _body[i].velocity();
			a = _body[i].force();

			v = v + (a * dt);
			v = v * _dampingFactor;
			r = r + v * dt;

			_body[i].position() = r;
			_body[i].velocity() = v;
			_all[_nBodies * 4 * _steps * evo + _nBodies * 4 * turn + 4 * i] = r.x();
			_all[_nBodies * 4 * _steps * evo + _nBodies * 4 * turn + 4 * i + 1] = r.y();
			_all[_nBodies * 4 * _steps*evo + _nBodies * 4 * turn + 4 * i + 2] = r.z();
			_all[_nBodies * 4 * _steps* evo + _nBodies * 4 * turn + 4 * i + 3] = 1.0f;
		}
	}

	void Integrator::RKIntegration(float dt, int evo, int turn){

		for(size_t i = 0; i < _nBodies; i++){
			Vector3f k1v, k2v, k3v, k4v, k1r, k2r, k3r, k4r, v, r;
			v = _body[i].velocity();
			r = _body[i].position();
			//std::cout << r.x() << "what is x position  \n";
			//std::cout << v.y() << "what is y velocity  \n";


		  	k1r = dt*v;
			k1v = dt*grav(r, i);
		  	k2r = dt*(v + k1v / 2.0f);
		  	k2v = dt*(grav(r + k1r / 2.0f, i));
		  	k3r = dt*(v + k2v / 2.0f);
		  	k3v = dt*(grav(r + k2r / 2.0f, i));
		  	k4r = dt*(v + k3v);
		  	k4v = dt*(grav(r + k3r, i));
		  	v = v + (k1v + 2.0f * k2v + 2.0f * k3v + k4v) / 6.0f;
		  	r = r + (k1r + 2.0f * k2r + 2.0f * k3r + k4r) / 6.0f;
		  	_body[i].position() = r;
		  	_body[i].velocity() = v;

			_all[_nBodies * 4 * _steps * evo + _nBodies * 4 * turn + 4 * i] = r.x();
			_all[_nBodies* 4 * _steps * evo + _nBodies * 4 * turn + 4 * i + 1] = r.y();
			_all[_nBodies * 4 * _steps * evo + _nBodies * 4 * turn + 4 * i + 2] = r.z();
			_all[_nBodies * 4 * _steps * evo + _nBodies * 4 * turn + 4 * i + 3] = 1.0f;

		}
	}

	Vector3f Integrator::grav(const Vector3f x, size_t i){
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

	void Integrator::integrateSystem(float dt, Mode mode, int evo, int turn){

		if (mode == RUNGE_KUTTA){
			Integrator::RKIntegration(dt, evo, turn);
		}
		else{
			Integrator::basicIntegration(dt, evo, turn);
		}
	}
} // namespace nbody
