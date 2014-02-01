#include <nbody/Integrator.h>
#include <nbody/Vector3.h>
#include <nbody/Body.h>
#include <nbody/constants.h>


namespace nbody{
	
	void Integrator::RKIntegration(float dt){
		for(size_t f = 0; f < _nBodies; f++){
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







} // namespace nbody
