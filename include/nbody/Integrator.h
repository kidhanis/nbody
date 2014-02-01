#include <nbody/Vector3.h>
#include <nbody/Body.h>

namespace nbody{

	class Integrator{
	public:
		void RKIntegration(Body & _nBodies, float dt, size_t nBodies);
		Vector3f grav(const Vector3f x, size_t i);
	};



} //namespace nbody

#endif // _NBODY_INTEGRATOR_H