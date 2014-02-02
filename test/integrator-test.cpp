#include <nbody/Vector3.h>
#include <nbody/Integrator.h>
#include <nbody/Body.h>
#include <gtest/gtest.h>


TEST( integrator-test, RKIntegration_singleBody){

    	float softFactor = 1e-9f;
    	float dampingFactor = 1.0f;
	size_t nBodies = 1;
	
	Body * bodies = new Body[nBodies];
	Vector3f positionA = new Vector3f{0,0,0};
	Vector3f velocityA = new Vector3f{1,0,0};
	Vector3f forceA = new Vector3f{0,0,0};
	float massA = 10f;
	bodies[0] = new Body{positionA, velocityA, forceA, massA};
	
	auto integrator = Integrator(bodies, nBodies, softFactor, dampingFactor);
	auto integrator -> integrateSystem(dt, nbody::RUNGE_KUTTA);

	ASSERT_FLOAT_EQ(bodies[0].position().x(), 1f);
	ASSERT_FLOAT_EQ(bodies[0].position().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].position().z(), 0f);

	ASSERT_FLOAT_EQ(bodies[0].velocity().x(), 1f);
	ASSERT_FLOAT_EQ(bodies[0].velocity().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].velocity().z(), 0f);


}


TEST( integrator-test, simpleIntegration_singleBody){

    	float softFactor = 1e-9f;
    	float dampingFactor = 1.0f;
	size_t nBodies = 1;
	
	Body * bodies = new Body[nBodies];
	Vector3f positionA = new Vector3f{0,0,0};
	Vector3f velocityA = new Vector3f{1,0,0};
	Vector3f forceA = new Vector3f{0,0,0};
	float massA = 10f;
	bodies[0] = new Body{positionA, velocityA, forceA, massA};



	auto integrator = Integrator(bodies, nBodies, softFactor, dampingFactor);
	auto integrator -> integrateSystem(dt, nbody::SIMPLE);

	ASSERT_FLOAT_EQ(bodies[0].position().x(), 1f);
	ASSERT_FLOAT_EQ(bodies[0].position().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].position().z(), 0f);

	ASSERT_FLOAT_EQ(bodies[0].velocity().x(), 1f);
	ASSERT_FLOAT_EQ(bodies[0].velocity().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].velocity().z(), 0f);


}



TEST( integrator-test, RKIntegration_twoBodies){
    	
	float softFactor = 1e-9f;
    	float dampingFactor = 1.0f;
	size_t nBodies = 2;
	
	Body * bodies = new Body[nBodies];
	Vector3f positionA = new Vector3f{0,0,0};
	Vector3f velocityA = new Vector3f{0,0,0};
	Vector3f forceA = new Vector3f{0,0,0};
	float massA = 10f;
	bodies[0] = new Body{positionA, velocityA, forceA, massA};

	Vector3f positionB = new Vector3f{10,0,0};
	Vector3f velocityB = new Vector3f{0,0,0};
	Vector3f forceB = new Vector3f{0,0,0};
	float massB = 10f;
	bodies[1] = new Body{positionB, velocityB, forceB, massB};




	auto integrator = Integrator(bodies, nBodies, softFactor, dampingFactor);
	auto integrator -> integrateSystem(dt, nbody::RUNGE_KUTTA);

	ASSERT_FLOAT_EQ(bodies[0].position().x(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].position().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].position().z(), 0f);

	ASSERT_FLOAT_EQ(bodies[0].velocity().x(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].velocity().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].velocity().z(), 0f);


	ASSERT_FLOAT_EQ(bodies[1].position().x(), 0f);
	ASSERT_FLOAT_EQ(bodies[1].position().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[1].position().z(), 0f);

	ASSERT_FLOAT_EQ(bodies[1].velocity().x(), 0f);
	ASSERT_FLOAT_EQ(bodies[1].velocity().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[1].velocity().z(), 0f);



}



TEST( integrator-test, simpleIntegration_twoBodies){

    	float softFactor = 1e-9f;
    	float dampingFactor = 1.0f;
	size_t nBodies = 2;
	

	Body * bodies = new Body[nBodies];
	Vector3f positionA = new Vector3f{0,0,0};
	Vector3f velocityA = new Vector3f{0,0,0};
	Vector3f forceA = new Vector3f{0,0,0};
	float massA = 10f;
	bodies[0] = new Body{positionA, velocityA, forceA, massA};

	Vector3f positionB = new Vector3f{10,0,0};
	Vector3f velocityB = new Vector3f{0,0,0};
	Vector3f forceB = new Vector3f{0,0,0};
	float massB = 10f;
	bodies[1] = new Body{positionB, velocityB, forceB, massB};




	auto integrator = Integrator(bodies, nBodies, softFactor, dampingFactor);
	auto integrator -> integrateSystem(dt, nbody::SIMPLE);

	ASSERT_FLOAT_EQ(bodies[0].position().x(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].position().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].position().z(), 0f);

	ASSERT_FLOAT_EQ(bodies[0].velocity().x(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].velocity().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[0].velocity().z(), 0f);


	ASSERT_FLOAT_EQ(bodies[1].position().x(), 0f);
	ASSERT_FLOAT_EQ(bodies[1].position().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[1].position().z(), 0f);

	ASSERT_FLOAT_EQ(bodies[1].velocity().x(), 0f);
	ASSERT_FLOAT_EQ(bodies[1].velocity().y(), 0f);
	ASSERT_FLOAT_EQ(bodies[1].velocity().z(), 0f);



}

