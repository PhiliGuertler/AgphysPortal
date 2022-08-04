#include "fluidCreator.h"

#include "rigidBodyManager.h"

#include "../imguiOptions.h"

// ######################################################################### //
// ### FluidCreator ######################################################## //
// ######################################################################### //

FluidCreator::FluidCreator(ParticleSystem& particleSystem)
	: m_particleSystem(particleSystem)
{
	// empty
}

FluidData FluidCreator::create(int numParticles, int x, int z, vec3 corner) {
	HostParticles particles(numParticles);
	initParticles(particles, x, z, corner);
	HostFluids fluids(1, numParticles);

	// init fluid per-particle data
#pragma omp parallel for
	for(int i = 0; i < numParticles; ++i) {
		fluids.d_lambdas[i].isFluid = 1;
	}
	
	return { particles, fluids };
}
		
FluidData FluidCreator::create(const Saiga::Ray& ray, float strength, int numParticles) {
	ImGuiOptions& options = ImGuiOptions::get();

	HostParticles particles(numParticles);
	HostFluids fluids(1, numParticles);

	// init fluid per-particle data
#pragma omp parallel for
	for(int i = 0; i < numParticles; ++i) {
		fluids.d_lambdas[i].isFluid = 1;
		particles.d_positionRadius[i].position = ray.origin;
		particles.d_positionRadius[i].radius = options.radiusReset;
		particles.d_guessedPosition[i] = particles.d_positionRadius[i];
		particles.d_color[i].color = vec4(0.06f, 0.18f, 0.95f, 1.f);
		particles.d_momentumMass[i].massinv = 1.f;
		particles.d_momentumMass[i].momentumVelocity = ray.direction * strength;
	}
	
	return { particles, fluids };
}

FluidData FluidCreator::create(const Saiga::Ray& ray, float strength, int width, int height) {
	const float EPSTEIN = 0.001f;
	ImGuiOptions& options = ImGuiOptions::get();

	int numParticles = width * height * width;
	HostParticles particles(numParticles);
	HostFluids fluids(1, numParticles);

	float distance = options.radiusReset * 2.f;

	quat rayOrientation = lookAt(-ray.direction, vec3(0,1,0));

	vec3 off = vec3(width, 0, height) * 0.5f * distance;

	// init fluid per-particle data
#pragma omp parallel for
	for(int i = 0; i < numParticles; ++i) {
		int particleY = (i / (width * height));
		float offset = ((particleY % 2 == 0) ? 1.f : -1.f) * EPSTEIN;
		//
		// position in cube formation
		vec3 position = vec3(0.f,0.f,0.f);
		position[0] = ((i % width) * distance) + offset;
		position[1] = (((i / width) % height) * distance) + offset;
		position[2] = ((i / (width * height)) * distance);
		position -= off;
		position = rayOrientation * position;
		position += ray.origin;
		
		fluids.d_lambdas[i].isFluid = 1;
		particles.d_positionRadius[i].position = position;
		particles.d_positionRadius[i].radius = options.radiusReset;
		particles.d_guessedPosition[i] = particles.d_positionRadius[i];
		particles.d_color[i].color = vec4(0.06f, 0.18f, 0.95f, 1.f);
		particles.d_momentumMass[i].massinv = 1.f;
		particles.d_momentumMass[i].momentumVelocity = ray.direction * strength;
	}
	
	return { particles, fluids };
}


void FluidCreator::initParticles(HostParticles& particles, int x, int z, vec3 corner) {
	const float EPSTEIN = 0.001f;
	ImGuiOptions& options = ImGuiOptions::get();
	float distance = options.radiusReset * 2.f;

#pragma omp parallel for
	for(int i = 0; i < particles.d_positionRadius.size(); ++i) {
		int particleY = (i / (x * z));
		float offset = ((particleY % 2 == 0) ? 1.f : -1.f) * EPSTEIN;

		// position in cube formation
		vec3 position = vec3(0.f,0.f,0.f);
		position[0] = corner[0] + ((i % x) * distance) + offset;
		position[1] = corner[1] + ((i / (x * z)) * distance);
		position[2] = corner[2] + (((i / x) % z) * distance) + offset;
		
		particles.d_positionRadius[i].position = position;
		// default radius (set in the ui)
		particles.d_positionRadius[i].radius = options.radiusReset;
		// guessed position == original position
		particles.d_guessedPosition[i].position = particles.d_positionRadius[i].position;
		// default radius yet again
		particles.d_guessedPosition[i].radius = particles.d_positionRadius[i].radius;
		// default color
		particles.d_color[i].color = vec4(0.06f, 0.18f, 0.95f, 1.f);
		// default mass
		particles.d_momentumMass[i].massinv = 1.f;
	}
}
