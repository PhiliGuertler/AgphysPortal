#pragma once

#include "particleSystem/particleSystem.h"

class FluidCreator {
	public:
		/**
		 *	constructor
		 */
		FluidCreator(ParticleSystem& particleSystem);
		/**
		 *	default destructor
		 */
		virtual ~FluidCreator() = default;

		/**
		 *	creates numParticles many fluid particles in a cuboid layout with
		 *	given dimensions in x- and z-direction and one corner at corner.
		 */
		FluidData create(int numParticles, int x, int z, vec3 corner);
		/**
		 *	creates numParticles many fluid particles in the center of ray which
		 *	are flying in ray direction with the speed 'strength'
		 */
		FluidData create(const Saiga::Ray& ray, float strength, int numParticles);

		FluidData create(const Saiga::Ray& ray, float strength, int width, int height);
	private:
		/**
		 *	initializes the host particles' position, radius, color and mass
		 */
		void initParticles(HostParticles& particles, int x, int z, vec3 corner);

	private:
		ParticleSystem& m_particleSystem;
};
