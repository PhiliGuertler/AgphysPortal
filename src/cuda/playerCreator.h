#pragma once 

#include "particleSystem/particleSystem.h"


class Player{

public:
		/**
		 *	constructor
		 */
		Player();
		Player(ParticleSystem& particleSystem,int index);
		/**
		 *	default destructor
		 */
		virtual ~Player() = default;
		Player(const Player& x);
		//Player create(const Saiga::Ray& ray, float strength, int numParticles);
		

	protected:

		// reference to the particle system
		ParticleSystem& m_particleSystem;
	public:
		/**
		 *	initializes the host particles' position, radius, color and mass, 0 = back, 1 = left, 2 = right, 3 = front, 4 = up
		 */
		void init();
		void move(const Saiga::Ray& cameraRay, int direction);
		int bodyIndex;
		vec3 position;
			
};
