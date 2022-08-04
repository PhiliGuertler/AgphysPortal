#pragma once

#include "particleSystem/particleSystem.h"

class ClothCreator {
	public:
		/**
		 *	constructor
		 */
		ClothCreator(ParticleSystem& particleSystem, int N, int M);
		/**
		 *	default destructor
		 */
		virtual ~ClothCreator() = default;

		/**
		 *	creates the data nescessary to create a cloth with its center at 'position', an 'orientation' and fixed edges with infinite mass or not
		 */
		ClothData create(vec3 center, quat orientation, bool fixedEdges = true);
	private:
		/**
		 *	computes distance constraints for the cloth and fills the input with them
		 *	the input vector must be resized correctly, with one entry for each particle of the cloth
		 */
		void fillWithDistanceConstraints(std::vector<ClothNeighbors>& filler);
		/**
		 *	computes the bending constraints for the cloth and fills the input vector with them
		 *	the input vector must be sized correctly ((N-1)*(M-1))
		 */
		void fillWithBendingConstraints(std::vector<ClothBendingConstraint>& constraints);
		/**
		 *	sets the values for each particle of the cloth to the initial state
		 */
		void initParticles(HostParticles& particles, vec3 position, quat orientation, bool fixedEdges);

	private:
		ParticleSystem& m_particleSystem;
		int m_N;
		int m_M;
};
