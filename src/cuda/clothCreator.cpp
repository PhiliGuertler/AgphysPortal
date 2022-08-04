#include "clothCreator.h"

#include "../imguiOptions.h"

// ######################################################################### //
// ### ClothCreator ######################################################## //
// ######################################################################### //

ClothCreator::ClothCreator(ParticleSystem& particleSystem, int N, int M)
	: m_particleSystem(particleSystem)
	, m_N(N)
	, m_M(M)
{
	// empty
}

ClothData ClothCreator::create(vec3 position, quat orientation, bool fixedEdges) {
	int numParticles = m_N * m_M;
	HostParticles particles(numParticles);
	initParticles(particles, position, orientation, fixedEdges);
	HostCloths cloths(numParticles, (m_N-1) * (m_M-1));

	// create collision constraints
	fillWithDistanceConstraints(cloths.d_clothNeighbors);

	// create bending constraints
	fillWithBendingConstraints(cloths.d_clothBendingConstraints);

	return {particles, cloths};
}

void ClothCreator::fillWithDistanceConstraints(std::vector<ClothNeighbors>& filler) {
	// filler is already as big as it should be
	size_t insertionIndex = m_particleSystem.particlesEnd(ParticleType::ClothParticle);

#pragma omp parallel for
	for(int x = 0; x < m_N; ++x) {
		for(int y = 0; y < m_M; ++y) {
			int currentIndex = x*m_M + y;
			ClothNeighbors neighbors;
			if(x != m_N-1) {
				int rightNeighborIndex = currentIndex + m_M;
				neighbors.rightNeighborCollision = {(int)(insertionIndex+currentIndex), (int)(insertionIndex+rightNeighborIndex)};
			}

			if(y != m_M-1) {
				int topNeighborIndex = currentIndex+1;
				neighbors.topNeighborCollision = {(int)(insertionIndex+currentIndex), (int)(insertionIndex+topNeighborIndex)};
			}

			filler[x*m_M + y] = neighbors;
		}
	}
}

// helper function to linearize a 2D-index pair
inline int linearizeIndex(int x, int y, int maxX, int maxY) {
	return x*maxY + y;
}

void ClothCreator::fillWithBendingConstraints(std::vector<ClothBendingConstraint>& constraints) {
	constraints.resize((m_N-1)*(m_M-1));
	size_t insertionIndex = m_particleSystem.particlesEnd(ParticleType::ClothParticle);

#pragma omp parallel for
	for(int x = 0; x < m_N-1; ++x) {
		for(int y = 0; y < m_M-1; ++y) {
			ClothBendingConstraint constraint;
			constraint.particleID1 = linearizeIndex(x+1, y, m_N, m_M) + insertionIndex;
			constraint.particleID2 = linearizeIndex(x, y+1, m_N, m_M) + insertionIndex;
			constraint.particleID3 = linearizeIndex(x, y, m_N, m_M) + insertionIndex;
			constraint.particleID4 = linearizeIndex(x+1, y+1, m_N, m_M) + insertionIndex;

			int constraintIndex = linearizeIndex(x, y, m_N-1, m_M-1);
			constraints[constraintIndex] = constraint;
		}
	}

	if(ImGuiOptions::get().enablePrints) {
		for(int i = 0; i < constraints.size(); ++i) {
			ClothBendingConstraint constraint = constraints[i];
			std::cout << "ParticleID1: " << constraint.particleID1;
			std::cout << ", ParticleID2: " << constraint.particleID2;
			std::cout << ", ParticleID3: " << constraint.particleID3;
			std::cout << ", ParticleID4: " << constraint.particleID4 << std::endl;
		}
		std::cout << constraints.size() << std::endl;
	}
}

void ClothCreator::initParticles(HostParticles& particles, vec3 position, quat orientation, bool fixedEdges) {
	ImGuiOptions& options = ImGuiOptions::get();
	std::vector<vec3> positions(particles.d_positionRadius.size());
	vec3 center = {0.f,0.f,0.f};
	for(int x = 0; x < m_N; ++x) {
		for(int y = 0; y < m_M; ++y) {
			int currentIndex = x*m_M + y;
			// TODO: make the distance of the cloth particles variable in the GUI
			vec3 currentPosition = vec3((float)x, (float)y, 0.f) * (options.radiusReset * 2.f);
			positions[currentIndex] = currentPosition;
			center += currentPosition;
		}
	}
	center /= positions.size();

	// Initialize particles
#pragma omp parallel for
	for(int i = 0; i < particles.d_positionRadius.size(); ++i) {
		// random position
		vec3 currentPosition = positions[i] - center;
		currentPosition = orientation * currentPosition;
		particles.d_positionRadius[i].position = currentPosition + position;
		// default radius (set in the ui)
		particles.d_positionRadius[i].radius = options.radiusReset;
		// guessed position == original position
		particles.d_guessedPosition[i].position = particles.d_positionRadius[i].position;
		// default radius yet again
		particles.d_guessedPosition[i].radius = particles.d_positionRadius[i].radius;
		// default color
		particles.d_color[i].color = vec4(0.7f, 0.8f, 0.2f, 1.f);
		// default mass
		if(fixedEdges && (i == 0 || i == m_M-1 || i == particles.d_momentumMass.size()-1 || i == particles.d_momentumMass.size()-m_M)) {
			// the corner particles are infinitely heavy
			particles.d_momentumMass[i].massinv = 0.f;
		} else {
			particles.d_momentumMass[i].massinv = options.massinv;
		}

	}
}
