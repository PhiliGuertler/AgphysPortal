#include "particleSystem_stabilization.h"

#include "particleSystem_collisionResolver.h"

#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"

// ######################################################################### //
// ### CUDA Kernels ######################################################## //
// ######################################################################### //

__global__ void stabilizeMe(ArrayView<ParticlePositionRadius> absolutePositions, ArrayView<ParticlePositionRadius> particlesGuessed, ArrayView<ParticleDeltaValues> particleDeltas, float dt) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlesGuessed.size()) return;

	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+ti.thread_id, &guessedPos);

	ParticlePositionRadius absolutePos;
	Saiga::CUDA::vectorCopy(absolutePositions.data()+ti.thread_id, &absolutePos);

	ParticleDeltaValues delta;
	Saiga::CUDA::vectorCopy(particleDeltas.data()+ti.thread_id, &delta);
	
	// update the guessed position
	guessedPos.position += delta.delta;
	absolutePos.position += delta.delta;
	delta.delta = {0.f,0.f,0.f};

	Saiga::CUDA::vectorCopy(&absolutePos, absolutePositions.data()+ti.thread_id);
	Saiga::CUDA::vectorCopy(&guessedPos, particlesGuessed.data()+ti.thread_id);
	Saiga::CUDA::vectorCopy(&delta, particleDeltas.data()+ti.thread_id);
}


// ######################################################################### //
// ### C++ ParticleSystem ################################################## //
// ######################################################################### //

void ParticleSystem::stabilizationStep(float dt, int planeCollisionLength, int tryeckCollisionLength, int particleCollisionLength) {
	ImGuiOptions& options = ImGuiOptions::get();
	for(int i = 0; i < options.numStabilizationSteps; ++i) {
		// resolve particle-plane collisions
		if(planeCollisionLength > 0 && options.resolvePlanes) {
			resolveCollisionConstraintsPositionBasedWithFriction<<<THREAD_BLOCK(planeCollisionLength, BLOCK_SIZE)>>>(m_particles.d_positionRadius
					, m_particles.d_positionRadius
					, m_particles.d_momentumMass
					, m_particles.d_deltaValues
					, d_planes
					, d_activePlaneCollisions
					, m_particles.d_portalFlags
					, m_rigidBodies.d_rigidBodiesParticleOffsets
					, m_rigidBodies.d_rigidBodyPortalFlags
					, *m_bluePortal
					, *m_orangePortal
					, particlesBegin(ParticleType::FluidParticle)
					, particlesEnd(ParticleType::FluidParticle)
					, dt
					, planeCollisionLength
					, options.staticFriction
					, options.kineticFriction
					, options.enableFriction);
		}

		// resolve particle-tryeck collisions
		if(tryeckCollisionLength > 0 && options.resolvePlanes) {
			resolveTriangleCollisions<<<THREAD_BLOCK(tryeckCollisionLength, BLOCK_SIZE)>>>(m_particles.d_positionRadius
					, m_particles.d_momentumMass
					, m_particles.d_deltaValues
					, m_particles.d_color
					, d_activeTryeckCollisions
					, d_tryecke				
					, m_particles.d_positionRadius
					, m_particles.d_portalFlags
					, m_rigidBodies.d_rigidBodiesParticleOffsets
					, m_rigidBodies.d_rigidBodyPortalFlags
					, *m_bluePortal 
					, *m_orangePortal
					, particlesBegin(ParticleType::FluidParticle)
					, particlesEnd(ParticleType::FluidParticle)
					, dt
					, options.relaxation
					, tryeckCollisionLength
					, options.staticFriction
					, options.kineticFriction
					, options.enableFriction);
		}

		// resolve particle-particle collisions
		if(particleCollisionLength > 0 && options.resolveParticles) {
			if(options.resolveWithSDF) {
				resolveCollisionConstraintsPositionBasedWithFriction<<<THREAD_BLOCK(particleCollisionLength, BLOCK_SIZE)>>>(m_particles.d_positionRadius
						, m_particles.d_momentumMass
						, m_particles.d_deltaValues
						, d_activeParticleCollisions
						, m_rigidBodies.d_rigidBodiesParticles
						, m_rigidBodies.d_rigidBodiesOrientation
						, m_rigidBodies.d_rigidBodiesParticleNormals
						, m_particles.d_positionRadius
						, m_particles.d_portalFlags
						, m_rigidBodies.d_rigidBodiesParticleOffsets
						, m_rigidBodies.d_rigidBodyPortalFlags
						, *m_bluePortal
						, *m_orangePortal
						, particlesBegin(ParticleType::FluidParticle)
						, particlesEnd(ParticleType::FluidParticle)
						, dt
						, options.relaxation
						, particleCollisionLength
						, options.staticFriction
						, options.kineticFriction
						, options.enableFriction);

			} else {
				resolveCollisionConstraintsPositionBased<<<THREAD_BLOCK(particleCollisionLength, BLOCK_SIZE)>>>(m_particles.d_guessedPosition
						, m_particles.d_momentumMass
						, m_particles.d_deltaValues
						, d_activeParticleCollisions
						, m_particles.d_portalFlags
						, m_rigidBodies.d_rigidBodiesParticleOffsets
						, m_rigidBodies.d_rigidBodyPortalFlags
						, *m_bluePortal
						, *m_orangePortal
						, dt
						, options.relaxation
						, particleCollisionLength);
			}
		}

		// update both the guessed position and the absolute position
		stabilizeMe<<<THREAD_BLOCK(m_particles.d_positionRadius.size(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_guessedPosition, m_particles.d_deltaValues, dt);
	}
}
