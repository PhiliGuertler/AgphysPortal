#include "Player.h"
#include "cuda/particleSystem/particleSystem.h"

#include "cuda/particle.h"

#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"

#include "imguiOptions.h"

__global__ void acceleratePlayer(ArrayView<ParticleMomentumMass> moms
		, ArrayView<RigidBodyParticleData> particleData
		, float maxSpeed
		, vec3 direction
		, float force) {
	Saiga::CUDA::ThreadInfo<> ti;

	RigidBodyParticleData data;
	Saiga::CUDA::vectorCopy(particleData.data(), &data);

	if(ti.thread_id >= data.numParticles) return;

	ParticleMomentumMass mom;
	Saiga::CUDA::vectorCopy(moms.data()+ti.thread_id, &mom);

	mom.momentumVelocity += direction * force;
	mom.massinv = 0.01f;
	float length = mom.momentumVelocity.norm();
	if(length > maxSpeed) {
		mom.momentumVelocity *= maxSpeed/length;
	}

	Saiga::CUDA::vectorCopy(&mom, moms.data()+ti.thread_id);
}

__global__ void elevatePlayer(ArrayView<ParticlePositionRadius> positions
		, ArrayView<ParticlePositionRadius> guessedPositions
		, ArrayView<RigidBodyParticleData> particleData
		, float height) {
	Saiga::CUDA::ThreadInfo<> ti;

	RigidBodyParticleData data;
	Saiga::CUDA::vectorCopy(particleData.data(), &data);

	if(ti.thread_id >= data.numParticles) return;

	ParticlePositionRadius pos;
	Saiga::CUDA::vectorCopy(positions.data()+ti.thread_id, &pos);

	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(guessedPositions.data()+ti.thread_id, &guessedPos);

	pos.position.y() += height;
	guessedPos.position.y() += height;

	Saiga::CUDA::vectorCopy(&pos, positions.data()+ti.thread_id);
	Saiga::CUDA::vectorCopy(&guessedPos, guessedPositions.data()+ti.thread_id);
}

namespace Controls {

	RigidBodyOrientation Player::copyOrientation() {
		cudaDeviceSynchronize();
		RigidBodyOrientation currentOrientaion = m_particleSystem.m_rigidBodies.d_rigidBodiesOrientation[0];
		cudaDeviceSynchronize();
		return currentOrientaion;
	}

	void Player::movePlayerParticles(vec3 direction, float force) {
		int numRigidBodyParticles = m_particleSystem.getNumParticles(ParticleType::RigidBodyParticle);
		if(numRigidBodyParticles == 0) {
			return;
		}

		ImGuiOptions& options = ImGuiOptions::get();

		// move the player a little bit in positive y direction
		elevatePlayer<<<THREAD_BLOCK(numRigidBodyParticles, BLOCK_SIZE)>>>(m_particleSystem.m_particles.d_positionRadius
				, m_particleSystem.m_particles.d_guessedPosition
				, m_particleSystem.m_rigidBodies.d_rigidBodiesParticles
				, m_hoverHeight);

		acceleratePlayer<<<THREAD_BLOCK(numRigidBodyParticles, BLOCK_SIZE)>>>(m_particleSystem.m_particles.d_momentumMass, m_particleSystem.m_rigidBodies.d_rigidBodiesParticles, m_maxSpeed, direction, force);
		CUDA_SYNC_CHECK_ERROR();
	}
}
