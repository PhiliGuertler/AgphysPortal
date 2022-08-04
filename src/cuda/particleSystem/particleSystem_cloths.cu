#include "particleSystem_cloths.h"

#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"


// ######################################################################### //
// ### CUDA Kernels ######################################################## //
// ######################################################################### //

__device__ inline void clothHelper(ArrayView<ParticlePositionRadius> particlesGuessed
					, ArrayView<ParticleMomentumMass> moms
					, ArrayView<ParticleDeltaValues> deltas
					, float dt
					, float relaxationFactor
					, float spacing
					, int particleID1
					, int particleID2) {
	// get the two particles
	ParticlePositionRadius guessedPosition1;
	ParticlePositionRadius guessedPosition2;

	Saiga::CUDA::vectorCopy(particlesGuessed.data()+particleID1, &guessedPosition1);
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+particleID2, &guessedPosition2);

	// compute their overlapping distance
	// compute the normals for each particle
	float distance = (guessedPosition1.position - guessedPosition2.position).norm() - spacing;

	vec3 normal = (guessedPosition1.position - guessedPosition2.position).normalized();

	ParticleMomentumMass mass1;
	ParticleMomentumMass mass2;
	Saiga::CUDA::vectorCopy(moms.data()+particleID1, &mass1);
	Saiga::CUDA::vectorCopy(moms.data()+particleID2, &mass2);
	
	ParticleDeltaValues& delta1 = deltas[particleID1];
	ParticleDeltaValues& delta2 = deltas[particleID2];

	// compute amount vector of displacement to resolve the collision
	vec3 positionChange = normal * distance * relaxationFactor;
	vec3 positionChange1 = -mass1.massinv / (mass1.massinv + mass2.massinv) * positionChange;
	vec3 positionChange2 = mass2.massinv / (mass1.massinv + mass2.massinv) * positionChange;

	atomicAdd(&delta1.delta[0], positionChange1[0]);
	atomicAdd(&delta1.delta[1], positionChange1[1]);
	atomicAdd(&delta1.delta[2], positionChange1[2]);
	
	atomicAdd(&delta2.delta[0], positionChange2[0]);
	atomicAdd(&delta2.delta[1], positionChange2[1]);
	atomicAdd(&delta2.delta[2], positionChange2[2]);

}

__global__ void resolveClothCollisionConstraints(ArrayView<ParticlePositionRadius> particlesGuessed
												, ArrayView<ParticleMomentumMass> moms
												, ArrayView<ParticleDeltaValues> deltas
												, ArrayView<ClothNeighbors> clothCollisionData
												, float dt
												, float relaxationFactor
												, float spacing
												, int clothParticlesBegin
												, int clothParticlesEnd) {
	Saiga::CUDA::ThreadInfo<> ti;
	int particleID = ti.thread_id + clothParticlesBegin;
	if(particleID >= clothParticlesEnd) return;

	ClothNeighbors data;
	Saiga::CUDA::vectorCopy(clothCollisionData.data()+particleID, &data);

	int a = data.topNeighborCollision.particleID1;
	int b = data.topNeighborCollision.particleID2;
	if(a != -1 && b != -1) {
		clothHelper(particlesGuessed, moms, deltas, dt, relaxationFactor, spacing, a, b);
	}
	a = data.rightNeighborCollision.particleID1;
	b = data.rightNeighborCollision.particleID2;
	if(a != -1 && b != -1) {
		clothHelper(particlesGuessed, moms, deltas, dt, relaxationFactor, spacing, a, b);
	}

}

__device__ inline vec3 finalCorrection(float massinv, float d, vec3 qi, float weightedSum) {
	return -((massinv * sqrt(1.f-(d*d)) * (acos(d)-M_PI)) / weightedSum) * qi;
}

__global__ void resolveClothBendingConstraints(ArrayView<ClothBendingConstraint> bendingConstraints
												, ArrayView<ParticlePositionRadius> particlesGuessed
												, ArrayView<ParticleMomentumMass> moms
												, ArrayView<ParticleDeltaValues> deltas
												, float omega
												, float bendFactor) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= bendingConstraints.size()) return;

	const float EPSTEIN = 0.0001f;

	ClothBendingConstraint constraint;
	Saiga::CUDA::vectorCopy(bendingConstraints.data()+ti.thread_id, &constraint);

	if(constraint.particleID1 >= particlesGuessed.size() || constraint.particleID2 >= particlesGuessed.size() || constraint.particleID3 >= particlesGuessed.size() || constraint.particleID4 >= particlesGuessed.size()) {
		// FIXME: this should not happen, but moving the constraints seems breaks it
		printf("[Bending Constraint Error]: BendingConstraint: %d, %d, %d, %d", constraint.particleID1, constraint.particleID2, constraint.particleID3, constraint.particleID4);
		return;
	}

	// variable names are the same as in the paper

	ParticlePositionRadius guessedPosition1;
	ParticlePositionRadius guessedPosition2;
	ParticlePositionRadius guessedPosition3;
	ParticlePositionRadius guessedPosition4;
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+constraint.particleID1, &guessedPosition1);
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+constraint.particleID2, &guessedPosition2);
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+constraint.particleID3, &guessedPosition3);
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+constraint.particleID4, &guessedPosition4);

	ParticleMomentumMass mass1;
	ParticleMomentumMass mass2;
	ParticleMomentumMass mass3;
	ParticleMomentumMass mass4;
	Saiga::CUDA::vectorCopy(moms.data()+constraint.particleID1, &mass1);
	Saiga::CUDA::vectorCopy(moms.data()+constraint.particleID2, &mass2);
	Saiga::CUDA::vectorCopy(moms.data()+constraint.particleID3, &mass3);
	Saiga::CUDA::vectorCopy(moms.data()+constraint.particleID4, &mass4);

	ParticleDeltaValues& d1 = deltas[constraint.particleID1];
	ParticleDeltaValues& d2 = deltas[constraint.particleID2];
	ParticleDeltaValues& d3 = deltas[constraint.particleID3];
	ParticleDeltaValues& d4 = deltas[constraint.particleID4];

	vec3 p1 = guessedPosition1.position;
	vec3 p2 = guessedPosition2.position - p1;
	vec3 p3 = guessedPosition3.position - p1;
	vec3 p4 = guessedPosition4.position - p1;
	p1 = vec3(0,0,0);

	vec3 p2CrossP3 = p2.cross(p3);
	vec3 n1 = p2CrossP3.normalized();
	if(n1.norm() < EPSTEIN) return;

	vec3 p2CrossP4 = p2.cross(p4);
	vec3 n2 = p2CrossP4.normalized();
	if(n2.norm() < EPSTEIN) return;

	float d = n1.dot(n2);
	// to prevent a NaN in finalCorrection (sqrt(1-d^2)), clamp d to a bit less than abs(1)
	d = clamp(d, -1.f+EPSTEIN, 1.f-EPSTEIN);

	vec3 q3 = (p2.cross(n2) + (n1.cross(p2) * d)) / p2CrossP3.norm();
	vec3 q4 = (p2.cross(n1) + (n2.cross(p2) * d)) / p2CrossP4.norm();
	vec3 q2 = -((p3.cross(n2) + (n1.cross(p3) * d)) / p2CrossP3.norm()) - ((p4.cross(n1) + (n2.cross(p4) * d)) / p2CrossP4.norm());
	vec3 q1 = -q2 - q3 - q4;

	float weightedSum = mass1.massinv * q1.dot(q1) + mass2.massinv * q2.dot(q2) + mass3.massinv * q3.dot(q3) + mass4.massinv * q4.dot(q4);

	if(-EPSTEIN < weightedSum || weightedSum < EPSTEIN) {
		// don't divide by zero in finalCorrection
		return;
	}
	
	vec3 delta1, delta2, delta3, delta4;
	delta1 = finalCorrection(mass1.massinv, d, q1, weightedSum) * omega * bendFactor;
	delta2 = finalCorrection(mass2.massinv, d, q2, weightedSum) * omega * bendFactor;
	delta3 = finalCorrection(mass3.massinv, d, q3, weightedSum) * omega * bendFactor;
	delta4 = finalCorrection(mass4.massinv, d, q4, weightedSum) * omega * bendFactor;

	for(int i = 0; i < 3; ++i) {
		atomicAdd(&d1.delta[i], delta1[i]);
		atomicAdd(&d2.delta[i], delta2[i]);
		atomicAdd(&d3.delta[i], delta3[i]);
		atomicAdd(&d4.delta[i], delta4[i]);
	}
}


// ######################################################################### //
// ### C++ ParticleSystem ################################################## //
// ######################################################################### //

void ParticleSystem::clothStep(float dt) {
	// skip the cloth step if there are no cloth particles
	if(getNumParticles(ParticleType::ClothParticle) > 0) {
		ImGuiOptions& options = ImGuiOptions::get();
		// resolve the cloth collision constraints
		resolveClothCollisionConstraints<<<THREAD_BLOCK(getNumParticles(ParticleType::ClothParticle), BLOCK_SIZE)>>>(m_particles.d_guessedPosition
				, m_particles.d_momentumMass
				, m_particles.d_deltaValues
				, m_cloths.d_clothNeighbors
				, dt
				, options.relaxation
				, options.clothSpacing
				, particlesBegin(ParticleType::ClothParticle)
				, particlesEnd(ParticleType::ClothParticle));

		if(options.resolveBendingConstraints) {
			// resolve the bending constraints
			resolveClothBendingConstraints<<<THREAD_BLOCK(m_cloths.d_clothBendingConstraints.size(), BLOCK_SIZE)>>>(m_cloths.d_clothBendingConstraints
					, m_particles.d_guessedPosition
					, m_particles.d_momentumMass
					, m_particles.d_deltaValues
					, options.relaxation
					, options.bendFactor);
		}
	}
}
