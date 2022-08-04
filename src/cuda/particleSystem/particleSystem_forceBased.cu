#include "particleSystem.h"
#include "saiga/core/util/assert.h"
#include "saiga/cuda/thread_info.h"

#include "particleSystem_collisions.inl"

#include "particleSystem_forceBased.h"

// ######################################################################### //
// ### CUDA Kernels ######################################################## //
// ######################################################################### //

// ######################################################################### //
// ### Force Computation: Force based Dynamics ############################# //

__device__ inline vec3 elasticForce(ParticleMomentumMass particle1, ParticleMomentumMass particle2, vec3 collisionNormal, float elasticCoefficient) {
	float fact = 1.f + elasticCoefficient;
	vec3 moment = (particle2.momentumVelocity * particle2.massinv - particle1.momentumVelocity * particle1.massinv);
	vec3 pe = collisionNormal * ((fact * moment.dot(collisionNormal))/ (particle1.massinv + particle2.massinv));

	return pe;
}

__device__ inline vec3 springForce(vec3 springNormal, float distance, float springCoefficient, float deltaTime) {
	// compute the actual force
	return distance * springCoefficient * springNormal * deltaTime;;
}

__device__ inline vec3 frictionForce(ParticleMomentumMass momentum1, ParticleMomentumMass momentum2, vec3 collisionNormal, float frictionCoefficient) {
	// compute tangential momentum of p1 and p2
	vec3 pt1 = momentum1.momentumVelocity - momentum1.momentumVelocity.dot(collisionNormal) * collisionNormal;
	vec3 pt2 = momentum2.momentumVelocity - momentum2.momentumVelocity.dot(collisionNormal) * collisionNormal;

	// compute relative tangential impulse
	vec3 ptr = pt1 - pt2;
	float x = ptr.norm();

	// compute quadratic function f(x)
	float f_x = (x < frictionCoefficient) ? ((-(x*x))/(2.f * frictionCoefficient) + x) : (frictionCoefficient * 0.5f);

	// compute tangent
	vec3 t = pt1.normalized();
	return t * (-f_x);
}

/**
 *	returns true if the collision will be resolved on its own.
 *	should only be called with colliding objects in force based dynamics.
 */
 __device__ inline float collisionIsAlive(ParticleMomentumMass particle1, ParticleMomentumMass particle2, vec3 collisionNormal) {
	float left = particle1.momentumVelocity.dot(collisionNormal) * particle1.massinv;
	float right = particle2.momentumVelocity.dot(collisionNormal) * particle2.massinv;
	return left < right ? 1.f : 0.f;
}

// ######################################################################### //
// ### Collision Resolver: Force based Dynamics ############################ //

__device__ inline vec3 resolveCollision(Saiga::Plane plane
									, ParticlePositionRadius particlePosition
									, ParticleMomentumMass particleMomentum
									, float dt
									, float elasticCoefficient
									, float springCoefficient
									, float frictionCoefficient) {
	// check for collision
	float collisionDistance = collideSpherePlane(particlePosition, plane);
	
	vec3 force = vec3(0,0,0);
	// check for aliveness
	if(collisionDistance < 0.f) {

		// elastic force
		ParticleMomentumMass planeMomentum = {vec3(0.f,0.f,0.f), 0.f};
		float collisionAliveFact = collisionIsAlive(particleMomentum, planeMomentum, plane.normal);
		force += elasticForce(particleMomentum, planeMomentum, plane.normal, elasticCoefficient) * collisionAliveFact;

		// spring force
		force += springForce(plane.normal, -collisionDistance, springCoefficient, dt);

		// friction force
		force += frictionForce(particleMomentum, planeMomentum, plane.normal, frictionCoefficient);
	}

	return force;
}

__device__ inline vec3 resolveCollision(ParticlePositionRadius particlePosition1
										, ParticleMomentumMass particleMomentum1
										, ParticlePositionRadius particlePosition2
										, ParticleMomentumMass particleMomentum2
										, float dt
										, float elasticCoefficient
										, float springCoefficient
										, float frictionCoefficient) {
	// check for collision
	float collisionDistance = collideSphereSphere(particlePosition1, particlePosition2);
	// compute collision normal
	vec3 normal = (particlePosition1.position - particlePosition2.position).normalized();

	vec3 force = vec3(0,0,0);
	// check for aliveness
	if(collisionDistance < 0.f) {
		
		// elastic force
		float collisionAliveFact = collisionIsAlive(particleMomentum1, particleMomentum2, normal);
		force += elasticForce(particleMomentum1, particleMomentum2, normal, elasticCoefficient) * collisionAliveFact;

		// spring force
		force += springForce(normal, -collisionDistance, springCoefficient, dt);

		// friction force
		force += frictionForce(particleMomentum1, particleMomentum2, normal, frictionCoefficient);
	}

	return force;
}

// ######################################################################### //
// ### Integration ######################################################### //

/**
 *  the euler integration step used in force based dynamics
 */
__global__ void forceBasedIntegration(ArrayView<ParticlePositionRadius> positions
									, ArrayView<ParticleMomentumMass> momMass
									, ArrayView<ParticleDeltaValues> deltas
									, vec3 acceleration
									, float deltaTime) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= positions.size()) return;
	ParticlePositionRadius& positionRadius = positions[ti.thread_id];
	ParticleMomentumMass& momentumMass = momMass[ti.thread_id];
	ParticleDeltaValues& delta = deltas[ti.thread_id];

	// update the particles momentum according to its deltaMomentum
	momentumMass.momentumVelocity += delta.delta;
	delta.delta = vec3(0.f,0.f,0.f);

	// integrate euler style
	positionRadius.position = positionRadius.position + momentumMass.momentumVelocity * deltaTime;
	momentumMass.momentumVelocity = momentumMass.momentumVelocity + acceleration * deltaTime;
}


// ######################################################################### //
// ### Kernels that handle Force based Dynamics with Brute Force ########### //

__global__ void resolveCollisionsBruteForceBased(ArrayView<ParticlePositionRadius> particlePositions
								, ArrayView<ParticleMomentumMass> momentumMass
								, ArrayView<ParticleDeltaValues> deltas
								, ArrayView<Plane> planes
								, float dt
								, float elasticCoefficient
								, float springCoefficient
								, float frictionCoefficient) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePositions.size()) return;

	// get particle data
	ParticlePositionRadius& position = particlePositions[ti.thread_id];
	ParticleMomentumMass& momentum = momentumMass[ti.thread_id];
	ParticleDeltaValues& delta = deltas[ti.thread_id];

	// collide particle with every plane
	vec3 summSumm = vec3(0,0,0);
	for(int i = 0; i < planes.size(); ++i) {
		summSumm += resolveCollision(planes[i], position, momentum, dt, elasticCoefficient, springCoefficient, frictionCoefficient);
	}

	// update particle momentum
	momentum.momentumVelocity += summSumm;
	delta.delta = vec3(0,0,0);
}

__global__ void resolveCollisionsBruteForceBased(ArrayView<ParticlePositionRadius> particlePositions
									, ArrayView<ParticleMomentumMass> momentumMass
									, ArrayView<ParticleDeltaValues> deltas
									, float dt
									, float elasticCoefficient
									, float springCoefficient
									, float frictionCoefficient) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePositions.size()) return;

	// get particle data
	ParticlePositionRadius& position1 = particlePositions[ti.thread_id];
	ParticleMomentumMass& momentum1 = momentumMass[ti.thread_id];
	ParticleDeltaValues& delta1 = deltas[ti.thread_id];

	// collide particle with every particle
	vec3 summSumm = vec3(0,0,0);
	for(int i = 0; i < particlePositions.size(); ++i) {
		ParticlePositionRadius& positionOther = particlePositions[i];
		ParticleMomentumMass& momentumOther = momentumMass[i];
		vec3 currentForce = resolveCollision(position1, momentum1, positionOther, momentumOther, dt, elasticCoefficient, springCoefficient, frictionCoefficient);
		summSumm += currentForce;
	}
	// remove the impact of the particle with itself
	summSumm -= resolveCollision(position1, momentum1, position1, momentum1, dt, elasticCoefficient, springCoefficient, frictionCoefficient);

	// update particle momentum
	deltas[ti.thread_id].delta += summSumm;
}

// ######################################################################### //
// ### Resolvers of force based dynamics using Collision Constraints ####### //

__global__ void resolveCollisionConstraintsForceBased(ArrayView<ParticlePositionRadius> particlePositions
														, ArrayView<ParticleMomentumMass> momMass
														, ArrayView<ParticleDeltaValues> deltas
														, ArrayView<Plane> planes
														, ArrayView<CollisionPlaneData> collisionPlaneData
														, float dt
														, float elasticCoefficient
														, float springCoefficient
														, float frictionCoefficient
														, int numCollisions) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= numCollisions) return;

	// get current collisiondata
	CollisionPlaneData data = collisionPlaneData[ti.thread_id];

	// get data for the particle from collisiondata
	ParticlePositionRadius& position = particlePositions[data.particleID];
	ParticleMomentumMass& momentum = momMass[data.particleID];
	ParticleDeltaValues& delta = deltas[data.particleID];

	// get plane from collisiondata
	Plane plane = planes[data.planeID];

	// collide particle with plane
	vec3 force = resolveCollision(plane, position, momentum, dt, elasticCoefficient, springCoefficient, frictionCoefficient);

	// update particle momentum
	// there is no need for atomics when handling plane collisions, i guess
	delta.delta += force;
}

__global__ void resolveCollisionConstraintsForceBased(ArrayView<ParticlePositionRadius> particlePositions
														, ArrayView<ParticleMomentumMass> momMass
														, ArrayView<ParticleDeltaValues> deltas
														, ArrayView<CollisionParticleData> collisionParticleData
														, float dt
														, float elasticCoefficient
														, float springCoefficient
														, float frictionCoefficient
														, int numCollisions) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= numCollisions) return;

	// get current collisiondata
	CollisionParticleData data = collisionParticleData[ti.thread_id];

	// get data for particle 1
	ParticlePositionRadius& position1 = particlePositions[data.particleID1];
	ParticleMomentumMass& momentum1 = momMass[data.particleID1];
	ParticleDeltaValues& delta1 = deltas[data.particleID1];

	// get data for particle 2
	ParticlePositionRadius& position2 = particlePositions[data.particleID2];
	ParticleMomentumMass& momentum2 = momMass[data.particleID2];
	ParticleDeltaValues& delta2 = deltas[data.particleID2];

	// collide particle with planes
	vec3 currentForce = resolveCollision(position1, momentum1, position2, momentum2, dt, elasticCoefficient, springCoefficient, frictionCoefficient);

	// update particle1 momentum
	atomicAdd(&delta1.delta[0], currentForce[0]);
	atomicAdd(&delta1.delta[1], currentForce[1]);
	atomicAdd(&delta1.delta[2], currentForce[2]);

	// update particle2 momentum
	atomicAdd(&delta2.delta[0], -currentForce[0]);
	atomicAdd(&delta2.delta[1], -currentForce[1]);
	atomicAdd(&delta2.delta[2], -currentForce[2]);
}

// ######################################################################### //
// ### C++ ParticleSystem ################################################## //
// ######################################################################### //

void ParticleSystem::forceBasedBruteForce(float dt) {
	const int BLOCK_SIZE = 128;

	ImGuiOptions& options = ImGuiOptions::get();

	forceBasedIntegration<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_momentumMass, m_particles.d_deltaValues, options.getGravity(), dt);
	
	resolveCollisionsBruteForceBased<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_momentumMass, m_particles.d_deltaValues, d_planes, dt, options.elasticCoefficient, options.springCoefficient, options.frictionCoefficient);
	resolveCollisionsBruteForceBased<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_momentumMass, m_particles.d_deltaValues, dt, options.elasticCoefficient, options.springCoefficient, options.frictionCoefficient);
}

template <int NumCollisions>
void ParticleSystem::forceBasedCollisionConstraints(float dt) {
	d_activeParticleCollisions.resize(NumCollisions * m_particleCounts.sum());
	d_activePlaneCollisions.resize(NumCollisions * m_particleCounts.sum());

	const int BLOCK_SIZE = 128;

	ImGuiOptions& options = ImGuiOptions::get();

	forceBasedIntegration<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_momentumMass, m_particles.d_deltaValues, options.getGravity(), dt);

	// create particle-plane collision constraints
	//createCollisionConstraintsPlanes<NumCollisions><<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_portalFlags, d_planes, d_activePlaneCollisions);
	createCollisionConstraintsPlanes<NumCollisions><<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition, m_particles.d_portalFlags, m_rigidBodies.d_rigidBodiesParticleOffsets, m_rigidBodies.d_rigidBodyPortalFlags, d_planes, d_activePlaneCollisions);
	auto newEndPlaneCollisions = thrust::remove_if(d_activePlaneCollisions.begin(), d_activePlaneCollisions.end(), CollisionPlaneData());
	int planeCollisionLength = thrust::distance(d_activePlaneCollisions.begin(), newEndPlaneCollisions);

	// create particle-particle collision constraints
	createCollisionConstraintsParticles<NumCollisions><<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, d_activeParticleCollisions, m_rigidBodies.d_rigidBodiesParticleOffsets, m_fluids.d_lambdas);
	auto newEndParticleCollisions = thrust::remove_if(d_activeParticleCollisions.begin(), d_activeParticleCollisions.end(), CollisionParticleData());
	int particleCollisionLength = thrust::distance(d_activeParticleCollisions.begin(), newEndParticleCollisions);

	// resolve particle-plane collision constraints
	if(planeCollisionLength > 0 && options.resolvePlanes) {
		resolveCollisionConstraintsForceBased<<<THREAD_BLOCK(planeCollisionLength, BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_momentumMass, m_particles.d_deltaValues, d_planes, d_activePlaneCollisions, dt, options.elasticCoefficient, options.springCoefficient, options.frictionCoefficient, planeCollisionLength);
	}

	// resolve particle-particle collision constraints
	if(particleCollisionLength > 0 && options.resolveParticles) {
		resolveCollisionConstraintsForceBased<<<THREAD_BLOCK(particleCollisionLength, BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_momentumMass, m_particles.d_deltaValues, d_activeParticleCollisions, dt, options.elasticCoefficient, options.springCoefficient, options.frictionCoefficient, particleCollisionLength);
	}
}

void ParticleSystem::forceBasedCollisionConstraintsHack(float dt) {
	forceBasedCollisionConstraints<16>(dt);
}