#include "particleSystem_collisionResolver.h"

#include "particleSystem_collisions.inl"

#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"


#define atomicAdd3(target, source) atomicAdd(&target[0], source[0]); atomicAdd(&target[1], source[1]); atomicAdd(&target[2], source[2])

#define VECCC(a) a.x(), a.y(), a.z()

// applies a position change to a duplicated particle
static inline __device__ void applyLotion(ArrayView<ParticleDeltaValues> deltas, ParticlePortalFlags flag, RigidBodyPortalFlags rbFlag, Portal bluePortal, Portal orangePortal, vec3 positionChange) {
	vec3 transformedChange;
	if(flag.otherParticle != -1) {
		if(flag.otherParticle < 0 || flag.otherParticle >= deltas.size()) {
			printf("Fuack: %d\n", flag.otherParticle);
		}
		ParticleDeltaValues& deltaOther = deltas[flag.otherParticle];
		if(flag.portalHit == 1) {
			// blue portal hit
			transformedChange = Portal::transformVector(vec4(VECCC(positionChange), 0.f), bluePortal, orangePortal);
		} else if(flag.portalHit == 2) {
			// orange portal hit
			transformedChange = Portal::transformVector(vec4(VECCC(positionChange), 0.f), orangePortal, bluePortal);
		} else {
			if(rbFlag.portalHit == 1) {
				// blue portal hit
				transformedChange = Portal::transformVector(vec4(VECCC(positionChange), 0.f), bluePortal, orangePortal);
			} else if(rbFlag.portalHit == 2) {
				// orange portal hit
				transformedChange = Portal::transformVector(vec4(VECCC(positionChange), 0.f), orangePortal, bluePortal);
			} else {
				printf("Flag is invalid, %d, %d, %d\n", flag.otherParticle, flag.portalHit, rbFlag.portalHit);
			}
		}
		atomicAdd3(deltaOther.delta, transformedChange);
	}
}


__global__ void resolveCollisionConstraintsPositionBased(ArrayView<ParticlePositionRadius> particlesGuessed
												, ArrayView<ParticleDeltaValues> deltas
												, ArrayView<Saiga::Plane> planes
												, ArrayView<CollisionPlaneData> collisionPlaneData
												, ArrayView<ParticlePortalFlags> flags
												, Portal bluePortal
												, Portal orangePortal
												, float dt
												, int numCollisions) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= numCollisions) return;
	
	// Load collision data from memory
	CollisionPlaneData data;
	Saiga::CUDA::vectorCopy<CollisionPlaneData, int2>(collisionPlaneData.data()+ti.thread_id, &data);

	// Load plane and particle from memory
	Saiga::Plane plane;
	Saiga::CUDA::vectorCopy(planes.data()+data.planeID, &plane);

	ParticlePositionRadius guessedPosition;
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+data.particleID, &guessedPosition);

	ParticleDeltaValues& delta = deltas[data.particleID];
	
	// compute overlap distance
	float distance = collideSpherePlane(guessedPosition, plane);
	if(distance > 0.f) {
		// this is not actually a collision, which might have been caused by iterating
		return;
	}

	// compute amount vector of displacement to resolve the collision
	vec3 positionChange = plane.normal * -distance;

	atomicAdd3(delta.delta, positionChange);
}

/**
 *	resolves Particle-Plane Collisions
 */
__global__ void resolveCollisionConstraintsPositionBasedWithFriction(ArrayView<ParticlePositionRadius> particlesGuessed
												, ArrayView<ParticlePositionRadius> absolutePositions
												, ArrayView<ParticleMomentumMass> moms
												, ArrayView<ParticleDeltaValues> deltas
												, ArrayView<Saiga::Plane> planes
												, ArrayView<CollisionPlaneData> collisionPlaneData
												, ArrayView<ParticlePortalFlags> flags
												, ArrayView<RigidBodyParticleOffset> offsets
												, ArrayView<RigidBodyPortalFlags> rbFlags
												, Portal bluePortal
												, Portal orangePortal
												, int fluidsBegin
												, int fluidsEnd
												, float dt
												, int numCollisions
												, float staticFriction
												, float kineticFriction
												, bool enableFriction) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= numCollisions) return;
	
	// Load collision data from memory
	CollisionPlaneData data;
	Saiga::CUDA::vectorCopy<CollisionPlaneData, int2>(collisionPlaneData.data()+ti.thread_id, &data);

	// Load plane and particle from memory
	Saiga::Plane plane;
	Saiga::CUDA::vectorCopy(planes.data()+data.planeID, &plane);

	ParticlePositionRadius guessedPosition;
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+data.particleID, &guessedPosition);

	ParticleDeltaValues& delta = deltas[data.particleID];
	
	// compute overlap distance
	float distance = collideSpherePlane(guessedPosition, plane);
	if(distance > 0.f) {
		// this is not actually a collision, which might have been caused by iterating
		return;
	}

	// compute amount vector of displacement to resolve the collision
	vec3 positionChange = plane.normal * -distance;

	if(enableFriction) {
		ParticlePositionRadius absolutePos;
		Saiga::CUDA::vectorCopy(absolutePositions.data()+data.particleID, &absolutePos);

		ParticleMomentumMass mom;
		Saiga::CUDA::vectorCopy(moms.data()+data.particleID, &mom);


		// don't do the friction for fluid particles
		if(ti.thread_id < fluidsBegin || ti.thread_id >= fluidsEnd) {
			Friction friction = doTheFriction(positionChange, guessedPosition.position, absolutePos.position, vec3(0.f,0.f,0.f), vec3(0.f,0.f,0.f), mom.massinv, 0.f, staticFriction, kineticFriction);
			positionChange += friction.delta1;
		}
	}

	atomicAdd3(delta.delta, positionChange);

	// TODO: update duplicate-rigid-body's particle too
	ParticlePortalFlags flag;
	Saiga::CUDA::vectorCopy(flags.begin()+data.particleID, &flag);

	RigidBodyParticleOffset of;
	Saiga::CUDA::vectorCopy(offsets.begin()+data.particleID, &of);

	RigidBodyPortalFlags rbFlag;
	if(of.bodyIndex != -1) {
		Saiga::CUDA::vectorCopy(rbFlags.begin()+of.bodyIndex, &rbFlag);
	}
	applyLotion(deltas, flag, rbFlag, bluePortal, orangePortal, positionChange);
}



/**
 *	resolves particle-particle collisions with friction
 */
__global__ void resolveCollisionConstraintsPositionBasedWithFriction(ArrayView<ParticlePositionRadius> particlesGuessed
												, ArrayView<ParticleMomentumMass> moms
												, ArrayView<ParticleDeltaValues> deltas
												, ArrayView<CollisionParticleData> collisionParticleData
												, ArrayView<RigidBodyParticleData> rigidBodies
												, ArrayView<RigidBodyOrientation> rigidBodyOrientations
												, ArrayView<RigidBodyParticleNormal> rigidBodyNormals
												, ArrayView<ParticlePositionRadius> absolutePositions
												, ArrayView<ParticlePortalFlags> portalFlags
												, ArrayView<RigidBodyParticleOffset> particleOffset
												, ArrayView<RigidBodyPortalFlags> bodyPortalFlags
							  					, Portal bluePortal
							  					, Portal orangePortal
												, int fluidsBegin
												, int fluidsEnd
												, float dt
												, float relaxationFactor
												, int numCollisions
												, float staticFriction
												, float kineticFriction
												, bool enableFriction) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= numCollisions) return;

	CollisionParticleData data;
	Saiga::CUDA::vectorCopy<CollisionParticleData, int2>(collisionParticleData.data()+ti.thread_id, &data);

	// get the two particles
	ParticlePositionRadius guessedPosition1;
	ParticlePositionRadius guessedPosition2;
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+data.particleID1, &guessedPosition1);
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+data.particleID2, &guessedPosition2);

	ParticlePositionRadius absolutePos1;
	ParticlePositionRadius absolutePos2;
	Saiga::CUDA::vectorCopy(absolutePositions.data()+data.particleID1, &absolutePos1);
	Saiga::CUDA::vectorCopy(absolutePositions.data()+data.particleID2, &absolutePos2);

	RigidBodyParticleNormal normalParticle1;
	RigidBodyParticleNormal normalParticle2;
	Saiga::CUDA::vectorCopy(rigidBodyNormals.data()+data.particleID1, &normalParticle1);
	Saiga::CUDA::vectorCopy(rigidBodyNormals.data()+data.particleID2, &normalParticle2);

	ParticlePortalFlags flags1;
	ParticlePortalFlags flags2;
	Saiga::CUDA::vectorCopy(portalFlags.data()+data.particleID1, &flags1);	
	Saiga::CUDA::vectorCopy(portalFlags.data()+data.particleID2, &flags2);

	RigidBodyParticleOffset bodyOffset1;
	RigidBodyParticleOffset bodyOffset2;
	Saiga::CUDA::vectorCopy(particleOffset.data()+data.particleID1, &bodyOffset1);
	Saiga::CUDA::vectorCopy(particleOffset.data()+data.particleID2, &bodyOffset2);

	vec3 normal1 = vec3(0,0,0);
	float resolution1 = 0.f;
	if(normalParticle1.bodyIndex != -1) {
		// get the rigid body's sdf-normal
		RigidBodyOrientation rb;
		Saiga::CUDA::vectorCopy(rigidBodyOrientations.data()+normalParticle1.bodyIndex, &rb);

		normal1 = rb.rotation * normalParticle1.normal;
		resolution1 = rb.resolution;
	}

	vec3 normal2 = vec3(0,0,0);
	float resolution2 = 0.f;
	if(normalParticle2.bodyIndex != -1) {
		// get the rigid body's sdf-normal
		RigidBodyOrientation rb;
		Saiga::CUDA::vectorCopy(rigidBodyOrientations.data()+normalParticle2.bodyIndex, &rb);

		normal2 = rb.rotation * normalParticle2.normal;
		resolution2 = rb.resolution;
	}

	// compute their overlapping distance
	// compute the normals for each particle
	vec3 distance = collideSphereSphere(guessedPosition1, normal1, resolution1, guessedPosition2, normal2, resolution2);
	if(distance.norm() < 0.001f) {
		// this is not actually a collision, which might have been caused by iterating
		return;
	}

	ParticleMomentumMass mass1;
	ParticleMomentumMass mass2;
	Saiga::CUDA::vectorCopy(moms.data()+data.particleID1, &mass1);
	Saiga::CUDA::vectorCopy(moms.data()+data.particleID2, &mass2);
	
	ParticleDeltaValues& delta1 = deltas[data.particleID1];
	ParticleDeltaValues& delta2 = deltas[data.particleID2];

	Friction friction;
	if(data.particleID1 >= fluidsBegin && data.particleID1 < fluidsEnd) {
		//printf("ti.thread_id: %d, fluidsBegin: %d, fluidsEnd: %d\n", ti.thread_id, fluidsBegin, fluidsEnd);
	} else if(data.particleID2 >= fluidsBegin && data.particleID2 < fluidsEnd) {
		//printf("ti.thread_id: %d, fluidsBegin: %d, fluidsEnd: %d\n", ti.thread_id, fluidsBegin, fluidsEnd);
	} else if(enableFriction) {
		// don't do the friction for fluid particles
		friction = doTheFriction(distance, guessedPosition1.position, absolutePos1.position, guessedPosition2.position, absolutePos2.position, mass1.massinv, mass2.massinv, staticFriction, kineticFriction);
	}

	// compute amount vector of displacement to resolve the collision
	vec3 positionChange = distance;

	vec3 positionChange1 = -mass1.massinv / (mass1.massinv + mass2.massinv) * positionChange;
	positionChange1 += friction.delta1;
	positionChange1 *= relaxationFactor;

	vec3 positionChange2 = mass2.massinv / (mass1.massinv + mass2.massinv) * positionChange;
	positionChange2 += friction.delta2;
	positionChange2 *= relaxationFactor;

	atomicAdd3(delta1.delta, positionChange1);
	
	atomicAdd3(delta2.delta, positionChange2);

	// TODO: update duplicate-rigid-body's particle too
	ParticlePortalFlags flag, flag2;
	Saiga::CUDA::vectorCopy(portalFlags.begin()+data.particleID1, &flag);
	Saiga::CUDA::vectorCopy(portalFlags.begin()+data.particleID2, &flag2);

	RigidBodyPortalFlags rbFlag, rbFlag2;
	if(bodyOffset1.bodyIndex != -1) {
		Saiga::CUDA::vectorCopy(bodyPortalFlags.begin()+bodyOffset1.bodyIndex, &rbFlag);
	}
	if(bodyOffset2.bodyIndex != -1) {
		Saiga::CUDA::vectorCopy(bodyPortalFlags.begin()+bodyOffset2.bodyIndex, &rbFlag2);
	}

	applyLotion(deltas, flag, rbFlag, bluePortal, orangePortal, positionChange1);
	applyLotion(deltas, flag2, rbFlag2, bluePortal, orangePortal, positionChange2);

}

/**
 *	resolves particle-particle collisions without friction
 */
__global__ void resolveCollisionConstraintsPositionBased(ArrayView<ParticlePositionRadius> particlesGuessed
												, ArrayView<ParticleMomentumMass> moms
												, ArrayView<ParticleDeltaValues> deltas
												, ArrayView<CollisionParticleData> collisionParticleData
												, ArrayView<ParticlePortalFlags> flags
												, ArrayView<RigidBodyParticleOffset> offsets
												, ArrayView<RigidBodyPortalFlags> rbFlags
												, Portal bluePortal
												, Portal orangePortal
												, float dt
												, float relaxationFactor
												, int numCollisions) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= numCollisions) return;

	CollisionParticleData data;
	Saiga::CUDA::vectorCopy<CollisionParticleData,int2>(collisionParticleData.data()+ti.thread_id, &data);

	// get the two particles
	ParticlePositionRadius guessedPosition1;
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+data.particleID1, &guessedPosition1);

	ParticlePositionRadius guessedPosition2;
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+data.particleID2, &guessedPosition2);

	// compute their overlapping distance
	float distance = collideSphereSphere(guessedPosition1, guessedPosition2);
	if(distance > 0.f) {
		// this is not actually a collision, which might have been caused by iterating
		return;
	}

	ParticleMomentumMass mass1;
	Saiga::CUDA::vectorCopy(moms.data()+data.particleID1, &mass1);

	ParticleMomentumMass mass2;
	Saiga::CUDA::vectorCopy(moms.data()+data.particleID2, &mass2);
	
	ParticleDeltaValues& delta1 = deltas[data.particleID1];
	ParticleDeltaValues& delta2 = deltas[data.particleID2];

	// compute amount vector of displacement to resolve the collision
	vec3 positionChange = ((guessedPosition1.position - guessedPosition2.position).normalized() * (-distance)) * relaxationFactor;
	vec3 positionChange1 = mass1.massinv / (mass1.massinv + mass2.massinv) * positionChange;
	vec3 positionChange2 = -mass2.massinv / (mass1.massinv + mass2.massinv) * positionChange;

	atomicAdd(&delta1.delta[0], positionChange1[0]);
	atomicAdd(&delta1.delta[1], positionChange1[1]);
	atomicAdd(&delta1.delta[2], positionChange1[2]);
	
	atomicAdd(&delta2.delta[0], positionChange2[0]);
	atomicAdd(&delta2.delta[1], positionChange2[1]);
	atomicAdd(&delta2.delta[2], positionChange2[2]);

	// TODO: update duplicate-rigid-body's particle too
	ParticlePortalFlags flag, flag2;
	Saiga::CUDA::vectorCopy(flags.begin()+data.particleID1, &flag);
	Saiga::CUDA::vectorCopy(flags.begin()+data.particleID2, &flag2);

	RigidBodyParticleOffset of, of2;
	Saiga::CUDA::vectorCopy(offsets.begin()+data.particleID1, &of);
	Saiga::CUDA::vectorCopy(offsets.begin()+data.particleID2, &of2);

	RigidBodyPortalFlags rbFlag, rbFlag2;
	if(of.bodyIndex != -1) {
		Saiga::CUDA::vectorCopy(rbFlags.begin()+of.bodyIndex, &rbFlag);
	}
	if(of2.bodyIndex != -1) {
		Saiga::CUDA::vectorCopy(rbFlags.begin()+of2.bodyIndex, &rbFlag2);
	}

	applyLotion(deltas, flag, rbFlag, bluePortal, orangePortal, positionChange1);
	applyLotion(deltas, flag2, rbFlag2, bluePortal, orangePortal, positionChange2);
}

/**
 *	resolves triangle-particle collisions
 */
__global__ void resolveTriangleCollisions(ArrayView<ParticlePositionRadius> particlesGuessed
												, ArrayView<ParticleMomentumMass> moms
												, ArrayView<ParticleDeltaValues> deltas
												, ArrayView<ParticleColor> colors
												, ArrayView<CollisionTryeckData> collisionTryeckData
												, ArrayView<Tryeck> tryecke
												, ArrayView<ParticlePositionRadius> absolutePositions
												, ArrayView<ParticlePortalFlags> portalFlags
												, ArrayView<RigidBodyParticleOffset> particleOffset
												, ArrayView<RigidBodyPortalFlags> bodyPortalFlags
							  					, Portal bluePortal
							  					, Portal orangePortal
												, int fluidsBegin
												, int fluidsEnd
												, float dt
												, float relaxationFactor
												, int numCollisions
												, float staticFriction
												, float kineticFriction
												, bool enableFriction) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= numCollisions) return;

	// Load collision data from memory
	CollisionTryeckData data;
	Saiga::CUDA::vectorCopy<CollisionTryeckData, int2>(collisionTryeckData.data()+ti.thread_id, &data);

	// Load tryeck
	Tryeck tryeck;
	Saiga::CUDA::vectorCopy(tryecke.data()+data.tryeckID, &tryeck);

	if(data.particleID < 0 || data.particleID >= particlesGuessed.size()) {
		printf("Fuck off: %d\n", data.particleID);
	}

	// Load particle
	ParticlePositionRadius guessedPosition;
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+data.particleID, &guessedPosition);

	ParticlePositionRadius originalPosition;
	Saiga::CUDA::vectorCopy(absolutePositions.data()+data.particleID, &originalPosition);

	ParticlePortalFlags flags;
	Saiga::CUDA::vectorCopy(portalFlags.data()+data.particleID, &flags);
	// if this particle is intersecting a portal, ignore this intersection
	if(flags.portalHit != 0) return;


	ParticleDeltaValues& delta = deltas[data.particleID];

	vec3 movementDirection = guessedPosition.position - originalPosition.position;

	// check if an intersection is occuring at all
	vec3 triangleNormal = tryeck.normal();

	vec3 proj = intersectParticleTriangle(originalPosition, movementDirection, tryeck.a, tryeck.b, tryeck.c, triangleNormal);
	if(proj.x() != proj.x()) {
		return;
	}
	vec3 distance = proj - guessedPosition.position;
	distance += triangleNormal * originalPosition.radius;
	
	// compute amount vector of displacement to resolve the collision
	vec3 positionChange = distance;

	if(enableFriction) {
		ParticlePositionRadius absolutePos;
		Saiga::CUDA::vectorCopy(absolutePositions.data()+data.particleID, &absolutePos);

		ParticleMomentumMass mom;
		Saiga::CUDA::vectorCopy(moms.data()+data.particleID, &mom);


		// don't do the friction for fluid particles
		if(ti.thread_id < fluidsBegin || ti.thread_id >= fluidsEnd) {
			Friction friction = doTheFriction(positionChange, guessedPosition.position, absolutePos.position, vec3(0.f,0.f,0.f), vec3(0.f,0.f,0.f), mom.massinv, 0.f, staticFriction, kineticFriction);
			positionChange += friction.delta1;
		}
	}

	atomicAdd(&delta.delta[0], positionChange[0]);
	atomicAdd(&delta.delta[1], positionChange[1]);
	atomicAdd(&delta.delta[2], positionChange[2]);

	ParticlePortalFlags flag;
	Saiga::CUDA::vectorCopy(portalFlags.begin()+data.particleID, &flag);

	RigidBodyParticleOffset bodyOffset;
	Saiga::CUDA::vectorCopy(particleOffset.data()+data.particleID, &bodyOffset);

	RigidBodyPortalFlags rbFlag;
	if(bodyOffset.bodyIndex != -1) {
		Saiga::CUDA::vectorCopy(bodyPortalFlags.begin()+bodyOffset.bodyIndex, &rbFlag);
	}

	applyLotion(deltas, flag, rbFlag, bluePortal, orangePortal, positionChange);
}

__global__ void	resolveTriangleRayCollision(ParticlePositionRadius particle, vec3 movementDirection,ArrayView<vec3> colTryeck, ArrayView<Tryeck> tryecke){
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= tryecke.size()) return;
	
	Tryeck tmp = tryecke[ti.thread_id];

	colTryeck[ti.thread_id] = intersectParticleTriangle(particle,movementDirection, tmp.a, tmp.b, tmp.c, tmp.normal());

}
