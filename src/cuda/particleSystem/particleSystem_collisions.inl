#pragma once

#include "../particle.h"

#include "saiga/cuda/memory.h"

#include "saiga/cuda/thread_info.h"
#include "../../AgphysCudaConfig.h"

// ######################################################################### //
// ### particleSystem_collisions.inl ####################################### //
// ### Defines functions that will handle collision detection.           ### //
// ######################################################################### //

#if 0
__host__ __device__
inline void intersectFlo(vec3 origin, vec3 direction, vec3 point, vec3 normal, vec3& wpos, float& t) {
	direction.normalize();
	normal.normalize();
	t = (point - origin).dot(normal) / normal.dot(direction);
	wpos = origin + direction * t;
}

__host__ __device__
inline vec3 intersectFlosTriangle(ParticlePositionRadius origin, vec3 direction, vec3 A, vec3 B, vec3 C) {
	vec3 normal = (B-A).cross(C-A);
	normal.normalize();

	float dist;
	vec3 wpos;
	intersectFlo(origin.position, direction, A, normal, wpos, dist);
	if(dist >= 0 && dist <= direction.norm()) {
		return wpos;
	}
	intersectFlo(origin.position, -normal, A, normal, wpos, dist);
	if(dist < origin.radius) {
		return wpos;
	}
	intersectFlo(origin.position+direction, -normal, A, normal, wpos, dist);
	if(dist < origin.radius) {
		return wpos;
	}

	return vec3(NAN, NAN, NAN);
}
#endif

// ######################################################################### //
// ### Friction ############################################################ //
// ######################################################################### //

struct Friction {
	vec3 delta1 = vec3(0,0,0);
	vec3 delta2 = vec3(0,0,0);
};


__device__ inline Friction doTheFriction(vec3 normal, vec3 guessedPos1, vec3 pos1, vec3 guessedPos2, vec3 pos2, float massinv1, float massinv2, float staticFriction, float kineticFriction) {
	vec3 tangential = guessedPos1 - pos1 - guessedPos2 + pos2;
	tangential = tangential - tangential.dot(normal.normalized()) * normal.normalized();
	float tangentialLength = tangential.norm();

	Friction result;
	if(tangentialLength < normal.norm() * staticFriction) {
		result.delta1 = -massinv1 / (massinv1 + massinv2) * tangential;
	} else {
		float fact = min(kineticFriction * normal.norm() / tangential.norm(), 1.f);
		result.delta1 = -massinv1 / (massinv1 + massinv2) * tangential * fact;
	}
	result.delta2 = -massinv2 / (massinv1 + massinv2) * result.delta1;
	return result;
}


// ######################################################################### //
// ### Collision Detection: Sphere - Triangle ############################## //

__host__ __device__
inline vec3 projectPointOnPlane(vec3 point, vec3 planeNormal, vec3 planePoint) {
	vec3 tmp = planePoint - point;
	float distance = tmp.dot(planeNormal);
	vec3 resultPoint = point + distance * planeNormal;
	return resultPoint;
}

__host__ __device__
inline vec3 projectPointOnPlaneInDirection(vec3 point, vec3 direction, vec3 planeNormal, vec3 planePoint) {
	if(direction.norm() < 0.0001f) {
		return vec3(NAN, NAN, NAN);
	}
	direction.normalize();

	float tmp = direction.dot(-planeNormal);
	if(-0.001f < tmp && tmp < 0.001f) {
		// direction and plane normal are in a right angle to each other, so there is no valid projection
		return vec3(NAN, NAN, NAN);
	}

	vec3 pointOnPlane = projectPointOnPlane(point, planeNormal, planePoint);

	vec3 pointToPointOnPlane = pointOnPlane - point;
	float fact = pointToPointOnPlane.norm();
	float fact2 = pointToPointOnPlane.normalized().dot(direction);
	vec3 resultPoint = point + direction * (fact/fact2);

	return resultPoint;
}

__host__ __device__
inline vec3 computeBarycentricCoordinates(vec3 point, vec3 A, vec3 B, vec3 C, vec3 triangleNormal) {
	vec3 result = vec3(0,0,0);

	float abc = triangleNormal.dot((B-A).cross(C-A));
	float pbc = triangleNormal.dot((B-point).cross(C-point));
	float pca = triangleNormal.dot((C-point).cross(A-point));

	result.x() = pbc / abc;
	result.y() = pca / abc;
	result.z() = 1.f - result.x() - result.y();

	return result;
}

__host__ __device__
inline bool isInTriangle(vec3 barycentricCoordinates) {
	return (0.f <= barycentricCoordinates.x() && barycentricCoordinates.x() <= 1.f) &&
			(0.f <= barycentricCoordinates.y() && barycentricCoordinates.y() <= 1.f) &&
			(0.f <= barycentricCoordinates.z() && barycentricCoordinates.z() <= 1.f);
}

// not really intersecting a parallel plane...
__host__ __device__
inline bool intersectParallelPlane(ParticlePositionRadius particle, vec3 movementDirection, vec3 A, vec3 B, vec3 C, vec3 triangleNormal) {

	bool result = false;
	vec3 endPosition = particle.position + movementDirection;
	vec3 endProjection = projectPointOnPlane(endPosition, triangleNormal, A);
	float distanceFromPlane = (endPosition - endProjection).norm();
	if(distanceFromPlane < particle.radius) {
		vec3 bary = computeBarycentricCoordinates(endProjection, A, B, C, triangleNormal);
		bool intersectionIsHappening = isInTriangle(bary);

		result |= intersectionIsHappening;
	}

	return result;
}

// distance is negative if the particle intersects the triangle
__host__ __device__
inline float distanceFromTrianglePlane(ParticlePositionRadius particle, vec3 movementDirection, vec3 A, vec3 B, vec3 C, vec3 triangleNormal) {
	vec3 endPosition = particle.position + movementDirection;
	vec3 endProjection = projectPointOnPlane(endPosition, triangleNormal, A);
	float distanceFromPlane = (endPosition - endProjection).norm();
	if(distanceFromPlane < particle.radius) {
		vec3 bary = computeBarycentricCoordinates(endProjection, A, B, C, triangleNormal);
		if(isInTriangle(bary)) {
			return distanceFromPlane - particle.radius;
		}
	}

	return NAN;
}

// returns the point of intersection on the triangle
__host__ __device__
inline vec3 intersectParticleTriangle(ParticlePositionRadius particle, vec3 movementDirection, vec3 A, vec3 B, vec3 C, vec3 triangleNormal) {
	triangleNormal.normalize();

	// don't only intersect the triangle plane, also intersect a parallel plane that is offset by the particle's radius
	vec3 particleOnPlane = projectPointOnPlaneInDirection(particle.position, movementDirection, triangleNormal, A);

	vec3 result = { NAN, NAN, NAN };

	// check if the movementDirection is intersecting the triangle plane
	float intersectionDistance = (particle.position - particleOnPlane).norm();
	bool intersectionIsValid = particleOnPlane.x() == particleOnPlane.x();
	bool intersectionIsReachable = intersectionDistance <= movementDirection.norm();
	bool intersectionDirectionIsCorrect = (particle.position-particleOnPlane).dot(movementDirection) > 0.f;
	if(intersectionIsValid && intersectionIsReachable && intersectionDirectionIsCorrect) {
		// the movementDirection is intersecting the triangle plane
		// compute barycentric coorinates for the collision
		vec3 bary = computeBarycentricCoordinates(particleOnPlane, A, B, C, triangleNormal);
		if(isInTriangle(bary)) {
			// move the particle to the intersection point on the triangle
			result = particleOnPlane;
		}
	} else {
		vec3 finalPosition = particle.position + movementDirection;
		vec3 otherParticleOnPlane = projectPointOnPlane(finalPosition, triangleNormal, A);
		if((finalPosition - otherParticleOnPlane).norm() < particle.radius) {
			vec3 bary = computeBarycentricCoordinates(otherParticleOnPlane, A, B, C, triangleNormal);
			if(isInTriangle(bary)) {
				result = otherParticleOnPlane;
			}
		}
		if(result.x() != result.x()) {
			vec3 fuckOff = projectPointOnPlane(particle.position, triangleNormal, A);
			if((particle.position - fuckOff).norm() < particle.radius) {
				vec3 bary = computeBarycentricCoordinates(fuckOff, A, B, C, triangleNormal);
				if(isInTriangle(bary)) {
					result = fuckOff;
				}
			}
		}
	}

	return result;
}

__host__ __device__
inline bool particleIntersectsTriangle(ParticlePositionRadius particle, vec3 movementDirection, vec3 A, vec3 B, vec3 C, vec3 triangleNormal) {
	triangleNormal.normalize();

	vec3 result = intersectParticleTriangle(particle, movementDirection, A, B, C, triangleNormal);
	return result.x() == result.x();
}

// ######################################################################### //
// ### Collision Detection: Sphere - Plane ################################# //

/**
 *	returns the distance between the plane and the sphere.
 *	they are colliding if the distance is negative.
 */
__host__ __device__
inline float collideSpherePlane(ParticlePositionRadius sphere, Saiga::Plane plane) {
	return -plane.sphereOverlap(sphere.position, sphere.radius);
}

// ######################################################################### //
// ### Collision Detection: Sphere - Sphere ################################ //

/**
 *	returns the distance between p1 and p2.
 *	if they are colliding, the distance is negative.
 */
__host__ __device__
inline float collideSphereSphere(ParticlePositionRadius particle1, ParticlePositionRadius particle2) {
	float distance = (particle1.position - particle2.position).norm();
	return distance - particle1.radius - particle2.radius;
}

__host__ __device__
inline vec3 computeBorderNormal(vec3 xij, vec3 nij) {
	vec3 result;
	float dot = xij.dot(nij);
	if(dot < 0.f) {
		result = xij - 2.f * dot * nij;
	} else {
		result = xij;
	}
	return result;
}

/**
 *	returns the directed unweighted distance between p1 and p2 that should be used to move them.
 *	if they are not colliding, the length of the resulting vector is 0.
 */
__host__ __device__
inline vec3 collideSphereSphere(ParticlePositionRadius particle1, vec3 particle1Normal, float resolution1, ParticlePositionRadius particle2, vec3 particle2Normal, float resolution2) {
	vec3 collisionNormal = vec3(0.f,0.f,0.f);
	// if the normal of particle1 is shorter than the one of particle 2, take the first one
	vec3 distanceVector = particle2.position - particle1.position;
	float distance = particle1.radius + particle2.radius - distanceVector.norm();
	if(distance < 0.f) {
		// no collision
		return vec3(0.f,0.f,0.f);
	}
	float res = 0.f;
	bool doRigidBodyThings = particle1Normal.norm()*resolution1 > 0.f || particle2Normal.norm()*resolution2 > 0.f;

	if(!doRigidBodyThings) {
		// compute the normal as usual
		collisionNormal = distanceVector.normalized();
	} else {
		if(particle1Normal.norm()*resolution1 > 0.f) {
			if(particle2Normal.norm() > 0.f) {
				// both particles are rigid bodies
				bool useFirstNormal = particle1Normal.norm()*resolution1 < particle2Normal.norm()*resolution2;
				if(useFirstNormal) {
					collisionNormal = particle1Normal;
					res = resolution1;
				} else {
					collisionNormal = -particle2Normal;
					res = resolution2;
				}
			} else {
				// only particle 1 is a rigid body particle, use its normal
				collisionNormal = particle1Normal;
				res = resolution1;
			}
		} else {
			// only particle 2 is a rigid body particle, us its normal
			collisionNormal = -particle2Normal;
			res = resolution2;
		}

		bool isBorderParticle = collisionNormal.norm() <= 1.f;
		distance = collisionNormal.norm() * res;
		collisionNormal.normalize();
		if(isBorderParticle) {
			// special case for boundary particles
			collisionNormal = computeBorderNormal(distanceVector, collisionNormal);
			distance = particle1.radius + particle2.radius - distanceVector.norm();
		}
	}
	collisionNormal = collisionNormal * distance;
	return collisionNormal;
}

// ######################################################################### //
// ### Collision Detection: Sphere - Ray ################################### //

__host__ __device__
inline CollisionRayData collideSphereRayHelper(ParticlePositionRadius particle, Saiga::Ray ray) {
	CollisionRayData data;
	
	vec3 z = ray.origin - particle.position;
	data.distanceToCenter = z.norm();
	float dot1 = ray.direction.dot(z);
	float Q = dot1 * dot1 - z.dot(z) + particle.radius * particle.radius;
	if(Q > 0.f) {
		data.distance = -sqrt(Q);
		// collision! compute intersection points
	} else {
		data.distance = sqrt(Q);
	}

	return data;
}

/**
 *	returns a Collision Object consisting of a ray and a particle.
 *	if they are colliding, the distance is negative.
 */
__host__ __device__
inline CollisionRayData collideSphereRay(ParticlePositionRadius particle, Saiga::Ray ray) {
	CollisionRayData data = collideSphereRayHelper(particle, ray);
	return data;
}

// ######################################################################### //
// ### Collision Constraint Generators ##################################### //

/**
 *  Fills 'collisionTryeckData' with collisions of particles and tryecks.
 *  Every Particle will create at most 'NumCollisions' many Collision Constraints.
 *  The result should be compacted with thrust::remove_if
 */
template <int NumCollisions>
__global__ void createCollisionConstraintsTryecks(ArrayView<ParticlePositionRadius> particles, 
												 ArrayView<ParticlePositionRadius> guessedPositions, 
												 ArrayView<ParticleColor> colors, 
												 ArrayView<ParticlePortalFlags> particlePortalFlags, 
												 ArrayView<RigidBodyParticleOffset> bodyOffset, 
												 ArrayView<RigidBodyPortalFlags> bodyFlags,
												 ArrayView<Tryeck> tryecks, 
												 ArrayView<CollisionTryeckData> collisionTryeckData) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particles.size()) return;

	ParticlePositionRadius particle;
	Saiga::CUDA::vectorCopy(particles.data()+ti.thread_id,&particle);

	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(guessedPositions.data()+ti.thread_id,&guessedPos);

	ParticlePortalFlags portalFlags;
	Saiga::CUDA::vectorCopy(particlePortalFlags.data()+ti.thread_id,&portalFlags);

	RigidBodyParticleOffset rbParticleOffset;
	Saiga::CUDA::vectorCopy(bodyOffset.data()+ti.thread_id,&rbParticleOffset);

	// DEBUG
	colors[ti.thread_id].color = vec4(0,0,1,1);
	// /DEBUG

	RigidBodyPortalFlags rbPortalFlags;
	if(rbParticleOffset.bodyIndex != -1) {
		Saiga::CUDA::vectorCopy(bodyFlags.data()+rbParticleOffset.bodyIndex,&rbPortalFlags);

		if(rbPortalFlags.state == Regular) {
			colors[ti.thread_id].color = vec4(1,0,0,1);
		} else if(rbPortalFlags.state == IsDuplicate) {
			colors[ti.thread_id].color = vec4(1,1,0,1);
			return;
		} else if(rbPortalFlags.state == HasDuplicate) {
			colors[ti.thread_id].color = vec4(0,1,0,1);
		}
	}


	// create a local list of collisions with other particles
	int currentIndex = 0;
	
	// update global array with invalid collisions
	CollisionTryeckData empty;
#pragma unroll NumCollisions
	for(int i = NumCollisions - 1; i >= currentIndex; --i) {
		Saiga::CUDA::vectorCopy<CollisionTryeckData, int2>(&empty, collisionTryeckData.data()+ti.thread_id*NumCollisions + i);
	}

	if(portalFlags.oldParticle != 0 || portalFlags.newParticle != 0) {
		return;
	}

	vec3 movementDirection = guessedPos.position - particle.position;
	// collide with tryecks
	for(int i = 0; i < tryecks.size(); ++i) {
		Tryeck tryeck;
		Saiga::CUDA::vectorCopy(tryecks.data()+i, &tryeck);
		bool collisionIsHappening = particleIntersectsTriangle(particle, movementDirection, tryeck.a, tryeck.b, tryeck.c, tryeck.normal());
		if(collisionIsHappening) {
			// collision! add collision to the local array
			CollisionTryeckData c;
			c.particleID = ti.thread_id;
			c.tryeckID = i;

			//colors[ti.thread_id].color = vec4(1,0,0,1);

			Saiga::CUDA::vectorCopy<CollisionTryeckData, int2>(&c, collisionTryeckData.data()+ti.thread_id*NumCollisions+currentIndex);
			++currentIndex;

			if(currentIndex == NumCollisions) return;
		}
	}
}


/**
 *  Fills 'collisionPlaneData' with collisions of particles and planes.
 *  Every Particle will create at most 'NumCollisions' many Collision Constraints.
 *  The result should be compacted with thrust::remove_if
 */
template <int NumCollisions>
__global__ void createCollisionConstraintsPlanes(ArrayView<ParticlePositionRadius> particles, 
												 ArrayView<ParticlePortalFlags> particlePortalFlags, 
												 ArrayView<RigidBodyParticleOffset> bodyOffset, 
												 ArrayView<RigidBodyPortalFlags> bodyFlags,
												 ArrayView<Plane> planes, 
												 ArrayView<CollisionPlaneData> collisionPlaneData) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particles.size()) return;

	ParticlePositionRadius particle;
	Saiga::CUDA::vectorCopy(particles.data()+ti.thread_id,&particle);

	ParticlePortalFlags portalFlags;
	Saiga::CUDA::vectorCopy(particlePortalFlags.data()+ti.thread_id,&portalFlags);

	RigidBodyParticleOffset rbParticleOffset;
	Saiga::CUDA::vectorCopy(bodyOffset.data()+ti.thread_id,&rbParticleOffset);

	// create a local list of collisions with other particles
	int currentIndex = 0;
	
	// update global array
	CollisionPlaneData empty;
#pragma unroll NumCollisions
	for(int i = NumCollisions - 1; i >= currentIndex; --i) {
		Saiga::CUDA::vectorCopy<CollisionPlaneData, int2>(&empty, collisionPlaneData.data()+ti.thread_id*NumCollisions + i);
	}

	if(portalFlags.oldParticle != 0 || portalFlags.newParticle != 0) {
		return;
	}

	// collide with planes
#pragma unroll 6
	for(int i = 0; i < planes.size(); ++i) {
		Saiga::Plane plane;
		Saiga::CUDA::vectorCopy(planes.data()+i, &plane);
		float distance = collideSpherePlane(particle, plane);
		if(distance <= 0.f) {
			// collision! add collision to the local array
			CollisionPlaneData c;
			c.particleID = ti.thread_id;
			c.planeID = i;

			Saiga::CUDA::vectorCopy<CollisionPlaneData, int2>(&c, collisionPlaneData.data()+ti.thread_id*NumCollisions+currentIndex);
			++currentIndex;

			if(currentIndex == NumCollisions) return;
		}
	}
}

/**
 *  Fills 'collisionParticleData' with collisions of particles and particles.
 *  Every Particle will create at most 'NumCollisions' many Collision Constraints.
 *  The result should be compacted with thrust::remove_if
 */
template <int NumCollisions>
__global__ void createCollisionConstraintsParticles(ArrayView<ParticlePositionRadius> particles
													, ArrayView<CollisionParticleData> collisionParticleData
													, ArrayView<RigidBodyParticleOffset> offset
													, ArrayView<FluidLambda> lambdas) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particles.size()) return;
	
	// create a local list of collisions with other particles
	int currentIndex = 0;

	// collide with particles
	ParticlePositionRadius particle1;
	Saiga::CUDA::vectorCopy(particles.data()+ti.thread_id, &particle1);
		
	FluidLambda lambda;
	Saiga::CUDA::vectorCopy<FluidLambda, int2>(lambdas.data()+ti.thread_id, &lambda);
	if(lambda.isFluid != 1) {
		for(int i = 0; i < ti.thread_id; ++i) {
			ParticlePositionRadius particle2;
			Saiga::CUDA::vectorCopy(particles.data()+i, &particle2);

			FluidLambda lambda2;
			Saiga::CUDA::vectorCopy<FluidLambda, int2>(lambdas.data()+i, &lambda2);
			if(lambda2.isFluid == 1) continue;

			float distance = collideSphereSphere(particle1, particle2);
			if(distance <= 0.f && offset[ti.thread_id].bodyIndex != offset[i].bodyIndex) {
				// collision! add collision to the local array
				CollisionParticleData data;
				data.particleID1 = ti.thread_id;
				data.particleID2 = i;
				Saiga::CUDA::vectorCopy<CollisionParticleData, int2>(&data, collisionParticleData.data()+ti.thread_id*NumCollisions+currentIndex);
				++currentIndex;

				if(currentIndex == NumCollisions) return;
			}
		}
	}

	// update global array
	for(int i = NumCollisions-1; i > currentIndex; --i) {
		CollisionParticleData data;
		Saiga::CUDA::vectorCopy<CollisionParticleData, int2>(&data, collisionParticleData.data()+ti.thread_id*NumCollisions+i);
	}
}
