#include "particleSystem.h"

// ######################################################################### //
// ### Collision Resolvers that don't consider friction #################### //


/**
 *	Resolves Plane-Particle Collisions without Friction
 */
__global__ void resolveCollisionConstraintsPositionBased(ArrayView<ParticlePositionRadius> particlesGuessed
												, ArrayView<ParticleDeltaValues> deltas
												, ArrayView<Saiga::Plane> planes
												, ArrayView<CollisionPlaneData> collisionPlaneData
												, ArrayView<ParticlePortalFlags> flags
												, Portal bluePortal
												, Portal orangePortal
												, float dt
												, int numCollisions);
	
/**
 *	Resolves Particle-Particle Collisions without Friction
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
												, int numCollisions);


// ######################################################################### //
// ### Collision Resolvers that consider friction ########################## //
// FIXME: Friction and Stabilization cause Rigid Bodies to behave in a weird manner
/**
 *	Resolves Plane-Particle Collisions with Friction
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
												, bool enableFriction);

/**
 *	Resolves Particle-Particle Collisions with Friction
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
												, bool enableFriction);

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
												, bool enableFriction);

__global__ void	resolveTriangleRayCollision(ParticlePositionRadius particle, vec3 movementDirection,ArrayView<vec3> colTryeck,ArrayView<Tryeck> tryecke);
