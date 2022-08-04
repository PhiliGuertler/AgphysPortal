#pragma once

#include "saiga/core/geometry/plane.h"
#include "saiga/core/math/math.h"
#include "saiga/core/math/random.h"
#include "saiga/cuda/cudaHelper.h"

#include "../AgphysCudaConfig.h"

#include "collisionData.h"


// ######################################################################### //
// ### Particle Types ###################################################### //

enum class ParticleType {
	RegularParticle,
	RigidBodyParticle,
	ClothParticle,
	FluidParticle
};

enum RigidBodyState : int {
	Regular = 0,
	HasDuplicate = 1,
	IsDuplicate = 2
};


// ######################################################################### //
// ### Regular Particles ################################################### //
// ######################################################################### //

// ######################################################################### //
// ### Particle Attributes packed in 4*4 Byte-sized structs ################ //

struct SAIGA_ALIGN(16) ParticlePositionRadius {
    vec3 position = {0.f,0.f,0.f};
    float radius = 0.f;
};

struct SAIGA_ALIGN(16) ParticleColor {
    vec4 color = {0.f,1.f,0.f,1.f};
};

struct SAIGA_ALIGN(16) ParticleMomentumMass {
    vec3 momentumVelocity = {0.f,0.f,0.f}; // to be used as either momentum (force based) or velocity (position based)
    float massinv = 0.f;
};

struct SAIGA_ALIGN(16) ParticleDeltaValues {
    vec3 delta = {0.f,0.f,0.f}; // to be used as either deltaMomentum or deltaPosition
    int effectFlag = -1; // use for effect particle
};

// ######################################################################### //
// ### Particle Attributes for portal packed in 4*4 Byte-sized structs ##### //
/**
 * portalHit: 0=no hit, 1=blue portal hit, 2=orange portal hit
 * oldParticle: flag to show a particle is an old one and is deleted after it looses portal contact, also no plane intersection
 * newParticle: flag to show a particle is a new one, no plane intersection as long as it touches a portal
 */
struct SAIGA_ALIGN(16) ParticlePortalFlags {
	int portalHit = 0;
	int oldParticle = 0;
	int newParticle = 0;
	int otherParticle = -1;
};

struct HostParticles {
	HostParticles(int numParticles) 
		: d_positionRadius(numParticles)
		, d_color(numParticles)
		, d_momentumMass(numParticles)
		, d_deltaValues(numParticles)
		, d_guessedPosition(numParticles)
		, d_portalFlags(numParticles)
	{}

    std::vector<ParticlePositionRadius> d_positionRadius;
    std::vector<ParticleColor> d_color;
    std::vector<ParticleMomentumMass> d_momentumMass;
    std::vector<ParticleDeltaValues> d_deltaValues;
    std::vector<ParticlePositionRadius> d_guessedPosition;
	std::vector<ParticlePortalFlags> d_portalFlags;
};

inline std::ostream& operator<<(std::ostream& os, const HostParticles& particles) {
	int numParticles = particles.d_positionRadius.size();
	for(int i = 0; i < numParticles; ++i) {
		os << "ParticlePosition: [" 
		<< particles.d_positionRadius[i].position.x() << ", "
		<< particles.d_positionRadius[i].position.y() << ", "
		<< particles.d_positionRadius[i].position.z() << "]";
	}
	return os;
}

struct Particles {
    // interop'd OpenGL-Buffers
    ArrayView<ParticlePositionRadius> d_positionRadius;
    ArrayView<ParticleColor> d_color;
    // data buffers
    thrust::device_vector<ParticleMomentumMass> d_momentumMass;
    thrust::device_vector<ParticleDeltaValues> d_deltaValues;
    thrust::device_vector<ParticlePositionRadius> d_guessedPosition;
	thrust::device_vector<ParticlePortalFlags> d_portalFlags;
};



// ######################################################################### //
// ### Rigid Body Particles ################################################ //
// ######################################################################### //

// rigid bodies
struct SAIGA_ALIGN(16) RigidBodyParticleOffset {
    vec3 relativeOffset = {0.f, 0.f, 0.f};
    int bodyIndex = -1;
};

struct SAIGA_ALIGN(16) RigidBodyParticleNormal {
    vec3 normal = {0.f,0.f,0.f};
    int bodyIndex = -1;
};

struct SAIGA_ALIGN(16) RigidBodyParticleData {
    int firstParticleIndex = -1; // the index of the particle in struct Particles
    int numParticles = -1;
};

// TODO: split this up in chunks of 4 bytes each
struct SAIGA_ALIGN(16) RigidBodyOrientation {
    vec3 position = {0.f, 0.f, 0.f};
    float resolution = 1.f;
    quat rotation = quat(0.f,0.f,0.f,0.f);
	vec3 scale = {1.f,1.f,1.f};
	float empty = 0.f;
};

/**
 * state: 0 = original RB, 1 = original with duplicate RB, 2 = duplicated RB
 * switched: set to 1 after dup and original got swapped, as long as center intersects and swapped is set, don't swap again
 * duplicateId: ID of the RB duplikat
 * portalHit: 0 = no hit, 1 = blue portal hit, 2 = orange portal hit
 */

struct SAIGA_ALIGN(16) RigidBodyPortalFlags {
	RigidBodyState state = Regular;
	int switched = 0;
	int duplicateId = -1;
	int portalHit = 0;
};

struct SAIGA_ALIGN(16) RigidBodyAssetHash {
	size_t assetHash = 0;
};

struct HostRigidBodies {
	HostRigidBodies(int numBodies, int numParticles)
		: d_rigidBodiesOrientation(numBodies)
		, d_rigidBodiesParticles(numBodies)
		, d_assetHash(numBodies)
		, d_rigidBodyPortalFlags(numBodies)
		, d_rigidBodiesParticleOffsets(numParticles)
		, d_rigidBodiesParticleNormals(numParticles)
	{}

	// per body attributes
    std::vector<RigidBodyOrientation> d_rigidBodiesOrientation;
    std::vector<RigidBodyParticleData> d_rigidBodiesParticles;
	std::vector<RigidBodyAssetHash> d_assetHash;
	std::vector<RigidBodyPortalFlags> d_rigidBodyPortalFlags;
	// per particle attributes
    std::vector<RigidBodyParticleOffset> d_rigidBodiesParticleOffsets;
    std::vector<RigidBodyParticleNormal> d_rigidBodiesParticleNormals;
};

struct RigidBodies {
	// per body attributes
    // contains the translation and orientation of a rigid body wrt {0,0,0}
    thrust::device_vector<RigidBodyOrientation> d_rigidBodiesOrientation;
    thrust::device_vector<RigidBodyParticleData> d_rigidBodiesParticles;
	thrust::device_vector<RigidBodyAssetHash> d_assetHash;
	thrust::device_vector<RigidBodyPortalFlags> d_rigidBodyPortalFlags;
	// per particle attributes
    thrust::device_vector<RigidBodyParticleOffset> d_rigidBodiesParticleOffsets;
    thrust::device_vector<RigidBodyParticleNormal> d_rigidBodiesParticleNormals;
};


struct RigidBodyData {
	HostParticles particles;
	HostRigidBodies rigidBodies;
};

// ######################################################################### //
// ### Cloth Particles ##################################################### //
// ######################################################################### //

struct SAIGA_ALIGN(16) ClothBendingConstraint {
	int particleID1 = 0;
	int particleID2 = 0;
	int particleID3 = 0;
	int particleID4 = 0;
};

struct SAIGA_ALIGN(16) ClothNeighbors {
	// store the neighbor to the right and the one to the top to avoid resolving twice
	CollisionParticleData rightNeighborCollision;
	CollisionParticleData topNeighborCollision;
};

struct HostCloths {
	HostCloths(int numParticles, int numBendingConstraints)
		: d_clothBendingConstraints(numBendingConstraints)
		, d_clothNeighbors(numParticles)
	{}

	// per cloth attributes
	std::vector<ClothBendingConstraint> d_clothBendingConstraints;
	// per particle attributes
	std::vector<ClothNeighbors> d_clothNeighbors;
};

struct Cloths {
	// per cloth attributes
	thrust::device_vector<ClothBendingConstraint> d_clothBendingConstraints;
	// per particle attributes
	thrust::device_vector<ClothNeighbors> d_clothNeighbors;
};

struct ClothData {
	HostParticles particles;
	HostCloths cloths;
};

// ######################################################################### //
// ### Fluid Particles ##################################################### //
// ######################################################################### //

struct SAIGA_ALIGN(8) FluidLambda {
	float lambda = -1.f;
	int isFluid = 0;
};

struct SAIGA_ALIGN(16) FluidVorticity {
	vec3 voriticity = vec3(0.f,0.f,0.f);
	float density = 0.f;
};

struct SAIGA_ALIGN(16) FluidViscosity {
	vec3 viscosity = vec3(0.f,0.f,0.f);
	float freeStorage;
};

struct HostFluids {
	HostFluids(int numFluids, int numParticles) 
		: d_lambdas(numParticles)
		, d_voriticities(numParticles)
		, d_viscosities(numParticles)
	{}

	// per fluid attributes: None
	// per particle attributes
	std::vector<FluidLambda> d_lambdas;
	std::vector<FluidVorticity> d_voriticities;
	std::vector<FluidViscosity> d_viscosities;
};

struct Fluids {
	// per fluid attributes: None
	// per particle attributes
	thrust::device_vector<FluidLambda> d_lambdas;
	thrust::device_vector<FluidVorticity> d_voriticities;
	thrust::device_vector<FluidViscosity> d_viscosities;
};

struct FluidData {
	HostParticles particles;
	HostFluids fluids;
};

