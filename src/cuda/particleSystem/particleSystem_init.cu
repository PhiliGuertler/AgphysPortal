#include "particleSystem.h"
#include "saiga/cuda/device_helper.h"
#include "saiga/core/util/assert.h"
#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"

#include "saiga/core/math/Eigen_Compile_Checker.h"

#include <saiga/opengl/shader/shaderLoader.h>

#include "../../glBuffers.h"

// ######################################################################### //
// ### CUDA Kernels ######################################################## //
// ######################################################################### //

// ######################################################################### //
// ### Cloth Reference Updates ############################################# //

/**
 *	moves the references of all cloth neighbor constraints by numNewParticles
 */
__global__ void updateClothParticleReferences(ArrayView<ClothNeighbors> neighbors, int numNewParticles, int clothParticlesBegin, int clothParticlesEnd) {
	Saiga::CUDA::ThreadInfo<> ti;
	int particleID = ti.thread_id + clothParticlesBegin;
	if(particleID >= clothParticlesEnd) return;
	
	// load neighborData
	ClothNeighbors neighborData;
	Saiga::CUDA::vectorCopy(neighbors.data()+particleID, &neighborData);

	// move right neighbor collision
	if(neighborData.rightNeighborCollision.particleID1 != -1) {
		neighborData.rightNeighborCollision.particleID1 += numNewParticles;
	}
	if(neighborData.rightNeighborCollision.particleID2 != -1) {
		neighborData.rightNeighborCollision.particleID2 += numNewParticles;
	}

	// move top neighbor collision
	if(neighborData.topNeighborCollision.particleID1 != -1) {
		neighborData.topNeighborCollision.particleID1 += numNewParticles;
	}
	if(neighborData.topNeighborCollision.particleID2 != -1) {
		neighborData.topNeighborCollision.particleID2 += numNewParticles;
	}

	// store result
	Saiga::CUDA::vectorCopy(&neighborData, neighbors.data()+particleID);
}

/**
 *	moves the references of all cloth bending constraints by numNewParticles
 */
__global__ void updateClothBendingParticleReferences(ArrayView<ClothBendingConstraint> bendingConstraints, int numNewParticles, int clothParticlesBegin, int clothParticlesEnd) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= bendingConstraints.size()) return;
	
	// load neighborData
	ClothBendingConstraint bendingConstraint;
	Saiga::CUDA::vectorCopy(bendingConstraints.data()+ti.thread_id, &bendingConstraint);

	if(bendingConstraint.particleID1 != -1) {
		bendingConstraint.particleID1 += numNewParticles;
	}
	if(bendingConstraint.particleID2 != -1) {
		bendingConstraint.particleID2 += numNewParticles;
	}
	if(bendingConstraint.particleID3 != -1) {
		bendingConstraint.particleID3 += numNewParticles;
	}
	if(bendingConstraint.particleID4 != -1) {
		bendingConstraint.particleID4 += numNewParticles;
	}

	// store result
	Saiga::CUDA::vectorCopy(&bendingConstraint, bendingConstraints.data()+ti.thread_id);
}

// ######################################################################### //
// ### ParticleSystem (C++) ################################################ //
// ######################################################################### //

inline void setupTessShader(std::shared_ptr<Saiga::MVPShader>& shader) {
	ImGuiOptions& options = ImGuiOptions::get();

	shader->bind();
	GLint location = shader->getUniformLocation("u_tessLevelOuter");
	shader->upload(location, options.tessLevelOuter);
	location = shader->getUniformLocation("u_tessLevelInner");
	shader->upload(location, options.tessLevelInner);
	location = shader->getUniformLocation("u_tessAlpha");
	shader->upload(location, options.tessAlpha);
	shader->unbind();
}

/**
 *	constructor
 */
ParticleSystem::ParticleSystem(int _particleCount)
	: m_particleCounts{_particleCount, 0}
	, m_firstRigidBodyHasHitSomeWalls(false)
	//, m_duplicatedParticles(0)
{
	initConstantMemory();

	CUDA_SYNC_CHECK_ERROR();
	m_bluePortal = std::make_shared<Portal>();
	m_orangePortal = std::make_shared<Portal>();

}

/**
 *	destructor
 */
ParticleSystem::~ParticleSystem()
{}


// ######################################################################### //
// ### Plane Initialization ################################################ //
// ######################################################################### //

void ParticleSystem::registerPlanes(const std::vector<Saiga::Plane>& planes) {
	d_planes = thrust::device_vector<Saiga::Plane>(planes);
}

void ParticleSystem::registerLevel(const Controls::Level& level) {
	d_tryecke = thrust::device_vector<Tryeck>(level.getTryeckList());
}
void ParticleSystem::registerWallOverlay(const Controls::Level& level) {
	d_tryeckeWall = thrust::device_vector<Tryeck>(level.getTryeckList());
}


// ######################################################################### //
// ### Particle Initialization ############################################# //
// ######################################################################### //

void ParticleSystem::setParticleColors(void *colorPtr) {
	m_particles.d_color = ArrayView<ParticleColor>((ParticleColor*) colorPtr, m_particleCounts.sum());
}

void ParticleSystem::setParticlePositions(void *positionPtr) {
	m_particles.d_positionRadius = ArrayView<ParticlePositionRadius>((ParticlePositionRadius*) positionPtr, m_particleCounts.sum());
}


__global__ void initParticles(ArrayView<ParticlePositionRadius> particlePositions
		, ArrayView<ParticleMomentumMass> velos
		, ArrayView<ParticleDeltaValues> deltas
		, ArrayView<ParticlePositionRadius> particleGuessedPositions
		, int x
		, int y
		, int z
		, vec3 corner
		, float distance
		, float radius
		, float massinv) {
	const float EPSTEIN = 0.001f;

	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePositions.size()) return;

	ParticlePositionRadius& pos = particlePositions[ti.thread_id];
	ParticleMomentumMass& velo = velos[ti.thread_id];
	ParticleDeltaValues& delta = deltas[ti.thread_id];
	ParticlePositionRadius& guessedPosition = particleGuessedPositions[ti.thread_id];

	int particleY = (ti.thread_id / (x * z));
	float offset = ((particleY % 2 == 0) ? 1.f : -1.f) * EPSTEIN;
	pos.position[0] = corner[0] + ((ti.thread_id % x) * distance) + offset;
	pos.position[1] = corner[1] + ((ti.thread_id / (x * z)) * distance);
	pos.position[2] = corner[2] + (((ti.thread_id / x) % z) * distance) + offset;
	pos.radius = radius;

	velo.momentumVelocity = {0.f,0.f,0.f};
	delta.delta = {0.f,0.f,0.f};

	guessedPosition.position = pos.position;
	guessedPosition.radius = pos.radius;

	velo.massinv = massinv;
}


// ######################################################################### //
// ### Particle Memory Layout Functions #################################### //

int ParticleSystem::particlesBegin(ParticleType type) {
	int beginIndex = 0;

	if(type == ParticleType::RigidBodyParticle) {
		return beginIndex;
	}
	beginIndex += m_particleCounts.rigidBodyParticles;

	if(type == ParticleType::ClothParticle) {
		return beginIndex;
	}
	beginIndex += m_particleCounts.clothParticles;

	if(type == ParticleType::FluidParticle) {
		return beginIndex;
	}
	beginIndex += m_particleCounts.fluidParticles;

	// other particle types go here

	if(type == ParticleType::RegularParticle) {
		return beginIndex;
	}
	beginIndex += m_particleCounts.regularParticles;


	std::cerr << "Unknown ParticleType! [" << __FUNCTION__ << "]" << std::endl;
	return beginIndex;
}

int ParticleSystem::particlesEnd(ParticleType type) {
	int endIndex = 0;

	endIndex += m_particleCounts.rigidBodyParticles;
	if(type == ParticleType::RigidBodyParticle) {
		return endIndex;
	}

	endIndex += m_particleCounts.clothParticles;
	if(type == ParticleType::ClothParticle) {
		return endIndex;
	}

	endIndex += m_particleCounts.fluidParticles;
	if(type == ParticleType::FluidParticle) {
		return endIndex;
	}

	// other particle types go here

	endIndex += m_particleCounts.regularParticles;
	if(type == ParticleType::RegularParticle) {
		return endIndex;
	}

	std::cerr << "Unknown ParticleType! [" << __FUNCTION__ << "]" << std::endl;
	return endIndex;
}

int ParticleSystem::getNumParticles(ParticleType type) {
	switch(type) {
	case ParticleType::RegularParticle:
		return m_particleCounts.regularParticles;
	case ParticleType::RigidBodyParticle:
		return m_particleCounts.rigidBodyParticles;
	case ParticleType::ClothParticle:
		return m_particleCounts.clothParticles;
	case ParticleType::FluidParticle:
		return m_particleCounts.fluidParticles;
	// other particle types go here
	default:
		std::cerr << "Unknown ParticleType! [" << __FUNCTION__ << "]" << std::endl;
	}
	return -1;
}

void ParticleSystem::addParticlesToCount(ParticleType type, int amount) {
	switch(type) {
	case ParticleType::RegularParticle:
		m_particleCounts.regularParticles += amount;
		break;
	case ParticleType::RigidBodyParticle:
		m_particleCounts.rigidBodyParticles += amount;
		break;
	case ParticleType::ClothParticle:
		m_particleCounts.clothParticles += amount;
		break;
	case ParticleType::FluidParticle:
		m_particleCounts.fluidParticles += amount;
		break;
	default:
		std::cerr << "Unknown ParticleType! [" << __FUNCTION__ << "]" << std::endl;
	}
}


// ######################################################################### //
// ### Particle Spawning ################################################### //

template<typename T>
inline thrust::host_vector<T> updateArrayView(ArrayView<T>& target, const std::vector<T>& source, int insertionIndex) {
	// copy the data of particlePositions to the host
	thrust::host_vector<T> helper(target.size());
	thrust::copy(target.device_begin(), target.device_end(), helper.begin());
	// append new data
	helper.insert(helper.begin()+insertionIndex, source.begin(), source.end());

	return helper;
}


/**
 *	spawns particles
 */
void ParticleSystem::spawnParticles(const HostParticles& particles, ParticleType type) {
	if(particles.d_positionRadius.size() < 1) return;

	// insert the particles at the end of a particle range depending on the particle type
	int insertionIndex = particlesEnd(type);
	int numNewParticles = particles.d_momentumMass.size();

	int clothParticlesBegin = particlesBegin(ParticleType::ClothParticle);
	int clothParticlesEnd = particlesEnd(ParticleType::ClothParticle);
	// update particle counts
	addParticlesToCount(type, numNewParticles);

	// append data to all non-ArrayViews
	// --- Regular Particle Data --- //
	m_particles.d_momentumMass.insert(m_particles.d_momentumMass.begin()+insertionIndex, particles.d_momentumMass.begin(), particles.d_momentumMass.end());
	m_particles.d_deltaValues.insert(m_particles.d_deltaValues.begin()+insertionIndex, particles.d_deltaValues.begin(), particles.d_deltaValues.end());
	m_particles.d_guessedPosition.insert(m_particles.d_guessedPosition.begin()+insertionIndex, particles.d_guessedPosition.begin(), particles.d_guessedPosition.end());
	m_particles.d_portalFlags.insert(m_particles.d_portalFlags.begin()+insertionIndex, particles.d_portalFlags.begin(), particles.d_portalFlags.end());

	// --- Resize Rigid Body Particle Data --- //
	std::vector<RigidBodyParticleOffset> dummyOffsetData(numNewParticles);
	m_rigidBodies.d_rigidBodiesParticleOffsets.insert(m_rigidBodies.d_rigidBodiesParticleOffsets.begin()+insertionIndex, dummyOffsetData.begin(), dummyOffsetData.end());
	std::vector<RigidBodyParticleNormal> dummyNormalData(numNewParticles);
	m_rigidBodies.d_rigidBodiesParticleNormals.insert(m_rigidBodies.d_rigidBodiesParticleNormals.begin()+insertionIndex, dummyNormalData.begin(), dummyNormalData.end());

	// --- Resize Fluid Particle Data --- //
	std::vector<FluidLambda> dummyLambdaData(numNewParticles);
	m_fluids.d_lambdas.insert(m_fluids.d_lambdas.begin()+insertionIndex, dummyLambdaData.begin(), dummyLambdaData.end());
	std::vector<FluidVorticity> dummyVorticities(numNewParticles);
	m_fluids.d_voriticities.insert(m_fluids.d_voriticities.begin()+insertionIndex, dummyVorticities.begin(), dummyVorticities.end());
	std::vector<FluidViscosity> dummyViscosities(numNewParticles);
	m_fluids.d_viscosities.insert(m_fluids.d_viscosities.begin()+insertionIndex, dummyViscosities.begin(), dummyViscosities.end());

	if(type == ParticleType::RigidBodyParticle) {
		if(getNumParticles(ParticleType::ClothParticle) > 0) {
			std::cout << "NumNewParticles: " << numNewParticles << ", clothParticlesBegin: " << clothParticlesBegin << ", clothParticlesEnd: " << clothParticlesEnd << ", numClothParticles: " << getNumParticles(ParticleType::ClothParticle) << std::endl;
			updateClothParticleReferences<<<THREAD_BLOCK(getNumParticles(ParticleType::ClothParticle), BLOCK_SIZE)>>>(m_cloths.d_clothNeighbors, numNewParticles, clothParticlesBegin, clothParticlesEnd);
			updateClothBendingParticleReferences<<<THREAD_BLOCK(m_cloths.d_clothBendingConstraints.size(), BLOCK_SIZE)>>>(m_cloths.d_clothBendingConstraints, numNewParticles, clothParticlesBegin, clothParticlesEnd);
		}
	}

	// --- Resize Cloth Particle Data --- //
	std::vector<ClothNeighbors> dummyNeighborData(numNewParticles);
	m_cloths.d_clothNeighbors.insert(m_cloths.d_clothNeighbors.begin()+insertionIndex, dummyNeighborData.begin(), dummyNeighborData.end());

	// --- Update ArrayViews of interop'd gl-buffers --- //
	GLBufferManager& bufferManager = GLBufferManager::get();

	// update the positionRadius ArrayView
	auto updatedBuffer = updateArrayView<ParticlePositionRadius>(m_particles.d_positionRadius, particles.d_positionRadius, insertionIndex);
	// update the color ArrayView
	auto updatedColorBuffer = updateArrayView<ParticleColor>(m_particles.d_color, particles.d_color, insertionIndex);

	bufferManager.resetInteropReferences();

	// update the positionRadius vertex buffer
	bufferManager.getParticlePositions().set(updatedBuffer, GL_DYNAMIC_DRAW);
	// update the color vertex buffer
	bufferManager.getParticleColors().set(updatedColorBuffer, GL_DYNAMIC_DRAW);

	// update interops
	bufferManager.updateInteropReferences();
	bufferManager.map();
	bufferManager.unmap();
}


// ######################################################################### //
// ### Rigid Body Initialization ########################################### //
// ######################################################################### //

// ######################################################################### //
// ### Rigid Body Spawning ################################################# //

void ParticleSystem::spawnRigidBodies(const RigidBodyData& data) {
	if(data.particles.d_positionRadius.size() < 1) return;

	int insertionIndex = particlesEnd(ParticleType::RigidBodyParticle);

	// first, spawn the needed particles
	spawnParticles(data.particles, ParticleType::RigidBodyParticle);

	// append new rigid bodies at the end of the rigid body buffers
	m_rigidBodies.d_rigidBodiesOrientation.insert(m_rigidBodies.d_rigidBodiesOrientation.end(), data.rigidBodies.d_rigidBodiesOrientation.begin(), data.rigidBodies.d_rigidBodiesOrientation.end());
	m_rigidBodies.d_rigidBodiesParticles.insert(m_rigidBodies.d_rigidBodiesParticles.end(), data.rigidBodies.d_rigidBodiesParticles.begin(), data.rigidBodies.d_rigidBodiesParticles.end());
	m_rigidBodies.d_assetHash.insert(m_rigidBodies.d_assetHash.end(), data.rigidBodies.d_assetHash.begin(), data.rigidBodies.d_assetHash.end());
	m_rigidBodies.d_rigidBodyPortalFlags.insert(m_rigidBodies.d_rigidBodyPortalFlags.end(), data.rigidBodies.d_rigidBodyPortalFlags.begin(), data.rigidBodies.d_rigidBodyPortalFlags.end());

	// append the rigid body particle bonus data.
	// These have already been resized by spawnParticles(), so the new data must be copied into the buffers with an offset
	thrust::host_vector<RigidBodyParticleOffset> offsetData = data.rigidBodies.d_rigidBodiesParticleOffsets;
	thrust::copy(offsetData.begin(), offsetData.end(), m_rigidBodies.d_rigidBodiesParticleOffsets.begin()+insertionIndex);

	thrust::host_vector<RigidBodyParticleNormal> normalData = data.rigidBodies.d_rigidBodiesParticleNormals;
	thrust::copy(normalData.begin(), normalData.end(), m_rigidBodies.d_rigidBodiesParticleNormals.begin()+insertionIndex);

	m_particleCounts.numRigidBodies += data.rigidBodies.d_rigidBodiesOrientation.size();

	initRigidCubes();
}


// ######################################################################### //
// ### Cloth Initialization ################################################ //
// ######################################################################### //

// ######################################################################### //
// ### Cloth Spawning ###################################################### //

void ParticleSystem::spawnCloth(const ClothData& data) {
	if(data.particles.d_positionRadius.size() < 1) return;

	int insertionIndex = particlesEnd(ParticleType::ClothParticle);

	// first spawn the needed particles
	spawnParticles(data.particles, ParticleType::ClothParticle);

	// insert the clothdata in the already resized m_cloths.d_clothNeighbors device vector
	thrust::host_vector<ClothNeighbors> neighborData = data.cloths.d_clothNeighbors;
	thrust::copy(neighborData.begin(), neighborData.end(), m_cloths.d_clothNeighbors.begin()+insertionIndex);

	// insert clothBendingConstraints
	m_cloths.d_clothBendingConstraints.insert(m_cloths.d_clothBendingConstraints.end(), data.cloths.d_clothBendingConstraints.begin(), data.cloths.d_clothBendingConstraints.end());
}


// ######################################################################### //
// ### Fluid Initialization ################################################ //
// ######################################################################### //

// ######################################################################### //
// ### Fluid Spawning ###################################################### //

void ParticleSystem::spawnFluid(const FluidData& data) {
	if(data.particles.d_positionRadius.size() < 1) return;

	int insertionIndex = particlesEnd(ParticleType::FluidParticle);

	// first spawn the needed particles
	spawnParticles(data.particles, ParticleType::FluidParticle);

	// insert the fluid data in the already resized
	thrust::host_vector<FluidLambda> lambdaData = data.fluids.d_lambdas;
	thrust::copy(lambdaData.begin(), lambdaData.end(), m_fluids.d_lambdas.begin()+insertionIndex);

	thrust::host_vector<FluidVorticity> vorticityData = data.fluids.d_voriticities;
	thrust::copy(vorticityData.begin(), vorticityData.end(), m_fluids.d_voriticities.begin()+insertionIndex);

	thrust::host_vector<FluidViscosity> viscosityData = data.fluids.d_viscosities;
	thrust::copy(viscosityData.begin(), viscosityData.end(), m_fluids.d_viscosities.begin()+insertionIndex);
}


// ######################################################################### //
// ### Resetting ########################################################### //

void ParticleSystem::resetParticles(int x, int z, const vec3& corner, float distance) {
	// compute the y direction
	ImGuiOptions& options = ImGuiOptions::get();

	float yFloat = static_cast<float>(m_particleCounts.sum()) / static_cast<float>(x * z);
	int y = std::ceil(yFloat);

	if(m_particleCounts.sum() > 0) {
		initParticles<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_momentumMass, m_particles.d_deltaValues, m_particles.d_guessedPosition, x, y, z, corner, distance, options.radiusReset, options.massinv);
	}
}
