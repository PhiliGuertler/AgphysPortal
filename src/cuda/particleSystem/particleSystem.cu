#include "particleSystem.h"
#include "saiga/core/util/assert.h"
#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"

#include "particleSystem_collisions.inl"
#include "rigidBodyManager.h"

// ### Particlesystem submodules ### //
#include "particleSystem_mouseInteraction.h"
#include "particleSystem_collisionResolver.h"
#include "particleSystem_rigidBodies.h"
#include "particleSystem_cloths.h"
#include "particleSystem_fluids.h"
#include "particleSystem_stabilization.h"
// ### /Particlesystem submodules ### //

#include <thrust/gather.h>
#include <thrust/sort.h>
#include <algorithm> 

#include "../svd/svd3_wrapper.h"
#include "../../glBuffers.h"

#include "../../profiling/Profiler.h"


// ######################################################################### //
// ### CUDA Kernels ######################################################## //
// ######################################################################### //

__global__ void fillMeUpScotty(ArrayView<int> indices) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= indices.size()) return;

	indices[ti.thread_id] = ti.thread_id;
}

// ######################################################################### //
// ### Integration ######################################################### //

__global__ void positionBasedIntegration(ArrayView<ParticlePositionRadius> particlePositions
										, ArrayView<ParticleMomentumMass> velos
										, ArrayView<ParticlePositionRadius> particleGuessedPositions
										, vec3 acceleration
										, float dt) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePositions.size()) return;

	// load data from memory
	ParticlePositionRadius position;
	Saiga::CUDA::vectorCopy(particlePositions.data()+ti.thread_id, &position);

	ParticleMomentumMass velo;
	Saiga::CUDA::vectorCopy(velos.data()+ti.thread_id, &velo);

	ParticlePositionRadius guessedPosition;
	Saiga::CUDA::vectorCopy(particleGuessedPositions.data()+ti.thread_id, &guessedPosition);

	// step 1: integrate velocities
	// acceleration is actually a momentum, which is force/mass, in this case it is the gravity
	if(velo.massinv > 0) {
		velo.momentumVelocity += dt * acceleration;
	}

	// step 2: guess new position
	guessedPosition.position = position.position + dt * velo.momentumVelocity;

	// write back to memory
	Saiga::CUDA::vectorCopy(&velo, velos.data()+ti.thread_id);
	Saiga::CUDA::vectorCopy(&guessedPosition, particleGuessedPositions.data()+ti.thread_id);
}

__global__ void positionBasedVelocityUpdate(ArrayView<ParticlePositionRadius> particlePositions
		, ArrayView<ParticlePositionRadius> particlePositionsGuessed
		, ArrayView<ParticleMomentumMass> velos
		, float dt) {

	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePositions.size()) return;

	// load data from memory
	ParticlePositionRadius pos;
	Saiga::CUDA::vectorCopy(particlePositions.data()+ti.thread_id, &pos);

	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(particlePositionsGuessed.data()+ti.thread_id, &guessedPos);

	ParticleMomentumMass velo;
	Saiga::CUDA::vectorCopy(velos.data()+ti.thread_id, &velo);

	// update velocity
	velo.momentumVelocity = (guessedPos.position - pos.position) / dt;

	// write back
	Saiga::CUDA::vectorCopy(&velo, velos.data()+ti.thread_id);
}

__global__ void positionBasedPositionUpdate(ArrayView<ParticlePositionRadius> particlePositions
		, ArrayView<ParticlePositionRadius> particlePositionsGuessed
		, float dt) {

	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePositions.size()) return;

	// load data from memory
	ParticlePositionRadius pos;
	Saiga::CUDA::vectorCopy(particlePositions.data()+ti.thread_id, &pos);

	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(particlePositionsGuessed.data()+ti.thread_id, &guessedPos);

	// update position
	pos.position = guessedPos.position;

	// write back
	Saiga::CUDA::vectorCopy(&pos, particlePositions.data()+ti.thread_id);
}

// ######################################################################### //
// ### Resolvers of position based dynamics using Collision Constraints #### //

__global__ void updatePositionsGPU(ArrayView<ParticlePositionRadius> particlesGuessed
							  , ArrayView<ParticleDeltaValues> particleDeltas
							  , float dt) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlesGuessed.size()) return;

	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(particlesGuessed.data()+ti.thread_id, &guessedPos);

	ParticleDeltaValues delta;
	Saiga::CUDA::vectorCopy(particleDeltas.data()+ti.thread_id, &delta);

	// update the guessed position
	guessedPos.position += delta.delta;
	delta.delta = {0.f,0.f,0.f};

	Saiga::CUDA::vectorCopy(&guessedPos, particlesGuessed.data()+ti.thread_id);
	Saiga::CUDA::vectorCopy(&delta, particleDeltas.data()+ti.thread_id);
}

__global__ void printPosition(ArrayView<ParticlePositionRadius> particlePositions) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePositions.size()) return;

	ParticlePositionRadius ownPosition;
	Saiga::CUDA::vectorCopy(particlePositions.data()+ti.thread_id, &ownPosition);

	printf("pos:(%f,%f,%f) \n",ownPosition.position[0],ownPosition.position[1],ownPosition.position[2]);
}

// ######################################################################### //
// ### Updated rigid body duplicated position ############################## //
__global__ void updatePostionDuplicatedRigidBodys(ArrayView<ParticlePositionRadius> particlePositions
												, ArrayView<ParticlePortalFlags> portalFlags
												, ArrayView<ParticleMomentumMass> particleMomentumMass
												, ArrayView<RigidBodyPortalFlags> rigidBodyPortalFlags
												, ArrayView<RigidBodyParticleOffset> rigidBodyParticleOffset	
												, Portal portalOranje
												, Portal portalBleu) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePositions.size()) return;

	RigidBodyParticleOffset particleOffset;
	Saiga::CUDA::vectorCopy(rigidBodyParticleOffset.data()+ti.thread_id, &particleOffset);
	
	// non rigid body particles do nothing
	if(particleOffset.bodyIndex == -1) {
		return;
	} 

	RigidBodyPortalFlags rbPortalFlags;
	Saiga::CUDA::vectorCopy(rigidBodyPortalFlags.data()+particleOffset.bodyIndex, &rbPortalFlags);
	if(rbPortalFlags.state == 0 || rbPortalFlags.state == 2) {
		return;
	}

	ParticlePositionRadius ownPosition;
	Saiga::CUDA::vectorCopy(particlePositions.data()+ti.thread_id, &ownPosition);

	ParticlePortalFlags ownPortalFlags;
	Saiga::CUDA::vectorCopy(portalFlags.data()+ti.thread_id, &ownPortalFlags);

	ParticleMomentumMass ownMomentum;
	Saiga::CUDA::vectorCopy(particleMomentumMass.data()+ti.thread_id, &ownMomentum);

	ParticlePositionRadius duplicatedPosition;
	Saiga::CUDA::vectorCopy(particlePositions.data()+ownPortalFlags.otherParticle, &duplicatedPosition);

	ParticleMomentumMass duplicatedMomentum;
	Saiga::CUDA::vectorCopy(particleMomentumMass.data()+ownPortalFlags.otherParticle, &duplicatedMomentum);

#define VELC(a) a.x(), a.y(), a.z()

	vec3 posNew;
	vec3 newVelocity;
	if(rbPortalFlags.portalHit == 1) {
		// blue portal hit
		posNew = Portal::transformVector(vec4(VELC(ownPosition.position), 1), portalBleu, portalOranje);
		newVelocity = Portal::transformVector(vec4(VELC(ownMomentum.momentumVelocity), 1), portalBleu, portalOranje);
	} else {
		// orange portal hit
		posNew = Portal::transformVector(vec4(VELC(ownPosition.position), 1), portalOranje, portalBleu);
		newVelocity = Portal::transformVector(vec4(VELC(ownMomentum.momentumVelocity), 1), portalOranje, portalBleu);
	}

	duplicatedPosition.position = posNew;
	duplicatedMomentum.momentumVelocity = newVelocity;
	
	Saiga::CUDA::vectorCopy(&duplicatedPosition, particlePositions.data()+ownPortalFlags.otherParticle);
	Saiga::CUDA::vectorCopy(&duplicatedMomentum, particleMomentumMass.data()+ownPortalFlags.otherParticle);
	
}

// ######################################################################### //
// ### Linked Cell Kernels ################################################# //

__global__
void createLinkedCellStructure(ArrayView<ParticlePositionRadius> particlePositions, ArrayView<int> hashBuckets, ArrayView<int> hashOverflow, ArrayView<int> numParticlesPerBucket, float cellLength) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePositions.size()) return;

	ParticlePositionRadius particlePosition;
	Saiga::CUDA::vectorCopy(particlePositions.data()+ti.thread_id, &particlePosition);

    int hash = hashMe(particlePosition.position, cellLength, hashBuckets.size());

	hashOverflow[ti.thread_id] = atomicExch(&hashBuckets[hash], ti.thread_id);

	// add 1 to the number of particles in this bucket
	atomicAdd(&numParticlesPerBucket[hash], 1);
}

template <int NumCollisions>
__global__ void collideParticlesWithLinkedCell(ArrayView<ParticlePositionRadius> particleGuessedPositions
												, ArrayView<int> hashBuckets
												, ArrayView<int> hashOverflow
												, ArrayView<CollisionParticleData> collisionParticleData
												, ArrayView<FluidLambda> fluidLambdas
												, float cellLength) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particleGuessedPositions.size()) return;

	ParticlePositionRadius guessedPosition;
	Saiga::CUDA::vectorCopy(particleGuessedPositions.data()+ti.thread_id, &guessedPosition);

	FluidLambda fluidLambda;
	Saiga::CUDA::vectorCopy<FluidLambda, int2>(fluidLambdas.data()+ti.thread_id, &fluidLambda);

	// create a local list of collisions with other particles
	int currentIndex = 0;

	// update global array
	CollisionParticleData invalid = CollisionParticleData();
	for(int i = NumCollisions-1; i >= currentIndex; --i) {
		Saiga::CUDA::vectorCopy<CollisionParticleData, int2>(&invalid, collisionParticleData.data()+ti.thread_id*NumCollisions+i);
	}

    int ownHash = hashMe(guessedPosition.position, cellLength, hashBuckets.size());

	// TODO: increase performance
	// TODO: make this constant memory
	const vec3 collisionOffsets[13] = {
		vec3(-1.f,-1.f,-1.f),
		vec3(0.f,-1.f,-1.f),
		vec3(1.f,-1.f,-1.f),
		vec3(-1.f,0.f,-1.f),
		vec3(0.f,0.f,-1.f),
		vec3(1.f,0.f,-1.f),
		vec3(-1.f,1.f,-1.f),
		vec3(0.f,1.f,-1.f),
		vec3(1.f,1.f,-1.f),
		vec3(-1.f,-1.f,0.f),
		vec3(0.f,-1.f,0.f),
		vec3(1.f,-1.f,0.f),
		vec3(-1.f,0.f,0.f)
	};
	

	// iterate over the own bucket
	for(int otherParticleID = hashBuckets[ownHash]; otherParticleID != -1; otherParticleID = hashOverflow[otherParticleID]) {
		if(otherParticleID == ti.thread_id) {
			// Special case for the cell of the original particle: only check until the particle gets to itself.
			// This way, every collision will be resolved only once in this cell.
			break;
		}
		// if one or both are fluids, return
		FluidLambda fluidLambdaOther;
		Saiga::CUDA::vectorCopy<FluidLambda, int2>(fluidLambdas.data()+otherParticleID, &fluidLambdaOther);
		if(fluidLambdaOther.isFluid == 1 && fluidLambda.isFluid == 1) {
			// fluid-fluid collision must be enabled
		} else if(fluidLambdaOther.isFluid == 1 || fluidLambda.isFluid == 1) {
			// don't collide fluid particles with other particles
			//continue;
		}

		ParticlePositionRadius guessedPositionOther;
		Saiga::CUDA::vectorCopy(particleGuessedPositions.data()+otherParticleID,&guessedPositionOther);
		
		float distance = collideSphereSphere(guessedPosition, guessedPositionOther);
		if(distance <= 0.f) {
			// collision! add collision to the local array
			CollisionParticleData c;
			c.particleID1 = ti.thread_id;
			c.particleID2 = otherParticleID;
			collisionParticleData[ti.thread_id * NumCollisions + currentIndex] = c;
			
			++currentIndex;
			if (currentIndex == NumCollisions) return;
		}
	}	

	// check for collisions with all particles in the corresponding buckets
	#pragma unroll
	for(int i = 0; i < 13; ++i) {
		// get the hash of this cell
        int hash = hashMe(guessedPosition.position + (collisionOffsets[i]*cellLength), cellLength, hashBuckets.size());

		if(hash == ownHash) {
			// skip this bucket if it maps to the same as the original cell of this particle
			// this should not happen, but it does...
			// FIXME
			continue;
		}

		// iterate over the bucket
		for(int otherParticleID = hashBuckets[hash]; otherParticleID != -1; otherParticleID = hashOverflow[otherParticleID]) {
			ParticlePositionRadius guessedPositionOther;
			Saiga::CUDA::vectorCopy(particleGuessedPositions.data()+otherParticleID,&guessedPositionOther);
			
			FluidLambda fluidLambdaOther;
			Saiga::CUDA::vectorCopy<FluidLambda, int2>(fluidLambdas.data()+otherParticleID, &fluidLambdaOther);
			if(fluidLambdaOther.isFluid == 1 && fluidLambda.isFluid == 1) {
				// fluid-fluid collision must be enabled
			} else if(fluidLambdaOther.isFluid == 1 || fluidLambda.isFluid == 1) {
				// don't collide fluid particles with other particles
				//continue;
			}

			float distance = collideSphereSphere(guessedPosition, guessedPositionOther);

			if(distance <= 0.f) {
				// collision! add collision to the local array
				CollisionParticleData c;
				c.particleID1 = ti.thread_id;
				c.particleID2 = otherParticleID;
				collisionParticleData[ti.thread_id * NumCollisions + currentIndex] = c;
				
				++currentIndex;
				if (currentIndex == NumCollisions) return;
			}
		}		
	}

}

// ######################################################################### //
// ### C++ ParticleSystem ################################################## //
// ######################################################################### //

void ParticleSystem::update(float dt, Controls::Player& player)
{

	if(m_particleCounts.sum() < 1) return;

	CUDA_SYNC_CHECK_ERROR();

	PROFILE_FUNCTION();

	ImGuiOptions& options = ImGuiOptions::get();
	
	switch(options.physicsImplementation) {
	case PhysicsImplementation::ForceBasedBruteForce:
		forceBasedBruteForce(dt);
		break;
	case PhysicsImplementation::ForceBasedCollisionResolver:
		forceBasedCollisionConstraintsHack(dt);
		break;
	case PhysicsImplementation::WithRigidBodies:
		try {
			positionBasedRigid<10>(dt, player);
		} catch(int& i) {
			throw i;
		}
		break;
	default:
		std::cout << "This Implementation is unknown!" << std::endl;
	}
}

/**
 *	reorders buffer 'target' by gathering over the Indexbuffer 'indices'
 */
template <typename T, typename I>
inline void adjustBuffer(thrust::device_vector<T>& target, thrust::device_vector<I>& indices, int beginIndex) {
	PROFILE_FUNCTION();

	if(indices.size() < 2) return;
	thrust::device_vector<T> helper(indices.size());
	thrust::gather(indices.begin(), indices.end(), target.begin()+beginIndex, helper.begin());
	thrust::copy(helper.begin(), helper.end(), target.begin()+beginIndex);
}

/**
 *	reorders ArrayView 'target' by gathering over the Indexbuffer 'indices'
 */
template <typename T, typename I>
inline void adjustArrayView(ArrayView<T>& target, thrust::device_vector<I>& indices, int beginIndex) {
	PROFILE_FUNCTION();

	if(indices.size() < 2) return;
	thrust::device_vector<T> helper(indices.size());
	thrust::gather(indices.begin(), indices.end(), target.device_begin()+beginIndex, helper.begin());
	thrust::copy(helper.begin(), helper.end(), target.device_begin()+beginIndex);
}

/**
 *	Sorts all particles of a given type by their hashed position
 *	Calling this on cloths or rigid bodies will render them broken
 */
void ParticleSystem::sortParticles(ParticleType type, float cellLength) {
	PROFILE_FUNCTION();

	int numParticles = getNumParticles(type);
	thrust::device_vector<int> indices(numParticles);
	// fill with 0 to n
	fillMeUpScotty<<<THREAD_BLOCK(numParticles, BLOCK_SIZE)>>>(indices);

	int beginIndex = particlesBegin(type);
	int endIndex = particlesEnd(type);
	// sort the indices array while sorting the guessed positions
    thrust::sort_by_key(m_particles.d_guessedPosition.begin()+beginIndex, m_particles.d_guessedPosition.begin()+endIndex, indices.begin(), SortParticle(cellLength, d_hashBuckets.size()));

	// sort all other particle attributes as well
	adjustBuffer<ParticleMomentumMass, int>(m_particles.d_momentumMass, indices, beginIndex);
	adjustBuffer<ParticleDeltaValues, int>(m_particles.d_deltaValues, indices, beginIndex);
	adjustBuffer<ParticlePortalFlags, int>(m_particles.d_portalFlags, indices, beginIndex);
	
	adjustArrayView<ParticlePositionRadius, int>(m_particles.d_positionRadius, indices, beginIndex);
	adjustArrayView<ParticleColor, int>(m_particles.d_color, indices, beginIndex);
}

std::vector<std::pair<matrix4, size_t>> ParticleSystem::getModelMatricesOfBodies() {
	PROFILE_FUNCTION();

	// copy cpu data to host
	thrust::host_vector<RigidBodyOrientation> helper(m_rigidBodies.d_rigidBodiesOrientation.size());
	thrust::copy(m_rigidBodies.d_rigidBodiesOrientation.begin(), m_rigidBodies.d_rigidBodiesOrientation.end(), helper.begin());
	
	thrust::host_vector<RigidBodyAssetHash> hashes(m_rigidBodies.d_assetHash.size());
	thrust::copy(m_rigidBodies.d_assetHash.begin(), m_rigidBodies.d_assetHash.end(), hashes.begin());
	
	std::vector<std::pair<matrix4, size_t>> result(hashes.size());

#pragma omp parallel for
	for(int i = 0; i < helper.size(); ++i) {
		RigidBodyOrientation& orient = helper[i];
		vec4 translation = vec4(orient.position.x(), orient.position.y(), orient.position.z(), 1.f);
		vec4 scale = vec4(orient.scale.x(), orient.scale.y(), orient.scale.z(), 1.f);
		matrix4 model = createTRSmatrix(translation, orient.rotation, scale);

		std::pair<matrix4, size_t> entry;
		entry.first = model;
		entry.second = hashes[i].assetHash;

		result[i] = entry;
	}

	return result;
}

void ParticleSystem::resolvePlanesStep(float dt, int planeCollisionLength) {
	PROFILE_FUNCTION();

	ImGuiOptions& options = ImGuiOptions::get();
	if(planeCollisionLength > 0 && options.resolvePlanes) {
		resolveCollisionConstraintsPositionBasedWithFriction<<<THREAD_BLOCK(planeCollisionLength, BLOCK_SIZE)>>>(m_particles.d_guessedPosition
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
}

void ParticleSystem::resolveTryeckStep(float dt, int tryeckCollisionLength) {
	PROFILE_FUNCTION();

	ImGuiOptions& options = ImGuiOptions::get();
	if(tryeckCollisionLength > 0 && options.resolvePlanes) {
		resolveTriangleCollisions<<<THREAD_BLOCK(tryeckCollisionLength, BLOCK_SIZE)>>>(m_particles.d_guessedPosition
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

	if(m_blueSet && m_orangeSet) {
		intersectPortalEdges<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius
				, m_particles.d_guessedPosition
				, m_particles.d_portalFlags
				, m_rigidBodies.d_rigidBodiesParticleOffsets
				, m_rigidBodies.d_rigidBodyPortalFlags
				, m_particles.d_deltaValues
				, m_particles.d_momentumMass
				, particlesBegin(ParticleType::FluidParticle)
				, particlesEnd(ParticleType::FluidParticle)
				, options.enableFriction
				, options.kineticFriction
				, options.staticFriction
				, *m_bluePortal
				, *m_orangePortal
				, options.edgeWidth);
	}
}

RayTriangleCollision ParticleSystem::getTriangleFromIntersection(ParticlePositionRadius particle, vec3 movementDirection){
	if(d_tryeckeWall.size() < 1) {
		RayTriangleCollision hit;
		hit.hit = false;
		return hit;
	}

	resolveTriangleRayCollision<<<THREAD_BLOCK(d_tryeckeWall.size(), BLOCK_SIZE)>>>(particle,movementDirection,d_collisionTryecke,d_tryeckeWall);
	vec3 minHit = vec3(FLT_MAX,FLT_MAX,FLT_MAX);
	float minValue = FLT_MAX;
	float minValue2 = FLT_MAX;
	Tryeck tmp0 = d_tryeckeWall[1];
	vec3 tryNormal = tmp0.normal();
	RayTriangleCollision result;
	result.hit = false;
	for(int i = 0; i< d_collisionTryecke.size();i++){
		Tryeck tmp = d_tryeckeWall[i];
		vec3 hitTryeck = d_collisionTryecke[i];
		if(hitTryeck.x() != hitTryeck.x())
			continue;
		vec3 dir = (particle.position- hitTryeck);
		float distance = abs((particle.position- hitTryeck).norm());
		float dott = dir.dot(movementDirection);
		if(dott < 0.f)
			continue;
		if(distance < minValue && distance < minValue2){
			minHit =  hitTryeck;
			minValue = distance;
			tryNormal = tmp.normal();
			result.hit = true;
		}
	}
	result.normal = tryNormal;
	result.hitPoint = minHit;
	return result;
}
void ParticleSystem::resolveParticleStep(float dt, int particleCollisionLength) {
	PROFILE_FUNCTION();

	ImGuiOptions& options = ImGuiOptions::get();
	if(particleCollisionLength > 0 && options.resolveParticles) {
		if(options.resolveWithSDF) {
			resolveCollisionConstraintsPositionBasedWithFriction<<<THREAD_BLOCK(particleCollisionLength, BLOCK_SIZE)>>>(m_particles.d_guessedPosition
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
}

void ParticleSystem::updatePositions(float dt) {
	updatePositionsGPU<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition, m_particles.d_deltaValues, dt);
}

template <int NumCollisions>
void ParticleSystem::positionBasedRigid(float dt, Controls::Player& player) {
	//std::cout << "--------------------------------" << std::endl;

	d_activeParticleCollisions.resize(NumCollisions * m_particleCounts.sum());
	d_activePlaneCollisions.resize(NumCollisions * m_particleCounts.sum());

	ImGuiOptions& options = ImGuiOptions::get();

	{
		// TODO: this can be improved by not resizing every frame
		PROFILE_SCOPE("Resizing");

		d_activeParticleCollisions.resize(NumCollisions * m_particleCounts.sum());
		d_activePlaneCollisions.resize(NumCollisions * m_particleCounts.sum());
		d_activeTryeckCollisions.resize(NumCollisions * m_particleCounts.sum());
		d_collisionTryecke.resize(d_tryeckeWall.size());
		thrust::fill(d_collisionTryecke.begin(),d_collisionTryecke.end(),vec3(FLT_MAX,FLT_MAX,FLT_MAX));
		d_collisionTryecke2.resize(d_tryecke.size());
		thrust::fill(d_collisionTryecke2.begin(),d_collisionTryecke2.end(),vec3(FLT_MAX,FLT_MAX,FLT_MAX));
		d_hashBuckets.resize(options.numHashBuckets, -1);
		d_numParticlesPerBucket.resize(options.numHashBuckets, 0);
		d_beginIndicesFluids.resize(options.numHashBuckets, 0);
		d_hashOverflow.resize(m_particleCounts.sum());
	}
	
	if(options.enableFixedCellLength) {
		m_maxRadius = options.fixedCellLength;
	} else {
		ParticlePositionRadius maxElement = *thrust::max_element(m_particles.d_positionRadius.device_begin(), m_particles.d_positionRadius.device_end(), MaxRadius());
		m_maxRadius = maxElement.radius;
	}

	
	{
		PROFILE_SCOPE("positionBasedIntegration");

		// --- 5. update velocity and position 1. + 2. integrate velocities and guess next positions --- //
		positionBasedIntegration<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_momentumMass, m_particles.d_guessedPosition, options.getGravity(), dt);
	}

	float cellLength;
	{
		PROFILE_SCOPE("Sorting");

		cellLength = options.fluidH;
		if(getNumParticles(ParticleType::RegularParticle) > 0) {
			sortParticles(ParticleType::RegularParticle, cellLength);
		}
		std::vector<std::pair<int,int>> debugHashes;
		if(getNumParticles(ParticleType::FluidParticle) > 0) {
			sortParticles(ParticleType::FluidParticle, cellLength);
		}
	}


	// --- 6. collide particles with portals --- //
	if(m_blueSet && m_orangeSet) {
		PROFILE_SCOPE("Portal Step");
		try {
			portalStep(player);
		} catch(int& i) {
			throw i;
		}
	}
	
	int planeCollisionLength;
	{
		PROFILE_SCOPE("Create Plane-Particle Collisions");
		// --- 3. create collision constraints --- //
		// --- create particle-plane collision constraints --- //
		createCollisionConstraintsPlanes<NumCollisions><<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition, m_particles.d_portalFlags, m_rigidBodies.d_rigidBodiesParticleOffsets, m_rigidBodies.d_rigidBodyPortalFlags, d_planes, d_activePlaneCollisions);
		cudaDeviceSynchronize();
		auto newEndPlaneCollisions = thrust::remove_if(d_activePlaneCollisions.begin(), d_activePlaneCollisions.end(), CollisionPlaneData());
		planeCollisionLength = thrust::distance(d_activePlaneCollisions.begin(), newEndPlaneCollisions);
	}


	{
		PROFILE_SCOPE("Create Linked Cell Structure (Particle-Particle)");
		// --- create particle-particle collision constraints --- //
		thrust::fill(d_hashBuckets.begin(), d_hashBuckets.end(), -1);
		thrust::fill(d_numParticlesPerBucket.begin(), d_numParticlesPerBucket.end(), 0);
		createLinkedCellStructure<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition, d_hashBuckets, d_hashOverflow, d_numParticlesPerBucket, cellLength);
		cudaDeviceSynchronize();

		thrust::exclusive_scan(d_numParticlesPerBucket.begin(), d_numParticlesPerBucket.end(), d_beginIndicesFluids.begin(), 0);

		collideParticlesWithLinkedCell<NumCollisions><<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition, d_hashBuckets, d_hashOverflow, d_activeParticleCollisions, m_fluids.d_lambdas, cellLength);
		
		
		cudaDeviceSynchronize();
	}

	int tryeckCollisionLength;
	{
		PROFILE_SCOPE("Create Particle-Tryeck Collisions");

		createCollisionConstraintsTryecks<NumCollisions><<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_guessedPosition, m_particles.d_color, m_particles.d_portalFlags, m_rigidBodies.d_rigidBodiesParticleOffsets, m_rigidBodies.d_rigidBodyPortalFlags, d_tryecke, d_activeTryeckCollisions);
		auto newEndTryEckCollisions = thrust::remove_if(d_activeTryeckCollisions.begin(), d_activeTryeckCollisions.end(), CollisionTryeckData());
		tryeckCollisionLength = thrust::distance(d_activeTryeckCollisions.begin(), newEndTryEckCollisions);
	}

	auto newEndParticleCollisions = thrust::remove_if(d_activeParticleCollisions.begin(), d_activeParticleCollisions.end(), CollisionParticleData());
	int particleCollisionLength = thrust::distance(d_activeParticleCollisions.begin(), newEndParticleCollisions);


	if(getNumParticles(ParticleType::RigidBodyParticle) >= 1) {
		// check if the player is hitting a wall to reset his jump
		m_firstRigidBodyHasHitSomeWalls = false;
		// get the player's rigid body which is the first rigid body in memory
		RigidBodyParticleData playerBody;
		thrust::copy(m_rigidBodies.d_rigidBodiesParticles.begin(), m_rigidBodies.d_rigidBodiesParticles.begin()+1, &playerBody);
		int numPlayerParticles = playerBody.numParticles;

		if(planeCollisionLength > 0) {
			// check if any of the plane collisions contain a particle id that is less than numPlayerParticles
			CollisionPlaneData firstHit;
			thrust::copy(d_activePlaneCollisions.begin(), d_activePlaneCollisions.begin()+1, &firstHit);
			// if yes, the player has hit a plane
			bool hitPlane = firstHit.particleID < numPlayerParticles;
			m_firstRigidBodyHasHitSomeWalls |= hitPlane;
		}

		if(tryeckCollisionLength > 0) {
			// check if any of the triangle collisions contain a particle id that is less than numPlayerParticles
			CollisionTryeckData firstHit;
			thrust::copy(d_activeTryeckCollisions.begin(), d_activeTryeckCollisions.begin()+1, &firstHit);
			// if yes, the player has hit a triangle
			bool hitTryeck = firstHit.particleID < numPlayerParticles;
			m_firstRigidBodyHasHitSomeWalls |= hitTryeck;
		}

		if(particleCollisionLength > 0){
			// check if the player has hit any other particle
			// if yes, the player may jump again
			CollisionParticleData firstHit;
			thrust::copy(d_activeParticleCollisions.begin(), d_activeParticleCollisions.begin()+1, &firstHit);
			bool hitADifferentParticle = (firstHit.particleID1 < numPlayerParticles) && (firstHit.particleID2 >= numPlayerParticles);
			hitADifferentParticle |= (firstHit.particleID2 < numPlayerParticles) && (firstHit.particleID1 >= numPlayerParticles);
			m_firstRigidBodyHasHitSomeWalls |= hitADifferentParticle;
		}
		
	}

	// --- Stabilization --- //
	if(options.stabilize) {
		PROFILE_SCOPE("Stabilization");

		stabilizationStep(dt, planeCollisionLength, tryeckCollisionLength, particleCollisionLength);
		if(options.enablePrints) std::cout << "stabilization step done!" << std::endl;
	}


	if(options.enablePrints) {
		std::cout << "Before Jacobi" << std::endl;
		printPosition<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition);
	}

	// --- 4. resolve collision constraints iteratively --- //
	{
		PROFILE_SCOPE("Jacobi Iterations");

		for(int i = 0; i < options.jacobiIterations; ++i) {
			{
				PROFILE_SCOPE("Plane-Particle Resolve Step");

				// resolve particle-plane collisions
				resolvePlanesStep(dt, planeCollisionLength);
				CUDA_SYNC_CHECK_ERROR();
			}

			{
				PROFILE_SCOPE("Position Update");

				// update the guessed position of the particles
				updatePositions(dt);
				CUDA_SYNC_CHECK_ERROR();
			}

			{
				PROFILE_SCOPE("Triangle Step");

				resolveTryeckStep(dt, tryeckCollisionLength);
				CUDA_SYNC_CHECK_ERROR();
			}

			{
				PROFILE_SCOPE("Position Update");

				// update the guessed position of the particles
				updatePositions(dt);
				CUDA_SYNC_CHECK_ERROR();
			}

			{
				PROFILE_SCOPE("Particle-Particle Resolve Step");

				// resolve particle-particle collisions
				resolveParticleStep(dt, particleCollisionLength);
				CUDA_SYNC_CHECK_ERROR();
			}


			{
				PROFILE_SCOPE("Cloth Step");

				// --- Cloth Simulation --- //
				if(options.resolveWithSDF) {
					clothStep(dt);
				} else {
					std::cout << "Cloth physics are only supported with Signed Distance Fields" << std::endl;
				}
				CUDA_SYNC_CHECK_ERROR();
			}

			{
				PROFILE_SCOPE("Position Update");

				// update the guessed position of the particles
				updatePositions(dt);
				CUDA_SYNC_CHECK_ERROR();
			}

			{
				PROFILE_SCOPE("Rigid Body Step");

				// --- Rigid Bodies --- //
				rigidBodyShapeConstraintStep();
				CUDA_SYNC_CHECK_ERROR();
			}

			{
				PROFILE_SCOPE("Fluid Step");

				// --- Fluids --- //
				fluidStep();
				CUDA_SYNC_CHECK_ERROR();
			}

			{
				PROFILE_SCOPE("Position Update");

				// update the guessed position of the particles
				updatePositions(dt);
				CUDA_SYNC_CHECK_ERROR();
			}
		}
	}

	if(options.enablePrints) {
		std::cout << "After Jacobi" << std::endl;
		printPosition<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition);
		CUDA_SYNC_CHECK_ERROR();
	}

	{
		PROFILE_SCOPE("Velocity Update");

		positionBasedVelocityUpdate<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_guessedPosition, m_particles.d_momentumMass, dt);
		CUDA_SYNC_CHECK_ERROR();
	}

	{
		PROFILE_SCOPE("Vorticity Step");

		vorticityStep(dt);
	}

	{
		PROFILE_SCOPE("Position Update");

		positionBasedPositionUpdate<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_guessedPosition, dt);
		CUDA_SYNC_CHECK_ERROR();
	}

	{
		PROFILE_SCOPE("Marching Cubes");

		// fluid rendering
		doMarchingCubes();
	}

	if(options.enablePrints) {
		std::cout << "Before Rigid Body Duplicate Update" << std::endl;
		printPosition<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition);
	}
	{
		PROFILE_SCOPE("Duplicate Position update");
		updatePostionDuplicatedRigidBodys<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, m_particles.d_portalFlags, m_particles.d_momentumMass, m_rigidBodies.d_rigidBodyPortalFlags, m_rigidBodies.d_rigidBodiesParticleOffsets, *m_orangePortal, *m_bluePortal);
		cudaDeviceSynchronize();
	}
	if(options.enablePrints) {
		std::cout << "After Rigid Body Duplicate Update" << std::endl;
		printPosition<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition);
	}
	cudaDeviceSynchronize();
}
