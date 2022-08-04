#include "particleSystem_rigidBodies.h"

#include "particleSystem_collisions.inl"

#include "saiga/core/util/assert.h"
#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"

#include <thrust/gather.h>
#include <thrust/sort.h>

#include "../svd/svd3_wrapper.h"
#include "saiga/cuda/thread_info.h"

// ######################################################################### //
// ### CUDA Kernels ######################################################## //


/**
 * called once for each rigid body.
*/
__global__ void initializeParticlePositions(ArrayView<RigidBodyParticleData> particleData,
                                        ArrayView<RigidBodyOrientation> bodyOrientation,
                                        ArrayView<RigidBodyParticleOffset> offsets,
                                        ArrayView<ParticlePositionRadius> guessedPositions,
                                        ArrayView<ParticlePositionRadius> absolutePositions) {
	Saiga::CUDA::ThreadInfo<> ti;
    if(ti.thread_id >= particleData.size()) return;

	RigidBodyParticleData bodyData;
	RigidBodyOrientation orientation;
	Saiga::CUDA::vectorCopy(particleData.data()+ti.thread_id, &bodyData);
	Saiga::CUDA::vectorCopy(bodyOrientation.data()+ti.thread_id, &orientation);

    // iterate over this rigid bodies' particles
    for(int i = bodyData.firstParticleIndex; i < bodyData.firstParticleIndex + bodyData.numParticles; ++i) {
		RigidBodyParticleOffset offset;
		ParticlePositionRadius absolutePos;
		ParticlePositionRadius guessedPos;
		Saiga::CUDA::vectorCopy(offsets.data()+i, &offset);
		Saiga::CUDA::vectorCopy(absolutePositions.data()+i, &absolutePos);
		Saiga::CUDA::vectorCopy(guessedPositions.data()+i, &guessedPos);

        vec3 orientedOffset = offset.relativeOffset;
        orientedOffset = orientation.position +  orientation.rotation * orientedOffset;
        guessedPos.position = orientedOffset;
        absolutePos.position = orientedOffset;

		Saiga::CUDA::vectorCopy(&offset, offsets.data()+i);
		Saiga::CUDA::vectorCopy(&absolutePos, absolutePositions.data()+i);
		Saiga::CUDA::vectorCopy(&guessedPos, guessedPositions.data()+i);
    }
}

/**
 * computes the center of gravity of the particles in [startIndex, endIndex)
*/
__device__ vec3 computeDeformedCenterOfGravity(ArrayView<ParticlePositionRadius> guessedPositions, int startIndex, int endIndex) {
    vec3 result = {0.f, 0.f, 0.f};
    for(int i = startIndex; i < endIndex; ++i) {
        result += guessedPositions[i].position;
    }
	if(startIndex == endIndex) return result;
    return result / (endIndex - startIndex);
}

__device__ mat3 computeCovarianceMatrix(ArrayView<RigidBodyParticleOffset> offsets
    , ArrayView<ParticlePositionRadius> guessedPositions    
    , vec3 deformedCenterOfGravity
    , int startIndex
    , int endIndex) {
    mat3 result = mat3::Zero();
    for(int i = startIndex; i < endIndex; ++i) {
        vec3 offset = offsets[i].relativeOffset;
        vec3 guessedPos = guessedPositions[i].position;
        result += (guessedPos - deformedCenterOfGravity) * offset.transpose();
    }
	return result;
}

__device__ void computePositionDeltas(ArrayView<ParticleDeltaValues> particleDeltas
    , ArrayView<RigidBodyParticleOffset> particleOffsets
    , ArrayView<ParticlePositionRadius> guessedPositions
    , mat3 Q
    , vec3 deformedCenterOfGravity
    , int startIndex
    , int endIndex) {
    for(int i = startIndex; i < endIndex; ++i) {
		// load from memory
        ParticleDeltaValues& delta = particleDeltas[i];
		RigidBodyParticleOffset offset;
		ParticlePositionRadius guessedPosition;
		Saiga::CUDA::vectorCopy(particleOffsets.data()+i, &offset);
		Saiga::CUDA::vectorCopy(guessedPositions.data()+i, &guessedPosition);
    
        vec3 deltaPosition = (Q * offset.relativeOffset + deformedCenterOfGravity) - guessedPosition.position;
        
        atomicAdd(&delta.delta[0], deltaPosition[0]);
        atomicAdd(&delta.delta[1], deltaPosition[1]);
        atomicAdd(&delta.delta[2], deltaPosition[2]);

		// write back
		Saiga::CUDA::vectorCopy(&offset, particleOffsets.data()+i);
		Saiga::CUDA::vectorCopy(&guessedPosition, guessedPositions.data()+i);
    }
}

/**
 * Called once per rigid body
*/
__global__ void createCoolerRigidBodyShapeConstraints(ArrayView<ParticlePositionRadius> guessedPositions,
    ArrayView<ParticleDeltaValues> particleDeltas,
    ArrayView<RigidBodyOrientation> bodyOrientation,
    ArrayView<RigidBodyParticleData> particleData,
    ArrayView<RigidBodyParticleOffset> particleOffsets,
    ArrayView<RigidBodyPortalFlags> bodyPortalFlags) {
        
    Saiga::CUDA::ThreadInfo<> ti;
    if(ti.thread_id >= particleData.size()) return;

	RigidBodyParticleData rigidBodyData;
    RigidBodyOrientation rigidBodyOrientation;    
	Saiga::CUDA::vectorCopy(particleData.data()+ti.thread_id, &rigidBodyData);
    Saiga::CUDA::vectorCopy(bodyOrientation.data()+ti.thread_id, &rigidBodyOrientation);

    // don't calculate anything if the body is a duplicate
    //RigidBodyPortalFlags rigidBodyPortalFlags;
    //Saiga::CUDA::vectorCopy(bodyPortalFlags.data()+ti.thread_id, &rigidBodyPortalFlags);
    //if(rigidBodyPortalFlags.state == 2) return;

    int particlesBegin = rigidBodyData.firstParticleIndex;
    int particlesEnd = rigidBodyData.firstParticleIndex + rigidBodyData.numParticles;
    // Step 1: compute the deformed center of mass.
    // TODO: move this to its own kernel and call it per particle
    vec3 deformedCenter = computeDeformedCenterOfGravity(guessedPositions, particlesBegin, particlesEnd);

    // Step 2: compute the covariance matrix A
    // TODO: move this to its own kernel and call it per particle
    mat3 A = computeCovarianceMatrix(particleOffsets, guessedPositions, deformedCenter, particlesBegin, particlesEnd);
    
    // Step 3: compute Rotation matrix
    // TODO: compute this cleverly
    mat3 Q = pd(A);

    // TODO: update the orientation of the whole rigid body in rigidBodyOrientation
    // this is untested
    rigidBodyOrientation.rotation = quat(Q);

	// TODO: update the position of the whole rigid body in rigidBodyOrientation
    // so far it lags one frame behind
    rigidBodyOrientation.position = deformedCenter;

    // Step 4: compute delta position
    // TODO: move this to its own kernel and call it per particle
    computePositionDeltas(particleDeltas, particleOffsets, guessedPositions, Q, deformedCenter, particlesBegin, particlesEnd);

	// write back
	Saiga::CUDA::vectorCopy(&rigidBodyOrientation, bodyOrientation.data()+ti.thread_id);
}


/**
 * called once for each rigid body particle
*/
__global__ void createRigidBodyShapeMatchingConstraints(ArrayView<ParticlePositionRadius> guessedPositions,
    ArrayView<ParticleDeltaValues> particleDeltas,
    ArrayView<RigidBodyOrientation> bodyOrientation,
    ArrayView<RigidBodyParticleData> particleData,
    ArrayView<RigidBodyParticleOffset> bodyOffsets,
    ArrayView<RigidBodyPortalFlags> bodyPortalFlags) {
       
    Saiga::CUDA::ThreadInfo<> ti;
    if(ti.thread_id >= guessedPositions.size()) return;

    // don't calculate anything if the body is a duplicate
    //RigidBodyPortalFlags rigidBodyPortalFlags;
    //Saiga::CUDA::vectorCopy(bodyPortalFlags.data()+ti.thread_id, &rigidBodyPortalFlags);
    //if(rigidBodyPortalFlags.state == 2) return;

    RigidBodyParticleOffset particleOffset = bodyOffsets[ti.thread_id];
    if(particleOffset.bodyIndex == -1) return;

	// Step 1: compute center of gravity of the deformed body
    vec3 c = {0.f,0.f,0.f};
    // get body data of particle
    RigidBodyParticleData bodyData = particleData[particleOffset.bodyIndex];
    // iterate over all particles of the body
    for(int i = bodyData.firstParticleIndex; i < bodyData.firstParticleIndex + bodyData.numParticles; ++i) {
        c += guessedPositions[i].position;
    }
    c /= bodyData.numParticles;

    // Step 2: compute the covariance matrix A
    mat3 A = mat3::Zero();
    for(int i = bodyData.firstParticleIndex; i < bodyData.firstParticleIndex + bodyData.numParticles; ++i) {
		vec3 offsetty = bodyOffsets[i].relativeOffset;
		vec3 a = guessedPositions[i].position - c;
		mat3 damn = a * offsetty.transpose();
		A += damn;
    }

	// Step 3: compute rotationmatrix Q with svd
	mat3 Q = pd(A);

    // Step 4: compute delta position
    ParticleDeltaValues& delta = particleDeltas[ti.thread_id];

    vec3 deltaPos = (Q * bodyOffsets[ti.thread_id].relativeOffset + c) - guessedPositions[ti.thread_id].position;
    
    atomicAdd(&delta.delta[0], deltaPos[0]);
    atomicAdd(&delta.delta[1], deltaPos[1]);
	atomicAdd(&delta.delta[2], deltaPos[2]);
}


// ######################################################################### //
// ### C++ ParticleSystem ################################################## //

void ParticleSystem::initRigidCubes() {
    // set the absolute position of particles in rigid bodies
    initializeParticlePositions<<<THREAD_BLOCK(m_rigidBodies.d_rigidBodiesParticles.size(), BLOCK_SIZE)>>>(m_rigidBodies.d_rigidBodiesParticles, m_rigidBodies.d_rigidBodiesOrientation, m_rigidBodies.d_rigidBodiesParticleOffsets, m_particles.d_guessedPosition, m_particles.d_positionRadius);
    cudaDeviceSynchronize();
}

void ParticleSystem::rigidBodyShapeConstraintStep() {
	if(m_rigidBodies.d_rigidBodiesParticles.size() > 0) {
		createCoolerRigidBodyShapeConstraints<<<THREAD_BLOCK(m_rigidBodies.d_rigidBodiesParticles.size(), BLOCK_SIZE)>>>(m_particles.d_guessedPosition, m_particles.d_deltaValues, m_rigidBodies.d_rigidBodiesOrientation, m_rigidBodies.d_rigidBodiesParticles, m_rigidBodies.d_rigidBodiesParticleOffsets, m_rigidBodies.d_rigidBodyPortalFlags);
		cudaDeviceSynchronize();
	}
}
