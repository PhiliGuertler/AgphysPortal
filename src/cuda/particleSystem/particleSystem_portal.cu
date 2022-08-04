#include "particleSystem.h"
#include "../rigidBodyManager.h"
#include "../fluidCreator.h"
#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"

#include "../../glBuffers.h"

#define PRINTDATA int f = 0;\
			std::cout << __FUNCTION__ << ":" << __LINE__ << std::endl;\
			for(auto& entry : m_duplicateMap) {\
				std::cout << f << " | [" << entry.first << "," << entry.second << "]" << std::endl;\
				++f;\
			}\
			for(f = 0; f < m_rigidBodysToDelete.size(); ++f) {\
				std::cout << f << " <" << m_rigidBodysToDelete[f] << ">" << std::endl;\
			}

__global__ void resetRigidBodyParticles(int numParticles, int firstParticleIndex, ArrayView<ParticlePortalFlags> particlePortalFlags) {
    Saiga::CUDA::ThreadInfo<> ti;
    if(ti.thread_id >= numParticles) return;
    
    ParticlePortalFlags portalFlags;
	Saiga::CUDA::vectorCopy(particlePortalFlags.data()+ti.thread_id + firstParticleIndex, &portalFlags);

    portalFlags.portalHit = 0;
    portalFlags.oldParticle = 0;
    portalFlags.newParticle = 0;
    portalFlags.otherParticle = -1;

    Saiga::CUDA::vectorCopy(&portalFlags, particlePortalFlags.data()+ti.thread_id + firstParticleIndex);
}

__global__ void updateDuplicateParticleID(ArrayView<ParticlePortalFlags> particlePortalFlags
                                        , ArrayView<RigidBodyParticleOffset> bodyParticleOffset
                                        , ArrayView<RigidBodyParticleNormal> bodyParticleNormal
                                        , int BodyIndex
                                        , int firstIndex
                                        , int particleNumber) {
    Saiga::CUDA::ThreadInfo<> ti;
    if(ti.thread_id >= particlePortalFlags.size()) return;
    
    // set duplicated ID to the new right one
    ParticlePortalFlags portalFlags;
    Saiga::CUDA::vectorCopy(particlePortalFlags.data()+ti.thread_id, &portalFlags);
    if(portalFlags.otherParticle >= firstIndex) {
        portalFlags.otherParticle -= particleNumber;
    }

    // set the new right body ID
    RigidBodyParticleOffset particleOffset;
    Saiga::CUDA::vectorCopy(bodyParticleOffset.data()+ti.thread_id, &particleOffset);

    RigidBodyParticleNormal particleNormal;
    Saiga::CUDA::vectorCopy(bodyParticleNormal.data()+ti.thread_id, &particleNormal);
    if(particleOffset.bodyIndex >= BodyIndex) {
        particleOffset.bodyIndex--;
        particleNormal.bodyIndex--;
    }

    Saiga::CUDA::vectorCopy(&portalFlags, particlePortalFlags.data()+ti.thread_id);
    Saiga::CUDA::vectorCopy(&particleOffset, bodyParticleOffset.data()+ti.thread_id);
    Saiga::CUDA::vectorCopy(&particleNormal, bodyParticleNormal.data()+ti.thread_id);
}

__global__ void updateRigidBodyData(ArrayView<RigidBodyParticleData> bodyParticleData
								  , ArrayView<RigidBodyPortalFlags> rbPortalFlags
                                  , int BodyIndex
                                  , int particleNumber){
    Saiga::CUDA::ThreadInfo<> ti;
    if(ti.thread_id >= bodyParticleData.size()) return;

	RigidBodyPortalFlags flags;
	Saiga::CUDA::vectorCopy(rbPortalFlags.data()+ti.thread_id, &flags);

	if(ti.thread_id < BodyIndex && flags.duplicateId < BodyIndex) {
		return;
	} else if(ti.thread_id < BodyIndex) {
		// update the index of the duplicate
		flags.duplicateId--;
		Saiga::CUDA::vectorCopy(&flags, rbPortalFlags.data()+ti.thread_id);
		return;
	}

    RigidBodyParticleData rbpd;
    Saiga::CUDA::vectorCopy(bodyParticleData.data()+ti.thread_id, &rbpd);
    rbpd.firstParticleIndex -= particleNumber;

    Saiga::CUDA::vectorCopy(&rbpd, bodyParticleData.data()+ti.thread_id);
}

void ParticleSystem::setBluePortal(vec3 normal, vec3 center, Scene& scene, vec3 viewDir) {   
	std::cout << "Blue Portal Plane is being updated" << std::endl;
	m_bluePortal->reposition(normal.normalized(), center, *m_orangePortal, viewDir);
	scene.updatePortalPlane(*m_bluePortal, false);
	m_blueSet = true;
	std::cout << "Blue Portal Plane update done" << std::endl;
}

void ParticleSystem::setOrangePortal(vec3 normal, vec3 center, Scene& scene, vec3 viewDir) {
	std::cout << "Orange Portal Plane is being updated" << std::endl;
	m_orangePortal->reposition(normal.normalized(), center, *m_bluePortal, viewDir);
	scene.updatePortalPlane(*m_orangePortal, true);
	m_orangeSet = true;
	std::cout << "Orange Portal Plane update done" << std::endl;
}

void ParticleSystem::portalStep(Controls::Player& player) {
    // device vector for particles to duplicate
    m_duplicateParticleLength = 0;
	d_particlesToDuplicate.resize(m_particleCounts.sum(), -1);
	thrust::fill(d_particlesToDuplicate.begin(), d_particlesToDuplicate.end(), -1);

	// device vector for particles to delete
	d_particlesToDelete.resize(m_particleCounts.sum(), -1);
	thrust::fill(d_particlesToDelete.begin(), d_particlesToDelete.end(), -1);

	// device vector for particles that are still intersecting a portal
	d_particlesInPortals.resize(m_particleCounts.sum(), -1);
	thrust::fill(d_particlesInPortals.begin(), d_particlesInPortals.end(), -1);
	
	portalParticleIntersection<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius
			, m_particles.d_color
			, m_particles.d_momentumMass
			, m_particles.d_guessedPosition
			, m_particles.d_portalFlags
			, d_particlesToDuplicate
			, d_particlesToDelete
			, d_particlesInPortals
			, *m_bluePortal
			, *m_orangePortal);
	cudaDeviceSynchronize();

	auto end2 = thrust::remove_if(d_particlesToDuplicate.begin(), d_particlesToDuplicate.end(), particlesDuplicateComp());
	int length2 = thrust::distance(d_particlesToDuplicate.begin(), end2);

	m_particlesToDuplicate.resize(length2);
	thrust::copy(d_particlesToDuplicate.begin(), d_particlesToDuplicate.begin()+length2, m_particlesToDuplicate.begin());
    

	auto end = thrust::remove_if(d_particlesToDelete.begin(), d_particlesToDelete.end(), particlesDuplicateComp());
	int length = thrust::distance(d_particlesToDelete.begin(), end);

	m_particlesToDelete.resize(length);
	thrust::copy(d_particlesToDelete.begin(), d_particlesToDelete.begin()+length, m_particlesToDelete.begin());
    
    // collide rigid bodys with portals
	if(m_particleCounts.numRigidBodies > 0) {
		try {
			collideRigidBodysPortal(player);
		} catch(int& i) {
			throw i;
		}
	}
}

void ParticleSystem::internalRegisterIntersectingBodies(std::vector<int>& allHitRigidBodys) {
	auto endParticlesInPortals = thrust::remove_if(d_particlesInPortals.begin(), d_particlesInPortals.end(), particlesDuplicateComp());
	int particlesInPortals = thrust::distance(d_particlesInPortals.begin(), endParticlesInPortals);

	thrust::host_vector<int> porticles(particlesInPortals);
	thrust::copy(d_particlesInPortals.begin(), endParticlesInPortals, porticles.begin());

	// add all corresponding rigid bodies of intersecting particles to allHitRigidBodys
	for(int i = 0; i < particlesInPortals; ++i) {
		int particleID = porticles[i];

		ParticlePortalFlags portalFlags;
		thrust::copy(m_particles.d_portalFlags.begin()+particleID, m_particles.d_portalFlags.begin()+particleID+1, &portalFlags);
		
        RigidBodyParticleOffset particleOffset = m_rigidBodies.d_rigidBodiesParticleOffsets[particleID];
		int bodyID = particleOffset.bodyIndex;

        // search for rb in duplicated rbs for this frame
        auto index = std::find(allHitRigidBodys.begin(), allHitRigidBodys.end(), bodyID);
		if(index == allHitRigidBodys.end()) {
			allHitRigidBodys.push_back(bodyID);
		}
	}
}

void ParticleSystem::internalRegisterDuplicationParticles(std::vector<int>& allHitRigidBodys, std::vector<int>& newRigidBodys) {
	int rbEnd = particlesEnd(ParticleType::RigidBodyParticle);

    for(int i = 0; i < m_duplicateParticleLength; ++i) {
		int particleID = m_particlesToDuplicate[i];
        if(particleID >= rbEnd) {
			// this is not a rigid-body particle, this doesn't need to be handled here
            break;
        }

        ParticlePortalFlags portalFlags;
		thrust::copy(m_particles.d_portalFlags.begin()+particleID, m_particles.d_portalFlags.begin()+particleID+1, &portalFlags);
			
        RigidBodyParticleOffset particleOffset = m_rigidBodies.d_rigidBodiesParticleOffsets[particleID];
		int bodyID = particleOffset.bodyIndex;

        // search for rb in duplicated rbs for this frame
        auto index = std::find(allHitRigidBodys.begin(), allHitRigidBodys.end(), bodyID);

        // rb wasn't marked in this frame yet
        if(index == allHitRigidBodys.end()) {
            RigidBodyPortalFlags rbPortalFlags = m_rigidBodies.d_rigidBodyPortalFlags[particleOffset.bodyIndex];

            allHitRigidBodys.push_back(bodyID);
            if(rbPortalFlags.state == Regular) {
				// rigid body is no duplicate and wasn't duplicated before
                newRigidBodys.push_back(particleOffset.bodyIndex);
				rbPortalFlags.portalHit = portalFlags.portalHit;
				m_rigidBodies.d_rigidBodyPortalFlags[particleOffset.bodyIndex] = rbPortalFlags;
			}
		}
	}

}

void ParticleSystem::collideRigidBodysPortal(Controls::Player& player) {
    // find new rigid bodys that need duplication
    int rbBegin = particlesBegin(ParticleType::RigidBodyParticle);
    int rbEnd = particlesEnd(ParticleType::RigidBodyParticle);

    std::vector<int> newRigidBodys;
    std::vector<int> allHitRigidBodys;

	m_duplicateParticleLength = m_particlesToDuplicate.size();

	internalRegisterIntersectingBodies(allHitRigidBodys);
	CUDA_SYNC_CHECK_ERROR();

	internalRegisterDuplicationParticles(allHitRigidBodys, newRigidBodys);
	CUDA_SYNC_CHECK_ERROR();

	m_rigidBodysToDuplicate = newRigidBodys;

	thrust::device_vector<int> d_centerOfRigidBodys = thrust::device_vector<int>();
	d_centerOfRigidBodys.resize(m_particleCounts.numRigidBodies, -1);

	// check if a rigid-body-center traverses a portal
	intersectRigidBodyCenterPortal<<<THREAD_BLOCK(m_particleCounts.numRigidBodies, BLOCK_SIZE)>>>(m_rigidBodies.d_rigidBodiesOrientation
			, d_centerOfRigidBodys
			, m_rigidBodies.d_rigidBodiesParticles
			, m_particles.d_momentumMass
			, m_particles.d_positionRadius
			, m_particles.d_guessedPosition
			, 0.001f
			, *m_bluePortal
			, *m_orangePortal);
	CUDA_SYNC_CHECK_ERROR();

	thrust::host_vector<int> h_centerOfRigidBodys = thrust::host_vector<int>(d_centerOfRigidBodys);

	internalCheckForInstantSwaps(h_centerOfRigidBodys, player);
	CUDA_SYNC_CHECK_ERROR();

	internalCheckForSwaps(h_centerOfRigidBodys, player);
	CUDA_SYNC_CHECK_ERROR();

	try {
		internalRemoveDuplicates(allHitRigidBodys);
	} catch(int& i) {
		throw i;
	}
	CUDA_SYNC_CHECK_ERROR();
}

void ParticleSystem::internalCheckForInstantSwaps(thrust::host_vector<int>& h_centerOfRigidBodys, Controls::Player& player) {

	// check all rigid bodys, if it has no duplicate, but the center hits the traverses the portal anyway
	for(int i = 0; i < m_particleCounts.numRigidBodies; ++i){
		
		auto index = m_duplicateMap.find(i);
		RigidBodyPortalFlags bodyPortalFlags = m_rigidBodies.d_rigidBodyPortalFlags[i];

		if(h_centerOfRigidBodys[i] == 1 && index == m_duplicateMap.end() && bodyPortalFlags.portalHit != 0) {
			// body crosses the portal in this step, but wasn't duplicated before
			
			RigidBodyOrientation bodyOrientation = m_rigidBodies.d_rigidBodiesOrientation[i];
			RigidBodyParticleData bodyData = m_rigidBodies.d_rigidBodiesParticles[i];
			
			// particle teleport
			int beg = bodyData.firstParticleIndex;
			int end = bodyData.firstParticleIndex + bodyData.numParticles;
			for(int j = beg; j < end; ++j) {
				ParticlePositionRadius posRad;//
				thrust::copy(m_particles.d_positionRadius.device_begin()+beg, m_particles.d_positionRadius.device_begin()+beg+1, &posRad);
				ParticleMomentumMass moms = m_particles.d_momentumMass[j];
				ParticlePositionRadius guessedPosRad = m_particles.d_guessedPosition[j];

				vec3 pos = posRad.position;
				vec3 guessedPos = guessedPosRad.position;
				vec3 velocity = moms.momentumVelocity;
			
				vec3 newVelocity;
				vec3 posNew;
				vec3 guessedPosNew;
				if(bodyPortalFlags.portalHit == 1) {
					// blue portal hit
					transformEverything(posNew, guessedPosNew, newVelocity, *m_bluePortal, *m_orangePortal, pos, guessedPos, velocity);
			
				} else if(bodyPortalFlags.portalHit == 2) {
					// orange portal hit
					transformEverything(posNew, guessedPosNew, newVelocity, *m_orangePortal, *m_bluePortal, pos, guessedPos, velocity);
			
				} else {
					std::cout << "Das darf nicht passieren" << std::endl;
				}
				
				// set changed values
				posRad.position = posNew;
				guessedPosRad.position = guessedPosNew;
				moms.momentumVelocity = newVelocity;

				//m_particles.d_positionRadius[j] = posRad;
				thrust::copy(&posRad, &posRad+1, m_particles.d_positionRadius.device_begin()+j);
				m_particles.d_momentumMass[j] = moms;
				m_particles.d_guessedPosition[j] = guessedPosRad;
			}

			// transform player view port
			// FIXME: the player's camera should only be teleported at the exact moment of crossing a portal
			// That might be a bit too sophisticated though
			if(i == 0) {
				// player needs to be swapped
				if(bodyPortalFlags.portalHit == 1) {
					player.transformView(*m_bluePortal, *m_orangePortal);
				} else if(bodyPortalFlags.portalHit == 2) {
					player.transformView(*m_orangePortal, *m_bluePortal);
				}
			}
						
			// set the needed rigid body portal flags
			// orientation teleport
			// center teleport
			if(bodyPortalFlags.portalHit == 1) {
				bodyOrientation.position = Portal::transformVector(vec4(bodyOrientation.position.x(), bodyOrientation.position.y(), bodyOrientation.position.z(), 1), *m_bluePortal, *m_orangePortal);
				bodyOrientation.rotation = Portal::transformQuat(bodyOrientation.rotation, *m_bluePortal, *m_orangePortal);
				bodyPortalFlags.portalHit = 2;
			} else {
				bodyOrientation.position = Portal::transformVector(vec4(bodyOrientation.position.x(), bodyOrientation.position.y(), bodyOrientation.position.z(), 1), *m_bluePortal, *m_orangePortal);
				bodyOrientation.rotation = Portal::transformQuat(bodyOrientation.rotation, *m_bluePortal, *m_orangePortal);
				bodyPortalFlags.portalHit = 1;
			}
			m_rigidBodies.d_rigidBodyPortalFlags[i] = bodyPortalFlags;
			m_rigidBodies.d_rigidBodiesOrientation[i] = bodyOrientation;

		}
	}

}

#define VECPOS(a) vec4(a.x(), a.y(), a.z(), 1.f)
#define VECDIR(a) vec4(a.x(), a.y(), a.z(), 0.f)

static inline __device__ void transformComponents(vec3& pos, vec3& guessedPos, vec3& mom, Portal portalIn, Portal portalOut) {
	pos = Portal::transformVector(VECPOS(pos), portalIn, portalOut);
	guessedPos = Portal::transformVector(VECPOS(guessedPos), portalIn, portalOut);
	mom = Portal::transformVector(VECDIR(mom), portalIn, portalOut);
}

static inline __device__ void teleportParticle(ArrayView<ParticlePositionRadius> positions
		, ArrayView<ParticlePositionRadius> guessedPositions 
		, ArrayView<ParticleMomentumMass> moms
		, ArrayView<ParticlePortalFlags> flags
		, Portal portalIn
		, Portal portalOut
		, int particleID) {

	// load data of original particle
	ParticlePositionRadius origPos;
	Saiga::CUDA::vectorCopy(positions.data()+particleID, &origPos);
	
	ParticlePositionRadius origGuessedPos;
	Saiga::CUDA::vectorCopy(guessedPositions.data()+particleID, &origGuessedPos);

	ParticleMomentumMass origMom;
	Saiga::CUDA::vectorCopy(moms.data()+particleID, &origMom);

	ParticlePortalFlags origFlag;
	Saiga::CUDA::vectorCopy(flags.data()+particleID, &origFlag);

	if(origFlag.portalHit == 1) {
		// original hits the blue portal, swap it to the orange portal
		origFlag.portalHit = 2;
	} else if(origFlag.portalHit == 2) {
		origFlag.portalHit = 1;
	}

	// transform the data
	transformComponents(origPos.position, origGuessedPos.position, origMom.momentumVelocity, portalIn, portalOut);

	// write back
	Saiga::CUDA::vectorCopy(&origPos, positions.data()+particleID);
	Saiga::CUDA::vectorCopy(&origGuessedPos, guessedPositions.data()+particleID);
	Saiga::CUDA::vectorCopy(&origMom, moms.data()+particleID);
	Saiga::CUDA::vectorCopy(&origFlag, flags.data()+particleID);
}

__global__ void swapParticles(ArrayView<ParticlePositionRadius> positions
		, ArrayView<ParticlePositionRadius> guessedPositions 
		, ArrayView<ParticleMomentumMass> moms
		, ArrayView<ParticlePortalFlags> flags
		, Portal bluePortal
		, Portal orangePortal
		, RigidBodyPortalFlags rbFlags
		, int firstParticleOriginal
		, int firstParticleDuplicate
		, int numParticles) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= numParticles) return;
	int particleID = firstParticleOriginal + ti.thread_id;
	int duplicateID = firstParticleDuplicate + ti.thread_id;

	if(rbFlags.portalHit == 1) {
		// blue portal hit
		teleportParticle(positions, guessedPositions, moms, flags, bluePortal, orangePortal, particleID);
		teleportParticle(positions, guessedPositions, moms, flags, orangePortal, bluePortal, duplicateID);
	} else if(rbFlags.portalHit == 2) {
		// orange portal hit
		teleportParticle(positions, guessedPositions, moms, flags, orangePortal, bluePortal, particleID);
		teleportParticle(positions, guessedPositions, moms, flags, bluePortal, orangePortal, duplicateID);
	}
}

void ParticleSystem::internalCheckForSwaps(thrust::host_vector<int>& h_centerOfRigidBodys, Controls::Player& player) {
	//PRINTDATA
	// FIXME: this is crashing sometimes

	// check all rigid bodys with duplicates, if it needs to be swapped
	for(auto& entry : m_duplicateMap) {
		int current = entry.first;
		RigidBodyPortalFlags rbpf = m_rigidBodies.d_rigidBodyPortalFlags[current];
		if(h_centerOfRigidBodys[current] == 1 && rbpf.state == HasDuplicate) {
			// swap position and movement of rb and rbDuplicated particles
			RigidBodyPortalFlags otherPortalFlags = m_rigidBodies.d_rigidBodyPortalFlags[rbpf.duplicateId];

			RigidBodyParticleData ownRbpd = m_rigidBodies.d_rigidBodiesParticles[current];
			RigidBodyParticleData duplicatedRbpd = m_rigidBodies.d_rigidBodiesParticles[rbpf.duplicateId];

			int firstOwn = ownRbpd.firstParticleIndex;
			int numParts = ownRbpd.numParticles;
			int firstOther = duplicatedRbpd.firstParticleIndex;

			swapParticles<<<THREAD_BLOCK(numParts, BLOCK_SIZE)>>>(m_particles.d_positionRadius
					, m_particles.d_guessedPosition
					, m_particles.d_momentumMass
					, m_particles.d_portalFlags
					, *m_bluePortal
					, *m_orangePortal
					, rbpf
					, firstOwn
					, firstOther
					, numParts);

			if(rbpf.portalHit == 1) {
				otherPortalFlags.portalHit = 1;
				rbpf.portalHit = 2;
			} else {
				otherPortalFlags.portalHit = 2;
				rbpf.portalHit = 1;
			}
			m_rigidBodies.d_rigidBodyPortalFlags[current] = rbpf;
			m_rigidBodies.d_rigidBodyPortalFlags[rbpf.duplicateId] = otherPortalFlags;

			// swap rigid body orientation
			RigidBodyOrientation tmp = m_rigidBodies.d_rigidBodiesOrientation[current];
			m_rigidBodies.d_rigidBodiesOrientation[current] = m_rigidBodies.d_rigidBodiesOrientation[rbpf.duplicateId];
			m_rigidBodies.d_rigidBodiesOrientation[rbpf.duplicateId] = tmp;

			if(current == 0) {
				// player needs to be swapped
				std::cout << "player needs to be swapped!" << std::endl;
				if(rbpf.portalHit == 1) {
					player.transformView(*m_bluePortal, *m_orangePortal);
				} else if(rbpf.portalHit == 2) {
					player.transformView(*m_orangePortal, *m_bluePortal);

				}
			}
		}
	}
	CUDA_SYNC_CHECK_ERROR();
}

__global__ void colorizeBody(ArrayView<ParticleColor> colors, ArrayView<RigidBodyParticleData> data, int bodyID, vec4 color) {
	Saiga::CUDA::ThreadInfo<> ti;

	RigidBodyParticleData rbData;
	Saiga::CUDA::vectorCopy(data.data()+bodyID, &rbData);

	if(ti.thread_id < rbData.firstParticleIndex || ti.thread_id >= rbData.firstParticleIndex+rbData.numParticles) return;

	colors[ti.thread_id].color = color;
}

void ParticleSystem::internalRemoveDuplicates(std::vector<int>& allHitRigidBodys) {
	// remove duplicates that have not had any collision
	for(auto entry: m_duplicateMap) {
		int bodyID = entry.first;
		// check if this body or its duplicate have had any portal collision in this frame
		auto index = std::find(allHitRigidBodys.begin(), allHitRigidBodys.end(), bodyID);
		bool bodyHasHitThePortalInTheLastFrames = index != allHitRigidBodys.end();
		if(!bodyHasHitThePortalInTheLastFrames) {
			
			// reset portal flags of rigid body
			RigidBodyPortalFlags rbpf = m_rigidBodies.d_rigidBodyPortalFlags[bodyID];
			// add duplicate ID to be deleted
			if(rbpf.duplicateId == -1) {
				std::cout << "Fehler gefunden! " << bodyID << std::endl;
				
				colorizeBody<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_color, m_rigidBodies.d_rigidBodiesParticles, bodyID, vec4(1.f, 0.5f, 0.8f, 1.f));
				throw 1;
			}

			m_rigidBodysToDelete.push_back(bodyID);

			rbpf.state = Regular;
			rbpf.duplicateId = -1;
			rbpf.portalHit = 0;
			m_rigidBodies.d_rigidBodyPortalFlags[bodyID] = rbpf;

			// reset portal flags of all particle
			RigidBodyParticleData rbpd = m_rigidBodies.d_rigidBodiesParticles[bodyID];
			resetRigidBodyParticles<<<THREAD_BLOCK(rbpd.numParticles, BLOCK_SIZE)>>>(rbpd.numParticles, rbpd.firstParticleIndex, m_particles.d_portalFlags);

		}
	}
	CUDA_SYNC_CHECK_ERROR();
}


void ParticleSystem::duplicateRigidBodys() {
	RigidBodyManager &rbManager = RigidBodyManager::get(); 

	// get GLBufferManager to read from gl memory
	GLBufferManager& manager = GLBufferManager::get();
	manager.map();

	for(int i = 0; i < m_rigidBodysToDuplicate.size(); ++i) {
		int bodyID = m_rigidBodysToDuplicate[i];

		// get this body's portal flags
		RigidBodyPortalFlags rbpf;
		thrust::copy(m_rigidBodies.d_rigidBodyPortalFlags.begin()+bodyID, m_rigidBodies.d_rigidBodyPortalFlags.begin()+bodyID+1, &rbpf);

		if(rbpf.duplicateId != -1) {
			// body was already duplicated
            continue;
        }

		// get this body's data
		RigidBodyOrientation rbo;
		thrust::copy(m_rigidBodies.d_rigidBodiesOrientation.begin()+bodyID, m_rigidBodies.d_rigidBodiesOrientation.begin()+bodyID+1, &rbo);

		RigidBodyParticleData rbpd;
		thrust::copy(m_rigidBodies.d_rigidBodiesParticles.begin()+bodyID, m_rigidBodies.d_rigidBodiesParticles.begin()+bodyID+1, &rbpd);			

		RigidBodyAssetHash rbah;
		thrust::copy(m_rigidBodies.d_assetHash.begin()+bodyID, m_rigidBodies.d_assetHash.begin()+bodyID+1, &rbah);

        rbpf.state = HasDuplicate;
        rbpf.duplicateId = m_particleCounts.numRigidBodies;

		m_duplicateMap[bodyID] = rbpf.duplicateId;

		// update portal flags of this body
		thrust::copy(&rbpf, &rbpf+1, m_rigidBodies.d_rigidBodyPortalFlags.begin()+bodyID);

        // duplicated rigid body
        RigidBodyData duplicatedBody = rbManager.duplicateRigidBody(*this,rbo,rbpd,rbah);
        
        // set correct portal flags for new rigid body 
        duplicatedBody.rigidBodies.d_rigidBodyPortalFlags[0].state = IsDuplicate;
        duplicatedBody.rigidBodies.d_rigidBodyPortalFlags[0].duplicateId = bodyID;
        duplicatedBody.rigidBodies.d_rigidBodyPortalFlags[0].portalHit = rbpf.portalHit == 1 ? 2 : 1;
    
        // fill rigid body particle with needed data
        int particleCountDup = rbpd.numParticles;
		duplicatedBody.rigidBodies.d_rigidBodiesParticles[0].numParticles = particleCountDup;

		// copy all particles of this rigid body
		thrust::host_vector<ParticlePortalFlags> oldFlags(particleCountDup);
		thrust::copy(m_particles.d_portalFlags.begin()+rbpd.firstParticleIndex, m_particles.d_portalFlags.begin()+rbpd.firstParticleIndex+particleCountDup, oldFlags.begin());

		int beg = rbpd.firstParticleIndex;
		int end = beg+particleCountDup;
		thrust::copy(m_particles.d_positionRadius.device_begin()+beg, m_particles.d_positionRadius.device_begin()+end, duplicatedBody.particles.d_positionRadius.data());
		thrust::copy(m_particles.d_color.device_begin()+beg, m_particles.d_color.device_begin()+end, duplicatedBody.particles.d_color.data());
		thrust::copy(m_particles.d_momentumMass.begin()+beg, m_particles.d_momentumMass.begin()+end, duplicatedBody.particles.d_momentumMass.data());
		thrust::copy(m_particles.d_deltaValues.begin()+beg, m_particles.d_deltaValues.begin()+end, duplicatedBody.particles.d_deltaValues.data());
		thrust::copy(m_particles.d_guessedPosition.begin()+beg, m_particles.d_guessedPosition.begin()+end, duplicatedBody.particles.d_guessedPosition.data());
		
#pragma omp parallel for
		for(int k = 0; k < particleCountDup; ++k) {
			oldFlags[k].otherParticle = getNumParticles(ParticleType::RigidBodyParticle) + k;

			ParticlePortalFlags flag;
			flag.portalHit = oldFlags[k].portalHit;
			flag.oldParticle = 0;
			flag.newParticle = 1;
			flag.otherParticle = beg+k;
			duplicatedBody.particles.d_portalFlags[k] = flag;
		
			vec3 pos = duplicatedBody.particles.d_positionRadius[k].position;
            vec3 guessedPos = duplicatedBody.particles.d_guessedPosition[k].position;
            vec3 velocity = duplicatedBody.particles.d_momentumMass[k].momentumVelocity;

            vec3 posNew = pos;
            vec3 guessedPosNew = guessedPos;
            vec3 newVelocity = velocity;
            if(rbpf.portalHit == 1) {
                // blue portal hit
				transformEverything(posNew, guessedPosNew, newVelocity, *m_bluePortal, *m_orangePortal, pos, guessedPos, velocity);
            } else if(rbpf.portalHit == 2) {
                // orange portal hit
				transformEverything(posNew, guessedPosNew, newVelocity, *m_orangePortal, *m_bluePortal, pos, guessedPos, velocity);
            } else {
                std::cout << "Kann nicht passieren" << std::endl;
            }

            // set changed values
            duplicatedBody.particles.d_positionRadius[k].position = posNew;
            duplicatedBody.particles.d_guessedPosition[k].position = guessedPosNew;
			duplicatedBody.particles.d_momentumMass[k].momentumVelocity = newVelocity;
		}
		thrust::copy(oldFlags.begin(), oldFlags.end(), m_particles.d_portalFlags.begin()+beg);

		// transform the rigid body's position to be inside the other portal
		auto& orient = duplicatedBody.rigidBodies.d_rigidBodiesOrientation[0];
		if(rbpf.portalHit == 1) {
			// blue portal hit
			orient.position = Portal::transformVector(vec4(rbo.position.x(), rbo.position.y(), rbo.position.z(), 1), *m_bluePortal, *m_orangePortal);
			orient.rotation = Portal::transformQuat(rbo.rotation, *m_bluePortal, *m_orangePortal);
		} else if(rbpf.portalHit == 2) {
			// orange portal hit
			orient.position = Portal::transformVector(vec4(rbo.position.x(), rbo.position.y(), rbo.position.z(), 1), *m_orangePortal, *m_bluePortal);
			orient.rotation = Portal::transformQuat(rbo.rotation, *m_orangePortal, *m_bluePortal);
		} else {
			std::cout << "Das darf nicht passieren?" << std::endl;
		}


		manager.unmap();
		spawnRigidBodies(duplicatedBody);
		manager.map();
    }
	manager.unmap();

	m_rigidBodysToDuplicate.clear();
	CUDA_SYNC_CHECK_ERROR();
}

void ParticleSystem::transformEverything(vec3& outPos, vec3& outGuessedPos, vec3& outVelocity, const Portal& portalIn, const Portal& portalOut, vec3 pos, vec3 guessedPos, vec3 velocity) {
	outPos = Portal::transformVector(vec4(pos.x(), pos.y(), pos.z(), 1), portalIn, portalOut);
	outGuessedPos = Portal::transformVector(vec4(guessedPos.x(), guessedPos.y(), guessedPos.z(), 1), portalIn, portalOut);
	outVelocity = Portal::transformVector(vec4(velocity.x(), velocity.y(), velocity.z(), 0), portalIn, portalOut);
	CUDA_SYNC_CHECK_ERROR();
}

void ParticleSystem::dontDuplicateme(int i, int helpSum, HostParticles& h_duplicatedParticles) {
	int begin = m_particlesToDuplicate[i];
	//ParticlePositionRadius
	thrust::copy(m_particles.d_positionRadius.device_begin()+begin, m_particles.d_positionRadius.device_begin()+begin+1, &h_duplicatedParticles.d_positionRadius[i-helpSum]);
	//ParticleColor
	thrust::copy(m_particles.d_color.device_begin()+begin, m_particles.d_color.device_begin()+begin+1, &h_duplicatedParticles.d_color[i-helpSum]);

	//ParticleMomentumMass
	thrust::copy(m_particles.d_momentumMass.begin()+begin, m_particles.d_momentumMass.begin()+begin+1, &h_duplicatedParticles.d_momentumMass[i-helpSum]);

	//ParticleDeltaValues
	thrust::copy(m_particles.d_deltaValues.begin()+begin, m_particles.d_deltaValues.begin()+begin+1, &h_duplicatedParticles.d_deltaValues[i-helpSum]);

	//ParticlePositionRadius
	thrust::copy(m_particles.d_guessedPosition.begin()+begin, m_particles.d_guessedPosition.begin()+begin+1, &h_duplicatedParticles.d_guessedPosition[i-helpSum]);

	// move and rotate particle on right position
	ParticlePortalFlags particleFlags;
	thrust::copy(m_particles.d_portalFlags.begin()+begin, m_particles.d_portalFlags.begin()+begin+1, &particleFlags);


	//ParticlePortalFlags
	h_duplicatedParticles.d_portalFlags[i-helpSum].portalHit = particleFlags.portalHit;
	h_duplicatedParticles.d_portalFlags[i-helpSum].oldParticle = 0.f;
	h_duplicatedParticles.d_portalFlags[i-helpSum].newParticle = 1.f;

	vec3 pos = h_duplicatedParticles.d_positionRadius[i-helpSum].position;
	vec3 guessedPos = h_duplicatedParticles.d_guessedPosition[i-helpSum].position;
	vec3 velocity = h_duplicatedParticles.d_momentumMass[i-helpSum].momentumVelocity;

	vec3 newVelocity;
	vec3 posNew;
	vec3 guessedPosNew;
	if(particleFlags.portalHit == 1) {
		// blue portal hit
		transformEverything(posNew, guessedPosNew, newVelocity, *m_bluePortal, *m_orangePortal, pos, guessedPos, velocity);

	} else if(particleFlags.portalHit == 2) {
		// orange portal hit
		transformEverything(posNew, guessedPosNew, newVelocity, *m_orangePortal, *m_bluePortal, pos, guessedPos, velocity);

	} else {
		std::cout << "Das darf nicht passieren" << std::endl;
	}

	// set changed values
	h_duplicatedParticles.d_positionRadius[i-helpSum].position = posNew;
	h_duplicatedParticles.d_guessedPosition[i-helpSum].position = guessedPosNew;
	h_duplicatedParticles.d_momentumMass[i-helpSum].momentumVelocity = newVelocity;

	m_particlesToDuplicate[i] = -1;

}

void ParticleSystem::duplicateParticles() {

	// only for normal particles for now
    int duplicateParticleLength = m_duplicateParticleLength;
	
	int fluidsBegin = particlesBegin(ParticleType::FluidParticle);
	int fluidsEnd = particlesEnd(ParticleType::FluidParticle);
	int rbBegin = particlesBegin(ParticleType::RigidBodyParticle);
	int rbEnd = particlesEnd(ParticleType::RigidBodyParticle);
	int normalBegin = particlesBegin(ParticleType::RegularParticle);
	int normalEnd = particlesEnd(ParticleType::RegularParticle);
	int clothBegin = particlesBegin(ParticleType::ClothParticle);
	int clothEnd = particlesEnd(ParticleType::ClothParticle);

	int rbpCount = 0;
	int clothCount = 0;
	int newFluids = 0;
	int newRegular = 0;
	std::vector<int> rigidBodys;
	std::vector<int> portalHitPerBody;

	if(duplicateParticleLength > 0) {
		//calclate how many of which particle needs to be duplicated
		for(int i = 0; i < duplicateParticleLength; ++i) {
			int actIndex = m_particlesToDuplicate[i];
			if(rbBegin <= actIndex && actIndex < rbEnd) {		
				rbpCount++;
			} else if(clothBegin <= actIndex && actIndex < clothEnd) {
				// cloth hit portal, don`t know what to do
				clothCount++;
			} else if(fluidsBegin <= actIndex && actIndex < fluidsEnd) {
				// new fluid particles needed
				newFluids++;
			} else if(normalBegin <= actIndex && actIndex < normalEnd) {
				// new normal Particle needed
				newRegular++;
			} else {
				std::cout << "this should not happen!" << std::endl;
			}
		}

		// get GLBufferManager to read from gl memory
		GLBufferManager& manager = GLBufferManager::get();
		manager.map();
		// Regular particles
        HostParticles h_duplicatedParticles(newRegular);
		int helpSum = rbpCount + clothCount + newFluids;
		for(int i = rbpCount + clothCount + newFluids; i < rbpCount + clothCount + newFluids + newRegular; ++i) {
			dontDuplicateme(i, helpSum, h_duplicatedParticles);
		}

		//FLuid particles
		FluidCreator creator(*this);
		FluidData h_duplicatedFluidParticles = creator.create(newFluids, 1, 1, vec3(0.f,0.f,0.f));
        helpSum = rbpCount + clothCount;
		for(int i = rbpCount + clothCount; i < rbpCount + clothCount + newFluids;++i) {
			int begin = m_particlesToDuplicate[i];
			dontDuplicateme(i, helpSum, h_duplicatedFluidParticles.particles);

			//FluidLambda
			thrust::copy(m_fluids.d_lambdas.begin()+begin, m_fluids.d_lambdas.begin()+begin+1, &h_duplicatedFluidParticles.fluids.d_lambdas[i-helpSum]);

			//FluidVorticity
			thrust::copy(m_fluids.d_voriticities.begin()+begin, m_fluids.d_voriticities.begin()+begin+1, &h_duplicatedFluidParticles.fluids.d_voriticities[i-helpSum]);

			//FluidViscosity
			thrust::copy(m_fluids.d_viscosities.begin()+begin, m_fluids.d_viscosities.begin()+begin+1, &h_duplicatedFluidParticles.fluids.d_viscosities[i-helpSum]);
		}

		manager.unmap();
		spawnParticles(h_duplicatedParticles);	
		spawnFluid(h_duplicatedFluidParticles);
	}	
}

void ParticleSystem::deleteParticles() {

    // remove all fluid particles and normal particles 
    int fluidsBegin = particlesBegin(ParticleType::FluidParticle);
    int normalBegin = particlesBegin(ParticleType::RegularParticle);
    for(int i = m_particlesToDelete.size()-1; i >= 0; --i) {
        
        if(m_particlesToDelete[i] < fluidsBegin) {
            break;
        }

        removeParticlesFromBuffers(m_particlesToDelete[i], m_particlesToDelete[i]+1);
    }

}

template<typename T>
inline thrust::host_vector<T> eraseFromArrayView(ArrayView<T>& target, int firstIndex, int lastIndex) {
	// copy the data of particlePositions to the host
	thrust::host_vector<T> helper(target.size());
	thrust::copy(target.device_begin(), target.device_end(), helper.begin());
    // erase data
	helper.erase(helper.begin()+firstIndex, helper.begin()+lastIndex);

	return helper;
}

void ParticleSystem::deleteRigidBodys() {
    RigidBodyManager &rbManager = RigidBodyManager::get(); 

    // rigid bodys to delete:
	for(int x = 0; x < m_rigidBodysToDelete.size(); ++x) {
		int bodyID = m_duplicateMap[m_rigidBodysToDelete[x]];

        RigidBodyParticleData rbpd;
        thrust::copy(m_rigidBodies.d_rigidBodiesParticles.begin()+bodyID, m_rigidBodies.d_rigidBodiesParticles.begin()+bodyID+1, &rbpd);

        RigidBodyAssetHash assetH;
        thrust::copy(m_rigidBodies.d_assetHash.begin()+bodyID, m_rigidBodies.d_assetHash.begin()+bodyID+1, &assetH);			

        int firstParticleIndex = rbpd.firstParticleIndex;
        int particleNumber = rbpd.numParticles;
        
        updateDuplicateParticleID<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_portalFlags
				, m_rigidBodies.d_rigidBodiesParticleOffsets
				, m_rigidBodies.d_rigidBodiesParticleNormals
				, bodyID
				, firstParticleIndex
				, particleNumber);
		CUDA_SYNC_CHECK_ERROR();

        // remove all rigid body particles
        removeParticlesFromBuffers(firstParticleIndex, firstParticleIndex+particleNumber);

        updateRigidBodyData<<<THREAD_BLOCK(m_particleCounts.numRigidBodies, BLOCK_SIZE)>>>(m_rigidBodies.d_rigidBodiesParticles
				, m_rigidBodies.d_rigidBodyPortalFlags
				, bodyID
				, particleNumber);

        m_particleCounts.numRigidBodies--;
        m_rigidBodies.d_rigidBodiesOrientation.erase(m_rigidBodies.d_rigidBodiesOrientation.begin()+bodyID);
        m_rigidBodies.d_rigidBodiesParticles.erase(m_rigidBodies.d_rigidBodiesParticles.begin()+bodyID);
        m_rigidBodies.d_assetHash.erase(m_rigidBodies.d_assetHash.begin()+bodyID);
        m_rigidBodies.d_rigidBodyPortalFlags.erase(m_rigidBodies.d_rigidBodyPortalFlags.begin()+bodyID);

        rbManager.decreaseAssetCount(assetH.assetHash);

		removeFromDuplicationMap(bodyID);
    }

    m_rigidBodysToDelete.clear();
}

void ParticleSystem::removeFromDuplicationMap(int duplicationID) {
	// get the original body of the deleted duplicate
	auto originalIter = std::find_if(m_duplicateMap.begin(), m_duplicateMap.end(), [duplicationID](const std::pair<int, int>& entry){ return entry.second == duplicationID; });
	// check if this is a valid entry (which should always be the case
	if(originalIter == m_duplicateMap.end()) {
		std::cout << "This Duplicate was not registered in m_duplicateMap..." << std::endl;
		throw -1;
	} else {
		// copy the body-duplicate pair
		std::pair<int, int> enter = *originalIter;
		//std::cout << "Removing [" << enter.first << "," << enter.second << "] from m_duplicateMap!" << std::endl;

		// update the deletion list
		for(int z = 0; z < m_rigidBodysToDelete.size(); ++z) {
			int id = m_rigidBodysToDelete[z];
			if(id > duplicationID) {
				m_rigidBodysToDelete[z]--;
			}
		}

		// remove the pair from the map
		m_duplicateMap.erase(originalIter);
		std::vector<std::pair<int,int>> updateNecessary;
		std::vector<int> removingNecessary;

		// go over the list and check which entries must be updated
		for(auto& entry : m_duplicateMap) {
			if(entry.first > duplicationID) {
				// entry.first has to be updated
				int body = entry.first-1;
				int duplicate = entry.second;
				if(entry.second > duplicationID) {
					// entry.second has to be updated too
					duplicate--;
				}
				updateNecessary.push_back(std::pair<int,int>(body,duplicate));
				removingNecessary.push_back(entry.first);
			} else if(entry.second > duplicationID) {
				// only entry.second has to be updated
				m_duplicateMap[entry.first]--;
			}
		}
		for(auto index : removingNecessary) {
			// remove outdated entries from the map
			m_duplicateMap.erase(index);
		}
		for(auto& newEntry : updateNecessary) {
			// add updated entries to the map
			m_duplicateMap[newEntry.first] = newEntry.second;
		}

	}
}

void ParticleSystem::removeParticlesFromBuffers(int firstIndex, int lastIndex) {
    // loop over all buffers and remove all particles from firstIndex to lastIndex
    // all buffers in m_particles
    GLBufferManager& manager = GLBufferManager::get();
    manager.map();
    auto updatedPositionRadius = eraseFromArrayView<ParticlePositionRadius>(m_particles.d_positionRadius, firstIndex, lastIndex);
    manager.getParticlePositions().set(updatedPositionRadius, GL_DYNAMIC_DRAW);
    auto updatedColor = eraseFromArrayView<ParticleColor>(m_particles.d_color, firstIndex, lastIndex);
    manager.getParticleColors().set(updatedColor, GL_DYNAMIC_DRAW);
    
    m_particles.d_momentumMass.erase(m_particles.d_momentumMass.begin()+firstIndex,m_particles.d_momentumMass.begin()+lastIndex);
    m_particles.d_deltaValues.erase(m_particles.d_deltaValues.begin()+firstIndex,m_particles.d_deltaValues.begin()+lastIndex);
    m_particles.d_guessedPosition.erase(m_particles.d_guessedPosition.begin()+firstIndex,m_particles.d_guessedPosition.begin()+lastIndex);
    m_particles.d_portalFlags.erase(m_particles.d_portalFlags.begin()+firstIndex,m_particles.d_portalFlags.begin()+lastIndex);
    
    // all rigid body buffers
    m_rigidBodies.d_rigidBodiesParticleOffsets.erase(m_rigidBodies.d_rigidBodiesParticleOffsets.begin()+firstIndex,m_rigidBodies.d_rigidBodiesParticleOffsets.begin()+lastIndex);
    m_rigidBodies.d_rigidBodiesParticleNormals.erase(m_rigidBodies.d_rigidBodiesParticleNormals.begin()+firstIndex,m_rigidBodies.d_rigidBodiesParticleNormals.begin()+lastIndex);

    // all cloth buffers
    m_cloths.d_clothNeighbors.erase(m_cloths.d_clothNeighbors.begin()+firstIndex,m_cloths.d_clothNeighbors.begin()+lastIndex);

    // all fluid buffers
    m_fluids.d_lambdas.erase(m_fluids.d_lambdas.begin()+firstIndex,m_fluids.d_lambdas.begin()+lastIndex);
    m_fluids.d_voriticities.erase(m_fluids.d_voriticities.begin()+firstIndex,m_fluids.d_voriticities.begin()+lastIndex);
    m_fluids.d_viscosities.erase(m_fluids.d_viscosities.begin()+firstIndex,m_fluids.d_viscosities.begin()+lastIndex);

    int fluidsBegin = particlesBegin(ParticleType::FluidParticle);
    int normalBegin = particlesBegin(ParticleType::RegularParticle);
    int clothBegin = particlesBegin(ParticleType::ClothParticle);

	int numParticles = lastIndex - firstIndex;
    if(firstIndex >= normalBegin) {
        m_particleCounts.regularParticles -= numParticles;
    } else if(firstIndex >= fluidsBegin) {
        m_particleCounts.fluidParticles -= numParticles;
	} else if(firstIndex >= clothBegin) {
		m_particleCounts.clothParticles -= numParticles;
    } else if(firstIndex < clothBegin) {
        m_particleCounts.rigidBodyParticles -= numParticles;
    }

    manager.unmap();
    manager.updateInteropReferences();
    manager.map();
    manager.unmap();
}

