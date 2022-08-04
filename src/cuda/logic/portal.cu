#include "portal.h"

#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"
#include "saiga/core/geometry/intersection.h"

#include "../particleSystem/particleSystem_collisions.inl"

#define PORTALSIZE 5
#define PORTALWIDTH 5
#define PORTALHEIGHT 5

__host__ __device__ 
inline bool particleIntersectsPortal(ParticlePositionRadius particle, vec3 movementDirection, Portal portal) {
	bool result = false;

	vec3 bottomLeft = portal.bottomLeft();
	vec3 bottomRight = portal.bottomRight();
	vec3 topLeft = portal.topLeft();
	vec3 topRight = portal.topRight();
	vec3 normal = portal.normal();

	result = particleIntersectsTriangle(particle, movementDirection, bottomLeft, bottomRight, topRight, normal);
	result |= particleIntersectsTriangle(particle, movementDirection, bottomLeft, topRight, topLeft, normal);

	return result;
}

__host__ __device__ 
inline vec3 particleIntersectsPortal2(ParticlePositionRadius particle, vec3 movementDirection, Portal portal) {
	vec3 bottomLeft = portal.bottomLeft();
	vec3 bottomRight = portal.bottomRight();
	vec3 topLeft = portal.topLeft();
	vec3 topRight = portal.topRight();
	vec3 normal = portal.normal();

	vec3 result = intersectParticleTriangle(particle, movementDirection, bottomLeft, bottomRight, topRight, normal);
	if(result.x() != result.x()) {
		result = intersectParticleTriangle(particle, movementDirection, bottomLeft, topRight, topLeft, normal);
	}
	return result;
}

// ######################################################################### //
// ############# Portal #################################################### //
// ######################################################################### //

Portal::Portal()
	: m_model(mat4::Identity())
{
	// empty
}


bool Portal::intersectsPortal(vec3 position, vec3 movementVector) {
	ParticlePositionRadius dummy;
	dummy.position = position;
	dummy.radius = 0.f;
	vec3 intersectionPoint = particleIntersectsPortal2(dummy, movementVector, *this);
	return intersectionPoint.x() == intersectionPoint.x();
}

bool Portal::reposition(vec3 normal, vec3 centerPos, Portal &otherPortal, vec3 viewDir) {

	if(normal.dot(otherPortal.normal()) > 0.99f) {
		vec3 minDistance = otherPortal.center()-otherPortal.bottomLeft();
		if((centerPos - otherPortal.center()).norm() < minDistance.norm()*2.f) {
			std::cerr << "Portal cannot be placed here!" << std::endl;
			return false;
		}
	}
    
    // calculate vectors in portal plane: up and normal are enough
    vec3 helper = normal.cross(vec3(0.f,1.f,0.f)).normalized();
	vec3 up;
	vec3 axis;
	float angle;
	Eigen::AngleAxis<float> rot;
    if(helper.norm() < 0.001) {
        // boden oder decke
        up = vec3(viewDir.x(),0,viewDir.z()).normalized();

		axis = vec3(1,0,0);
		angle = M_PI*0.5f;
		float sign = (viewDir.dot(vec3(0,1,0))) > 0.f ? -1.f : 1.f;
		rot = Eigen::AngleAxis<float>(sign*angle, axis);

		axis = vec3(0,1,0);
		angle = acos(up.dot(vec3(0,0,1)));
		sign = (up-vec3(0,0,1)).x() > 0 ? 1.f : -1.f;
		rot = Eigen::AngleAxis<float>(sign*angle, axis) * rot;

    } else {
        up = normal.cross(vec3(0.f,1.f,0.f)).cross(normal).normalized();

		axis = vec3(1,0,0);
		angle = acos(up.dot(vec3(0,1,0)));
		float sign = (up-vec3(0,1,0)).z() > 0 ? 1.f : -1.f;
		rot = Eigen::AngleAxis<float>(sign*angle, axis);

		vec3 toll = vec3(normal.x(), 0, normal.z()).normalized();

		axis = vec3(0,1,0);
		angle = acos(toll.dot(vec3(0,0,1)));
		sign = (toll-vec3(0,0,1)).x() > 0 ? 1.f : -1.f;
		rot = Eigen::AngleAxis<float>(sign*angle, axis) * rot;
    }

	quat rotation = quat(rot);
	m_model = createTRSmatrix(vec4(centerPos.x(), centerPos.y(), centerPos.z(), 1), rotation, vec4(PORTALWIDTH, PORTALHEIGHT, 1, 1));

    std::cout << "new portal set: [" << centerPos.x() << "," << centerPos.y() << "," << centerPos.z() << "], normal: [" << normal.x() << "," << normal.y() << "," << normal.z() << "]" << std::endl;
	return true;
}


// ######################################################################### //
// ### CUDA Kernels ######################################################## //
// ######################################################################### //

#define DEBUG_PORTAL_INTERSECTION 0

__global__ void portalParticleIntersection(ArrayView<ParticlePositionRadius> particlePositions
                                          , ArrayView<ParticleColor> particleColor
                                          , ArrayView<ParticleMomentumMass> velos
                                          , ArrayView<ParticlePositionRadius> particleGuessedPositions
                                          , ArrayView<ParticlePortalFlags> particlePortalFlags
                                          , ArrayView<int> particlesToDuplicateIndices
                                          , ArrayView<int> particlesToDeleteIndices
										  , ArrayView<int> particlesStillIntersecting
                                          , Portal firstPortal
                                          , Portal secondPortal) {

    Saiga::CUDA::ThreadInfo<> ti;
    if(ti.thread_id >= particlePositions.size()) return;

	// load particle data from memory
	ParticlePositionRadius position;
	Saiga::CUDA::vectorCopy(particlePositions.data()+ti.thread_id, &position);

	ParticleMomentumMass velo;
	Saiga::CUDA::vectorCopy(velos.data()+ti.thread_id, &velo);

	ParticlePositionRadius guessedPosition;
    Saiga::CUDA::vectorCopy(particleGuessedPositions.data()+ti.thread_id, &guessedPosition);

    ParticlePortalFlags portalFlags;
    Saiga::CUDA::vectorCopy(particlePortalFlags.data()+ti.thread_id, &portalFlags);

    // test intersection with both portals
    vec3 pos = position.position;
    vec3 guessedPos = guessedPosition.position;
    vec3 moveVector = guessedPos - pos;

    // test with first portal
    vec3 firstPortalHit = particleIntersectsPortal2(position, moveVector, firstPortal);
	bool firstTrue = firstPortalHit.x() == firstPortalHit.x();

    // test with second portal
    vec3 secondPortalHit = particleIntersectsPortal2(position, moveVector, secondPortal);
	bool secondTrue = secondPortalHit.x() == secondPortalHit.x();
    
    bool newHit = (firstTrue) || (secondTrue);
    if(firstTrue) {
        // blue
        portalFlags.portalHit = 1;
    } else if(secondTrue) {
        // orange
        portalFlags.portalHit = 2;
    } else {
		// no hit
        portalFlags.portalHit = 0;
    }

	if(!newHit) {
		// this particle is not hitting a portal
		if(portalFlags.newParticle == 1) {
			// this particle has a duplicate. Flag it to return to normal
			portalFlags.newParticle = 0;
			portalFlags.oldParticle = 0;
		} else if(portalFlags.oldParticle == 1) {
			// this particle is a duplicate. Flag it for deletion.
			particlesToDeleteIndices[ti.thread_id] = ti.thread_id;
		}
	} else if(portalFlags.oldParticle == 0 && portalFlags.newParticle == 0) {
		// this particle is hitting a portal and needs a duplicate. Flag it for duplication
        portalFlags.oldParticle = 1;
		particlesToDuplicateIndices[ti.thread_id] = ti.thread_id;
	} else {
		// This particle is hitting a portal and has a duplicate. Flag it anyways
		particlesStillIntersecting[ti.thread_id] = ti.thread_id;
	}
    Saiga::CUDA::vectorCopy(&portalFlags, particlePortalFlags.data()+ti.thread_id);
}

__global__ void intersectRigidBodyCenterPortal(ArrayView<RigidBodyOrientation> rigidBodyOrientation
                                             , ArrayView<int> rigidBodyIndex
                                             , ArrayView<RigidBodyParticleData> particleData
                                             , ArrayView<ParticleMomentumMass> moms
                                             , ArrayView<ParticlePositionRadius> particlePositions
                                             , ArrayView<ParticlePositionRadius> particleGuessedPositions
                                             , float radius
                                             , Portal firstPortal
                                             , Portal secondPortal) {
    Saiga::CUDA::ThreadInfo<> ti;
    if(ti.thread_id >= rigidBodyOrientation.size()) return;

    RigidBodyOrientation rbOrientation;
    Saiga::CUDA::vectorCopy(rigidBodyOrientation.data()+ti.thread_id, &rbOrientation);

    RigidBodyParticleData data;
    Saiga::CUDA::vectorCopy(particleData.data()+ti.thread_id, &data);

    ParticlePositionRadius positionCenter;
    positionCenter.position = rbOrientation.position;
    positionCenter.radius = radius;

	vec3 velo = vec3(0,0,0);
	for(int i = data.firstParticleIndex; i < data.firstParticleIndex+data.numParticles; ++i) {
        ParticlePositionRadius pos;
        ParticlePositionRadius guessedPos;
        Saiga::CUDA::vectorCopy(particlePositions.data()+i, &pos);
        Saiga::CUDA::vectorCopy(particleGuessedPositions.data()+i, &guessedPos);

        velo += (guessedPos.position - pos.position);
		
    }
    velo /= data.numParticles; 

    bool firstPortalHit = particleIntersectsPortal(positionCenter, velo, firstPortal);

    bool secondPortalHit = particleIntersectsPortal(positionCenter, velo, secondPortal);

    if(firstPortalHit || secondPortalHit) {
        rigidBodyIndex[ti.thread_id] = 1;
    }

}

template <typename T>
__device__ inline void swap(int firstParticle, int secondParticle, ArrayView<T> data) {
    T tmp;
    Saiga::CUDA::vectorCopy(data.data()+firstParticle, &tmp);
    Saiga::CUDA::vectorCopy(data.data()+secondParticle, data.data()+firstParticle);
    Saiga::CUDA::vectorCopy(&tmp, data.data()+secondParticle);
}

__global__ void swapRigidBodys(int numberParticles
                             , int firstOwnParticle
                             , int firstDuplicatedParticle
                             , ArrayView<ParticlePositionRadius> particlePositions
                             , ArrayView<ParticleMomentumMass> velos
                             , ArrayView<ParticlePositionRadius> particleGuessedPositions
                             , ArrayView<ParticlePortalFlags> particlePortalFlags) {
    
    Saiga::CUDA::ThreadInfo<> ti;
    if(ti.thread_id >= numberParticles) return;

    int first = ti.thread_id+firstOwnParticle;
    int second = ti.thread_id+firstDuplicatedParticle;

    swap<ParticlePositionRadius>(first, second, particlePositions);
    swap<ParticleMomentumMass>(first, second, velos);
    swap<ParticlePositionRadius>(first, second, particleGuessedPositions);
   
    ParticlePortalFlags ownPortalFlags;
    Saiga::CUDA::vectorCopy(particlePortalFlags.data()+ti.thread_id + firstOwnParticle, &ownPortalFlags);
    if(ownPortalFlags.oldParticle == 1) {
        ownPortalFlags.newParticle = 1;
        ownPortalFlags.oldParticle = 0;
    }   
    Saiga::CUDA::vectorCopy(&ownPortalFlags, particlePortalFlags.data()+ti.thread_id + firstOwnParticle);

    ParticlePortalFlags duplicatedPortalFlags;
    Saiga::CUDA::vectorCopy(particlePortalFlags.data()+ti.thread_id + firstDuplicatedParticle, &duplicatedPortalFlags);
    if(duplicatedPortalFlags.newParticle == 1) {
        duplicatedPortalFlags.newParticle = 0;
        duplicatedPortalFlags.oldParticle = 1;
    } 
    Saiga::CUDA::vectorCopy(&duplicatedPortalFlags, particlePortalFlags.data()+ti.thread_id + firstDuplicatedParticle);

}

enum class PortalEdge {
	Bottom,
	Left,
	Top,
	Right
};

struct EdgePlane {
	Tryeck t1;
	Tryeck t2;
};

__device__ inline EdgePlane getPortalEdge(Portal portal, PortalEdge edge, float width) {
	// create the two edge tryecks
	EdgePlane result;

	vec3 bl = portal.bottomLeft();
	vec3 br = portal.bottomRight();
	vec3 tl = portal.topLeft();
	vec3 tr = portal.topRight();
	vec3 normal = portal.normal();

	vec3 left, right;
	switch(edge) {
		case PortalEdge::Bottom:
			left = bl;
			right = br;
			break;
		case PortalEdge::Left:
			left = tl;
			right = bl;
			break;
		case PortalEdge::Top:
			left = tr;
			right = tl;
			break;
		case PortalEdge::Right:
			left = br;
			right = tr;
			break;
		default:
			left = vec3(0,0,0);
			right = vec3(0,0,0);
			break;
	}

	result.t1.a = left + normal * width * 0.5f;
	result.t1.b = right - normal * width * 0.5f;
	result.t1.c = left - normal * width * 0.5f;

	result.t2.a = left + normal * width * 0.5f;
	result.t2.b = right + normal * width * 0.5f;
	result.t2.c = right - normal * width * 0.5f;

	return result;
}

#define atomicAdd3(target, source) atomicAdd(&target[0], source[0]); atomicAdd(&target[1], source[1]); atomicAdd(&target[2], source[2])

__device__ void updatePortalEdgeDelta(int particleID
		, vec3 intersectionPoint
		, ParticlePositionRadius guessedPos
		, vec3 tryeckNormal
		, ParticleDeltaValues& delta
		, vec3 pos
		, ParticleMomentumMass mom
		, int fluidsBegin
		, int fluidsEnd
		, bool enableFriction
		, float staticFriction
		, float kineticFriction) {

	if(intersectionPoint.x() != intersectionPoint.x()) {
		return;
	}

	vec3 distance = intersectionPoint - guessedPos.position;
	distance += tryeckNormal * guessedPos.radius;

	if(enableFriction) {
		// don't do the friction for fluid particles
		if(particleID < fluidsBegin || particleID >= fluidsEnd) {
			Friction friction = doTheFriction(distance, guessedPos.position, pos, vec3(0.f,0.f,0.f), vec3(0.f,0.f,0.f), mom.massinv, 0.f, staticFriction, kineticFriction);
			distance += friction.delta1;
		}
	}


	atomicAdd3(delta.delta, distance);
}

__global__ void intersectPortalEdges(ArrayView<ParticlePositionRadius> positions
									, ArrayView<ParticlePositionRadius> guessedPositions
									, ArrayView<ParticlePortalFlags> portalFlags
									, ArrayView<RigidBodyParticleOffset> offsets
									, ArrayView<RigidBodyPortalFlags> rbPortalFlags
									, ArrayView<ParticleDeltaValues> deltas
									, ArrayView<ParticleMomentumMass> moms
									, int fluidsBegin
									, int fluidsEnd
									, bool enableFriction
									, float kineticFriction
									, float staticFriction
									, Portal bluePortal
									, Portal orangePortal
									, float edgeWidth) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= positions.size()) return;

	ParticlePortalFlags flags;
	Saiga::CUDA::vectorCopy(portalFlags.data()+ti.thread_id, &flags);

	Portal *portal;

	// only handle particles that are intersecting a portal
	if(flags.portalHit == 1) {
		// blue portal hit
		portal = &bluePortal;
	} else if(flags.portalHit == 2) {
		// orange portal hit
		portal = &orangePortal;
	} else {
		// check if this particle has a rigid body
		RigidBodyParticleOffset offset;
		Saiga::CUDA::vectorCopy(offsets.data()+ti.thread_id, &offset);

		if(offset.bodyIndex != -1) {
			// this particle is part of a rigid body
			// load the portalflags of this rigid body
			RigidBodyPortalFlags rbFlags;
			Saiga::CUDA::vectorCopy(rbPortalFlags.data()+offset.bodyIndex, &rbFlags);

			if(rbFlags.portalHit == 1) {
				// blue portal hit
				portal = &bluePortal;
			} else if(rbFlags.portalHit == 2) {
				// orange portal hit
				portal = &orangePortal;
			} else {
				// neither this particle nor its rigid body intersect a rigid body
				return;
			}
		} else {
			// the particle is not part of a rigid body and is not intersecting a portal
			return;
		}
	}

	PortalEdge edges[4];
	edges[0] = PortalEdge::Top;
	edges[1] = PortalEdge::Left;
	edges[2] = PortalEdge::Bottom;
	edges[3] = PortalEdge::Right;

	ParticlePositionRadius pos;
	Saiga::CUDA::vectorCopy(positions.data()+ti.thread_id, &pos);

	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(guessedPositions.data()+ti.thread_id, &guessedPos);

	ParticleMomentumMass mom;
	Saiga::CUDA::vectorCopy(moms.data()+ti.thread_id, &mom);

	ParticleDeltaValues& delta = deltas[ti.thread_id];

	vec3 movementVector = guessedPos.position - pos.position;

	for(int i = 0; i < 4; ++i) {
		EdgePlane plane = getPortalEdge(*portal, edges[i], edgeWidth);

		vec3 normal = plane.t1.normal();

		vec3 intersection = intersectParticleTriangle(pos, movementVector, plane.t1.a, plane.t1.b, plane.t1.c, normal);
		updatePortalEdgeDelta(ti.thread_id
		, intersection
		, guessedPos
		, normal
		, delta
		, pos.position
		, mom
		, fluidsBegin
		, fluidsEnd
		, enableFriction
		, staticFriction
		, kineticFriction);

//updatePortalEdgeDelta(intersection, guessedPos, normal, delta);

		intersection = intersectParticleTriangle(pos, movementVector, plane.t2.a, plane.t2.b, plane.t2.c, plane.t2.normal());
		updatePortalEdgeDelta(ti.thread_id
		, intersection
		, guessedPos
		, normal
		, delta
		, pos.position
		, mom
		, fluidsBegin
		, fluidsEnd
		, enableFriction
		, staticFriction
		, kineticFriction);

//updatePortalEdgeDelta(intersection, guessedPos, normal, delta);
	}
}
