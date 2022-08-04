#pragma once

#include "../../AgphysCudaConfig.h"
#include "saiga/cuda/cudaHelper.h"
#include "saiga/cuda/interop.h"

#include "../particle.h"

#include <functional>

/* TODO:
 * Rigid Bodys sollen Portale nur durchqueren, wenn sie komplett durchpassen
 * Nicht mehr benutzte Partikel sollen entfernt werden
 * Partikel beeinflussen sich gegenseitig
 * Funktionen in Unterfunktionen auslagern
 * Orientierung der Rigid Bodys anpassen
 */


class Portal {
    public:

		__host__ __device__
			inline static vec3 transformVector(const vec4& input, const Portal& portalIn, const Portal& portalOut) {
				// rotation in the portal-space to flip from portalIn's front to portalOut's back
				Eigen::AngleAxis<float> turn180(M_PI, vec3(0,1,0));
				mat4 rotation = mat4::Identity();
				rotation.block(0,0,3,3) = turn180.toRotationMatrix();

				vec4 result = portalOut.model() * rotation * portalIn.model().inverse() * input;
				return result.head<3>();
			}

		__host__ __device__
			inline static mat4 transformView(const mat4& input, const Portal& portalIn, const Portal& portalOut) {
				// rotation in the portal-space to flip from portalIn's front to portalOut's back
				Eigen::AngleAxis<float> turn180(M_PI, vec3(0,1,0));
				mat4 rotation = mat4::Identity();
				rotation.block(0,0,3,3) = turn180.toRotationMatrix();

				mat4 result = input * portalOut.model() * rotation * portalIn.model().inverse();
				return result;
			}

		__host__ __device__
			inline static quat transformQuat(const quat& input, const Portal& portalIn, const Portal& portalOut) {
				// rotation in the portal-space to flip from portalIn's front to portalOut's back
				Eigen::AngleAxis<float> turn180(M_PI, vec3(0,1,0));
				mat4 rotation = mat4::Identity();
				rotation.block(0,0,3,3) = turn180.toRotationMatrix();

				mat4 quatRotation = mat4::Identity();
				quatRotation.block(0,0,3,3) = input.toRotationMatrix();

				mat4 transform = portalOut.model() * rotation * portalIn.model().inverse() * quatRotation;
				mat3 resultRotation = transform.block(0,0,3,3);
				auto result = quat(resultRotation);
				return result;
			}

        /**
	     *	constructor.
		 *	The callback function will be called upon creation, e.g. to spawn some effect particles
	    */
        Portal();

		bool intersectsPortal(vec3 position, vec3 movementVector);

        /**
         *  methode to set the portal to a new position
         */
        bool reposition(vec3 normal, vec3 centerPos, Portal &otherPortal, vec3 viewDir);

		// getters for components in world-space
		__host__ __device__ inline vec3 normal() const {
			vec4 tmp = model()*vec4(0,0,1,0);
			return vec3(tmp.x(), tmp.y(), tmp.z());
		}
		__host__ __device__ inline vec3 center() const {
			vec4 tmp = model()*vec4(0,0,0,1);
			return vec3(tmp.x(), tmp.y(), tmp.z());
		}
		__host__ __device__ inline vec3 bottomLeft() const {
			vec4 tmp = model()*vec4(-1,-1,0,1);
			return vec3(tmp.x(), tmp.y(), tmp.z());
		}
		__host__ __device__ inline vec3 bottomRight() const {
			vec4 tmp = model()*vec4(1,-1,0,1);
			return vec3(tmp.x(), tmp.y(), tmp.z());
		}
		__host__ __device__ inline vec3 topLeft() const {
			vec4 tmp = model()*vec4(-1,1,0,1);
			return vec3(tmp.x(), tmp.y(), tmp.z());
		}
		__host__ __device__ inline vec3 topRight() const {
			vec4 tmp = model()*vec4(1,1,0,1);
			return vec3(tmp.x(), tmp.y(), tmp.z());
		}
		__host__ __device__ inline mat4 model() const {
			return m_model;
		}

    public:
		mat4 m_model;
        
		Tryeck m_edgeCollisionTryecke[8];
};

/**
 *  function for intersecting a portal with a particle
 */
__device__ bool intersectPortalParticle(Portal portal, vec3 pos, vec3 moveVector, vec3 guessedPos, float radius);

/**
 *  kernel for portal particle intersection
 */
 __global__ void portalParticleIntersection(ArrayView<ParticlePositionRadius> particlePositions
                                           , ArrayView<ParticleColor> particleColor
                                           , ArrayView<ParticleMomentumMass> velos
                                           , ArrayView<ParticlePositionRadius> particleGuessedPositions
                                           , ArrayView<ParticlePortalFlags> particlePortalFlags
                                           , ArrayView<int> particlesToDuplicateIndices
                                           , ArrayView<int> particlesToDeleteIndices
                                           , ArrayView<int> particlesStillIntersecting
                                           , Portal firstPortal
                                           , Portal secondPortal);

__device__ bool intersectPortalTriangleParticle(vec3 direction, vec3 origin, vec3 A, vec3 B, vec3 C, float radius);

__global__ void intersectRigidBodyCenterPortal(ArrayView<RigidBodyOrientation> rigidBodyOrientation
                                             , ArrayView<int> rigidBodyIndex
                                             , ArrayView<RigidBodyParticleData> particleData
                                             , ArrayView<ParticleMomentumMass> moms
											 , ArrayView<ParticlePositionRadius> particlePositions
                                             , ArrayView<ParticlePositionRadius> particleGuessedPositions
                                             , float radius
                                             , Portal firstPortal
                                             , Portal secondPortal); 

__global__ void swapRigidBodys(int numberParticles
                             , int firstOwnParticle
                             , int firstDuplicatedParticle
                             , ArrayView<ParticlePositionRadius> particlePositions
                             , ArrayView<ParticleMomentumMass> velos
                             , ArrayView<ParticlePositionRadius> particleGuessedPositions
                             , ArrayView<ParticlePortalFlags> particlePortalFlags);
            
__global__ void placeBorderParticles(int numberParticles                                   
                                    , int portalID
                                    , int firstParticle
                                    , vec3 directionX
                                    , vec3 directionY
                                    , vec3 leftCorner
                                    , vec3 color
                                    , ArrayView<ParticlePositionRadius> particlePositions
                                    , ArrayView<ParticleDeltaValues> particleDelta
                                    , ArrayView<ParticleColor> particleColor);

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
									, float edgeWidth);
