#pragma once

#include "saiga/core/geometry/aabb.h"
#include "saiga/cuda/cudaHelper.h"
#include "saiga/cuda/random.h"
#include "saiga/opengl/assets/all.h"
#include "saiga/opengl/assets/objAssetLoader.h"
#include "saiga/opengl/assets/simpleAssetObject.h"
#include "saiga/cuda/interop.h"

#include "saiga/core/geometry/ray.h"

#include "../../Player.h"

#include "../particle.h"

#include "../collisionData.h"
#include "../predicates.h"

#include "../../EigenConfig.h"

// Includes for Portal Game

#include "../logic/portal.h"
#include "../../rendering/Scene.h"


// ### namespace Controls includes ### //
#include "../../Level.h"
// ### /namespace Controls includes ### //

#include "../../imguiOptions.h"

#include <map>

using Saiga::ArrayView;
using Saiga::CUDA::getBlockCount;

struct ParticleCount {
	int regularParticles = 0;
	int rigidBodyParticles = 0;
	int clothParticles = 0;
	int fluidParticles = 0;
	int numRigidBodies = 0;
	
	inline int sum() const {
		return regularParticles + clothParticles + fluidParticles + rigidBodyParticles;
	}

};

struct particlesDuplicateComp {

	__host__ __device__
    bool operator()(const int v1) {
        return (v1 == -1);
    }
};
struct FluidBounds {
	vec3 min = vec3(INFINITY, INFINITY, INFINITY);
	vec3 max = vec3(-INFINITY, -INFINITY, -INFINITY);
};
struct RayTriangleCollision{
	vec3 normal;
	vec3 hitPoint;
	bool hit;

};
struct Tri {
	vec3 a = vec3(NAN, NAN, NAN);
	vec3 aSmoothed = vec3(NAN, NAN, NAN);
	vec3 aNormal = vec3(0,0,0);
	int aIndex = -1;
	float aVorticity = NAN;

	vec3 b = vec3(NAN, NAN, NAN);
	vec3 bSmoothed = vec3(NAN, NAN, NAN);
	vec3 bNormal = vec3(0,0,0);
	int bIndex = -1;
	float bVorticity = NAN;

	vec3 c = vec3(NAN, NAN, NAN);
	vec3 cSmoothed = vec3(NAN, NAN, NAN);
	vec3 cNormal = vec3(0,0,0);
	int cIndex = -1;
	float cVorticity = NAN;
};

// ######################################################################### //
// ### ParticleSystem ###################################################### //
// ######################################################################### //

#define FAME 5

class SAIGA_ALIGN(16) ParticleSystem
{
	public:
		ParticleCount m_particleCounts;

		std::shared_ptr<Portal> m_bluePortal;
		std::shared_ptr<Portal> m_orangePortal;
		bool m_blueSet = false;
		bool m_orangeSet = false;

		int m_portalID = 0;

		bool swapped = false;

		thrust::host_vector<int> m_particlesToDuplicate;
		int m_duplicateParticleLength = 0;
		thrust::host_vector<int> m_particlesToDelete;

		thrust::host_vector<int> m_rigidBodysToDuplicate;
		thrust::host_vector<int> m_rigidBodysToDelete;
		thrust::host_vector<int> m_prevAllHits[FAME];
		thrust::host_vector<int> m_originalRigidBodys;

		// maps bodyID to duplicateID
		// contains all bodies that currently have a duplicate
		std::map<int, int> m_duplicateMap;


	public:
		// ################################################################# //
		// ### particleSystem_init.cu ###################################### //
		/**
		 *	constructor
		*/
		ParticleSystem(int _particleCount);
		/**
		 *	destructor
		*/
		~ParticleSystem();

		/**
		 *	sets planes of the particle system that will be used for collision
		*/
		void registerPlanes(const std::vector<Saiga::Plane>& planes);
		/**
		 *	sets tryecks that will be used for collision
		*/
		void registerLevel(const Controls::Level& level);

		void registerWallOverlay(const Controls::Level& level); 

		/**
		 *	sets Walls of the particle system that will be used for collision
		*/
		//void registerWalls(const std::vector<vec3>& m_triangleList);


		/**
		 *	used to update the arrayview that interops the color gl-buffer
		*/
		void setParticleColors(void *colorPtr);
		/**
		 *	used to update the arrayview that interops the position-radius gl-buffer
		*/
		void setParticlePositions(void *positionPtr);

		/**
		 *	returns the begin index for a given particle type
		*/
		int particlesBegin(ParticleType type);
		/**
		 *	returns the end index for a given particle type
		*/
		int particlesEnd(ParticleType type);
		/**
		 *	returns the amount of particles of a given particle type
		*/
		int getNumParticles(ParticleType type);

		/**
		 *	spawns particles of a given type and inserts them at the end of their
		 *	buffer-location.
		*/
		void spawnParticles(const HostParticles& particles, ParticleType type = ParticleType::RegularParticle);
		/**
		 *	Spawns Rigid Bodies. RigidBodyData can be generated easily by using the
		 *	RigidBodyManager and RigidBodyCreators
		*/
		void spawnRigidBodies(const RigidBodyData& data);
		/**
		 *	Spawns Cloths. ClothData can be generated easily by using a ClothCreator.
		*/
		void spawnCloth(const ClothData& data);
		/**
		 *	Spawns Fluid Particles. FluidData can be generated easily by using a FluidCreator.
		*/
		void spawnFluid(const FluidData& data);

		/**
		 *	Resets the positions and velocities of all particles in the scene.
		*/
		void resetParticles(int x, int z, const vec3& corner, float distance);

		void renderPortals(Camera *cam);

		// ################################################################# //
		// ### particleSystem.cu ########################################### //
		/**
		 *	updates the scene by doing one physics-step
		*/
		void update(float dt, Controls::Player& player);
		/**
		 *	returns a list of model-matrices and their corresponding asset-hash.
		 *	This list can be used for the rendering of rigid bodies
		*/
		std::vector<std::pair<matrix4, size_t>> getModelMatricesOfBodies();
		/**
		 *	performs one step of the position based physics simulation including
		 *	rigid bodies, cloths and fluids
		*/
		template <int NumCollisions>
		void positionBasedRigid(float dt, Controls::Player& player);
		RayTriangleCollision getTriangleFromIntersection(ParticlePositionRadius particle, vec3 movementDirection);
		// ##################################################################### //
		// ### particleSystem_portal.cu ######################################## //
		void setBluePortal(vec3 normal, vec3 center, Scene& scene, vec3 viewDir);
		void setOrangePortal(vec3 normal, vec3 center, Scene& scene, vec3 viewDir);

		void duplicateParticles();
		void transformEverything(vec3& outPos, vec3& outGuessedPos, vec3& outVelocity, const Portal& portalIn, const Portal& portalOut, vec3 pos, vec3 guessedPos, vec3 velocity);

		std::pair<int, bool> createEffectParticles(const Portal& portal, bool isOrange);
		// ##################################################################### //
		// ### particleSystem_portal.cu ######################################## //
		void duplicateRigidBodys();

		void removeParticlesFromBuffers(int firstIndex, int lastIndex);
		void deleteRigidBodys();
		void deleteParticles();
		void printPlayer(Controls::Player& player);
		void removeFromDuplicationMap(int duplicationID);

		// ################################################################# //
		// ### particlesystem_io.cpp ####################################### //
		void renderGUI();
		void renderGravityOptions();
		void renderSpawnOptions();
		void renderForceBasedOptions();
		void renderPositionBasedOptions();
		void renderImplementationOptions();
		void renderLinkedCellOptions();
		void renderMouseGameOptions();
		void renderRigidBodyOptions();
		void renderClothOptions();
		void renderFluidOptions();
		void renderFluidRenderingOptions();
		void renderFluidRenderingUniformOptions();
		void renderPortalOptions();
		
		void renderFluidMesh(Camera *cam);
		void renderFluidMeshDepth(Camera *cam);

		// ##################################################################### //
		// ### particleSystem_mouseInteraction.cu ############################## //
		/**
		 *	intersects the scene with a ray and performs the current mouse-interaction-mode
		 *	on the first particle underneath the cursor
		*/
		void intersectRay(const Saiga::Ray& ray);
		//void intersectRayTriangle(const Saiga::Ray& ray);
		/**
		 *	Spawns particles in the center of the camera and shoots them in view direction
		*/
		void spawnParticlesFromCamera(const Saiga::Ray& cameraRay, int numParticles, float strength);
		/**
		 *	Spawns a rigid body in the center of the camera and shoots them in view direction
		*/
		void spawnRigidBodiesFromCamera(const Saiga::Ray& cameraRay, float strength);
		/**
		 *	Spawns a cloth in the center of the camera and shoots them in view direction
		*/
		void spawnClothFromCamera(const Saiga::Ray& cameraRay, float strength);
		/**
		 *	Spawns fluid particles in the center of the camera and shoots them in view direction
		*/
		void spawnFluidFromCamera(const Saiga::Ray& cameraRay, float strength);

#if 0
		/**
		 *	Move player in view direction
		*/
		void movePlayerFromCamera(const Saiga::Ray& cameraRay, int direction);
		/**
		 *	Get player position
		*/
		vec3 getPlayerPosition();
		void updatePlayerGravity(int dt);
#endif

		// ##################################################################### //
		// ### particleSystem_forceBased.cu #################################### //
		/**
		 *  To be called once per frame.
		 *  This function performs force based particle simulation using an
		 *  an elastic force, a spring force and a friction force.
		 */
		void forceBasedBruteForce(float dt);

		template <int NumCollisions>
		void forceBasedCollisionConstraints(float dt);

		/**
		 *	Calls forceBasedCollisionConstraints with a fixed template argument to
		 *	allow other files to call this function without putting the template
		 *	code in the header
		 */
		void forceBasedCollisionConstraintsHack(float dt);

		//void updateWallPlanes();
		//void renderWalls(Camera *cam);

	private:


		// ##################################################################### //
		// ### particleSystem_init.cu ########################################## //
		void addParticlesToCount(ParticleType type, int amount);

		//void updatePortalPlanes();

		// ##################################################################### //
		// ### particleSystem.cu ############################################### //
		/**
		 *	Sorts particles by type with a given cell length of the linked cell structure
		 */
		void sortParticles(ParticleType type, float cellLength = 1.f);

		// ##################################################################### //
		// ### particleSystem_rigidBodies.cu ################################### //
		void initRigidCubes();
		void rigidBodyShapeConstraintStep();

		// ##################################################################### //
		// ### particleSystem_cloth.cu ######################################### //
		void clothStep(float dt);
		
		// ##################################################################### //
		// ### particleSystem_fluids.cu ######################################## //
		void initConstantMemory();
		void fluidStep();
		void vorticityStep(float dt);

		// ##################################################################### //
		// ### particleSystem_stabilization.cu ################################# //
		void stabilizationStep(float dt, int planeCollisionLength, int tryeckCollisionLength, int particleCollisionLength);

		// ##################################################################### //
		// ### particleSystem.cu ############################################### //
		void resolveParticleStep(float dt, int particleCollisionLength);
		void resolvePlanesStep(float dt, int planeCollisionLength);
		void resolveWallsStep(float dt, int wallCollisionLength);
		void resolveTryeckStep(float dt, int tryeckCollisionLength);
		void updatePositions(float dt);


		// ##################################################################### //
		// ### particleSystem_portal.cu ######################################## //
		void portalStep(Controls::Player& player);
		void collideRigidBodysPortal(Controls::Player& player);

		// helpers
		void internalRegisterIntersectingBodies(std::vector<int>& allHitRigidBodys);
		void internalRegisterDuplicationParticles(std::vector<int>& allHitRigidBodys, std::vector<int>& newRigidBodys);
		void internalCheckForInstantSwaps(thrust::host_vector<int>& h_centerOfRigidBodys, Controls::Player& player);
		void internalCheckForSwaps(thrust::host_vector<int>& h_centerOfRigidBodys, Controls::Player& player);
		void internalRemoveDuplicates(std::vector<int>& allHitRigidBodys);
		

		inline float getFluidRadius() const { return m_fluidRadius; }

		// ##################################################################### //
		// ### particleSystem_marchingCubes.cu ################################# //
		FluidBounds computeFluidBounds(float gridSize);
		void resizeBuffers(ivec3 voxelDimensions, int numFluidParticles, ArrayView<vec2>& voxelValues, ArrayView<int>& buckets, ArrayView<int>& overflows, ArrayView<Tri>& triangles, ArrayView<int>& edgeIndices, ArrayView<int>& validTriangles);

		void doMarchingCubes();


		void dontDuplicateme(int i, int helpSum, HostParticles& h_duplicatedParticles);

	public:
		thrust::device_vector<Saiga::Plane> d_planes;
		thrust::device_vector<Tryeck> d_tryecke;
		thrust::device_vector<Tryeck> d_tryeckeWall;
		thrust::device_vector<vec3> d_collisionTryecke;
		thrust::device_vector<vec3> d_collisionTryecke2;
		thrust::device_vector<CollisionPlaneData> d_activePlaneCollisions;
		thrust::device_vector<CollisionParticleData> d_activeParticleCollisions;
		thrust::device_vector<CollisionTryeckData> d_activeTryeckCollisions;

	
		float m_maxRadius = 0.5f;
		// TODO: set this if the radius changes
		float m_fluidRadius = 0.5f;
		// an array of size N containing one entry per hashbucket

		thrust::device_vector<int> d_hashBuckets;
		// an array of the same size as d_particles used for bucket collisions
		thrust::device_vector<int> d_hashOverflow;
		// speedup for fluids:
		thrust::device_vector<int> d_beginIndicesFluids;
		thrust::device_vector<int> d_numParticlesPerBucket;

		Particles m_particles;
		RigidBodies m_rigidBodies;
		Cloths m_cloths;
		Fluids m_fluids;	
		const int BLOCK_SIZE = 128;

		// ### Portal Specifics ################################################## //
		thrust::device_vector<int> d_particlesToDuplicate;
		thrust::device_vector<int> d_particlesToDelete;
		thrust::device_vector<int> d_particlesInPortals;

		// ### Marching Cubes Specifics ########################################## //
		thrust::device_vector<vec2> d_marchingCubeVoxelValues;
		thrust::device_vector<Tri> d_marchingCubeTriangles;
		thrust::device_vector<int> d_marchingCubeBuckets;
		thrust::device_vector<int> d_marchingCubeOverflow;
		
		thrust::device_vector<int> d_marchingEdgeIndices;
		thrust::device_vector<int> d_validTriangles;
		thrust::device_vector<int> lel;

		bool m_firstRigidBodyHasHitSomeWalls;

		bool m_forcePause;
};
