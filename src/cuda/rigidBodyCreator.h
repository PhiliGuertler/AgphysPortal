#pragma once

#include <string>

#include "particleSystem/particleSystem.h"

#include "saiga/core/geometry/aabb.h"

#include "../EigenConfig.h"

// ######################################################################### //
// ### RigidBodyCreator #################################################### //
// ######################################################################### //

/**
 *	An interface of a RigidBodyCreator
 */
class RigidBodyCreator {
public:
	/**
	 *	constructor
	 */
	RigidBodyCreator(ParticleSystem& particleSystem, size_t assetID);
	/**
	 *	default destructor
	 */
	virtual ~RigidBodyCreator() = default;

	/**
	 *	creates numBodies Rigid Bodies at random positions and orientations
	*/
	virtual RigidBodyData create(int numBodies);

	/**
	 *	creates a single Rigid Body at a given position with a given orientation
	 */
	virtual RigidBodyData create(RigidBodyOrientation orientation);

	/**
	 *	Initializes the rigid body creator.
	 *	This function exists to allow the comparison with another RigidBodyCreator via hash() and initalize it only once when necessary.
	 *	In other words: Caching is easy with this function
	 */
	virtual void init() = 0;

	/**
	 *	Creates a hash for this RigidBodyCreator that is depending on scale, resolution and assetID.
	 *	Useful for caching.
	 */
	virtual size_t hash();

	inline std::shared_ptr<Saiga::Asset> getAsset() { return m_asset; }

protected:
	/**
	 *	initializes the host particles' position, radius, color and mass
	 */
	virtual void initParticleValues(HostParticles& particles);

	/**
	 *	Computes the signed distance field of this rigid body.
	 */
	void computeSignedDistanceField();

	/**
	 * adds a rigid body to target and sets the firstParticleIndex and
	 * bodyIndices to make it ready to be spawned in particleSystem.
	 */
	void appendHostRigidBody(HostRigidBodies& target, HostRigidBodies& source, int numParticles, int index);

	/**
	 *	creates a random orientation and a random position
	 */
	RigidBodyOrientation randomOrientation();

protected:
	// reference to the particle system
	ParticleSystem& m_particleSystem;

	// --- To be filled by extending classes --- //
	// the number of particles for one rigid body
	int m_numParticles;
	// rigid body offset and normal data wrt to particle 0
	HostRigidBodies m_relativeParticles;
	// width of the voxels
	float m_resolution;
	// asset mesh of this rigid body
	std::shared_ptr<Saiga::TexturedAsset> m_asset;
	// scale of the body
	vec3 m_scale;
	// hash of the asset file name
	size_t m_assetID;
	// 3D-vector of pairs of two ints, the first entry is the value of the sdf at this position, the second one is the particleID
	std::vector<std::vector<std::vector<std::pair<int,int>>>> m_voxelGrid;
};


// ######################################################################### //
// ### RigidCuboidCreator ################################################## //
// ######################################################################### //

/**
 *	Creates a new Rigid Body Cuboid
 */
class RigidCuboidCreator : public RigidBodyCreator {
public:
	/**
	 *	constructor
	 */
	RigidCuboidCreator(ParticleSystem& particleSystem, int xParticles, int yParticles, int zParticles, std::shared_ptr<Saiga::TexturedAsset> asset, size_t assetID);
	/**
	 *	default destructor
	 */
	virtual ~RigidCuboidCreator() = default;

	/**
	 *	override of initialization
	 */
	virtual void init() override;

protected:
	/**
	 *	initializes the voxel grid and spawns the cuboid's particles
	 */
	void initCuboid();

protected:
	int m_xParticles;
	int m_yParticles;
	int m_zParticles;
	vec3 m_centerOfGravity;
};


// ######################################################################### //
// ### RigidMeshCreator #################################################### //
// ######################################################################### //

/**
 *	Creates a new Rigid Body from a Mesh
 */
class RigidMeshCreator : public RigidBodyCreator {
public:
	/**
	 *	constructor
	 */
	RigidMeshCreator(ParticleSystem& particleSystem, float resolution, vec3 scale, std::shared_ptr<Saiga::TexturedAsset> asset, size_t assetID);
	/**
	 *	default destructor
	 */
	virtual ~RigidMeshCreator() = default;

	/**
	 *	override of initialization
	 */
	virtual void init() override;

protected:
	/**
	 *	checks for a given position if this position is inside of the mesh
	 *	by shooting rays in each if the six axis directions
	 */
	bool shootRays(vec3 position);
	/**
	 *	initializes the host particles' position, radius, color and mass
	 *	this override sets the radius depending on the resolution
	 */
	virtual void initParticleValues(HostParticles& particles) override;

	/**
	 *	loads a triangle mesh from filepath and transforms it according to scale
	 */
	Saiga::AABB initTriangleList();
	/**
	 *	computes the relative positions for each particle to be spawned
	 *	and sets the voxel grid accordingly
	 */
	void computeParticleOffsets(const Saiga::AABB& boundingBox);

protected:
	std::string m_filePath;
	std::vector<Saiga::Triangle> m_triangleList;
	vec3 m_centerOfGravity;
};
