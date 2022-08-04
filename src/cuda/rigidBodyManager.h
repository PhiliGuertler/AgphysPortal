#pragma once
#include "rigidBodyCreator.h"

#include <map>				// std::map
#include <functional>		// std::hash

/**
 *	turns a view direction and an up-vector into a quaternion
 */
static inline quat lookAt(vec3 viewDirection, vec3 up) {
	vec3 forward = vec3(0,0,1);
	vec3 rotationAxis = forward.cross(viewDirection);
	float dot = forward.dot(viewDirection);

	quat result;
	result.x() = rotationAxis.x();
	result.y() = rotationAxis.y();
	result.z() = rotationAxis.z();
	result.w() = dot+1;

	return result.normalized();
}


/**
 *	A singleton class that manages rigid body creation
 */
class RigidBodyManager {
public:
	/**
	 *	Singleton Getter
	 */
	static RigidBodyManager& get();

public:
	/**
	 *	default destructor
	 */
	~RigidBodyManager() = default;

	/**
	 *	Creates numMeshes many new rigid bodies for a given particleSystem.
	 *	Assets and scales/resolutions of the particles will be cached.
	 */
	void createRigidMesh(ParticleSystem& particleSystem, int numMeshes);
	/**
	 *	Creates 1 new rigid body with a specific orientation for a given particleSystem.
	 *	Assets and scales/resolutions of the particles will be cached.
	 */
	void createRigidMesh(ParticleSystem& particleSystem, RigidBodyOrientation orientation);
	/**
	 *	Creates 1 new rigid body in the center of the camera with a velocity towards the camera's view direction for a given particleSystem.
	 *	Assets and scales/resolutions of the particles will be cached.
	 */
	void createRigidMesh(ParticleSystem& particleSystem, const Saiga::Ray& cameraRay, float strength);

	/**
	 *	Creates numMeshes many new rigid cuboids for a given particleSystem.
	 *	Assets and scales/resolutions of the particles will be cached.
	 */
	void createRigidCuboid(ParticleSystem& particleSystem, int x, int y, int z);
	/**
	 *	Creates numMeshes many new rigid cuboids for a given particleSystem.
	 *	Assets and scales/resolutions of the particles will be cached.
	 */
	
	void createRigidCuboid(ParticleSystem& particleSystem, int numCuboids);
	/**
	 *	Creates 1 new rigid cuboid with a specific orientation for a given particleSystem.
	 *	Assets and scales/resolutions of the particles will be cached.
	 */
	void createRigidCuboid(ParticleSystem& particleSystem, RigidBodyOrientation orientation);
	
	/**
	 *	Creates 1 new rigid cuboid in the center of the camera with a velocity towards the camera's view direction for a given particleSystem.
	 *	Assets and scales/resolutions of the particles will be cached.
	 */
	void createRigidCuboid(ParticleSystem& particleSystem, const Saiga::Ray& cameraRay, float strength);
	/**
	 *  Duplicates a given rigid body
	 *	Returns the duplicated rigid body data
	 */
	RigidBodyData duplicateRigidBody(ParticleSystem& particleSystem
								   , RigidBodyOrientation bodyOrientation
								   , RigidBodyParticleData bodyParticleData
								   , RigidBodyAssetHash bodyAssetHash);
	/**
	 * Decrease the count of the specified assed
	 */
	void decreaseAssetCount(size_t hash);
	/**
	 *	Renders the meshes for the Rigid Bodies
	 */
	void drawRigidMeshes(Camera *cam, ParticleSystem& particleSystem);
	/**
	 *	renders the mesh shadows for the Rigid Bodies
	 */
	void drawRigidMeshesDepth(Camera *cam, ParticleSystem& particleSystem);
	/**
	 *	Clears all caches of this manager
	 */
	void clear();

private:
	/**
	 *	private constructor to force singleton
	 */
	RigidBodyManager() = default;
	/**
	 *	delete operator= to ensure the correct usage. (RigidBodyManager m = RigidBodyManager::get() will not work).
	 */
	RigidBodyManager& operator=(RigidBodyManager& manager) = delete;
	RigidBodyManager& operator=(RigidBodyManager&& manager) = delete;

	/**
	 *	Loads an asset from disk if it is not found in the cache and adds numInstances to its count
	 *	FIXME: numInstances would be needed with instanced-rendering, which is not implemented.
	 */
	std::shared_ptr<Saiga::TexturedAsset> loadAsset(const std::string& filePath, int numInstances);

	/**
	 *	Loads a creator from cache if it has already been created before 
	 *	or initializes the given creator and caches it for later use
	 */
	std::shared_ptr<RigidBodyCreator> loadCreator(std::shared_ptr<RigidBodyCreator> uninitializedCreator);

	/**
	 *	Helper functions for Rigid Cuboid creation
	 */
	RigidBodyData handleCuboidCreation(ParticleSystem& particleSystem, int x, int y, int z);
	RigidBodyData handleCuboidCreation(ParticleSystem& particleSystem, int numCuboids);
	RigidBodyData handleCuboidCreation(ParticleSystem& particleSystem, RigidBodyOrientation orientation);
	
	/**
	 *
	 *
	 */

	/**
	 *	Helper functions for Rigid Mesh Creation
	 */
	RigidBodyData handleMeshCreation(ParticleSystem& particleSystem, int numMeshes, const std::string& filePath);
	RigidBodyData handleMeshCreation(ParticleSystem& particleSystem, RigidBodyOrientation orientation, const std::string& filePath);
	RigidBodyData handleMeshCreation(ParticleSystem& particleSystem, RigidBodyOrientation orientation, size_t hash); 
private:
	// maps the hash of a filename to a colored asset and the number of instances that exist of it
	std::map<size_t, std::pair<std::shared_ptr<SimpleAssetObject>, int>> m_assetMap;
	// maps the hash of a rigid body creator to said creator
	std::map<size_t, std::shared_ptr<RigidBodyCreator>> m_creators;
};
