#include "rigidBodyManager.h"

#include "../imguiOptions.h"

// ######################################################################### //
// ### RigidBodyManager #################################################### //
// ######################################################################### //

RigidBodyManager& RigidBodyManager::get() {
	static RigidBodyManager instance;
	return instance;
}
	
// ######################################################################### //
// ### private helper functions ############################################ //

std::shared_ptr<Saiga::TexturedAsset> RigidBodyManager::loadAsset(const std::string& filePath, int numInstances) {
	size_t hash = std::hash<std::string>()(filePath);
	
	// check if this asset has already been loaded
	if(m_assetMap.find(hash) == m_assetMap.end()) {
		// this is a new asset, add it to the map
		ObjAssetLoader loader;
		//FIXME: this will crash if the .mtl file is broken
		auto asset = loader.loadTexturedAsset(filePath);
		std::shared_ptr<SimpleAssetObject> assetObject = std::make_shared<SimpleAssetObject>();
		assetObject->asset = asset;
		m_assetMap[hash] = std::pair<std::shared_ptr<SimpleAssetObject>, int>(assetObject, numInstances);
	} else {
		m_assetMap[hash].second += numInstances;
	}

	return std::dynamic_pointer_cast<Saiga::TexturedAsset>(m_assetMap[hash].first->asset);
}

std::shared_ptr<RigidBodyCreator> RigidBodyManager::loadCreator(std::shared_ptr<RigidBodyCreator> uninitializedCreator) {
	size_t hash = uninitializedCreator->hash();

	// check if this creator is already listed
	if(m_creators.find(hash) == m_creators.end()) {
		// this is a new creator, add it to the map
		uninitializedCreator->init();
		m_creators[hash] = uninitializedCreator;
	}

	return m_creators[hash];
}


RigidBodyData RigidBodyManager::handleCuboidCreation(ParticleSystem& particleSystem, int x, int y, int z) {
	std::string filePath = "./objs/box.obj";
	auto asset = loadAsset(filePath, 1);
	size_t hash = std::hash<std::string>()(filePath);
	
	// create a RigidBodyCreator
	std::shared_ptr<RigidBodyCreator> creator = std::make_shared<RigidCuboidCreator>(particleSystem, x, y, z, asset, hash);

	// spawn the rigid bodies in the particle system
	RigidBodyOrientation orient;
	//care this was changed for the player 
	orient.position = vec3(10.f,3,10.f);
	orient.rotation = quat(Eigen::AngleAxis<float>(0.0001f, vec3(0.f,1.f,0.f)));
	auto data = loadCreator(creator)->create(orient);
	return data;
}



RigidBodyData RigidBodyManager::handleCuboidCreation(ParticleSystem& particleSystem, RigidBodyOrientation orientation) {
	//ImGuiOptions& options = ImGuiOptions::get();

	std::string filePath = "./objs/companionCube.obj";
	auto asset = loadAsset(filePath, 1);
	size_t hash = std::hash<std::string>()(filePath);
	
	// create a RigidBodyCreator
	std::shared_ptr<RigidBodyCreator> creator = std::make_shared<RigidCuboidCreator>(particleSystem, 3, 3, 3, asset, hash);

	// spawn the rigid bodies in the particle system
	auto data = loadCreator(creator)->create(orientation);
	return data;
}


RigidBodyData RigidBodyManager::handleMeshCreation(ParticleSystem& particleSystem, int numMeshes, const std::string& filePath) {
	ImGuiOptions& options = ImGuiOptions::get();

	// retrieve the string of the mesh to be loaded
	auto asset = loadAsset(filePath, numMeshes);
	size_t hash = std::hash<std::string>()(filePath);

	// create a RigidBodyCreator
	std::shared_ptr<RigidBodyCreator> creator = std::make_shared<RigidMeshCreator>(particleSystem, options.meshResolution, options.meshScale, asset, hash);

	auto data = loadCreator(creator)->create(numMeshes);
	return data;
}

RigidBodyData RigidBodyManager::handleMeshCreation(ParticleSystem& particleSystem, RigidBodyOrientation orientation, const std::string& filePath) {
	ImGuiOptions& options = ImGuiOptions::get();

	// retrieve the string of the mesh to be loaded
	auto asset = loadAsset(filePath, 1);
	size_t hash = std::hash<std::string>()(filePath);

	// create a RigidBodyCreator
	std::shared_ptr<RigidBodyCreator> creator = std::make_shared<RigidMeshCreator>(particleSystem, options.meshResolution, options.meshScale, asset, hash);

	auto data = loadCreator(creator)->create(orientation);
	return data;
}

RigidBodyData RigidBodyManager::handleMeshCreation(ParticleSystem& particleSystem, RigidBodyOrientation orientation, size_t hash) {

	m_assetMap[hash].second++;
	auto asset = std::dynamic_pointer_cast<Saiga::TexturedAsset>(m_assetMap[hash].first->asset);

	// create a RigidBodyCreator
	std::shared_ptr<RigidBodyCreator> creator = std::make_shared<RigidMeshCreator>(particleSystem, orientation.resolution, orientation.scale, asset, hash);

	auto data = loadCreator(creator)->create(orientation);
	return data;
}

RigidBodyData RigidBodyManager::handleCuboidCreation(ParticleSystem& particleSystem, int numCuboids) {
       ImGuiOptions& options = ImGuiOptions::get();
 
       std::string filePath = "./objs/box.obj";
       auto asset = loadAsset(filePath, numCuboids);
       size_t hash = std::hash<std::string>()(filePath);
       
       // create a RigidBodyCreator
       std::shared_ptr<RigidBodyCreator> creator = std::make_shared<RigidCuboidCreator>(particleSystem, options.xParticles, options.yParticles, options.zParticles, asset, hash);

       // spawn the rigid bodies in the particle system
       auto data = loadCreator(creator)->create(numCuboids);
       return data;
}

// ######################################################################### //
// ### public methods ###################################################### //

void RigidBodyManager::decreaseAssetCount(size_t hash) {
	m_assetMap[hash].second--;
}

void RigidBodyManager::createRigidCuboid(ParticleSystem& particleSystem, int x, int y, int z) {
	auto data = handleCuboidCreation(particleSystem, x, y, z);
	particleSystem.spawnRigidBodies(data);
}

void RigidBodyManager::createRigidCuboid(ParticleSystem& particleSystem, int numCuboids) {
	auto data = handleCuboidCreation(particleSystem, numCuboids);
	particleSystem.spawnRigidBodies(data);
}

void RigidBodyManager::createRigidCuboid(ParticleSystem& particleSystem, RigidBodyOrientation orientation) {
	auto data = handleCuboidCreation(particleSystem, orientation);
	particleSystem.spawnRigidBodies(data);	
}

void RigidBodyManager::createRigidCuboid(ParticleSystem& particleSystem, const Saiga::Ray& cameraRay, float strength) {
	RigidBodyOrientation orientation;
	orientation.position = cameraRay.origin;
	orientation.rotation = lookAt(cameraRay.direction, vec3(0,1,0));
	
	auto data = handleCuboidCreation(particleSystem, orientation);

	// set the velocity of each particle to fly in the view direction of the camera
#pragma omp parallel for
	for(int i = 0; i < data.particles.d_momentumMass.size(); ++i) {
		data.particles.d_momentumMass[i].momentumVelocity = cameraRay.direction * strength;
		data.particles.d_color[i].color = vec4(0.9f, 0.1f, 0.12f, 1.f);
	}

	particleSystem.spawnRigidBodies(data);
}


void RigidBodyManager::createRigidMesh(ParticleSystem& particleSystem, int numMeshes) {
	ImGuiOptions& options = ImGuiOptions::get();
	std::string filePath = std::string(options.meshFilePath);
	auto data = handleMeshCreation(particleSystem, numMeshes, filePath);
	particleSystem.spawnRigidBodies(data);
}

void RigidBodyManager::createRigidMesh(ParticleSystem& particleSystem, RigidBodyOrientation orientation) {
	ImGuiOptions& options = ImGuiOptions::get();
	std::string filePath = std::string(options.meshFilePath);
	auto data = handleMeshCreation(particleSystem, orientation, filePath);
	particleSystem.spawnRigidBodies(data);
}

void RigidBodyManager::createRigidMesh(ParticleSystem& particleSystem, const Saiga::Ray& cameraRay, float strength) {
	RigidBodyOrientation orientation;
	orientation.position = cameraRay.origin;
	orientation.rotation = lookAt(cameraRay.direction, vec3(0,1,0));

	ImGuiOptions& options = ImGuiOptions::get();
	std::string filePath = std::string(options.meshFilePath);
	auto data = handleMeshCreation(particleSystem, orientation, filePath);

#pragma omp parallel for
	for(int i = 0; i < data.particles.d_momentumMass.size(); ++i) {
		data.particles.d_momentumMass[i].momentumVelocity = cameraRay.direction * strength;
	}

	particleSystem.spawnRigidBodies(data);
}

RigidBodyData RigidBodyManager::duplicateRigidBody(ParticleSystem& particleSystem, RigidBodyOrientation bodyOrientation, RigidBodyParticleData bodyParticleData, RigidBodyAssetHash bodyAssetHash) {
	auto data = handleMeshCreation(particleSystem, bodyOrientation, bodyAssetHash.assetHash);
	return data;
}


void RigidBodyManager::drawRigidMeshes(Camera *cam, ParticleSystem& particleSystem) {
	// FIXME: draw instanced and don't copy the model matrices.
	// instead, use an interop'd gl-buffer
	auto matrices = particleSystem.getModelMatricesOfBodies();
	
	for(auto& matrixPair : matrices) {
		auto& renderAsset = m_assetMap[matrixPair.second].first;
		if(renderAsset != nullptr) {
			renderAsset->model = matrixPair.first;
			renderAsset->render(cam);
		}
	}
}

void RigidBodyManager::drawRigidMeshesDepth(Camera *cam, ParticleSystem& particleSystem) {
	// FIXME: draw instanced and don't copy the model matrices.
	// instead, use an interop'd gl-buffer
	auto matrices = particleSystem.getModelMatricesOfBodies();
	
	for(auto& matrixPair : matrices) {
		auto& renderAsset = m_assetMap[matrixPair.second].first;
		if(renderAsset != nullptr) {
			renderAsset->model = matrixPair.first;
			renderAsset->renderDepth(cam);
		}
	}
}


void RigidBodyManager::clear() {
	m_assetMap.clear();
	m_creators.clear();
}
