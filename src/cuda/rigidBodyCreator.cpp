#include "rigidBodyCreator.h"

#include "imguiOptions.h"
#include "particle.h"

#include "saiga/core/geometry/intersection.h"

#include <functional>

// ######################################################################### //
// ### RigidBodyCreator #################################################### //
// ######################################################################### //

RigidBodyCreator::RigidBodyCreator(ParticleSystem& particleSystem, size_t assetID) 
	: m_particleSystem(particleSystem)
	, m_numParticles(0)
	, m_relativeParticles(0,0)
	, m_resolution(1.f)
	, m_asset()
	, m_scale(vec3(1.f,1.f,1.f))
	, m_assetID(assetID)
	, m_voxelGrid()
{}

size_t RigidBodyCreator::hash() {
	// consider resolution, scale and assetID
	std::hash<float> floatHasher;
	size_t scaleHash = floatHasher(m_scale.x()) + floatHasher(m_scale.y()) * floatHasher(m_scale.z());
	size_t resolutionHash = floatHasher(m_resolution);
	size_t result = m_assetID * scaleHash ^ resolutionHash;
	return result;
}

RigidBodyOrientation RigidBodyCreator::randomOrientation() {
	ImGuiOptions &options = ImGuiOptions::get();

	// create a random position and orientation for the body
	vec3 randomPosition = linearRand(vec3(-options.planeDistance, 0, -options.planeDistance), vec3(options.planeDistance, 40, options.planeDistance));

	// create random rotation matrix that is not uniformly distributed ...heh
	vec3 randomAxis = linearRand(vec3(-1.f, -1.f, -1.f), vec3(1.f, 1.f, 1.f));
	float randomAngle = linearRand(0.f, 2.f * static_cast<float>(M_PI));
	randomAxis.normalize();
	Eigen::AngleAxis<float> rotationMatrix(randomAngle, randomAxis);

	// build a quaternion from the rotation matrix and normalize it
	quat rotation = quat(rotationMatrix);
	rotation.normalize();

	// return the random orientation with the resolution that is stored in the creator
	return {randomPosition, m_resolution, rotation, m_scale, 0.f};
}

void RigidBodyCreator::initParticleValues(HostParticles& particles) {
	ImGuiOptions& options = ImGuiOptions::get();

	// Initialize particles
	for(int i = 0; i < particles.d_positionRadius.size(); ++i) {
		// random position
		particles.d_positionRadius[i].position = linearRand(vec3(-3, 0, -3), vec3(3, 6, 3));
		// default radius (set in the ui)
		particles.d_positionRadius[i].radius = options.radiusReset;
		// guessed position == original position
		particles.d_guessedPosition[i].position = particles.d_positionRadius[i].position;
		// default radius yet again
		particles.d_guessedPosition[i].radius = particles.d_positionRadius[i].radius;
		// default color
		particles.d_color[i].color = vec4(0.1f, 0.8f, 0.2f, 1.f);
		// default mass
		particles.d_momentumMass[i].massinv = options.massinv;
	}
}

void RigidBodyCreator::appendHostRigidBody(HostRigidBodies &target, HostRigidBodies& source, int numParticles, int index) {
	int bodyIndex = index + m_particleSystem.m_particleCounts.numRigidBodies;

	// append the data of m_cuboidParticles to target
	target.d_rigidBodiesOrientation.insert(target.d_rigidBodiesOrientation.end(), source.d_rigidBodiesOrientation.begin(), source.d_rigidBodiesOrientation.end());
	target.d_rigidBodiesParticles.insert(target.d_rigidBodiesParticles.end(), source.d_rigidBodiesParticles.begin(), source.d_rigidBodiesParticles.end());
	target.d_assetHash.insert(target.d_assetHash.end(), source.d_assetHash.begin(), source.d_assetHash.end());
	target.d_rigidBodyPortalFlags.insert(target.d_rigidBodyPortalFlags.end(), source.d_rigidBodyPortalFlags.begin(), source.d_rigidBodyPortalFlags.end());

	target.d_rigidBodiesParticleOffsets.insert(target.d_rigidBodiesParticleOffsets.end(), source.d_rigidBodiesParticleOffsets.begin(), source.d_rigidBodiesParticleOffsets.end());
	target.d_rigidBodiesParticleNormals.insert(target.d_rigidBodiesParticleNormals.end(), source.d_rigidBodiesParticleNormals.begin(), source.d_rigidBodiesParticleNormals.end());

	// set the correct firstParticleIndex of the newly appended rigid body
	int firstParticleIndex = index * numParticles + m_particleSystem.particlesEnd(ParticleType::RigidBodyParticle);

	target.d_rigidBodiesParticles.back().firstParticleIndex = firstParticleIndex;

	// update each particle's bodyIndex
	int localFirstParticle = numParticles * index;

	for (int i = 0; i < numParticles; ++i) {
		target.d_rigidBodiesParticleOffsets[i + localFirstParticle].bodyIndex = bodyIndex;
		target.d_rigidBodiesParticleNormals[i + localFirstParticle].bodyIndex = bodyIndex;
	}
}

RigidBodyData RigidBodyCreator::create(int numBodies) {
	// create new particles that should be appended to the existing particles
	HostParticles newParticles(m_numParticles * numBodies);
	initParticleValues(newParticles);

	// container of all rigid bodies that will be created by this call
	HostRigidBodies rigidBodies(0, 0);

	for (int i = 0; i < numBodies; ++i) {
		appendHostRigidBody(rigidBodies, m_relativeParticles, m_numParticles, i);
		rigidBodies.d_rigidBodiesOrientation.back() = randomOrientation();
	}

	return {newParticles, rigidBodies};
}

RigidBodyData RigidBodyCreator::create(RigidBodyOrientation orientation) {
	HostParticles resultParticles(m_numParticles);
	initParticleValues(resultParticles);

	// container of all rigid bodies that will be created by this call
	HostRigidBodies result(0, 0);

	orientation.resolution = m_resolution;
	orientation.scale = m_scale;

	appendHostRigidBody(result, m_relativeParticles, m_numParticles, 0);
	result.d_rigidBodiesOrientation.back() = orientation;
	result.d_rigidBodiesOrientation.back().resolution = m_resolution;

	return {resultParticles, result};
}

void RigidBodyCreator::computeSignedDistanceField() {
	bool noChangesAnymore = false;
	// compute the values of the sdf for each voxel
	// TODO: handle this in a cuda kernel to speed it up

	// copy the voxelGrid and set all of its sdf-values to 0
	auto copyWorkingMask = m_voxelGrid;
	for(int x = 0; x < copyWorkingMask.size(); ++x) {
		for(int y = 0; y < copyWorkingMask[x].size(); ++y) {
			for(int z = 0; z < copyWorkingMask[x][y].size(); ++z) {
				copyWorkingMask[x][y][z].first = 0;
			}
		}
	}
	// copy the voxelgrid again
	auto copyMask = m_voxelGrid;
	std::cout << "voxelGridSize: " << m_voxelGrid.size() << " y: " << m_voxelGrid[0].size() << " z: " << m_voxelGrid[0][0].size() << std::endl;

	// perform erosion on the voxelgrid
	while(!noChangesAnymore) {
		noChangesAnymore = true;
		copyMask = m_voxelGrid;

		// iterate over all inner gridVoxels
		for(int x = 0; x < m_voxelGrid.size(); ++x) {
			for(int y = 0; y < m_voxelGrid[x].size(); ++y) {
				for(int z = 0; z < m_voxelGrid[x][y].size(); ++z) {
					if(m_voxelGrid[x][y][z].first < 1) {
						// this voxel is already done with computing its sdf
						continue;
					} else {
						copyWorkingMask[x][y][z].first++;
						noChangesAnymore = false;
						// check the surrounding of this voxel
						int left = (x==0) ? 0 : m_voxelGrid[x-1][y][z].first;
						int right = (x==m_voxelGrid.size()-1) ? 0 : m_voxelGrid[x+1][y][z].first;
						int bottom = (y==0) ? 0 : m_voxelGrid[x][y-1][z].first;
						int top = (y==m_voxelGrid[x].size()-1) ? 0 : m_voxelGrid[x][y+1][z].first;
						int front = (z==0) ? 0 : m_voxelGrid[x][y][z-1].first;
						int back = (z==m_voxelGrid[x][y].size()-1) ? 0 : m_voxelGrid[x][y][z+1].first;
						if(left < 1 || right < 1 || top < 1 || bottom < 1 || front < 1 || back < 1) {
							// this voxel is being eroded
							copyMask[x][y][z].first = 0;
						}
					}
				}
			}
		}

		m_voxelGrid = copyMask;
	}
	// erosion completed, copyWorkingMask contains now the values of the sdf
	m_voxelGrid = copyWorkingMask;

	ImGuiOptions& options = ImGuiOptions::get();
	if(options.enablePrints) {
		std::cout << "SDF:" << std::endl;
		for(int z = 0; z < m_voxelGrid[0][0].size(); ++z) {
			for(int y = 0; y < m_voxelGrid[0].size(); ++y) {
				for(int x = 0; x < m_voxelGrid.size(); ++x) {
					std::cout << m_voxelGrid[x][y][z].first;
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
		}
	}

	// compute the gradient of the sdf for each voxel
	for(int x = 0; x < m_voxelGrid.size(); ++x) {
		for(int y = 0; y < m_voxelGrid[x].size(); ++y) {
			for(int z = 0; z < m_voxelGrid[x][y].size(); ++z) {
				// if this voxel has not spawned a particle, ignore it
				if(m_voxelGrid[x][y][z].second == -1) continue;

				// check the surrounding voxels to compute the gradient
				int left = (x==0) ? 0 : m_voxelGrid[x-1][y][z].first;
				int right = (x==m_voxelGrid.size()-1) ? 0 : m_voxelGrid[x+1][y][z].first;
				int bottom = (y==0) ? 0 : m_voxelGrid[x][y-1][z].first;
				int top = (y==m_voxelGrid[x].size()-1) ? 0 : m_voxelGrid[x][y+1][z].first;
				int front = (z==0) ? 0 : m_voxelGrid[x][y][z-1].first;
				int back = (z==m_voxelGrid[x][y].size()-1) ? 0 : m_voxelGrid[x][y][z+1].first;

				vec3 normal = vec3(0.f,0.f,0.f);
				normal.x() = (float)(left-right);
				normal.y() = (float)(bottom-top);
				normal.z() = (float)(front-back);
				normal.normalize();
				if(normal.norm() < 0.001f) {
					// in this case the direction does not matter
					normal = vec3(1.f,0.f,0.f);
				}
				// set the length of the gradient to be slightly less than the value of the sdf at this voxel
				normal *= m_voxelGrid[x][y][z].first * 0.997f;
				
				// store the gradient direction as the normal for this particle
				m_relativeParticles.d_rigidBodiesParticleNormals[m_voxelGrid[x][y][z].second].normal = normal;
			}
		}
	}

	if(options.enablePrints) {
		std::cout << "Normals:" << std::endl;
		for(int z = 0; z < m_voxelGrid[0][0].size(); ++z) {
			for(int y = 0; y < m_voxelGrid[0].size(); ++y) {
				for(int x = 0; x < m_voxelGrid.size(); ++x) {
					if(m_voxelGrid[x][y][z].second == -1) continue;
					vec3 normal = m_relativeParticles.d_rigidBodiesParticleNormals[m_voxelGrid[x][y][z].second].normal;
					std::cout << "[" << normal.x() << "," << normal.y() << "," << normal.z() << "]";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
		}
	}
}

// ######################################################################### //
// ### RigidCuboidCreator ################################################## //
// ######################################################################### //

RigidCuboidCreator::RigidCuboidCreator(ParticleSystem& particleSystem, int xParticles, int yParticles, int zParticles, std::shared_ptr<Saiga::TexturedAsset> asset, size_t assetID) 
	: RigidBodyCreator(particleSystem, assetID)
	, m_xParticles(xParticles)
	, m_yParticles(yParticles)
	, m_zParticles(zParticles)
	, m_centerOfGravity(vec3(0.f,0.f,0.f))
{
	m_scale = vec3(xParticles, yParticles, zParticles)*0.5f;
	m_asset = asset;
}

void RigidCuboidCreator::init() {
	initCuboid();
	computeSignedDistanceField();
}

void RigidCuboidCreator::initCuboid() {
	ImGuiOptions& options = ImGuiOptions::get();

	m_centerOfGravity = {0.f, 0.f, 0.f};
	// compute the relative positions of each particle of the cuboid and the center of gravity
	std::vector<vec3> positions;
	//int particleID = 0;
	for (int x = 0; x < m_xParticles; ++x) {
		std::vector<std::vector<std::pair<int,int>>> kek;
		for (int y = 0; y < m_yParticles; ++y) {
			std::vector<std::pair<int,int>> rolf;
			for (int z = 0; z < m_zParticles; ++z) {
				vec3 position = vec3((float)x,(float)y,(float)z) * (options.radiusReset * 2.f);
				m_centerOfGravity += position;
				rolf.push_back(std::pair<int,int>(1,positions.size()));
				positions.push_back(position);
			}
			kek.push_back(rolf);
		}
		m_voxelGrid.push_back(kek);
	}
	// compute the new center of gravity
	m_numParticles = positions.size();
	std::cout << "numParticles: " << m_numParticles << std::endl;
	// FIXME: don't divide by zero
	m_centerOfGravity /= (float)m_numParticles;

	// initialize m_relativeParticles
	m_relativeParticles = HostRigidBodies(1, m_numParticles);
	m_relativeParticles.d_rigidBodiesParticles[0].firstParticleIndex = 0;
	m_relativeParticles.d_rigidBodiesParticles[0].numParticles = m_numParticles;
	m_relativeParticles.d_assetHash[0].assetHash = m_assetID;

	// adjust offsets to be relative to the center of gravity
	for(int i = 0; i < m_numParticles; ++i) {
		m_relativeParticles.d_rigidBodiesParticleOffsets[i].relativeOffset = positions[i] - m_centerOfGravity;
	}
}

// ######################################################################### //
// ### RigidMeshCreator #################################################### //
// ######################################################################### //

RigidMeshCreator::RigidMeshCreator(ParticleSystem& particleSystem, float resolution, vec3 scale, std::shared_ptr<Saiga::TexturedAsset> asset, size_t assetID)
	: RigidBodyCreator(particleSystem, assetID)
	, m_filePath()
	, m_triangleList()
	, m_centerOfGravity(vec3(0,0,0))
{
	m_scale = scale;
	m_resolution = resolution;
	m_asset = asset;
}

void RigidMeshCreator::init() {
	Saiga::AABB boundingBox = initTriangleList();
	computeParticleOffsets(boundingBox);
	computeSignedDistanceField();
}

Saiga::AABB RigidMeshCreator::initTriangleList() {
	// create a transform of the asset with no translation, no rotation and a scale of m_scale
	Eigen::AngleAxis<float> rotationMatrix(0.f, vec3(0,1.f,0));
	quat rotation = quat(rotationMatrix);
	matrix4 scaling = createTRSmatrix(make_vec4(0.f), rotation, vec4(m_scale[0], m_scale[1], m_scale[2], 1.f));
	matrix4 inverseScaling = createTRSmatrix(make_vec4(0.f), rotation, vec4(1.f/m_scale[0], 1.f/m_scale[1], 1.f/m_scale[2], 1.f));

	// transform the asset and get its triangleList
	m_asset->transform(scaling);
	m_asset->toTriangleList(m_triangleList);
	Saiga::AABB boundingBox = m_asset->aabb();

	// reset the scaling of the asset (as it will be used for instanced drawing)
	m_asset->transform(inverseScaling);

	std::cout << m_triangleList.size() << std::endl;

	// return the axis-aligned bounding-box of the mesh
	return boundingBox;
}

void RigidMeshCreator::computeParticleOffsets(const Saiga::AABB& boundingBox) {
	// every voxel-center that should spawn a new particle will be pushed in here
	std::vector<vec3> positions;

	// iterate over the bounding box grid
	// TODO: handle this in a cuda kernel to speed it up
	for(float x = boundingBox.min.x(); x < boundingBox.max.x() - m_resolution * 0.5f; x += m_resolution) {
		std::vector<std::vector<std::pair<int,int>>> kek;
		for(float y = boundingBox.min.y(); y < boundingBox.max.y() - m_resolution * 0.5f; y += m_resolution) {
			std::vector<std::pair<int,int>> rolf;
			for(float z = boundingBox.min.z(); z < boundingBox.max.z() - m_resolution * 0.5f; z += m_resolution) {
				// compute the voxel center position
				vec3 position = vec3(x,y,z) + make_vec3(m_resolution * 0.5f);
				// cast 6 rays, one in each cardinal direction
				bool letsSpawnAParticle = shootRays(position);
				if(letsSpawnAParticle) {
					// spawn a new particle and set the voxelgrid to have a 1 at this position
					m_centerOfGravity += position;
					rolf.push_back(std::pair<int,int>(1,positions.size()));
					positions.push_back(position);
				} else {
					// don't spawn a new particle and set the voxelgrid to have a 0 at this position
					rolf.push_back(std::pair<int,int>(0,-1));
				}
			}
			kek.push_back(rolf);
		}
		m_voxelGrid.push_back(kek);
	}
	// compute the new center of gravity
	m_numParticles = positions.size();
	std::cout << "numParticles: " << m_numParticles << std::endl;
	// FIXME: don't divide by zero
	m_centerOfGravity /= (float)m_numParticles;

	// initialize m_relativeParticles
	m_relativeParticles = HostRigidBodies(1, m_numParticles);
	m_relativeParticles.d_rigidBodiesParticles[0].firstParticleIndex = 0;
	m_relativeParticles.d_rigidBodiesParticles[0].numParticles = m_numParticles;
	m_relativeParticles.d_assetHash[0].assetHash = m_assetID;

	// adjust offsets to be relative to the center of gravity
	for(int i = 0; i < m_numParticles; ++i) {
		m_relativeParticles.d_rigidBodiesParticleOffsets[i].relativeOffset = positions[i] - m_centerOfGravity;
	}
}

bool RigidMeshCreator::shootRays(vec3 position) {
	// create the six rays to iterate over
	const std::vector<Saiga::Ray> rays = {
		Saiga::Ray(vec3(1,0,0), position),
		Saiga::Ray(vec3(-1,0,0), position),
		Saiga::Ray(vec3(0,1,0), position),
		Saiga::Ray(vec3(0,-1,0), position),
		Saiga::Ray(vec3(0,0,1), position),
		Saiga::Ray(vec3(0,0,-1), position)
	};

	// TODO: handle this in a cuda kernel to speed it up
	for(auto& ray: rays) {
		// save the closest intersection of this ray
		Saiga::Intersection::RayTriangleIntersection minIntersection;
		minIntersection.valid = false;
		minIntersection.backFace = false;
		// iterate over all triangles in the mesh and intersect them with the ray
		for(auto& triangle: m_triangleList) {
			auto intersection = Saiga::Intersection::RayTriangle(ray, triangle);
			if(intersection.valid && intersection.t < minIntersection.t) {
				// update minIntersection if a closer intersection has been found
				minIntersection = intersection;
			}
		}
		// early out by returning true if the particle is inside of the mesh
		if(minIntersection.valid && minIntersection.backFace) return true;
	}
	return false;
}

void RigidMeshCreator::initParticleValues(HostParticles& particles) {
	RigidBodyCreator::initParticleValues(particles);
	// set the radius to half of the resolution
	for(int i = 0; i < particles.d_positionRadius.size(); ++i) {
		particles.d_positionRadius[i].radius = m_resolution * 0.5f;
		particles.d_guessedPosition[i].radius = m_resolution * 0.5f;
	}
}
