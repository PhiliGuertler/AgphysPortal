#include "particleSystem_mouseInteraction.h"

#include "saiga/core/util/assert.h"
#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"

#include "particleSystem_collisions.inl"
#include "saiga/core/geometry/intersection.h"
#include "../rigidBodyManager.h"
#include "../clothCreator.h"
#include "../fluidCreator.h"

#include "../../imguiOptions.h"

// ######################################################################### //
// ### CUDA Kernels ######################################################## //
// ######################################################################### //

// ######################################################################### //
// ### Handle Rays ######################################################### //

__global__ void handleMouseRay(ArrayView<ParticlePositionRadius> particlePos, Saiga::Ray ray, ArrayView<CollisionRayData> collisionData) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particlePos.size()) return;

	ParticlePositionRadius particle = particlePos[ti.thread_id];

	CollisionRayData intersectionData = collideSphereRay(particle, ray);
	intersectionData.particleID = ti.thread_id;
	intersectionData.rayOrigin = ray.origin;

	collisionData[ti.thread_id] = intersectionData;
}



// ######################################################################### //
// ### Other ############################################################### //

template <typename T>
__device__ T powDevice(T base, T exponent) {
	T result = base;
	for(T i = 0; i < exponent; i += 1) {
		result *= base;
	}
	return result;
}

// ######################################################################### //
// ### Mouse Games ######################################################### //

__global__ void colorizeParticle(ArrayView<ParticleColor> particleColors, int index, vec4 color, bool resetRemaining, vec4 resetColor) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particleColors.size()) return;

	ParticleColor& particleColor = particleColors[ti.thread_id];

	if(ti.thread_id == index)  {
		particleColor.color = color;
	} else if(resetRemaining) {
		particleColor.color = resetColor;
	}
}

__global__ void growParticle(ArrayView<ParticleMomentumMass> particleMass
							, ArrayView<ParticlePositionRadius> particlePos
							, ArrayView<ParticlePositionRadius> particleGuessedPos
							, int index, float growthFactor) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particleMass.size()) return;

	ParticleMomentumMass& mass = particleMass[ti.thread_id];
	ParticlePositionRadius& pos = particlePos[ti.thread_id];
	ParticlePositionRadius& guessedPos = particleGuessedPos[ti.thread_id];

	if(ti.thread_id == index)  {
		float radius = pos.radius + (growthFactor-1.f);
		radius = max(radius, 0.0001f);

		pos.radius = radius;
		guessedPos.radius = radius;

		mass.massinv = 1.f / ( (4.f/3.f) * M_PI * powDevice<float>(radius + 1.f, 5.f));
	}
}

__global__ void shootParticle(ArrayView<ParticleMomentumMass> particleVelocities, int index, vec3 direction, float strength) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particleVelocities.size()) return;

	ParticleMomentumMass& velo = particleVelocities[ti.thread_id];
	if(ti.thread_id == index) {
		velo.momentumVelocity += direction * strength;
		
	}
}
__global__ void moveParticle(ArrayView<ParticlePositionRadius> rorientation,int index, vec3 direction){
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= rorientation.size()) return;

	ParticlePositionRadius& pos = rorientation[ti.thread_id];
	if(ti.thread_id == index) {
		pos.position += direction;
		
	}

}
__global__ void setGravity(ArrayView<ParticleMomentumMass> particleVelocities,int index,float dt, vec3 acceleration) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= particleVelocities.size()) return;

	ParticleMomentumMass& velo = particleVelocities[ti.thread_id];
	if(ti.thread_id == index) {
		velo.momentumVelocity += dt * acceleration;
	}
}

// ######################################################################### //
// ### C++ ParticleSystem ################################################## //
// ######################################################################### //

static void initParticleValues(HostParticles& particles) {
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
		particles.d_color[i].color = vec4(0.9f, 0.1f, 0.12f, 1.f);
		// default mass
		particles.d_momentumMass[i].massinv = options.massinv;
	}
}

void ParticleSystem::spawnParticlesFromCamera(const Saiga::Ray& cameraRay, int numParticles, float strength) {
	vec3 origin = cameraRay.origin;
	vec3 direction = cameraRay.direction * strength;

	HostParticles particles(numParticles);
	initParticleValues(particles);
#pragma omp parallel for
	for(int i = 0; i < numParticles; ++i) {
		particles.d_momentumMass[i].momentumVelocity = direction;
		particles.d_guessedPosition[i].position = origin;
		particles.d_positionRadius[i].position = origin;
	}

	spawnParticles(particles);
}

void ParticleSystem::spawnRigidBodiesFromCamera(const Saiga::Ray& cameraRay, float strength) {
	ImGuiOptions& options = ImGuiOptions::get();
	RigidBodyManager& manager = RigidBodyManager::get();
	if(options.spawnCubes) {
		manager.createRigidCuboid(*this, cameraRay, strength);
	} else {
		manager.createRigidMesh(*this, cameraRay, strength);
	}
}

void ParticleSystem::spawnClothFromCamera(const Saiga::Ray& cameraRay, float strength) {
	ImGuiOptions& options = ImGuiOptions::get();

	quat orientation = lookAt(cameraRay.direction, vec3(0,1,0));
	ClothCreator creator = ClothCreator(*this, options.clothN, options.clothM);
	ClothData data = creator.create(cameraRay.origin, orientation, false);

#pragma omp parallel for
	for(int i = 0; i < data.particles.d_momentumMass.size(); ++i) {
		vec3 velo = cameraRay.direction * strength;
		data.particles.d_momentumMass[i].momentumVelocity = velo;
	}

	spawnCloth(data);
}

void ParticleSystem::spawnFluidFromCamera(const Saiga::Ray& cameraRay, float strength) {
	ImGuiOptions& options = ImGuiOptions::get();

	FluidCreator creator(*this);
	auto data = creator.create(cameraRay, strength, options.fluidWidth, options.fluidHeight);
	spawnFluid(data);

}

void ParticleSystem::intersectRay(const Saiga::Ray& ray) {
	ImGuiOptions& options = ImGuiOptions::get();

	CollisionRayData minElement;
	if(m_particleCounts.sum() > 0) {
		thrust::device_vector<CollisionRayData> collisionData(m_particleCounts.sum());
		handleMouseRay<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_positionRadius, ray, collisionData);
		minElement = *thrust::min_element(collisionData.begin(), collisionData.end(), CollisionRayData());
	}

	switch(options.mouseGame) {
	case MouseClickGame::Colorize:
		if(m_particleCounts.sum() > 0) {
			colorizeParticle<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_color, minElement.particleID, options.highlightParticleColor, options.resetHighlights, options.defaultParticleColor);
		}
		break;
	case MouseClickGame::Grow:
		if(m_particleCounts.sum() > 0) {
			growParticle<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_momentumMass, m_particles.d_positionRadius, m_particles.d_guessedPosition, minElement.particleID, options.growthFactor);
		}
		break;
	case MouseClickGame::Shoot:
		if(m_particleCounts.sum() > 0) {
			shootParticle<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_momentumMass, minElement.particleID, ray.direction, options.shootStrength);
		}
		break;
	case MouseClickGame::SpawnParticles:
		spawnParticlesFromCamera(ray, options.numParticlesToBeSpawned, options.shootStrength);
		break;
	case MouseClickGame::SpawnRigidBodies:
		spawnRigidBodiesFromCamera(ray, options.shootStrength);
		break;
#if 0
	case MouseClickGame::SpawnCloths:
		spawnClothFromCamera(ray, options.shootStrength);
		break;
#endif
	case MouseClickGame::SpawnFluids:
		spawnFluidFromCamera(ray, options.shootStrength);
		break;
	default:
		std::cout << "No such Mousegame yet!" << std::endl;
	}
}
