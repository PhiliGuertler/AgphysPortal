#include "agphys.h"

#include "saiga/core/geometry/triangle_mesh_generator.h"
#include "saiga/core/imgui/imgui.h"
#include "saiga/core/math/random.h"
#include "saiga/opengl/shader/shaderLoader.h"

#include "saiga/opengl/assets/assetLoader.h"
#include "cuda_profiler_api.h"

#include "cuda/rigidBodyManager.h"
#include "cuda/clothCreator.h"
#include "cuda/fluidCreator.h"


#include "rendering/WaterRenderer.h"
#include "rendering/PortalRenderer.h"

#include "profiling/Profiler.h"

#include "hacky.hack"

#include "rendering/Scene.h"

#include "cuda/particleSystem/particleSystem.h"

#define COLOR_IN_PARTICLE


Agphys *Agphys::s_instance = nullptr;


Agphys::Agphys() 
	: StandaloneWindow("config.ini")
	, physicsGraph("Physics", 80)
	, enc(window.get())
{
	s_instance = this;

	WaterRenderer::init();
	m_scene = std::make_unique<Scene>();
	//m_portalWalls = std::make_unique<Scene>();
	//HUD
	Image hudImg("objs/Hud.png");
	if(!hudTexture.fromImage(hudImg,true,false)){
		std::cout << "Fehler beim laden\n"<< std::endl;
	}
	hudTexture.generateMipmaps();

	Image hudImg2("objs/Hud2.png");
	if(!hudTexture2.fromImage(hudImg2,true,false)){
		std::cout << "Fehler beim laden\n"<< std::endl;
	}
	hudTexture2.generateMipmaps();

	// init particles
	initParticles();

	std::cout << "Agphys Initialized!" << std::endl;
}

Agphys::~Agphys()
{
	GLBufferManager::shutdown();
	WaterRenderer::shutdown();
	PortalRenderer::shutdown();
	s_instance = nullptr;
	std::cout << "~Agphys" << std::endl;
}

void Agphys::setParticleBuffersToDefaultValues(HostParticles& particles) {
	// Initialize particles
	for(int i = 0; i < particles.d_positionRadius.size(); ++i) {
		// random position
		float distance = 2;
		float offset = 0.1;
		int x = 10;
		int z = 10;
		vec3 corner(30,0,55);
		particles.d_positionRadius[i].position[0] = corner[0] + ((i % x) * distance) + offset;
		particles.d_positionRadius[i].position[1] = corner[1] + ((i / (x * z)) * distance);
		particles.d_positionRadius[i].position[2] = corner[2] + (((i / x) % z) * distance) + offset;
		// default radius (set in the ui)
		particles.d_positionRadius[i].radius = 1.5;
		// guessed position == original position
		particles.d_guessedPosition[i].position = particles.d_positionRadius[i].position;
		// default radius yet again
		particles.d_guessedPosition[i].radius = particles.d_positionRadius[i].radius;
		// default color
		particles.d_color[i].color = make_vec4(linearRand(vec3(0, 0, 0), vec3(1, 1, 1)),1.f);
		// default mass
		particles.d_momentumMass[i].massinv = 5;
	}
}

void Agphys::setupSceneObjects(){
	RigidBodyManager& manager = RigidBodyManager::get();
	//rigidBodys trump wall
	RigidBodyOrientation orientation;
	//height

	orientation.position = vec3(-5.f,1,30.f);
	manager.createRigidCuboid(*particleSystem, orientation);
	//waterfall
	//spawnFluidWithPosition(100,vec3(23.f,30.f,-10.f),10,1);
	//river
	//spawnFluidWithPosition(1000,vec3(40.f,4.f,-10.f),50,1);*/
	spawnParticles(40);
}

void Agphys::initParticles()
{
	ImGuiOptions& options = ImGuiOptions::get();

	// initialize option values
	static bool initializing = true;
	if(initializing) {
		// initialize position of the spawning particles to be centered int the plane-tube
		initializing = false;
		options.massinv = 1.f / ((4.f/3.f) * M_PI * (options.RADIUS_DEFAULT));
		options.distanceReset = 2.f * (options.RADIUS_DEFAULT + 0.1f);
		options.cornerReset = vec3(-options.xReset * 0.5f, options.distanceReset, -options.zReset * 0.5f);
		options.cornerReset *= options.distanceReset;
	}

	// create an empty particleSystem
	particleSystem = std::make_shared<ParticleSystem>(0);
	m_scene->m_planes.clear();

	// register the particle system in the glBufferManager
	GLBufferManager& manager = GLBufferManager::get();
	manager.setParticleSystem(particleSystem);

	// spawn an initial amount of particles
	HostParticles particles(numberParticles);
	setParticleBuffersToDefaultValues(particles);
	particleSystem->spawnParticles(particles);
	
	// relocate the particles
	resetParticles();

	m_ownplayer = std::make_unique<Controls::Player>(*particleSystem, window->getAspectRatio());
	window->setCamera(m_ownplayer->getCamera());

	// update rendering stuff
	manager.getParticlePositions().setDrawMode(GL_POINTS);
	manager.updateInteropReferences();

#if LOAD_PLANES
	// create planes
	m_scene->createPlanes(options.numPlanes, options.planeDistance);
#else
	// load level
	m_scene->loadLevel("objs/level0.obj");
	m_scene->loadWallLevel("objs/level0.obj");
#endif

	//setupSceneObjects();
}

void Agphys::spawnParticles(int numParticles) {
	// create lists of attributes that should be appended to the existing particles
	HostParticles newParticles(numParticles);
	setParticleBuffersToDefaultValues(newParticles);

	// append the data to all non-ArrayViews and retrieve the data of the rest
	particleSystem->spawnParticles(newParticles);
}

void Agphys::spawnRigidCubes(int numCubes) {
	// let the RigidBodyManager create the rigid cubes
	RigidBodyManager& manager = RigidBodyManager::get();
	manager.createRigidCuboid(*particleSystem, numCubes);
}

void Agphys::spawnRigidMeshes(int numInstances) {
	// let the RigidBodyManager create the rigid meshes
	RigidBodyManager& manager = RigidBodyManager::get();
	manager.createRigidMesh(*particleSystem, numInstances);
}

void Agphys::spawnInterlockedCubes() {
	// let the RigidBodyManager create the rigid cubes
	RigidBodyManager& manager = RigidBodyManager::get();
	RigidBodyOrientation position1, position2; 
	// offset the two cubes by the width of a radius in all dimensions
	position1.position = vec3(0.f,5.f,0.f);
	position2.position = vec3(0.5f,5.5f,0.5f);
	manager.createRigidCuboid(*particleSystem, position1);
	manager.createRigidCuboid(*particleSystem, position2);
}

void Agphys::spawnCloth(int N, int M) {
	// get options from the gui
	ImGuiOptions& options = ImGuiOptions::get();
	quat q = quat(Eigen::AngleAxis<float>(options.clothAngle,options.clothNormal));
	// use a ClothCreator to spawn a cloth
	ClothCreator creator(*particleSystem, N, M);
	// spawn the cloth
	auto data = creator.create(options.clothCenter, q, options.fixedClothEdges);
	particleSystem->spawnCloth(data);
}
void Agphys::spawnFluidWithPosition(int numParticles,vec3 corner,float length,float width) {
	FluidCreator creator(*particleSystem);
	//std::cout << numParticles << std::endl;
	auto data = creator.create(numParticles, length, width, corner);

	particleSystem->spawnFluid(data);
}	
void Agphys::spawnFluid(int numParticles) {
	ImGuiOptions& options = ImGuiOptions::get();

	FluidCreator creator(*particleSystem);
	//std::cout << numParticles << std::endl;
	auto data = creator.create(numParticles, options.numFluidParticlesX, options.numFluidParticlesZ, options.cornerFluid);

	particleSystem->spawnFluid(data);
}


void Agphys::destroyParticles()
{
	// reset the gl buffers and CUDA interops
	GLBufferManager& manager = GLBufferManager::get();
	manager.resetInteropReferences();

	// delete the particle system
	particleSystem.reset();
	manager.setParticleSystem(nullptr);

	m_ownplayer->m_capturesCamera = false;
		
	// clear asset and RigidBodyCreator caches
	RigidBodyManager::get().clear();
}

void Agphys::update(float dt) {

#ifdef ENABLE_PROFILING
	Profiling::Profiler& profiler = Profiling::Profiler::getInstance();
	// check if profiling is set
	if(!profiler.sessionIsRunning() && profiler.getNProfileEndFrame() != -1) {
		profiler.beginNSession();
	}
#endif

	// as this would bind interesting buffers, it is not being called in a render function
	WaterRenderer::getInstance().clearBuffers();
	PortalRenderer::get().clearBuffers();
	PortalRenderer::get().updatePortalEffects();

	ImGuiOptions& options = ImGuiOptions::get();
	options.setWindowSize(window->getWidth(), window->getHeight());
	if(options.resolutionChanged) {
		WaterRenderer::getInstance().resize(options.windowWidth, options.windowHeight);
		PortalRenderer::get().resize(options.windowWidth, options.windowHeight);
	}

	if (!ImGui::captureKeyboard()){
		m_ownplayer->updateCamera(dt, *window);
	}

	// update shadow area
	m_scene->m_sun->fitShadowToCamera(m_ownplayer->getCamera());
	m_scene->m_sun->fitNearPlaneToScene(AABB(vec3(-50, 0, -50), vec3(50, 50, 50)));

	// update video encoding thing
	enc.update();

	// don't do anything else if the program is paused
	if (pause) return;

	// map CUDA interops
	if(particleSystem->m_particleCounts.sum() > 0) {
		GLBufferManager& manager = GLBufferManager::get();
		manager.map();

		m_ownplayer->move();

		float t;
		{
			Saiga::CUDA::CudaScopedTimer tim(t);

			PROFILE_SCOPE("ParticleSystem Update");

			try {
				particleSystem->update(dt, *m_ownplayer);
			} catch(int& i) {
				pause = true;
			}
		}

		if(!pause) {
			m_ownplayer->updatePlayerCamera(dt);

			// update the graph displaying physics time and FPS
			physicsGraph.addTime(t);

			// unmap CUDA interops for rendering
			manager.unmap();

			// duplicated everythings that needs to be duplicated
			particleSystem->duplicateParticles();
			particleSystem->duplicateRigidBodys();

			// remove everything thats need to be removed
			particleSystem->deleteParticles();
			particleSystem->deleteRigidBodys();
		}
	}

	// handles input once per frame
	pollInputs();

	// pause after every frame in step mode
	if(m_frameByFrame) {
		pause = true;
	}

#ifdef ENABLE_PROFILING
	if(profiler.sessionIsRunning() && getNumFrames() >= profiler.getNProfileEndFrame()) {
		profiler.endSession();
	}
#endif
	//poll Hud
	if(particleSystem->d_tryeckeWall.size() > 0) {
		vec3 newtmp = m_ownplayer->m_camera->getDirection().head<3>();
		ParticlePositionRadius ray;
		ray.position = vec3(insertVec3(m_ownplayer->m_camera->position));
		ray.radius = 0.f;
		vec3 moveVec = newtmp*100000;
		rayTriangleCollision = particleSystem->getTriangleFromIntersection(ray, moveVec);
	}
}

void Agphys::interpolate(float dt, float interpolation) {
	if (!ImGui::captureMouse()) m_ownplayer->interpolateCamera(dt, interpolation);
}

void Agphys::render(Camera* cam) {
	PortalRenderer::get().renderScene(cam);
}

void Agphys::resetParticles() {
	if(particleSystem->m_particleCounts.sum() > 0) {
		// reset the particleSystem
		GLBufferManager& manager = GLBufferManager::get();
		manager.map();
		ImGuiOptions& options = ImGuiOptions::get();
		// FIXME: this breaks with fluids somehow
		particleSystem->resetParticles(options.xReset, options.zReset, options.cornerReset, options.distanceReset);
		manager.unmap();
	}
}

void Agphys::renderDepth(Camera* cam) {
	// do nothing
}

void Agphys::renderOverlay(Camera* cam) {
	// do nothing
}

void Agphys::renderFinal(Camera* cam) {
	PortalRenderer::get().renderToWindow(window->getWidth(), window->getHeight());

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  
	ivec2 sizeofHud (100,100);
	if(rayTriangleCollision.hit) {
		hudDisplay.render(&hudTexture,ivec2((window->getWidth()/2),(window->getHeight()/2))-sizeofHud/2, sizeofHud, false);
	} else {
		hudDisplay2.render(&hudTexture2,ivec2((window->getWidth()/2),(window->getHeight()/2))-sizeofHud/2, sizeofHud, false);
	}
		
	glDisable(GL_BLEND);

	m_scene->renderImGui();
}

template <>
void VertexBuffer<ParticlePositionRadius>::setVertexAttributes()
{
	// setting the vertex attributes correctly is required, so that the particle shader knows how to read the input
	// data. adding or removing members from the particle class may or may not requires you to change the
	// vertexAttribPointers.
	glEnableVertexAttribArray(0);
	// position radius
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(ParticlePositionRadius), NULL);
}
