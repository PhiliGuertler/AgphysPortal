#include "Scene.h"

#include "../agphys.h"
#include "../imguiOptions.h"
#include "../cuda/fluidCreator.h"
#include "../profiling/Profiler.h"
#include "../cuda/rigidBodyManager.h"
#include "../hacky.hack"

#include "WaterRenderer.h"
#include "PortalRenderer.h"

#define loadShader(x) Saiga::shaderLoader.load<Saiga::MVPShader>(x)

#define VECCC(a) a.x(), a.y(), a.z()

// ######################################################################### //
// ### TexturedPortal ###################################################### //
// ######################################################################### //

void TexturedPortal::fromPortal(const Portal& portal) {
	loadDefaultShaders();

	// create vertex data
	std::vector<VertexNT> vertexData;
	VertexNT bl(vec4(-1,-1,0,1.f), vec4(0,0,1,0.f), vec2(0,0));
	VertexNT br(vec4(1,-1,0,1.f), vec4(0,0,1,0.f), vec2(0,1));
	VertexNT tl(vec4(-1,1,0,1.f), vec4(0,0,1,0.f), vec2(1,0));
	VertexNT tr(vec4(1,1,0,1.f), vec4(0,0,1,0.f), vec2(1,1));
	vertexData.push_back(bl);
	vertexData.push_back(br);
	vertexData.push_back(tl);
	vertexData.push_back(tr);

	vertices.resize(4);
	for(int i = 0; i < 4; ++i) {
		vertices[i] = vertexData[i];
	}

	// create index data
	std::vector<uint32_t> indices = {0,1,2, 1,3,2};

	// FIXME: this line breaks
	//buffer.set(vertexData, indices, GL_STATIC_DRAW);
	buffer.fromMesh(*this);
	buffer.set(vertices, indices, GL_DYNAMIC_DRAW);
	buffer.setDrawMode(GL_TRIANGLES);
}

static inline vec2 vertexToTC(Camera *cam, const mat4& model, const vec4& vertex) {
	ImGuiOptions& options = ImGuiOptions::get();
	vec2 result = cam->projectToScreenSpace(make_vec3(vertex), options.windowWidth, options.windowHeight);
	result.x() /= options.windowWidth;
	result.y() /= options.windowHeight;

	return result;
}
		
void TexturedPortal::updateTextureCoords(Camera *cam, const mat4& model) {
	// do nothing
}

void TexturedPortal::loadDefaultShaders() {
	this->shader          = shaderLoader.load<MVPShader>(deferredShaderStr);
	this->forwardShader   = shaderLoader.load<MVPShader>(forwardShaderStr);
	this->depthshader     = shaderLoader.load<MVPShader>(depthShaderStr);
	this->wireframeshader = shaderLoader.load<MVPShader>(wireframeShaderStr);
}

void TexturedPortal::render(Camera *cam, const mat4& model) {
	if(shader == nullptr) return;

	ImGuiOptions& options = ImGuiOptions::get();
	vec2 screenSize = vec2(options.windowWidth, options.windowHeight);

	shader->bind();
	shader->uploadModel(model);

	updateTextureCoords(cam, model);

	auto location = shader->getUniformLocation("image");
	portalTex->bind(1);
	shader->upload(location, 1);

	location = shader->getUniformLocation("u_screenSize");
	shader->upload(location, screenSize);

	buffer.bindAndDraw();
	buffer.unbind();

	shader->unbind();
}

void TexturedPortal::renderDepth(Camera *cam, const mat4& model) {
	if(depthshader == nullptr) return;

	depthshader->bind();
	depthshader->uploadModel(model);

	auto location = shader->getUniformLocation("image");
	portalTex->bind(1);
	shader->upload(location, 1);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);
	buffer.bindAndDraw();
	buffer.unbind();

	depthshader->unbind();
}

// ######################################################################### //
// ### Scene ############################################################### //
// ######################################################################### //

Scene::Scene()
	: m_objAssetLoader()
	, m_groundPlane()
	, m_planes()
	, m_skybox()
	, m_orangePortalAsset()
	, m_bluePortalAsset()
	, m_sun()
	, m_particleShader()
	, m_particleShaderFast()
	, m_particleDepthShader()
	, m_particleDepthShaderFast()
	, m_mockupCamera()
	, m_renderingOrange(true)
	, m_shouldRenderGUI(true)
	, m_showSaigaGUI(false)
	, m_renderParticles(true)
	, m_renderShadows(true)
{
	ImGuiOptions& options = ImGuiOptions::get();

	// get agphys instance
	auto agphys = Agphys::getInstance();

	// set up mockup camera
	m_mockupCamera = std::make_unique<PerspectivePlayerCamera>(agphys->window->getAspectRatio());

#if LOAD_PLANES
	// set up ground plane
	m_groundPlane.asset = m_objAssetLoader.loadDebugPlaneAsset(vec2(500, 500), 1.f);
#endif

	// set up light source of the sun
	vec3 lightDir = vec3(-1,-3,-2).normalized();
	options.lightDirection = lightDir;

	m_sun = agphys->renderer->lighting.createDirectionalLight();
	m_sun->setDirection(lightDir);
	m_sun->setColorDiffuse(vec4(.8f,.8f,1.f,.6f));
	m_sun->setAmbientIntensity(.3f);

	m_sun->createShadowMap(2048,2048,3);
	m_sun->enableShadows();


	// initialize shaders for particles
	Saiga::ShaderPart::ShaderCodeInjections si = { Saiga::ShaderCodeInjection(GL_FRAGMENT_SHADER, "#define WRITE_DEPTH", 2) };

	m_particleShader = loadShader("shader/particleBoost.glsl");
	m_particleDepthShader = loadShader("shader/particlesDepth.glsl");
	m_particleShaderFast = loadShader("shader/particles.glsl");
	m_particleDepthShaderFast = loadShader("shader/particlesDepth.glsl");

	// create the skybox
	// step 1: load the cube map images
	std::vector<Image> skyboxImages;
	skyboxImages.push_back(Image("textures/canyon_0.jpg"));
	skyboxImages.push_back(Image("textures/canyon_1.jpg"));
	skyboxImages.push_back(Image("textures/canyon_4.jpg"));
	skyboxImages.push_back(Image("textures/canyon_5.jpg"));
	skyboxImages.push_back(Image("textures/canyon_2.jpg"));
	skyboxImages.push_back(Image("textures/canyon_3.jpg"));
	// step 2: create the cube map itself
	m_skybox.cube_texture = std::make_shared<TextureCube2>();
	m_skybox.cube_texture->fromImage(skyboxImages);
	// step 3: set shader and model of the skybox
	m_skybox.shader = Saiga::shaderLoader.load<MVPTextureShader>("shader/skybox.glsl");
	m_skybox.model(3,3) = 1.f;

	PortalRenderer& p = PortalRenderer::get();
	p.m_recursiveRenderer->m_lighting.addDirectionalLight(m_sun);
	p.registerRenderingInterface(this);
}


Scene::~Scene() {
	// FIXME: this will cause a segmentation fault after the program has started its shutdown sequence
#define WE_WANT_TO_USE_DIFFERENT_SCENES_DYNAMICALLY 0
#if WE_WANT_TO_USE_DIFFERENT_SCENES_DYNAMICALLY
	PortalRenderer& p = PortalRenderer::get();
	p.m_recursiveRenderer->m_lighting.removeLight(m_sun);
	p.m_recursiveRenderer->rendering = nullptr;
#endif
}

void Scene::updatePortalPlane(const Portal& portal, bool isOrange) {
	std::shared_ptr<TexturedPortal> asset;
	if(isOrange) {
		if(m_orangePortalAsset.asset != nullptr) {
			asset = std::dynamic_pointer_cast<TexturedPortal>(m_orangePortalAsset.asset);
		} else {
			// initialize the orange portal asset object
			asset = std::make_shared<TexturedPortal>();
			asset->portalTex = PortalRenderer::get().m_orangePortalColorTex[PortalRenderer::get().m_activeOrangePortalBuffer];
			asset->fromPortal(portal);
			m_orangePortalAsset.asset = asset;
		}
		// set portal model matrix
		m_orangePortalAsset.model = portal.model();
	} else {
		if(m_bluePortalAsset.asset != nullptr) {
			asset = std::dynamic_pointer_cast<TexturedPortal>(m_bluePortalAsset.asset);
		} else {
			// initialize the blue portal asset object
			asset = std::make_shared<TexturedPortal>();
			asset->portalTex = PortalRenderer::get().m_bluePortalColorTex[PortalRenderer::get().m_activeBluePortalBuffer];
			asset->fromPortal(portal);
			m_bluePortalAsset.asset = asset;
		}
		// set portal model matrix
		m_bluePortalAsset.model = portal.model();
	}

}



// ######################################################################### //
// ### Scene Rendering ##################################################### //
// ######################################################################### //

inline void setupMockupCamera(Camera *mockupCam, const Saiga::mat4& view, const Saiga::mat4& proj) {
	mockupCam->setView(view);
	mockupCam->setProj(proj);
}

void Scene::renderPortal(const Saiga::mat4& view, const Saiga::mat4& proj, bool isOrange) {
	setupMockupCamera(m_mockupCamera.get(), view, proj);
	if(isOrange) {
		if(m_orangePortalAsset.asset != nullptr) {
			m_orangePortalAsset.render(m_mockupCamera.get());
		}
	} else {
		if(m_bluePortalAsset.asset != nullptr) {
			m_bluePortalAsset.render(m_mockupCamera.get());
		}
	}
}

// --- DeferredRenderingInterface --- //
void Scene::render(Camera *cam) {
	PROFILE_FUNCTION();

	m_skybox.render(cam);
	auto agphys = Agphys::getInstance();

	ImGuiOptions& options = ImGuiOptions::get();

	// render the particles from the viewpoint of the camera
	if (m_renderParticles)
	{
		// set up the shader
		auto shader = (m_renderShadows) ? m_particleShader : m_particleShaderFast;
		shader->bind();
		shader->uploadModel(mat4::Identity());

		// bind the gl buffers
		GLBufferManager& manager = GLBufferManager::get();
		manager.bind();

		if(!options.renderMeshes) {
			// draw all particles
			manager.getParticlePositions().draw(0, agphys->particleSystem->m_particleCounts.sum());
		} else {
			// draw all particles except the rigid body and fluid particles
			manager.getParticlePositions().draw(agphys->particleSystem->particlesBegin(ParticleType::ClothParticle), agphys->particleSystem->getNumParticles(ParticleType::ClothParticle));
			manager.getParticlePositions().draw(agphys->particleSystem->particlesBegin(ParticleType::RegularParticle), agphys->particleSystem->getNumParticles(ParticleType::RegularParticle));
		}

		// special case for fluid particles
		if(!options.renderFluidMesh) {
			manager.getParticlePositions().draw(agphys->particleSystem->particlesBegin(ParticleType::FluidParticle), agphys->particleSystem->getNumParticles(ParticleType::FluidParticle));
		}

		
		// unbind the gl buffers
		manager.unbind();

		// unbind the shader
		shader->unbind();

	}

	if(options.renderMeshes) {
		// render the meshes
		RigidBodyManager& manager = RigidBodyManager::get();
		manager.drawRigidMeshes(cam, *agphys->particleSystem);
	}
#if LOAD_PLANES
	// render the horizontal ground plane checkerboard texture
	m_groundPlane.render(cam);
#endif

	// render vertical planes without culling backfaces
	glDisable(GL_CULL_FACE);
	for(auto& plane: m_planes) {
		plane.renderWireframe(cam);
	}
	glEnable(GL_CULL_FACE);

	renderPortal(cam->view, cam->proj, true);
	renderPortal(cam->view, cam->proj, false);

	if(m_level != nullptr) {
		m_level->render(cam);
	}
	if(m_portalWalls != nullptr) {
		m_portalWalls->render(cam);
	}
	// TODO: add a third level that is slightly smaller than the original level and is invisible for intersection tests

	if(agphys->particleSystem->getNumParticles(ParticleType::FluidParticle) > 0) {
		if(options.renderFluidMesh) {
			//GBuffer *ptr = &(agphys->renderer.get()->*result<Rendererf>::ptr);
			GBuffer *ptr = &PortalRenderer::get().m_recursiveRenderer->gbuffer;
			WaterRenderer::getInstance().renderWater(ptr);
		}
	}

	PortalRenderer::get().renderEffectParticles(cam);


}

void Scene::renderDepth(Camera *cam) {
	PROFILE_FUNCTION();

	auto agphys = Agphys::getInstance();

	if(!m_renderShadows) return;

	ImGuiOptions& options = ImGuiOptions::get();
	if (m_renderParticles)
	{
		// bind the shadow shader
		auto shader = (m_renderShadows) ? m_particleDepthShader : m_particleDepthShaderFast;
		shader->bind();
		shader->uploadModel(mat4::Identity());

		// bind the gl buffers
		GLBufferManager& manager = GLBufferManager::get();
		manager.bind();

		if(!options.renderMeshes) {
			// draw all particles' shadows
			manager.getParticlePositions().draw(0, agphys->particleSystem->m_particleCounts.sum());
		} else {
			// draw all particles except the rigid body and fluid particles
			manager.getParticlePositions().draw(agphys->particleSystem->particlesBegin(ParticleType::ClothParticle), agphys->particleSystem->getNumParticles(ParticleType::ClothParticle));
			manager.getParticlePositions().draw(agphys->particleSystem->particlesBegin(ParticleType::RegularParticle), agphys->particleSystem->getNumParticles(ParticleType::RegularParticle));
		}

		if(!options.renderFluidMesh) {
			manager.getParticlePositions().draw(agphys->particleSystem->particlesBegin(ParticleType::FluidParticle), agphys->particleSystem->getNumParticles(ParticleType::FluidParticle));
		}

		// unbind gl buffers
		manager.unbind();

		// unbind the shader
		shader->unbind();
	}

	if(options.renderMeshes) {
		// render the meshes' shadows
		RigidBodyManager& manager = RigidBodyManager::get();
		manager.drawRigidMeshesDepth(cam, *agphys->particleSystem);
	}

#if LOAD_PLANES
#else
	if(m_level != nullptr) {
		m_level->renderDepth(cam);
	}
	if(m_portalWalls != nullptr) {
		m_portalWalls->renderDepth(cam);
	}
#endif

	// Water shadows will be rendered as usual
	if(agphys->particleSystem->getNumParticles(ParticleType::FluidParticle) > 0) {
		if(options.renderFluidMesh) {
			WaterRenderer::getInstance().renderWaterShadows(cam);
		}
	}

#if LOAD_PLANES
	// render the shadow of the ground plane
	m_groundPlane.renderDepth(cam);
#endif
}

void Scene::renderOverlay(Camera *cam) {
}

void Scene::renderFinal(Camera *cam) {
	// do nothing
}

void Scene::renderImGui() {
	auto agphys = Agphys::getInstance();

	agphys->m_ownplayer->m_camera->imguiStartup();

	if (m_showSaigaGUI) agphys->window->renderImGui();

	renderGUI();
}

// --- Plane Creation Methods --- //
std::pair<Saiga::Plane, SimpleAssetObject> Scene::createPlane(vec3 center, vec3 normal, float width, vec3 tubeCenter) {
	// create the plane
	Saiga::Plane plane(center, normal);

	// create the wireframe
	SimpleAssetObject currentPlane;
	std::vector<vec3> vertices;


	vec3 centerPoint = plane.closestPointOnPlane(tubeCenter);
	vec3 planeThing1 = plane.normal.cross(vec3(0,1,0)).normalized();
	vec3 planeThing2 = plane.normal.cross(planeThing1).normalized();

	// create vertices
	vertices.push_back(centerPoint + width * planeThing1 + 20.f * planeThing2);
	vertices.push_back(centerPoint - width * planeThing1 + 20.f * planeThing2);
	vertices.push_back(centerPoint + width * planeThing1 - 20.f * planeThing2);
	vertices.push_back(centerPoint - width * planeThing1 - 20.f * planeThing2);

	// create indices
	std::vector<GLuint> indices = {0,1,2, 1,3,2};

	// create geometry
	currentPlane.asset = m_objAssetLoader.nonTriangleMesh(vertices, indices);
	return std::pair<Saiga::Plane, SimpleAssetObject>(plane, currentPlane);
}

void Scene::createPlanes(int numPlanes, float planeDistance) {
	auto agphys = Agphys::getInstance();

	std::vector<Saiga::Plane> planes;

	vec3 tubeCenter = vec3(0.f, 20.f, 0.f);
	vec2 punkt = {0.f,planeDistance};
	for(int i = 0; i < numPlanes; ++i) {
		punkt = Eigen::Rotation2D<float>((2.f * M_PI / numPlanes)) * punkt;
		vec3 position = {punkt.x(), 0, punkt.y()};
		vec3 normal = {-punkt.normalized().x(), 0, -punkt.normalized().y()};
		float width = punkt.norm() * tan(M_PI / numPlanes);
		auto result = createPlane(position, normal, width, tubeCenter);

		planes.push_back(result.first);
		m_planes.push_back(result.second);
	}

	// create ground plane
	planes.push_back(Saiga::Plane({0,0,0}, {0,1,0}));

	// register the planes at the particle system
	agphys->particleSystem->registerPlanes(planes);
}

// --- Fluid Scene Setup Methods --- //
void Scene::setupFluidPlanes(int particleWidth) {
	ImGuiOptions& options = ImGuiOptions::get();
	auto agphys = Agphys::getInstance();

	// create planes
	std::vector<Saiga::Plane> planes;
	// create ground plane
	planes.push_back(Saiga::Plane({0.f,0.f,0.f}, {0.f,1.f,0.f}));

	float radius = options.radiusReset;

	vec3 tubeCenter = vec3(1.5f * particleWidth, 20.f, 2.f * particleWidth);

	auto plane1 = createPlane(vec3(particleWidth*1.5f,0.f,-radius), vec3(0,0,1), 1.5f * particleWidth + radius, tubeCenter);
	planes.push_back(plane1.first);
	m_planes.push_back(plane1.second);

	auto plane2 = createPlane(vec3(particleWidth*1.5f,0.f,particleWidth*4.f+radius), vec3(0,0,-1), 1.5f * particleWidth + radius, tubeCenter);
	planes.push_back(plane2.first);
	m_planes.push_back(plane2.second);

	auto plane3 = createPlane(vec3(-radius,0.f,2.f*particleWidth), vec3(1,0,0), 2.f*particleWidth + radius, tubeCenter);
	planes.push_back(plane3.first);
	m_planes.push_back(plane3.second);

	auto plane4 = createPlane(vec3(3.f*particleWidth+radius,0.f,2.f*particleWidth), vec3(-1,0,0), 2.f*particleWidth + radius, tubeCenter);
	planes.push_back(plane4.first);
	m_planes.push_back(plane4.second);

	// register the physic-part of the planes in the particle system
	agphys->particleSystem->registerPlanes(planes);
}

void Scene::setupFluidScene1(int particleWidth) {
	ImGuiOptions& options = ImGuiOptions::get();
	auto agphys = Agphys::getInstance();

	// create an empty particleSystem
	agphys->destroyParticles();
	agphys->particleSystem = std::make_shared<ParticleSystem>(0);
	// register the particle system in the glBufferManager
	GLBufferManager& manager = GLBufferManager::get();
	manager.setParticleSystem(agphys->particleSystem);
	// clear the physical planes
	m_planes.clear();

	//setupFluidPlanes(particleWidth);
	loadLevel("objs/defaultCube.obj");

	float radius = options.radiusReset;
	
	// spawn an initial amount of particles
	FluidCreator creator(*agphys->particleSystem);
	int upscale = ((4.f/3.f)*particleWidth - particleWidth);
	int numParticles = (upscale+particleWidth) * (upscale+particleWidth) * 3 * particleWidth;
	vec3 corner = vec3(0.f, radius, 0.f);
	auto data = creator.create(numParticles, 3*(int)particleWidth, (int)particleWidth+upscale, corner);

	agphys->particleSystem->spawnFluid(data);

	// update rendering stuff
	manager.getParticlePositions().setDrawMode(GL_POINTS);
	manager.updateInteropReferences();
}

void Scene::setupFluidScene2(int particleWidth) {
	ImGuiOptions& options = ImGuiOptions::get();
	auto agphys = Agphys::getInstance();

	// create an empty particleSystem
	agphys->destroyParticles();
	agphys->particleSystem = std::make_shared<ParticleSystem>(0);
	m_level = nullptr;
	m_portalWalls = nullptr;
	// register the particle system in the glBufferManager
	GLBufferManager& manager = GLBufferManager::get();
	manager.setParticleSystem(agphys->particleSystem);
	// clear the physical planes
	m_planes.clear();
	
	//setupFluidPlanes(particleWidth);
	loadLevel("objs/defaultCube.obj");

	float radius = options.radiusReset;
	
	// spawn an initial amount of particles
	FluidCreator creator(*agphys->particleSystem);
	int numParticles = particleWidth * particleWidth * particleWidth;
	vec3 corner = vec3(0.f, radius, 0.f);
	auto data = creator.create(numParticles, particleWidth, particleWidth, corner);
	agphys->particleSystem->spawnFluid(data);

	numParticles = particleWidth * 2.f * particleWidth * 2.f * particleWidth * 2.f;
	corner = vec3(particleWidth+2.f*radius, radius, particleWidth*2.f+2.f*radius);
	data = creator.create(numParticles, particleWidth*2, particleWidth*2, corner);
	agphys->particleSystem->spawnFluid(data);

	// update rendering stuff
	manager.getParticlePositions().setDrawMode(GL_POINTS);
	manager.updateInteropReferences();
}


// --- Stress Test Setup Methods --- //
void Scene::setupTestScene() {
	auto agphys = Agphys::getInstance();
	agphys->numberParticles = 1000000;
	vec3 camPosition = vec3(180,20,30);
	vec3 target = vec3(0,0,0);
	vec3 up = vec3(0,1,0);
	agphys->m_ownplayer->getCamera()->setView(camPosition, target, up);
	m_mockupCamera->setView(camPosition, target, up);
}

void Scene::loadLevel(const std::string& filePath) {
	m_level = std::make_shared<Controls::Level>(filePath);
	auto agphys = Agphys::getInstance();
	agphys->particleSystem->registerLevel(*m_level);
}
void Scene::loadWallLevel(const std::string& filePath) {
	m_portalWalls = std::make_shared<Controls::Level>(filePath);
	auto agphys = Agphys::getInstance();
	agphys->particleSystem->registerWallOverlay(*m_portalWalls);
}
