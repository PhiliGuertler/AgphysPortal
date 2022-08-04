#include "PortalRenderer.h"
#include "../imguiOptions.h"

#include "../agphys.h"

#include "saiga/core/geometry/triangle_mesh_generator.h"
#include "WaterRenderer.h"

#include <random>

#define NUMEFFECTPARTICLES 100

// ######################################################################### //
// ### PortalRenderer ###################################################### //
// ######################################################################### //

std::unique_ptr<PortalRenderer> PortalRenderer::s_instance = nullptr;

PortalRenderer& PortalRenderer::get() {
	if(s_instance == nullptr) s_instance = std::unique_ptr<PortalRenderer>(new PortalRenderer());
	return *s_instance;
}

void PortalRenderer::shutdown() {
	s_instance = nullptr;
}

static inline vec3 randomPositionOnPortalRim(const Portal& portal, std::mt19937 generator, std::uniform_int_distribution<int> distribution, std::uniform_real_distribution<float> distr) {
	vec3 bl = portal.bottomLeft();
	vec3 br = portal.bottomRight();
	vec3 tl = portal.topLeft();
	vec3 tr = portal.topRight();

	int edge = distribution(generator);
	float fact = distr(generator);

	vec3 position = vec3(0,0,0);
	switch(edge) {
		case 0:
			position = bl + (br-bl)*fact;
			break;
		case 1:
			position = br + (tr-br)*fact;
			break;
		case 2:
			position = tr + (tl-tr)*fact;
			break;
		case 3:
			position = tl + (bl-tl)*fact;
			break;
		default:
			std::cout << "Whoops!" << std::endl;
	}

	return position;
}

static inline void fillPortalData(std::vector<ParticlePositionRadius>& output, const Portal& portal) {
	std::random_device seeder;
	std::mt19937 generator(seeder());
	std::uniform_int_distribution<int> distribution(0,3);
	std::uniform_real_distribution<float> distr(0.f, 1.f);
	std::uniform_real_distribution<float> radical(.3f, .5f);

	for(int i = 0; i < NUMEFFECTPARTICLES; ++i) {
		ParticlePositionRadius vert;
		vert.position = randomPositionOnPortalRim(portal, generator, distribution, distr);
		vert.position += portal.normal() * 0.1;
		vert.radius = radical(generator);
		output.push_back(vert);
	}
}

void PortalRenderer::setOrangePortal(std::shared_ptr<Portal>& portal) {
	std::cout << __FUNCTION__ << std::endl;
	// don't increment the reference counter of the portal to force its deletion if the particleSystem is deleted.
	if(m_orangePortal == nullptr) {
		m_orangePortal = portal;
		auto orangeBundle = std::make_shared<EffectParticleBundle>();
		orangeBundle->m_color = vec4(1.f, 0.6f, .2f, 1.f);
		std::vector<ParticlePositionRadius> verts;
		fillPortalData(verts, *m_orangePortal);
		orangeBundle->setVertexData(verts);
		m_orangePortalEffectsIndex = m_effectParticleManager->registerEffectParticleBundle(orangeBundle);
	}
	m_orangePortal = portal;
}

void PortalRenderer::setBluePortal(std::shared_ptr<Portal>& portal) {
	std::cout << __FUNCTION__ << std::endl;
	// don't increment the reference counter of the portal to force its deletion if the particleSystem is deleted.
	if(m_bluePortal == nullptr) {
	m_bluePortal = portal;
	auto blueBundle = std::make_shared<EffectParticleBundle>();
		blueBundle->m_color = vec4(0.1f, 0.2f, 1.f, 1.f);
		std::vector<ParticlePositionRadius> verts;
		fillPortalData(verts, *m_bluePortal);
		blueBundle->setVertexData(verts);

		m_bluePortalEffectsIndex = m_effectParticleManager->registerEffectParticleBundle(blueBundle);
	}
	m_bluePortal = portal;
}

static inline RenderInfo createRenderInforFromCam(Camera *cam) {
	auto agphys = Agphys::getInstance();
	int width = agphys->window->getWidth();
	int height = agphys->window->getHeight();
	RenderInfo r;
	r.cameras.push_back({cam, ViewPort(ivec2(0,0), ivec2(width, height))});

	return r;
}

static inline void setupFrameBuffer(Framebuffer& buffer, std::shared_ptr<Texture>& colors, std::shared_ptr<Texture>& depth) {
	buffer.create();

	colors = std::make_shared<Texture>();
	depth = std::make_shared<Texture>();

	ImGuiOptions& options = ImGuiOptions::get();
	int width = options.windowWidth;
	int height = options.windowWidth;

	colors->create(width, height, GL_RGBA, GL_RGBA16, GL_UNSIGNED_SHORT);
	depth->create(width, height, GL_DEPTH_STENCIL, GL_DEPTH24_STENCIL8, GL_UNSIGNED_INT_24_8);

	buffer.attachTexture(colors);
	buffer.attachTextureDepthStencil(depth);
	buffer.drawToAll();
	buffer.check();
	buffer.unbind();
}

PortalRenderer::PortalRenderer() 
	: m_recursiveRenderer()
	, m_activeOrangePortalBuffer(0)
	, m_activeBluePortalBuffer(0)
	, m_maxRecursionDepth(1)
	, m_orangePortal()
	, m_orangePortalEffectsIndex(-1)
	, m_bluePortal()
	, m_bluePortalEffectsIndex(-1)
	, m_shootingEffectBundleIndex(-1)
	, m_framesToLive(0)
	, m_flightDirection(vec3(0,0,0))
	, m_flightPosition(vec3(NAN, NAN, NAN))
	, m_flightSpeed(4.f)
{
	Agphys *agphys = Agphys::getInstance();
	m_recursiveRenderer = std::make_shared<CustomDeferredRenderer>(agphys->window->getWidth(), agphys->window->getHeight());

	auto qb = TriangleMeshGenerator::createFullScreenQuadMesh();
	m_quadMesh.fromMesh(*qb);

	m_quadShader = Saiga::shaderLoader.load<Shader>("shader/screen.glsl");

	// setup post process shaders
	auto effect = WaterRenderer::getInstance().createPostProcessingShader();

	m_recursiveRenderer->postProcessor.addPostProcessingEffect(effect);

	setupFrameBuffer(m_bluePortalBuffer[0], m_bluePortalColorTex[0], m_bluePortalDepthTex[0]);
	setupFrameBuffer(m_bluePortalBuffer[1], m_bluePortalColorTex[1], m_bluePortalDepthTex[1]);

	setupFrameBuffer(m_orangePortalBuffer[0], m_orangePortalColorTex[0], m_orangePortalDepthTex[0]);
	setupFrameBuffer(m_orangePortalBuffer[1], m_orangePortalColorTex[1], m_orangePortalDepthTex[1]);

	m_effectParticleManager = std::shared_ptr<EffectParticleBundleManager>(new EffectParticleBundleManager());

	m_effectParticleShader = Saiga::shaderLoader.load<MVPShader>("shader/effectParticlesRender.glsl");
}
		
void PortalRenderer::renderToWindow(int width, int height) {
	glDisable(GL_DEPTH_TEST);

	m_quadShader->bind();

	GLint location = m_quadShader->getUniformLocation("texture");
	m_recursiveRenderer->m_colorBuffer->bind(0);
	m_quadShader->upload(location, 0);

	location = m_quadShader->getUniformLocation("depthTex");
	m_recursiveRenderer->m_depthBuffer->bind(1);
	m_quadShader->upload(location, 1);

	location = m_quadShader->getUniformLocation("u_screenSize");
	m_quadShader->upload(location, vec2(width, height));

	m_quadMesh.bindAndDraw();

	m_quadShader->unbind();
	glEnable(GL_DEPTH_TEST);
}

void PortalRenderer::resize(int width, int height) {
	if(m_recursiveRenderer != nullptr) {
		m_recursiveRenderer->resize(width, height);
	}
	m_orangePortalBuffer[0].resize(width, height);
	m_orangePortalBuffer[1].resize(width, height);

	m_bluePortalBuffer[0].resize(width, height);
	m_bluePortalBuffer[1].resize(width, height);

	m_effectParticleManager->resize(width, height);
}

void PortalRenderer::registerRenderingInterface(CustomDeferredRenderingInterface *interface) {
	m_recursiveRenderer->rendering = interface;
}

static inline int sign(float a) {
	return a > 0.f ? 1 : (a < 0.f) ? -1 : 0;
}

// computes an oblique view frustum that aligns the near and far planes with portalIn's plane
static inline mat4 createClippedProjection(PerspectiveCamera *cam, const Portal& portalIn) {
	ImGuiOptions& options = ImGuiOptions::get();

	vec3 normal = portalIn.normal();
	float dist = -portalIn.center().dot(portalIn.normal());
	dist += options.nearTweak;

	vec4 clipPlane = vec4(normal.x(), normal.y(), normal.z(), dist);
	clipPlane = cam->view.transpose().inverse() * clipPlane;

	if(clipPlane.w() > 0.f) return cam->proj;

	vec4 q = cam->proj.inverse() * vec4(sign(clipPlane.x()), sign(clipPlane.y()), 1.f, 1.f);

	vec4 c = clipPlane * (2.f / clipPlane.dot(q));

	mat4 newProj = cam->proj;
	newProj.row(2) = c.transpose() - newProj.row(3);
	
	return newProj;
}

// sets the camera's near plane and view matrix
static inline void computeTeleportedView(PerspectiveCamera *cam, const Portal& portalIn, const Portal& portalOut) {
	auto destinationView = Portal::transformView(cam->view, portalIn, portalOut);

	cam->setView(destinationView);
	cam->proj = createClippedProjection(cam, portalIn);
	cam->recalculateMatrices();
}

#define ORANGE .95f, .4f, .05f, 1.f
#define BLUE .05f, .2f, .95f, 1.f

void PortalRenderer::clearBuffers() {
	m_recursiveRenderer->m_frameBuffer.bind();
	glClearDepth(1.f);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	m_orangePortalBuffer[0].bind();
	glClearColor(ORANGE);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	m_orangePortalBuffer[1].bind();
	glClearColor(ORANGE);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	m_bluePortalBuffer[0].bind();
	glClearColor(BLUE);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	m_bluePortalBuffer[1].bind();
	glClearColor(BLUE);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
}

void PortalRenderer::renderEffectParticles(Camera *cam) {
	glEnable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	m_effectParticleShader->bind();

	// TODO: upload uniforms for the shader here
	m_effectParticleShader->uploadModel(mat4::Identity());

	m_effectParticleManager->render(cam, *m_effectParticleShader);
	m_effectParticleShader->unbind();
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
}

void PortalRenderer::renderScene(Camera *cam) {
	mat4 view = cam->view;
	mat4 proj = cam->proj;

	PerspectiveCamera *cameo = dynamic_cast<PerspectiveCamera*>(cam);

	// check if anything has not been set yet
	auto info = createRenderInforFromCam(cam);

	RenderInfo orangeInfo;
	if(m_orangePortal == nullptr) {
		// as the orange portal is not set, just render the scene as usual
		m_recursiveRenderer->render(info);
		return;
	} else if(m_bluePortal != nullptr) {
		
		m_orangeCam.setView(view);
		m_orangeCam.Saiga::Camera::setProj(proj);
		m_orangeCam.zFar = cam->zFar;
		m_orangeCam.zNear = cam->zNear;
		m_orangeCam.fovy = cameo->fovy;
		m_orangeCam.tang = cameo->tang;
		m_orangeCam.aspect = cameo->aspect;

		computeTeleportedView(&m_orangeCam, *m_bluePortal, *m_orangePortal);

		orangeInfo = createRenderInforFromCam(&m_orangeCam);
	}

	RenderInfo blueInfo;
	if(m_bluePortal == nullptr) {
		// as the blue portal is not set, just render the scene as usual
		m_recursiveRenderer->render(info);
		return;
	} else {
		m_blueCam.setView(view);
		m_blueCam.Saiga::Camera::setProj(proj);
		m_blueCam.zFar = cam->zFar;
		m_blueCam.zNear = cam->zNear;
		m_blueCam.fovy = cameo->fovy;
		m_blueCam.tang = cameo->tang;
		m_blueCam.aspect = cameo->aspect;

		computeTeleportedView(&m_blueCam, *m_orangePortal, *m_bluePortal);

		blueInfo = createRenderInforFromCam(&m_blueCam);
	}

	// register the cameras of the portals in agphys's scene
	auto agphys = Agphys::getInstance();
	auto& scene = *(agphys->m_scene);
	
	scene.updatePortalPlane(*m_orangePortal, true);
	scene.updatePortalPlane(*m_bluePortal, false);

	if(m_debugPerspective != 0) {
		if(m_debugPerspective == 1) {
			scene.m_renderingOrange = 1;
			// render the scene from the orange portal including the blue portal.
			// the blue portal's texture should be just blue in the first render pass, but the result of these rendering in the following ones
			m_recursiveRenderer->render(orangeInfo);
		} else if(m_debugPerspective == 2) {
			scene.m_renderingOrange = 2;
			// render the scene from the orange portal including the blue portal.
			// the blue portal's texture should be just blue in the first render pass, but the result of these rendering in the following ones
			m_recursiveRenderer->render(blueInfo);
		}
		return;
	}

	for(int i = 0; i < m_maxRecursionDepth; ++i) {
		scene.m_renderingOrange = 1;
		// render the scene from the orange portal including the blue portal.
		// the blue portal's texture should be just blue in the first render pass, but the result of these rendering in the following ones
		m_recursiveRenderer->render(orangeInfo, m_orangePortalBuffer[m_activeOrangePortalBuffer]);

		scene.m_renderingOrange = 2;
		// render the scene from the blue portal including the orange portal.
		// the orange portal's texture should be just blue in the first render pass, but the result of these rendering in the following ones
		m_recursiveRenderer->render(blueInfo, m_bluePortalBuffer[m_activeBluePortalBuffer]);


		// Swap active buffers
		scene.updatePortalPlane(*m_orangePortal, true);
		scene.updatePortalPlane(*m_bluePortal, false);

		m_activeBluePortalBuffer ^= m_activeBluePortalBuffer;
		m_activeOrangePortalBuffer ^= m_activeOrangePortalBuffer;
	}

	// render the scene like normal
	scene.m_renderingOrange = 0;
	m_recursiveRenderer->render(info);

}


// ######################################################################### //
// ### Shot Visualization ################################################## //

static inline ParticlePositionRadius randomShotPosition(std::mt19937& generator, std::normal_distribution<float>& distr, std::uniform_real_distribution<float>& radical, vec3 position) {
	vec3 randomOffset = position;
	randomOffset.x() += distr(generator);
	randomOffset.y() += distr(generator);
	randomOffset.z() += distr(generator);

	return { randomOffset, radical(generator) };
}

static inline void createParticleShotData(std::vector<ParticlePositionRadius>& output, vec3 position) {
	std::random_device seeder;
	std::mt19937 generator(seeder());
	std::normal_distribution<float> distr(-.4f, .4f);
	std::uniform_real_distribution<float> radical(.3f, .5f);

	for(int i = 0; i < NUMEFFECTPARTICLES; ++i) {
		ParticlePositionRadius vert;
		vert = randomShotPosition(generator, distr, radical, position);
		output.push_back(vert);
	}
}

void PortalRenderer::updatePortalEffects() {
	if(m_bluePortalEffectsIndex != -1) {
		std::vector<ParticlePositionRadius> verts;
		fillPortalData(verts, *m_bluePortal);
		auto& blueBundle = m_effectParticleManager->getBundle(m_bluePortalEffectsIndex);
		blueBundle.setVertexData(verts);
	}

	if(m_orangePortalEffectsIndex != -1) {
		std::vector<ParticlePositionRadius> verts;
		fillPortalData(verts, *m_orangePortal);
		auto& orangeBundle = m_effectParticleManager->getBundle(m_orangePortalEffectsIndex);
		orangeBundle.setVertexData(verts);
	}

	if(m_shootingEffectBundleIndex != -1) {
		if(m_framesToLive > 0) {
			m_flightPosition += m_flightDirection * m_flightSpeed;
			std::vector<ParticlePositionRadius> verts;
			createParticleShotData(verts, m_flightPosition);
			auto& bundle = m_effectParticleManager->getBundle(m_shootingEffectBundleIndex);
			bundle.setVertexData(verts);
			--m_framesToLive;
		} else if(m_framesToLive == 0) {
			std::vector<ParticlePositionRadius> verts;
			createParticleShotData(verts, vec3(NAN, NAN, NAN));
			auto& bundle = m_effectParticleManager->getBundle(m_shootingEffectBundleIndex);
			bundle.setVertexData(verts);
			--m_framesToLive;
		}
	}
}
		
void PortalRenderer::shootEffectParticles(int framesToLive, vec3 direction, vec3 origin, vec4 color) {
	if(m_shootingEffectBundleIndex == -1) {
		// create the particle bundle if it is not existing yet
		auto bundle = std::make_shared<EffectParticleBundle>();
		m_shootingEffectBundleIndex = m_effectParticleManager->registerEffectParticleBundle(bundle);
	}

	auto& bundle = m_effectParticleManager->getBundle(m_shootingEffectBundleIndex);
	bundle.m_color = color;

	m_flightDirection = direction;
	m_flightPosition = origin;

	m_framesToLive = framesToLive;
}
