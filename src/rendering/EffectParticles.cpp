#include "EffectParticles.h"

#include "../imguiOptions.h"
#include "PortalRenderer.h"

#include "saiga/opengl/shader/shaderLoader.h"

// ######################################################################### //
// ### EffectParticleShader ################################################ //
// ######################################################################### //

EffectParticleShader::EffectParticleShader() {
	// empty
}

EffectParticleShader::~EffectParticleShader() {
	// empty
}

void EffectParticleShader::uploadAdditionalUniforms() {
	int bindLocation = 0;

	auto location = getUniformLocation("particleDepth");
	m_depth->bind(++bindLocation);
	upload(location, bindLocation);

	location = getUniformLocation("particleColor");
	m_color->bind(++bindLocation);
	upload(location, bindLocation);
}



// ######################################################################### //
// ### EffectParticleBundle ################################################ //
// ######################################################################### //

EffectParticleBundle::EffectParticleBundle()
{
	m_particleDepth = std::make_shared<Saiga::Texture>();
	m_particleColor = std::make_shared<Saiga::Texture>();

	ImGuiOptions& options = ImGuiOptions::get();

	m_frameBuffer.create();
	m_particleColor->create(options.windowWidth, options.windowHeight, GL_RGBA, GL_RGBA16, GL_UNSIGNED_SHORT);
	m_particleDepth->create(options.windowWidth, options.windowHeight, GL_DEPTH_STENCIL, GL_DEPTH24_STENCIL8, GL_UNSIGNED_INT_24_8);

	m_frameBuffer.attachTexture(m_particleColor);
	m_frameBuffer.attachTextureDepthStencil(m_particleDepth);
	m_frameBuffer.drawToAll();
	m_frameBuffer.check();
	m_frameBuffer.unbind();

	m_vertexBuffer.bind();
	std::vector<ParticlePositionRadius> dummy(1);
	m_vertexBuffer.set(dummy, GL_DYNAMIC_DRAW);
	m_vertexBuffer.unbind();
	m_vertexBuffer.setDrawMode(GL_POINTS);

	m_color = vec4(0,1,0,1);
}

EffectParticleBundle::~EffectParticleBundle() {
	// empty
}

void EffectParticleBundle::render(Saiga::Camera *cam) {
	m_frameBuffer.bind();
	glClearColor(0,0,0,0);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	m_vertexBuffer.bindAndDraw();
	m_frameBuffer.unbind();
}

void EffectParticleBundle::resize(int width, int height) {
	m_frameBuffer.resize(width, height);
}

void EffectParticleBundle::setVertexData(std::vector<ParticlePositionRadius>& positions) {
	m_vertexBuffer.set(positions, GL_DYNAMIC_DRAW);
}

std::shared_ptr<EffectParticleShader> EffectParticleBundle::createPostProcessingShader() {
	Saiga::ShaderPart::ShaderCodeInjections injections;
	Saiga::ShaderPartLoader spl("shader/effectParticlesPost.glsl", injections);
	std::shared_ptr<EffectParticleShader> shader;
	if(spl.load()) {
		shader = spl.createShader<EffectParticleShader>();
	}

	shader->m_depth = m_particleDepth;
	shader->m_color = m_particleColor;
	return shader;
}


// ######################################################################### //
// ### EffectParticleBundleManager ######################################### //
// ######################################################################### //

EffectParticleBundleManager::EffectParticleBundleManager()
	: m_bundles()
{
	// empty
}

int EffectParticleBundleManager::registerEffectParticleBundle(std::shared_ptr<EffectParticleBundle> bundle) {
	int index = m_bundles.size();
	m_bundles.push_back(bundle);
	
	PortalRenderer& renderer = PortalRenderer::get();
	renderer.m_recursiveRenderer->postProcessor.addPostProcessingEffect(bundle->createPostProcessingShader());

	return index;
}

EffectParticleBundle& EffectParticleBundleManager::getBundle(int index) const {
	return *m_bundles[index];
}

void EffectParticleBundleManager::resize(int width, int height) {
	for(auto& bundle : m_bundles) {
		bundle->resize(width, height);
	}
}

void EffectParticleBundleManager::render(Saiga::Camera *cam, Saiga::Shader& shader) {
	for(auto& bundle : m_bundles) {
		auto loc = shader.getUniformLocation("u_color");
		shader.upload(loc, bundle->m_color);
		bundle->render(cam);
	}
}
