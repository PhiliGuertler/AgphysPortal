#include "WaterRenderer.h"

#include "WaterShaders.h"

#include "../imguiOptions.h"

#include "saiga/opengl/texture/TextureLoader.h"

// ######################################################################### //
// ### WaterMesh ########################################################### //

WaterMesh::WaterMesh() 
{
	m_buffer.bind();
}

void WaterMesh::render(std::shared_ptr<Saiga::MVPShader> shader) {
	// bind shader
	shader->bind();
	m_buffer.bindAndDraw();
	shader->unbind();
}

WaterMesh::~WaterMesh() {
}

// ######################################################################### //
// ### WaterRenderer ####################################################### //

std::unique_ptr<WaterRenderer> WaterRenderer::s_instance = nullptr;

WaterRenderer& WaterRenderer::getInstance() {
	if(s_instance == nullptr) {
		std::cerr << "WaterRenderer has not been initialized yet. Please call \"WaterRenderer::init()\" first!" << std::endl;
	}
	return *s_instance;
}

void WaterRenderer::init() {
	s_instance = std::unique_ptr<WaterRenderer>(new WaterRenderer());
}

inline void setupFrameBuffer(Framebuffer& fb, std::shared_ptr<Texture>& positionTex, std::shared_ptr<Texture>& normalTex, std::shared_ptr<Texture>& depthTex, std::shared_ptr<Texture>& vorticityTex) {
	ImGuiOptions& options = ImGuiOptions::get();

	fb.create();

	normalTex->create(options.windowWidth, options.windowHeight, GL_RGB, GL_RGB16, GL_UNSIGNED_SHORT);
	positionTex->create(options.windowWidth, options.windowHeight, GL_RGB, GL_RGB16, GL_UNSIGNED_SHORT);
	vorticityTex->create(options.windowWidth, options.windowHeight, GL_RED, GL_R32F, GL_FLOAT);
	depthTex->create(options.windowWidth, options.windowHeight, GL_DEPTH_STENCIL, GL_DEPTH24_STENCIL8, GL_UNSIGNED_INT_24_8);

	fb.attachTexture(positionTex);
	fb.attachTexture(normalTex);
	fb.attachTexture(vorticityTex);
	fb.attachTextureDepthStencil(depthTex);
	fb.drawToAll();
	fb.check();
	fb.unbind();
}

WaterRenderer::WaterRenderer()
	: m_tessShader()
	, m_tessShaderDepth()
	, m_waterNormalTex()
	, m_waterDepthTex()
	, m_mesh()
{
	m_tessShader = Saiga::shaderLoader.load<Saiga::MVPShader>("shader/tess.glsl");

	m_tessShaderDepth = Saiga::shaderLoader.load<Saiga::MVPShader>("shader/tess_depth.glsl");

	m_foamTexture = Saiga::TextureLoader::instance()->load("textures/foamTex.png");

	m_mesh.setDrawMode(GL_PATCHES);

	// --- set up waterFrameBuffer --- //
	m_waterPositionTex = std::make_shared<Texture>();
	m_waterNormalTex = std::make_shared<Texture>();
	m_waterDepthTex = std::make_shared<Texture>();
	m_waterVorticityTex = std::make_shared<Texture>();

	setupFrameBuffer(m_waterFrameBuffer, m_waterPositionTex, m_waterNormalTex, m_waterDepthTex, m_waterVorticityTex);

	// --- set up waterFrameBufferBack --- //
	m_waterPositionTexBack = std::make_shared<Texture>();
	m_waterNormalTexBack = std::make_shared<Texture>();
	m_waterDepthTexBack = std::make_shared<Texture>();
	m_waterVorticityTexBack = std::make_shared<Texture>();

	setupFrameBuffer(m_waterFrameBufferBackFaces, m_waterPositionTexBack, m_waterNormalTexBack, m_waterDepthTexBack, m_waterVorticityTexBack);

	// --- register post processing effects --- //
	/*
	m_depthShader = Saiga::shaderLoader.load<Saiga::Shader>("shader/depthShader.glsl");
	m_blurShader = Saiga::shaderLoader.load<Saiga::Shader>("shader/blurShader.glsl");
	m_thicknessShader = Saiga::shaderLoader.load<Saiga::Shader>("shader/thicknessShader.glsl");
	m_fluidFinalShader = Saiga::shaderLoader.load<Saiga::Shader>("shader/fluidFinalShader.glsl");
	m_foamDepthShader = Saiga::shaderLoader.load<Saiga::Shader>("shader/foamDepthShader.glsl");
	m_foamThicknessShader = Saiga::shaderLoader.load<Saiga::Shader>("shader/foamThicknessShader.glsl");
	m_foamIntensityShader = Saiga::shaderLoader.load<Saiga::Shader>("shader/foamIntensityShader.glsl");
	m_foamRadianceShader = Saiga::shaderLoader.load<Saiga::Shader>("shader/foamRadianceShader.glsl");
	m_finalShader = Saiga::shaderLoader.load<Saiga::Shader>("shader/finalShader.glsl");
	*/
}

std::shared_ptr<Saiga::PostProcessingShader> WaterRenderer::createPostProcessingShader() {
	auto postProcessing = Saiga::shaderLoader.load<WaterSmootherShader>("shader/postProcessingTest.glsl");
	postProcessing->m_waterPositionTex = m_waterPositionTex;
	postProcessing->m_waterNormalTex = m_waterNormalTex;
	postProcessing->m_waterDepthTex = m_waterDepthTex;
	postProcessing->m_waterVorticityTex = m_waterVorticityTex;

	postProcessing->m_waterPositionTexBack = m_waterPositionTexBack;
	postProcessing->m_waterDepthTexBack = m_waterDepthTexBack;

	postProcessing->m_foamTexture = m_foamTexture;

	return postProcessing;
}

WaterRenderer::~WaterRenderer() {
	// shaders will be deleted automatically
}

void WaterRenderer::shutdown() {
	s_instance = nullptr;
}

static inline void setupTessShader(std::shared_ptr<Saiga::MVPShader>& shader) {
	ImGuiOptions& options = ImGuiOptions::get();

	shader->bind();

	GLint location = shader->getUniformLocation("u_tessLevelOuter");
	shader->upload(location, options.tessLevelOuter);
	location = shader->getUniformLocation("u_tessLevelInner");
	shader->upload(location, options.tessLevelInner);
	location = shader->getUniformLocation("u_tessAlpha");
	shader->upload(location, options.tessAlpha);
	location = shader->getUniformLocation("u_gridSize");
	shader->upload(location, options.marchingCubesGridStep);

	location = shader->getUniformLocation("model");
	mat4 model = mat4::Identity();
	shader->upload(location, model);

	shader->unbind();
}

void WaterRenderer::renderWater(GBuffer *ptr) {

	// cam is already set for some reason
	ImGuiOptions& options = ImGuiOptions::get();
	setupTessShader(m_tessShader);

	m_tessShader->bind();

	if(options.fluidWireframe) {
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	}

	// render front faces
	m_waterFrameBuffer.bind();
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// bind gbuffer depth
	ptr->getTextureDepth()->bind(9);
	auto location = m_tessShader->getUniformLocation("gbufferDepth");
	m_tessShader->upload(location, 9);

	location = m_tessShader->getUniformLocation("u_shrink");
	m_tessShader->upload(location, options.shrink);

	glDisable(GL_CULL_FACE);
	m_mesh.render(m_tessShader);
	glEnable(GL_CULL_FACE);

	m_waterFrameBuffer.unbind();

	// render back faces
	m_waterFrameBufferBackFaces.bind();
	glClearDepth(0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthFunc(GL_GREATER);
	
	glCullFace(GL_FRONT);
	m_mesh.render(m_tessShader);
	glCullFace(GL_BACK);

	glClearDepth(1.0f);
	glDepthFunc(GL_LESS);
	m_waterFrameBufferBackFaces.unbind();

	if(options.fluidWireframe) {
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	}
}

void WaterRenderer::clearBuffers() {
	glClearDepth(0.f);
	m_waterFrameBuffer.bind();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearDepth(1.f);
	m_waterFrameBufferBackFaces.bind();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void WaterRenderer::renderWaterShadows(Saiga::Camera *cam) {
	// cam is already set for some reason

	setupTessShader(m_tessShaderDepth);

	glDisable(GL_CULL_FACE);
	m_mesh.render(m_tessShaderDepth);
	glEnable(GL_CULL_FACE);
}

