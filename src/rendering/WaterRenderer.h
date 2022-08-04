#pragma once

#include "saiga/opengl/shader/shaderLoader.h"
#include "saiga/opengl/shader/shader.h"
#include "saiga/core/camera/all.h"
#include "saiga/opengl/opengl.h"
#include "saiga/opengl/framebuffer.h"
#include "saiga/opengl/rendering/deferredRendering/postProcessor.h"
//#include "saiga/opengl/rendering/deferredRendering/deferred_renderer.h"

#include "../cuda/particleSystem/particleSystem.h"

#include "structures.h"

#include "CustomIndexedVertexBuffer.h"
#include "CustomDeferredRenderer.h"


// ######################################################################### //
// ### WaterMesh ########################################################### //

class WaterMesh {
	public:
		WaterMesh();
		~WaterMesh();

		void setData(thrust::device_vector<WaterVertex> vertices, thrust::device_vector<GLuint> indices);

		inline void setDrawMode(GLenum mode) { m_buffer.setDrawMode(mode); }

		inline CustomBuffer& getBuffer() { return m_buffer; }

		void render(std::shared_ptr<Saiga::MVPShader> shader);

	private:
		CustomBuffer m_buffer;
};


// ######################################################################### //
// ### WaterRenderer ####################################################### //

class WaterRenderer {
public:
	static WaterRenderer& getInstance();

	static void init();
	static void shutdown();

public:
	~WaterRenderer();

	// TODO: render water in a transparent way in a post-process
	void renderWater(GBuffer *gbuffer);
	void renderWaterShadows(Saiga::Camera *cam);
	void clearBuffers();

	inline WaterMesh& getMesh() { return m_mesh; }

	void updateMeshData(thrust::device_vector<WaterVertex> vertices, thrust::device_vector<GLuint> indices);

	inline void resize(int width, int height) {
		m_waterFrameBuffer.resize(width, height);
		m_waterFrameBufferBackFaces.resize(width, height);
	}

	std::shared_ptr<Saiga::PostProcessingShader> createPostProcessingShader();
private:
	static std::unique_ptr<WaterRenderer> s_instance;

private:
	WaterRenderer();

	WaterRenderer(const WaterRenderer& r) = delete;
	WaterRenderer(const WaterRenderer&& r) = delete;

	WaterRenderer& operator=(const WaterRenderer& r) = delete;
	WaterRenderer& operator=(const WaterRenderer&& r) = delete;

private:
	std::shared_ptr<Saiga::MVPShader> m_tessShader;
	std::shared_ptr<Saiga::MVPShader> m_tessShaderDepth;

	Saiga::Framebuffer m_waterFrameBuffer;
	std::shared_ptr<Saiga::Texture> m_waterNormalTex;
	std::shared_ptr<Saiga::Texture> m_waterPositionTex;
	std::shared_ptr<Saiga::Texture> m_waterDepthTex;
	std::shared_ptr<Saiga::Texture> m_waterVorticityTex;

	Saiga::Framebuffer m_waterFrameBufferBackFaces;
	std::shared_ptr<Saiga::Texture> m_waterNormalTexBack;
	std::shared_ptr<Saiga::Texture> m_waterPositionTexBack;
	std::shared_ptr<Saiga::Texture> m_waterDepthTexBack;
	std::shared_ptr<Saiga::Texture> m_waterVorticityTexBack;

	WaterMesh m_mesh;

	std::shared_ptr<Saiga::Texture> m_foamTexture;
	/*
	std::shared_ptr<Saiga::Shader> m_depthShader;
	std::shared_ptr<Saiga::Shader> m_blurShader;
	std::shared_ptr<Saiga::Shader> m_thicknessShader;
	std::shared_ptr<Saiga::Shader> m_fluidFinalShader;
	std::shared_ptr<Saiga::Shader> m_foamDepthShader;
	std::shared_ptr<Saiga::Shader> m_foamThicknessShader;
	std::shared_ptr<Saiga::Shader> m_foamIntensityShader;
	std::shared_ptr<Saiga::Shader> m_foamRadianceShader;
	std::shared_ptr<Saiga::Shader> m_finalShader;
	*/
};
