#include "MockupRenderer.h"

#include "saiga/core/camera/camera.h"
#include "saiga/opengl/shader/basic_shaders.h"

namespace Saiga {

	MockupRenderer::MockupRenderer(int width, int height)
		: outputWidth(width)
		, outputHeight(height)
	{
		cameraBuffer.createGLBuffer(nullptr, sizeof(CameraDataGLSL), GL_DYNAMIC_DRAW);

		// init framebuffer
		m_frameBuffer.create();
		m_colorBuffer = std::make_shared<Texture>();
		m_colorBuffer->create(width, height, GL_RGBA, GL_RGBA8, GL_UNSIGNED_SHORT);
		m_depthBuffer = std::make_shared<Texture>();
		m_depthBuffer->create(width, height, GL_DEPTH_STENCIL, GL_DEPTH24_STENCIL8, GL_UNSIGNED_INT_24_8);

		m_frameBuffer.attachTexture(m_colorBuffer);
		m_frameBuffer.attachTextureDepthStencil(m_depthBuffer);
		m_frameBuffer.drawToAll();
		m_frameBuffer.check();
		m_frameBuffer.unbind();

	}

	MockupRenderer::~MockupRenderer() {
		// empty
	}

	void MockupRenderer::resize(int width, int height) {
		outputWidth = width;
		outputHeight = height;
		m_frameBuffer.resize(width, height);
	}

	void MockupRenderer::bindCamera(Camera *cam) {
		CameraDataGLSL cd(cam);
		cameraBuffer.updateBuffer(&cd, sizeof(CameraDataGLSL), 0);
		cameraBuffer.bind(CAMERA_DATA_BINDING_POINT);
	}

}
