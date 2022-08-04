#pragma once

#include "saiga/config.h"
#include "saiga/core/window/Interfaces.h"
#include "saiga/opengl/rendering/program.h"
#include "saiga/opengl/uniformBuffer.h"
#include "saiga/opengl/rendering/renderer.h"
#include "saiga/opengl/framebuffer.h"

namespace Saiga {

	/**
	 *	This Class is supposed to be just like an OpenGLRenderer with two major differences:
	 *	1. Instead of into the window's framebuffer this renderer has its own framebuffer
	 *		The goal of this is to allow the use of a Renderer extending this class to do a whole
	 *		or multiple render passes of a scene within a single frame of a real DeferredRenderer.
	 *		The framebuffer will be filled at the end of a Renderpass and should be usable with
	 *		stencils.
	 *	2. There is no support for imgui rendering
	 */
	class MockupRenderer : public RendererBase {
		public:
			int outputWidth = -1, outputHeight = -1;
			UniformBuffer cameraBuffer;

			Framebuffer m_frameBuffer;
			std::shared_ptr<Texture> m_colorBuffer;
			std::shared_ptr<Texture> m_depthBuffer;

		public:
			MockupRenderer(int width, int height);
			virtual ~MockupRenderer();

			virtual void resize(int width, int height);

			virtual void printTimings() {}

			void bindCamera(Camera *cam);
	};

}
