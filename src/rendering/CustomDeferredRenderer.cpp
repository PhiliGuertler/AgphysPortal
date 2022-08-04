#include "saiga/core/camera/camera.h"
#include "saiga/core/geometry/triangle_mesh_generator.h"
#include "saiga/core/imgui/imgui.h"
#include "saiga/opengl/error.h"
#include "saiga/opengl/rendering/deferredRendering/deferredRendering.h"
#include "saiga/opengl/rendering/program.h"
#include "saiga/opengl/rendering/renderer.h"
#include "saiga/opengl/shader/shaderLoader.h"
#include "saiga/opengl/window/OpenGLWindow.h"

#include "CustomDeferredRenderer.h"
#include "PortalRenderer.h"

namespace Saiga {
	
void CustomDeferredRenderer::setViewPortMock() {
	// don't set the glViewport, instead render into the result framebuffer!
}

CustomDeferredRenderer::CustomDeferredRenderer(int width, int height, ParameterType _params) 
    : MockupRenderer(width, height),
      m_lighting(gbuffer),
      params(_params),
      renderWidth(width * _params.renderScale),
      renderHeight(height * _params.renderScale),
      ddo(width, height)
{
    if (params.useSMAA)
    {
        smaa = std::make_shared<CustomSMAA>(renderWidth, renderHeight);
        smaa->loadShader(params.smaaQuality);
    }

    {
        // create a 2x2 grayscale black dummy texture
        blackDummyTexture = std::make_shared<Texture>();
        std::vector<int> data(2 * 2, 0);
        blackDummyTexture->create(2, 2, GL_RED, GL_R8, GL_UNSIGNED_BYTE, (GLubyte*)data.data());
    }
    if (params.useSSAO)
    {
        ssao = std::make_shared<SSAO>(renderWidth, renderHeight);
    }
    m_lighting.ssaoTexture = ssao ? ssao->bluredTexture : blackDummyTexture;


    if (params.srgbWrites)
    {
        // intel graphics drivers on windows do not define this extension but srgb still works..
        // SAIGA_ASSERT(hasExtension("GL_EXT_framebuffer_sRGB"));

        // Mesa drivers do not respect the spec when blitting with srgb framebuffers.
        // https://lists.freedesktop.org/archives/mesa-dev/2015-February/077681.html

        // TODO check for mesa
        // If this is true some recording softwares record the image too dark :(
        params.blitLastFramebuffer = false;
    }



    gbuffer.init(renderWidth, renderHeight, params.gbp);

    m_lighting.shadowSamples = params.shadowSamples;
    m_lighting.clearColor    = params.lightingClearColor;
    m_lighting.init(renderWidth, renderHeight, params.useGPUTimers);
    m_lighting.loadShaders();



    postProcessor.init(renderWidth, renderHeight, &gbuffer, params.ppp, m_lighting.lightAccumulationTexture,
                       params.useGPUTimers);


    auto qb = TriangleMeshGenerator::createFullScreenQuadMesh();
    quadMesh.fromMesh(*qb);

    int numTimers = DeferredTimings::COUNT;
    if (!params.useGPUTimers) numTimers = 1;  // still use one rendering timer :)
    timers.resize(numTimers);
    for (auto& t : timers)
    {
        t.create();
    }



    blitDepthShader = shaderLoader.load<MVPTextureShader>("lighting/blitDepth.glsl");

    ddo.setDeferredFramebuffer(&gbuffer, m_lighting.volumetricLightTexture2);


    std::shared_ptr<PostProcessingShader> pps =
        shaderLoader.load<PostProcessingShader>("post_processing/post_processing.glsl");  // this shader does nothing
    std::vector<std::shared_ptr<PostProcessingShader> > defaultEffects;
    defaultEffects.push_back(pps);
    postProcessor.setPostProcessingEffects(defaultEffects);

    std::cout << "Deferred Renderer initialized. Render resolution: " << renderWidth << "x" << renderHeight
              << std::endl;
}

CustomDeferredRenderer::~CustomDeferredRenderer() {}

void CustomDeferredRenderer::resize(int windowWidth, int windowHeight)
{
    if (windowWidth <= 0 || windowHeight <= 0)
    {
        std::cerr << "Warning: The window size must be greater than zero." << std::endl;
        windowWidth  = std::max(windowWidth, 1);
        windowHeight = std::max(windowHeight, 1);
    }
	MockupRenderer::resize(windowWidth, windowHeight);
    this->renderWidth  = windowWidth * params.renderScale;
    this->renderHeight = windowHeight * params.renderScale;
    std::cout << "[CustomDeferredRenderer] Resizing Window to : " << windowWidth << "," << windowHeight << std::endl;
    std::cout << "[CustomDeferredRenderer] Framebuffer size: " << renderWidth << " " << renderHeight << std::endl;
    postProcessor.resize(renderWidth, renderHeight);
    gbuffer.resize(renderWidth, renderHeight);
    m_lighting.resize(renderWidth, renderHeight);

    if (ssao) ssao->resize(renderWidth, renderHeight);

    if (smaa)
    {
        smaa->resize(renderWidth, renderHeight);
    }
}

    
void CustomDeferredRenderer::render(const RenderInfo& _renderInfo, Framebuffer& buffer) {
    if (!rendering) return;


    Saiga::RenderInfo renderInfo = _renderInfo;

    SAIGA_ASSERT(rendering);
    SAIGA_ASSERT(renderInfo);

    // if we have multiple cameras defined the user has to specify the viewports of each individual camera
    SAIGA_ASSERT(params.userViewPort || renderInfo.cameras.size() == 1);


    if (renderInfo.cameras.size() == 1)
    {
        renderInfo.cameras.front().second = ViewPort({0, 0}, {renderWidth, renderHeight});
    }


    InterfaceType* renderingInterface = dynamic_cast<InterfaceType*>(rendering);
    SAIGA_ASSERT(renderingInterface);


    if (params.srgbWrites) glEnable(GL_FRAMEBUFFER_SRGB);

    startTimer(TOTAL);

    // When GL_FRAMEBUFFER_SRGB is disabled, the system assumes that the color written by the fragment shader
    // is in whatever colorspace the image it is being written to is. Therefore, no colorspace correction is performed.
    // If GL_FRAMEBUFFER_SRGB is enabled however, then if the destination image is in the sRGB colorspace
    // (as queried through glGetFramebufferAttachmentParameter(GL_FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING)​),
    // then it will assume the shader's output is in the linear RGB colorspace.
    // It will therefore convert the output from linear RGB to sRGB.
    //    if (params.srgbWrites)
    //        glEnable(GL_FRAMEBUFFER_SRGB); //no reason to switch it off

    clearGBuffer();

    m_lighting.initRender();
    for (auto c : renderInfo.cameras)
    {
        auto camera = c.first;
        camera->recalculatePlanes();
        bindCamera(camera);

        setViewPort(c.second);
        renderGBuffer(c);
        renderSSAO(c);

        m_lighting.cullLights(camera);
        renderDepthMaps();


        bindCamera(camera);
        setViewPort(c.second);
        renderLighting(c);
    }
    assert_no_glerror();


    //    return;



    if (params.writeDepthToOverlayBuffer)
    {
        //        writeGbufferDepthToCurrentFramebuffer();
    }
    else
    {
        glClear(GL_DEPTH_BUFFER_BIT);
    }

    startTimer(OVERLAY);

    for (auto c : renderInfo.cameras)
    {
        auto camera = c.first;
        bindCamera(camera);
        setViewPort(c.second);
        renderingInterface->renderOverlay(camera);
    }
    stopTimer(OVERLAY);

    glViewport(0, 0, renderWidth, renderHeight);



    m_lighting.applyVolumetricLightBuffer();

    postProcessor.nextFrame();
    postProcessor.bindCurrentBuffer();
    //    postProcessor.switchBuffer();


    startTimer(POSTPROCESSING);
    // postprocessor's 'currentbuffer' will still be bound after render
    postProcessor.render();
    stopTimer(POSTPROCESSING);


    if (params.useSMAA)
    {
        startTimer(SMAATIME);
        smaa->render(postProcessor.getCurrentTexture(), postProcessor.getTargetBuffer());
        postProcessor.switchBuffer();
        postProcessor.bindCurrentBuffer();
        stopTimer(SMAATIME);
    }

    // write depth to default framebuffer
    if (params.writeDepthToDefaultFramebuffer)
    {
        //        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        postProcessor.bindCurrentBuffer();
        writeGbufferDepthToCurrentFramebuffer();
    }
    // glBindFramebuffer(GL_FRAMEBUFFER, 0);
    //    glClear(GL_COLOR_BUFFER_BIT);
    startTimer(FINAL);
    glViewport(0, 0, renderWidth, renderHeight);
    if (renderDDO)
    {
        bindCamera(&ddo.layout.cam);
        ddo.render();
    }

    {
		// Don't render anything using ImGui here!

		renderingInterface->renderFinal(nullptr);

	}
    stopTimer(FINAL);

    glDisable(GL_BLEND);

    if (params.blitLastFramebuffer) {
		buffer.bind();
		glClearColor(1,0,1,1);
		glClear(GL_COLOR_BUFFER_BIT);

        postProcessor.blitLast(outputWidth, outputHeight, buffer);
	}
    else {
		buffer.bind();
		glClearColor(1,0,1,1);
		glClear(GL_COLOR_BUFFER_BIT);
		
        postProcessor.renderLast(outputWidth, outputHeight, buffer);
	}

    //    if (params.srgbWrites)
    //        glDisable(GL_FRAMEBUFFER_SRGB);

    if (params.useGlFinish) glFinish();

    stopTimer(TOTAL);



    assert_no_glerror();

}


// TODO: This has to be called to draw the Scene!
void CustomDeferredRenderer::render(const Saiga::RenderInfo& _renderInfo) {
	render(_renderInfo, m_frameBuffer);
}

void CustomDeferredRenderer::clearGBuffer()
{
    gbuffer.bind();

    glViewport(0, 0, renderWidth, renderHeight);

    glClearColor(params.clearColor[0], params.clearColor[1], params.clearColor[2], params.clearColor[3]);

    if (params.maskUsedPixels)
    {
        glClearStencil(0xFF);  // sets stencil buffer to 255
    }
    else
    {
        glClearStencil(0x00);
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    gbuffer.unbind();
}



void CustomDeferredRenderer::renderGBuffer(const std::pair<Saiga::Camera*, Saiga::ViewPort>& camera)
{
    startTimer(GEOMETRYPASS);

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    if (params.maskUsedPixels)
    {
        // mark all written pixels with 0 in the stencil buffer
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 0, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    }
    else
    {
        glDisable(GL_STENCIL_TEST);
    }

    gbuffer.bind();

    //    setViewPort(camera.second);
    //    glViewport(0, 0, renderWidth, renderHeight);



    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);


    if (params.offsetGeometry)
    {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(params.offsetFactor, params.offsetUnits);
    }

    if (params.wireframe)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(params.wireframeLineSize);
    }
    InterfaceType* renderingInterface = dynamic_cast<InterfaceType*>(rendering);
    renderingInterface->render(camera.first);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


    if (params.offsetGeometry)
    {
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    glDisable(GL_STENCIL_TEST);

    gbuffer.unbind();


    stopTimer(GEOMETRYPASS);

    assert_no_glerror();
}

void CustomDeferredRenderer::renderDepthMaps()
{
    startTimer(DEPTHMAPS);

    InterfaceType* renderingInterface = dynamic_cast<InterfaceType*>(rendering);
    m_lighting.renderDepthMaps(renderingInterface);


    stopTimer(DEPTHMAPS);

    assert_no_glerror();
}

void CustomDeferredRenderer::renderLighting(const std::pair<Saiga::Camera*, Saiga::ViewPort>& camera)
{
    startTimer(LIGHTING);

    m_lighting.render(camera.first, camera.second);

    stopTimer(LIGHTING);

    assert_no_glerror();
}

void CustomDeferredRenderer::renderSSAO(const std::pair<Saiga::Camera*, Saiga::ViewPort>& camera)
{
    startTimer(SSAOT);

    if (params.useSSAO) ssao->render(camera.first, camera.second, &gbuffer);


    stopTimer(SSAOT);

    assert_no_glerror();
}

void CustomDeferredRenderer::writeGbufferDepthToCurrentFramebuffer()
{
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_ALWAYS);
    blitDepthShader->bind();
    blitDepthShader->uploadTexture(gbuffer.getTextureDepth().get());
    quadMesh.bindAndDraw();
    blitDepthShader->unbind();
    glDepthFunc(GL_LESS);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    assert_no_glerror();
}



void CustomDeferredRenderer::printTimings()
{
    std::cout << "====================================" << std::endl;
    std::cout << "Geometry pass: " << getTime(GEOMETRYPASS) << "ms" << std::endl;
    std::cout << "SSAO: " << getTime(SSAOT) << "ms" << std::endl;
    std::cout << "Depthmaps: " << getTime(DEPTHMAPS) << "ms" << std::endl;
    std::cout << "Lighting: " << getTime(LIGHTING) << "ms" << std::endl;
    m_lighting.printTimings();
    //    std::cout<<"Light accumulation: "<<getTime(LIGHTACCUMULATION)<<"ms"<<endl;
    std::cout << "Overlay pass: " << getTime(OVERLAY) << "ms" << std::endl;
    std::cout << "Postprocessing: " << getTime(POSTPROCESSING) << "ms" << std::endl;
    postProcessor.printTimings();
    std::cout << "SMAA: " << getTime(SMAATIME) << "ms" << std::endl;
    std::cout << "Final pass: " << getTime(FINAL) << "ms" << std::endl;
    float total = getTime(TOTAL);
    std::cout << "Total: " << total << "ms (" << 1000 / total << " fps)" << std::endl;
    std::cout << "====================================" << std::endl;
}

}
