#pragma once

#include "saiga/opengl/framebuffer.h"
#include "saiga/opengl/rendering/deferredRendering/gbuffer.h"

#include "saiga/opengl/rendering/deferredRendering/lighting/deferred_lighting.h"
#include "CustomDeferredLighting.h"

#include "saiga/opengl/rendering/deferredRendering/lighting/ssao.h"
#include "saiga/opengl/rendering/deferredRendering/postProcessor.h"
#include "saiga/opengl/rendering/overlay/deferredDebugOverlay.h"
//#include "saiga/opengl/rendering/renderer.h"
#include "MockupRenderer.h"
#include "CustomPostProcessor.h"

#include "CustomSMAA.h"

#include "saiga/opengl/rendering/deferredRendering/deferred_renderer.h"

namespace Saiga {

struct CustomDeferredRenderingParameters : public RenderingParameters
{
    /**
     * When true the depth of the gbuffer is blitted to the default framebuffer.
     */
    bool writeDepthToDefaultFramebuffer = false;

    /**
     * When true the depth of the gbuffer is blitted to the default framebuffer.
     */
    bool writeDepthToOverlayBuffer = true;

    /**
     * Mark all pixels rendered in the geometry pass in the stencil buffer. These pixels then will not be affected by
     * directional lighting. This is especially good when alot of pixels do not need to be lit. For example when huge
     * parts of the screeen is covered by the skybox.
     */
    bool maskUsedPixels = true;


    float renderScale = 1.0f;  // a render scale of 2 is equivalent to 4xSSAA

    bool useGPUTimers = true;  // meassure gpu times of individual passes. This can decrease the overall performance


    bool useSSAO = false;

    bool useSMAA              = false;
    CustomSMAA::Quality smaaQuality = CustomSMAA::Quality::SMAA_PRESET_HIGH;

    vec4 lightingClearColor = vec4(0, 0, 0, 0);

    int shadowSamples = 16;

    bool offsetGeometry = false;
    float offsetFactor = 1.0f, offsetUnits = 1.0f;
    bool blitLastFramebuffer = true;

    GBufferParameters gbp;
    PostProcessorParameters ppp;

    void fromConfigFile(const std::string& file) {}
};


// TODO: use this renderer interface
class CustomDeferredRenderingInterface : public RenderingInterfaceBase {
	public:
		virtual ~CustomDeferredRenderingInterface() {}
	// the render pass consists of:
	virtual void render(Camera *cam) {}
	virtual void renderDepth(Camera *cam) {}
	virtual void renderOverlay(Camera *cam) {}
	virtual void renderFinal(Camera *cam) {}
};

class CustomDeferredRenderer : public MockupRenderer
{
   public:
    using InterfaceType = CustomDeferredRenderingInterface;
    using ParameterType = CustomDeferredRenderingParameters;

	CustomDeferredLighting m_lighting;
    CustomPostProcessor postProcessor;
    ParameterType params;

    CustomDeferredRenderer(int width, int height, ParameterType _params = ParameterType());
    CustomDeferredRenderer& operator=(CustomDeferredRenderer& l) = delete;
    virtual ~CustomDeferredRenderer();

    void render(const RenderInfo& renderInfo) override;
    void render(const RenderInfo& renderInfo, Framebuffer& target);

	void setViewPortMock();

    enum DeferredTimings
    {
        TOTAL = 0,
        GEOMETRYPASS,
        SSAOT,
        DEPTHMAPS,
        LIGHTING,
        POSTPROCESSING,
        LIGHTACCUMULATION,
        OVERLAY,
        FINAL,
        SMAATIME,
        COUNT,
    };

    float getTime(DeferredTimings timer)
    {
        if (!params.useGPUTimers && timer != TOTAL) return 0;
        return timers[timer].getTimeMS();
    }
    float getUnsmoothedTimeMS(DeferredTimings timer)
    {
        if (!params.useGPUTimers && timer != TOTAL) return 0;
        return timers[timer].MultiFrameOpenGLTimer::getTimeMS();
    }
    float getTotalRenderTime() override { return getUnsmoothedTimeMS(CustomDeferredRenderer::DeferredTimings::TOTAL); }

    void printTimings() override;
    void resize(int outputWidth, int outputHeight) override;

    int getRenderWidth() { return renderWidth; }
    int getRenderHeight() { return renderHeight; }

   public:
    int renderWidth, renderHeight;
    std::shared_ptr<SSAO> ssao;
    std::shared_ptr<CustomSMAA> smaa;
    GBuffer gbuffer;

    std::shared_ptr<MVPTextureShader> blitDepthShader;
    IndexedVertexBuffer<VertexNT, GLushort> quadMesh;
    std::vector<FilteredMultiFrameOpenGLTimer> timers;
    std::shared_ptr<Texture> blackDummyTexture;
    bool showLightingImgui = false;
    bool renderDDO         = false;
    DeferredDebugOverlay ddo;


    void clearGBuffer();
    void renderGBuffer(const std::pair<Camera*, ViewPort>& camera);
    void renderDepthMaps();  // render the scene from the lights perspective (don't need user camera here)
    void renderLighting(const std::pair<Camera*, ViewPort>& camera);
    void renderSSAO(const std::pair<Camera*, ViewPort>& camera);

    void writeGbufferDepthToCurrentFramebuffer();

    void startTimer(DeferredTimings timer)
    {
        if (params.useGPUTimers || timer == TOTAL) timers[timer].startTimer();
    }
    void stopTimer(DeferredTimings timer)
    {
        if (params.useGPUTimers || timer == TOTAL) timers[timer].stopTimer();
    }
};

}
