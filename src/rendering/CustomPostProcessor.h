#pragma once

#include "saiga/core/util/quality.h"
#include "saiga/opengl/framebuffer.h"
#include "saiga/opengl/indexedVertexBuffer.h"
#include "saiga/opengl/query/gpuTimer.h"
#include "saiga/opengl/rendering/deferredRendering/gbuffer.h"
#include "saiga/opengl/shader/basic_shaders.h"
#include "saiga/opengl/vertex.h"

#include "saiga/opengl/rendering/deferredRendering/postProcessor.h"

namespace Saiga
{
class CustomPostProcessor
{
   private:
    PostProcessorParameters params;
    int width, height;
    Framebuffer framebuffers[2];
    std::shared_ptr<Texture> textures[2];
    GBuffer* gbuffer;
    int currentBuffer = 0;
    int lastBuffer    = 1;
    IndexedVertexBuffer<VertexNT, GLushort> quadMesh;
    std::vector<std::shared_ptr<PostProcessingShader> > postProcessingEffects;
    std::shared_ptr<PostProcessingShader> passThroughShader;

    bool useTimers = false;
    std::vector<FilteredMultiFrameOpenGLTimer> shaderTimer;

    std::shared_ptr<Shader> computeTest;

    // the first post processing shader reads from the lightaccumulation texture.
    std::shared_ptr<Texture> LightAccumulationTexture = nullptr;
    bool first                                        = false;

    void createFramebuffers();
    void applyShader(std::shared_ptr<PostProcessingShader> postProcessingShader);

   public:
    void createTimers();

    void init(int width, int height, GBuffer* gbuffer, PostProcessorParameters params,
              std::shared_ptr<Texture> LightAccumulationTexture, bool _useTimers);

    void nextFrame();
    void bindCurrentBuffer();
    void switchBuffer();

    void render();

    void setPostProcessingEffects(const std::vector<std::shared_ptr<PostProcessingShader> >& postProcessingEffects);
	int addPostProcessingEffect(std::shared_ptr<PostProcessingShader> shader);
	void removePostProcessingEffect(int index);

    void printTimings();
    void resize(int width, int height);
    void blitLast(int windowWidth, int windowHeight);
    void blitLast(int windowWidth, int windowHeight, Framebuffer& buffer);
	void renderLast(int windowWidth, int windowHeight, Framebuffer& buffer);
    void renderLast(int windowWidth, int windowHeight);

    framebuffer_texture_t getCurrentTexture();
    Framebuffer& getTargetBuffer();
};

}  // namespace Saiga
