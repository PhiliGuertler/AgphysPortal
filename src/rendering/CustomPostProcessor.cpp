#include "CustomPostProcessor.h"

#include "saiga/core/geometry/triangle_mesh_generator.h"
#include "saiga/opengl/error.h"
#include "saiga/opengl/rendering/deferredRendering/deferredRendering.h"
#include "saiga/opengl/shader/shaderLoader.h"

namespace Saiga
{

void CustomPostProcessor::init(int width, int height, GBuffer* gbuffer, PostProcessorParameters params,
                         std::shared_ptr<Texture> LightAccumulationTexture, bool _useTimers)
{
    this->params    = params;
    this->width     = width;
    this->height    = height;
    this->gbuffer   = gbuffer;
    this->useTimers = _useTimers;
    //    this->gbufferDepth = gbuffer->getTextureDepth();
    //    this->gbufferNormals = gbuffer->getTextureNormal();
    //    this->gbufferColor = gbuffer->getTextureColor();
    createFramebuffers();

    auto qb = TriangleMeshGenerator::createFullScreenQuadMesh();
    quadMesh.fromMesh(*qb);



    this->LightAccumulationTexture = LightAccumulationTexture;

    passThroughShader = shaderLoader.load<PostProcessingShader>("post_processing/post_processing.glsl");

    //    computeTest = shaderLoader.load<Shader>("computeTest.glsl");
    assert_no_glerror();
}

void CustomPostProcessor::nextFrame()
{
    //    gbuffer->blitDepth(framebuffers[0].getId());
    currentBuffer = 0;
    lastBuffer    = 1;
}

void CustomPostProcessor::createFramebuffers()
{
    std::shared_ptr<Texture> depth = std::make_shared<Texture>();
    depth->create(width, height, GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT32, GL_UNSIGNED_SHORT);

    auto tex = framebuffer_texture_t(depth);

    for (int i = 0; i < 2; ++i)
    {
        framebuffers[i].create();

        //        if(i==0){
        //            std::shared_ptr<Texture> depth_stencil = new Texture();
        //            depth_stencil->create(width,height,GL_DEPTH_STENCIL,
        //            GL_DEPTH24_STENCIL8,GL_UNSIGNED_INT_24_8);
        //            framebuffers[i].attachTextureDepthStencil(depth_stencil);

        framebuffers[i].attachTextureDepth(tex);

        //           framebuffers[1].attachTextureDepth( tex );
        //        }


        textures[i] = std::make_shared<Texture>();
        if (params.srgb)
        {
            textures[i]->create(width, height, GL_RGBA, GL_SRGB8_ALPHA8, GL_UNSIGNED_BYTE);
        }
        else
        {
            switch (params.quality)
            {
                case Quality::LOW:
                    textures[i]->create(width, height, GL_RGBA, GL_RGBA8, GL_UNSIGNED_BYTE);
                    break;
                case Quality::MEDIUM:
                    textures[i]->create(width, height, GL_RGBA, GL_RGBA16, GL_UNSIGNED_SHORT);
                    break;
                case Quality::HIGH:
                    textures[i]->create(width, height, GL_RGBA, GL_RGBA16, GL_UNSIGNED_SHORT);
                    break;
            }
        }
        framebuffers[i].attachTexture(framebuffer_texture_t(textures[i]));
        framebuffers[i].drawToAll();
        framebuffers[i].check();
        framebuffers[i].unbind();
    }
    assert_no_glerror();
}

void CustomPostProcessor::bindCurrentBuffer()
{
    framebuffers[currentBuffer].bind();
}

void CustomPostProcessor::switchBuffer()
{
    lastBuffer    = currentBuffer;
    currentBuffer = (currentBuffer + 1) % 2;
}

void CustomPostProcessor::render()
{
    int effects = postProcessingEffects.size();

    if (effects == 0)
    {
        std::cout << "Warning no post processing effects specified. The screen will probably be black!" << std::endl;
        return;
    }


    first = true;


    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);

    for (int i = 0; i < postProcessingEffects.size(); ++i)
    {
        switchBuffer();
        if (useTimers) shaderTimer[i].startTimer();
        applyShader(postProcessingEffects[i]);
        if (useTimers) shaderTimer[i].stopTimer();
    }

    //    shaderTimer[effects-1].startTimer();
    //    applyShaderFinal(postProcessingEffects[effects-1]);
    //    shaderTimer[effects-1].stopTimer();

    glEnable(GL_DEPTH_TEST);


    assert_no_glerror();

    //    std::cout<<"Time spent on the GPU: "<< timer.getTimeMS() <<std::endl;
}

void CustomPostProcessor::setPostProcessingEffects(
    const std::vector<std::shared_ptr<PostProcessingShader> >& postProcessingEffects)
{
    assert_no_glerror();
    this->postProcessingEffects = postProcessingEffects;
    createTimers();
    assert_no_glerror();
}

int CustomPostProcessor::addPostProcessingEffect(std::shared_ptr<PostProcessingShader> shader) {
	int index = this->postProcessingEffects.size();
	assert_no_glerror();
	this->postProcessingEffects.push_back(shader);
    createTimers();
	assert_no_glerror();
	return index;
}

void CustomPostProcessor::removePostProcessingEffect(int index) {
	assert_no_glerror();
	this->postProcessingEffects.erase(this->postProcessingEffects.begin()+index);
    createTimers();
	assert_no_glerror();
}

void CustomPostProcessor::createTimers()
{
    shaderTimer.clear();

    if (!useTimers) return;

    shaderTimer.resize(postProcessingEffects.size());
    for (auto& t : shaderTimer)
    {
        t.create();
    }
}

void CustomPostProcessor::printTimings()
{
    if (!useTimers) return;
    for (unsigned int i = 0; i < postProcessingEffects.size(); ++i)
    {
        std::cout << "\t" << shaderTimer[i].getTimeMS() << "ms " << postProcessingEffects[i]->name << std::endl;
    }
}

void CustomPostProcessor::resize(int width, int height)
{
    this->width  = width;
    this->height = height;
    framebuffers[0].resize(width, height);
    framebuffers[1].resize(width, height);
    assert_no_glerror();
}

void CustomPostProcessor::applyShader(std::shared_ptr<PostProcessingShader> postProcessingShader)
{
    framebuffers[currentBuffer].bind();


    postProcessingShader->bind();
    vec4 screenSize(width, height, 1.0 / width, 1.0 / height);
    postProcessingShader->uploadScreenSize(screenSize);
    postProcessingShader->uploadTexture((first) ? LightAccumulationTexture : textures[lastBuffer]);
    postProcessingShader->uploadGbufferTextures(gbuffer);
    postProcessingShader->uploadAdditionalUniforms();
    quadMesh.bindAndDraw();
    postProcessingShader->unbind();

    //    framebuffers[currentBuffer].unbind();

    first = false;
    assert_no_glerror();
}

void CustomPostProcessor::blitLast(int windowWidth, int windowHeight)
{
    //    framebuffers[lastBuffer].blitColor(0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffers[currentBuffer].getId());
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, width, height, 0, 0, windowWidth, windowHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR);
    assert_no_glerror();
}
    
void CustomPostProcessor::blitLast(int windowWidth, int windowHeight, Framebuffer& buffer) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffers[currentBuffer].getId());
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, buffer.getId());
    glBlitFramebuffer(0, 0, width, height, 0, 0, windowWidth, windowHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR);
    assert_no_glerror();
}

void CustomPostProcessor::renderLast(int windowWidth, int windowHeight)
{
    glViewport(0, 0, windowWidth, windowHeight);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);
    passThroughShader->bind();
    vec4 screenSize(width, height, 1.0 / width, 1.0 / height);
    passThroughShader->uploadScreenSize(screenSize);
    passThroughShader->uploadTexture(textures[currentBuffer]);
    passThroughShader->uploadGbufferTextures(gbuffer);
    passThroughShader->uploadAdditionalUniforms();
    quadMesh.bindAndDraw();
    passThroughShader->unbind();
}
	
void CustomPostProcessor::renderLast(int windowWidth, int windowHeight, Framebuffer& buffer) {
    glViewport(0, 0, windowWidth, windowHeight);
    glBindFramebuffer(GL_FRAMEBUFFER, buffer.getId());
    glDisable(GL_DEPTH_TEST);
    passThroughShader->bind();
    vec4 screenSize(width, height, 1.0 / width, 1.0 / height);
    passThroughShader->uploadScreenSize(screenSize);
    passThroughShader->uploadTexture(textures[currentBuffer]);
    passThroughShader->uploadGbufferTextures(gbuffer);
    passThroughShader->uploadAdditionalUniforms();
    quadMesh.bindAndDraw();
    passThroughShader->unbind();
}

framebuffer_texture_t CustomPostProcessor::getCurrentTexture()
{
    return framebuffers[currentBuffer].getTextureColor(0);
}

Framebuffer& CustomPostProcessor::getTargetBuffer()
{
    return framebuffers[lastBuffer];
}

}  // namespace Saiga
