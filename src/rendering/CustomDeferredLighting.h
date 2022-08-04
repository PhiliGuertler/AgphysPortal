#pragma once

#include "saiga/core/camera/camera.h"
#include "saiga/opengl/framebuffer.h"
#include "saiga/opengl/indexedVertexBuffer.h"
#include "saiga/opengl/query/gpuTimer.h"
#include "saiga/opengl/shader/basic_shaders.h"
#include "saiga/opengl/uniformBuffer.h"
#include "saiga/opengl/vertex.h"

namespace Saiga
{

class PointLightShader;
class SpotLightShader;
class DirectionalLightShader;
class BoxLightShader;
class LightAccumulationShader;

class SpotLight;
class PointLight;
class DirectionalLight;
class BoxLight;

class CustomDeferredRenderingInterface;

struct CustomDeferredLightingShaderNames
{
    std::string pointLightShader       = "lighting/light_point.glsl";
    std::string spotLightShader        = "lighting/light_spot.glsl";
    std::string directionalLightShader = "lighting/light_directional.glsl";
    std::string boxLightShader         = "lighting/light_box.glsl";
    std::string debugShader            = "lighting/debugmesh.glsl";
    std::string stencilShader          = "lighting/stenciltest.glsl";
};


class CustomDeferredLighting
{
    friend class LightingController;

   private:
    int width, height;
    std::shared_ptr<MVPColorShader> debugShader;
    std::shared_ptr<MVPTextureShader> textureShader;
    std::shared_ptr<MVPTextureShader> volumetricBlurShader;
    std::shared_ptr<Shader> volumetricBlurShader2;
    UniformBuffer shadowCameraBuffer;

    // the vertex position is sufficient. no normals and texture coordinates needed.
    typedef IndexedVertexBuffer<Vertex, GLushort> lightMesh_t;



    std::shared_ptr<PointLightShader> pointLightShader, pointLightShadowShader, pointLightVolumetricShader;
    lightMesh_t pointLightMesh;
    std::vector<std::shared_ptr<PointLight> > pointLights;

    std::shared_ptr<SpotLightShader> spotLightShader, spotLightShadowShader, spotLightVolumetricShader;
    lightMesh_t spotLightMesh;
    std::vector<std::shared_ptr<SpotLight> > spotLights;

    std::shared_ptr<BoxLightShader> boxLightShader, boxLightShadowShader, boxLightVolumetricShader;
    lightMesh_t boxLightMesh;
    std::vector<std::shared_ptr<BoxLight> > boxLights;

    std::shared_ptr<DirectionalLightShader> directionalLightShader, directionalLightShadowShader;
    lightMesh_t directionalLightMesh;
    std::vector<std::shared_ptr<DirectionalLight> > directionalLights;

    ShaderPart::ShaderCodeInjections volumetricInjection;
    ShaderPart::ShaderCodeInjections shadowInjection;

    std::shared_ptr<MVPShader> stencilShader;
    GBuffer& gbuffer;


    bool lightDepthTest = true;
    bool stencilCulling = true;


    std::vector<FilteredMultiFrameOpenGLTimer> timers2;
    std::vector<std::string> timerStrings;
    void startTimer(int timer)
    {
        if (useTimers) timers2[timer].startTimer();
    }
    void stopTimer(int timer)
    {
        if (useTimers) timers2[timer].stopTimer();
    }
    float getTime(int timer)
    {
        if (!useTimers) return 0;
        return timers2[timer].getTimeMS();
    }

   public:
    vec4 clearColor = make_vec4(0);
    int totalLights;
    int visibleLights;
    int visibleVolumetricLights;
    int renderedDepthmaps;
    int currentStencilId = 0;

    int shadowSamples = 16;  // Quadratic number (1,4,9,16,...)

    bool drawDebug = false;

    bool useTimers = true;

    bool backFaceShadows     = false;
    float shadowOffsetFactor = 2;
    float shadowOffsetUnits  = 10;
    bool renderVolumetric    = false;

    std::shared_ptr<Texture> ssaoTexture;

    std::shared_ptr<Texture> lightAccumulationTexture;
    std::shared_ptr<Texture> volumetricLightTexture, volumetricLightTexture2;
    Framebuffer lightAccumulationBuffer, volumetricBuffer;

    CustomDeferredLighting(GBuffer& gbuffer);
    CustomDeferredLighting& operator=(CustomDeferredLighting& l) = delete;
    ~CustomDeferredLighting();

    void init(int width, int height, bool _useTimers);
    void resize(int width, int height);

    void loadShaders();

    void setRenderDebug(bool b) { drawDebug = b; }
    void createLightMeshes();

	void initDirectionalLightShader();
    std::shared_ptr<DirectionalLight> createDirectionalLight();
	void addDirectionalLight(std::shared_ptr<DirectionalLight> light);
    std::shared_ptr<PointLight> createPointLight();
    std::shared_ptr<SpotLight> createSpotLight();
    std::shared_ptr<BoxLight> createBoxLight();

    void removeLight(std::shared_ptr<DirectionalLight> l);
    void removeLight(std::shared_ptr<PointLight> l);
    void removeLight(std::shared_ptr<SpotLight> l);
    void removeLight(std::shared_ptr<BoxLight> l);


    void initRender();
    void render(Camera* cam, const ViewPort& viewPort);
    void postprocessVolumetric();
    void renderDepthMaps(CustomDeferredRenderingInterface* renderer);
    void renderDebug(Camera* cam);


    void setShader(std::shared_ptr<SpotLightShader> spotLightShader,
                   std::shared_ptr<SpotLightShader> spotLightShadowShader);
    void setShader(std::shared_ptr<PointLightShader> pointLightShader,
                   std::shared_ptr<PointLightShader> pointLightShadowShader);
    void setShader(std::shared_ptr<DirectionalLightShader> directionalLightShader,
                   std::shared_ptr<DirectionalLightShader> directionalLightShadowShader);
    void setShader(std::shared_ptr<BoxLightShader> boxLightShader,
                   std::shared_ptr<BoxLightShader> boxLightShadowShader);

    void setDebugShader(std::shared_ptr<MVPColorShader> shader);
    void setStencilShader(std::shared_ptr<MVPShader> stencilShader);

    // add the volumetric light texture that was previously rendered to the scene
    void applyVolumetricLightBuffer();


    void cullLights(Camera* cam);

    void printTimings();
    void renderImGui(bool* p_open = NULL);

   private:
    void blitGbufferDepthToAccumulationBuffer();
    void setupStencilPass();
    void setupLightPass(bool isVolumetric);

    template <typename T, typename shader_t>
    void renderLightVolume(lightMesh_t& mesh, T obj, Camera* cam, const ViewPort& vp, shader_t shader,
                           shader_t shaderShadow, shader_t shaderVolumetric);


    void renderDirectionalLights(Camera* cam, const ViewPort& vp, bool shadow);
};


template <typename T, typename shader_t>
inline void CustomDeferredLighting::renderLightVolume(lightMesh_t& mesh, T obj, Camera* cam, const ViewPort& vp,
                                                shader_t shaderNormal, shader_t shaderShadow, shader_t shaderVolumetric)
{
    if (!obj->shouldRender()) return;

    if (stencilCulling)
    {
        setupStencilPass();
        stencilShader->bind();

        obj->bindUniformsStencil(*stencilShader);
        mesh.bindAndDraw();
        stencilShader->unbind();
    }

    setupLightPass(obj->isVolumetric());
    shader_t shader = (obj->hasShadows() ? (obj->isVolumetric() ? shaderVolumetric : shaderShadow) : shaderNormal);
    shader->bind();
    shader->DeferredShader::uploadFramebuffer(&gbuffer);
    shader->uploadScreenSize(vp.getVec4());

    obj->bindUniforms(shader, cam);
    mesh.bindAndDraw();
    shader->unbind();
}

}  // namespace Saiga
