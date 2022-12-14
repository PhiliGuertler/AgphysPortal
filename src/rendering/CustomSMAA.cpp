/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "CustomSMAA.h"

#include "saiga/core/geometry/triangle_mesh_generator.h"
#include "saiga/core/image/imageGenerator.h"
#include "saiga/core/imgui/imgui.h"
#include "saiga/opengl/rendering/deferredRendering/gbuffer.h"
#include "saiga/opengl/shader/shaderLoader.h"
#include "saiga/opengl/smaa/AreaTex.h"
#include "saiga/opengl/smaa/SearchTex.h"

namespace Saiga
{
void CustomSMAABlendingWeightCalculationShader::checkUniforms()
{
    Shader::checkUniforms();
    location_edgeTex   = Shader::getUniformLocation("edgeTex");
    location_areaTex   = Shader::getUniformLocation("areaTex");
    location_searchTex = Shader::getUniformLocation("searchTex");
}

void CustomSMAABlendingWeightCalculationShader::uploadTextures(std::shared_ptr<TextureBase> edgeTex,
                                                         std::shared_ptr<TextureBase> areaTex,
                                                         std::shared_ptr<TextureBase> searchTex)
{
    edgeTex->bind(0);
    Shader::upload(location_edgeTex, 0);

    areaTex->bind(1);
    Shader::upload(location_areaTex, 1);

    searchTex->bind(2);
    Shader::upload(location_searchTex, 2);
}

void CustomSMAANeighborhoodBlendingShader::checkUniforms()
{
    Shader::checkUniforms();
    location_colorTex = Shader::getUniformLocation("colorTex");
    location_blendTex = Shader::getUniformLocation("blendTex");
}

void CustomSMAANeighborhoodBlendingShader::uploadTextures(std::shared_ptr<TextureBase> colorTex,
                                                    std::shared_ptr<TextureBase> blendTex)
{
    colorTex->bind(0);
    Shader::upload(location_colorTex, 0);

    blendTex->bind(1);
    Shader::upload(location_blendTex, 1);
}


CustomSMAA::CustomSMAA(int w, int h)
{
    screenSize = ivec2(w, h);
    stencilTex = framebuffer_texture_t(new Texture());


    // GL_STENCIL_INDEX may be used for format only if the GL version is 4.4 or higher.
    bool useStencilOnly = hasExtension("GL_ARB_texture_stencil8");
    if (useStencilOnly)
    {
        stencilTex->create(w, h, GL_STENCIL_INDEX, GL_STENCIL_INDEX8, GL_UNSIGNED_BYTE);
    }
    else
    {
        stencilTex->create(w, h, GL_DEPTH_STENCIL, GL_DEPTH24_STENCIL8, GL_UNSIGNED_INT_24_8);
        std::cerr << "Warning: OpenGL extension ARB_texture_stencil8 not found. Fallback to Depth Stencil Texture."
                  << std::endl;
    }

    edgesTex = framebuffer_texture_t(new Texture());
    edgesTex->create(w, h, GL_RGBA, GL_RGBA8, GL_UNSIGNED_BYTE);
    edgesFb.create();
    edgesFb.attachTexture(edgesTex);
    if (useStencilOnly)
        edgesFb.attachTextureStencil(stencilTex);
    else
        edgesFb.attachTextureDepthStencil(stencilTex);
    edgesFb.drawToAll();
    edgesFb.check();
    edgesFb.unbind();

    blendTex = framebuffer_texture_t(new Texture());
    blendTex->create(w, h, GL_RGBA, GL_RGBA8, GL_UNSIGNED_BYTE);
    blendFb.create();
    blendFb.attachTexture(blendTex);
    blendFb.attachTextureStencil(stencilTex);
    blendFb.drawToAll();
    blendFb.check();
    blendFb.unbind();

    areaTex = framebuffer_texture_t(new Texture());
    areaTex->create(AREATEX_WIDTH, AREATEX_HEIGHT, GL_RG, GL_RG8, GL_UNSIGNED_BYTE, areaTexBytes);

    searchTex = framebuffer_texture_t(new Texture());
    searchTex->create(SEARCHTEX_WIDTH, SEARCHTEX_HEIGHT, GL_RED, GL_R8, GL_UNSIGNED_BYTE, searchTexBytes);


    auto qb = TriangleMeshGenerator::createFullScreenQuadMesh();
    quadMesh.fromMesh(*qb);
}

void CustomSMAA::loadShader(CustomSMAA::Quality _quality)
{
    quality = _quality;
    // example:
    //#define SMAA_RT_METRICS float4(1.0 / 1280.0, 1.0 / 720.0, 1280.0, 720.0)
    vec4 rtMetrics(1.0f / screenSize[0], 1.0f / screenSize[1], screenSize[0], screenSize[1]);
    std::string rtMetricsStr = "#define SMAA_RT_METRICS float4(" + std::to_string(rtMetrics[0]) + "," +
                               std::to_string(rtMetrics[1]) + "," + std::to_string(rtMetrics[2]) + "," +
                               std::to_string(rtMetrics[3]) + ")";

    std::string qualityStr;
    switch (quality)
    {
        case Quality::SMAA_PRESET_LOW:
            qualityStr = "#define SMAA_PRESET_LOW";
            break;
        case Quality::SMAA_PRESET_MEDIUM:
            qualityStr = "#define SMAA_PRESET_MEDIUM";
            break;
        case Quality::SMAA_PRESET_HIGH:
            qualityStr = "#define SMAA_PRESET_HIGH";
            break;
        case Quality::SMAA_PRESET_ULTRA:
            qualityStr = "#define SMAA_PRESET_ULTRA";
            break;
    }

    ShaderPart::ShaderCodeInjections smaaInjection;
    smaaInjection.emplace_back(GL_VERTEX_SHADER, rtMetricsStr, 1);
    smaaInjection.emplace_back(GL_FRAGMENT_SHADER, rtMetricsStr, 1);
    smaaInjection.emplace_back(GL_VERTEX_SHADER, qualityStr, 2);
    smaaInjection.emplace_back(GL_FRAGMENT_SHADER, qualityStr, 2);

    smaaEdgeDetectionShader =
        shaderLoader.load<PostProcessingShader>("post_processing/smaa/SMAAEdgeDetection.glsl", smaaInjection);
    smaaBlendingWeightCalculationShader = shaderLoader.load<CustomSMAABlendingWeightCalculationShader>(
        "post_processing/smaa/SMAABlendingWeightCalculation.glsl", smaaInjection);
    smaaNeighborhoodBlendingShader = shaderLoader.load<CustomSMAANeighborhoodBlendingShader>(
        "post_processing/smaa/SMAANeighborhoodBlending.glsl", smaaInjection);

    shaderLoaded = true;
    assert_no_glerror();
}

void CustomSMAA::resize(int w, int h)
{
    screenSize = make_ivec2((float)w, (float)h);
    edgesFb.resize(w, h);
    blendFb.resize(w, h);
    shaderLoaded = false;
}

void CustomSMAA::render(framebuffer_texture_t input, Framebuffer& output)
{
    if (!shaderLoaded) loadShader(quality);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);
    glViewport(0, 0, screenSize[0], screenSize[1]);

    // write 1 to stencil if the pixel is not discarded
    glStencilFunc(GL_ALWAYS, 0x1, ~0);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    edgesFb.bind();
    glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    smaaEdgeDetectionShader->bind();
    smaaEdgeDetectionShader->uploadTexture(input);
    quadMesh.bindAndDraw();
    smaaEdgeDetectionShader->unbind();
    assert_no_glerror();


    // only work on pixels that are marked with 1
    glStencilFunc(GL_EQUAL, 0x1, ~0);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);


    blendFb.bind();
    glClear(GL_COLOR_BUFFER_BIT);
    smaaBlendingWeightCalculationShader->bind();
    smaaBlendingWeightCalculationShader->uploadTextures(edgesTex, areaTex, searchTex);
    quadMesh.bindAndDraw();
    smaaBlendingWeightCalculationShader->unbind();
    assert_no_glerror();


    glDisable(GL_STENCIL_TEST);

    output.bind();
    glClear(GL_COLOR_BUFFER_BIT);
    smaaNeighborhoodBlendingShader->bind();
    smaaNeighborhoodBlendingShader->uploadTextures(input, blendTex);
    quadMesh.bindAndDraw();
    smaaNeighborhoodBlendingShader->unbind();
    assert_no_glerror();


    glEnable(GL_DEPTH_TEST);
}

void CustomSMAA::renderImGui()
{
    ImGui::PushID("SMAA::renderImGui");
    static const char* items[4] = {"LOW", "MEDIUM", "HIGH", "ULTRA"};
    int currentItem             = (int)quality;

    if (ImGui::Combo("Quality", &currentItem, items, 4))
    {
        quality      = (Quality)currentItem;
        shaderLoaded = false;
    }
    ImGui::PopID();
}

}  // namespace Saiga
