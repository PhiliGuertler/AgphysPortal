#include "WaterShaders.h"

#include "../profiling/Time.h"
#include "../imguiOptions.h"
#include "../agphys.h"
#include "Scene.h"

#include "PortalRenderer.h"

WaterSmootherShader::WaterSmootherShader() 
{
	// empty for now
}

void WaterSmootherShader::uploadAdditionalUniforms() {

	int textureBindSpot = 0;
	GLint location;
	location = getUniformLocation("waterPositionTex");
	m_waterPositionTex->bind(++textureBindSpot);
	upload(location, textureBindSpot);

	location = getUniformLocation("waterNormalTex");
	m_waterNormalTex->bind(++textureBindSpot);
	upload(location, textureBindSpot);

	location = getUniformLocation("waterDepthTex");
	m_waterDepthTex->bind(++textureBindSpot);
	upload(location, textureBindSpot);

	location = getUniformLocation("waterVorticityTex");
	m_waterVorticityTex->bind(++textureBindSpot);
	upload(location, textureBindSpot);


	location = getUniformLocation("waterPositionTexBack");
	m_waterPositionTexBack->bind(++textureBindSpot);
	upload(location, textureBindSpot);

	location = getUniformLocation("waterDepthTexBack");
	m_waterDepthTexBack->bind(++textureBindSpot);
	upload(location, textureBindSpot);


	location = getUniformLocation("foamTexture");
	m_foamTexture->bind(++textureBindSpot);
	upload(location, textureBindSpot);


	location = getUniformLocation("u_skybox");
	Agphys::getInstance()->m_scene->m_skybox.cube_texture->bind(++textureBindSpot);
	upload(location, textureBindSpot);

	location = getUniformLocation("coolerGBufferDepth");
	PortalRenderer::get().m_recursiveRenderer->gbuffer.getTextureDepth()->bind(++textureBindSpot);
	upload(location, textureBindSpot);


	ImGuiOptions& options = ImGuiOptions::get();
	location = getUniformLocation("u_specularFactor");
	upload(location, options.specularFactor);

	location = getUniformLocation("u_diffuseFactor");
	upload(location, options.diffuseFactor);

	location = getUniformLocation("u_specularPower");
	upload(location, options.specularPower);

	location = getUniformLocation("u_fresnelPower");
	upload(location, options.fresnelPower);

	location = getUniformLocation("u_specularColor");
	upload(location, options.specularColor);

	location = getUniformLocation("u_diffuseColor");
	upload(location, options.diffuseColor);

	location = getUniformLocation("u_lightPosition");
	upload(location, options.lightPosition);

	location = getUniformLocation("u_lightDirection");
	upload(location, options.lightDirection);

	location = getUniformLocation("u_depthFalloff");
	upload(location, options.depthFalloff);

	location = getUniformLocation("u_enableFoam");
	upload(location, options.enableFoam);


#if 0
	location = getUniformLocation("waterDepthTex");
	m_waterDepthTex->bind(7);
	upload(location, 7);
#endif
}
