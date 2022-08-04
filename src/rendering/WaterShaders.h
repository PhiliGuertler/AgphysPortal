#pragma once
#include "saiga/opengl/rendering/deferredRendering/postProcessor.h"
#include "saiga/opengl/texture/Texture.h"

// this extends shader aswell, so it should be loadable via Saiga::shaderLoader.load<WaterSmootherShader>(fileName)
// Also this shader will be called on a screen-sized quad!
class WaterSmootherShader : public Saiga::PostProcessingShader {
	public:
		WaterSmootherShader();

		virtual void uploadAdditionalUniforms() override;
	public:
		// extra textures go here
		// GLint location_tex1, location_tex2, ...
		std::shared_ptr<Saiga::Texture> m_waterPositionTex;
		std::shared_ptr<Saiga::Texture> m_waterNormalTex;
		std::shared_ptr<Saiga::Texture> m_waterDepthTex;
		std::shared_ptr<Saiga::Texture> m_waterVorticityTex;

		std::shared_ptr<Saiga::Texture> m_waterPositionTexBack;
		std::shared_ptr<Saiga::Texture> m_waterDepthTexBack;

		std::shared_ptr<Saiga::Texture> m_foamTexture;
};
