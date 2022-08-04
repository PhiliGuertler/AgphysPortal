#pragma once

#include "saiga/core/camera/camera.h"
#include "saiga/opengl/framebuffer.h"
#include "saiga/opengl/texture/Texture.h"
#include "saiga/opengl/vertexBuffer.h"
#include "saiga/opengl/rendering/deferredRendering/postProcessor.h"

#include "../cuda/particle.h"
#include "../AgphysCudaConfig.h"

class Portal;
class PortalRenderer;

class EffectParticleShader : public Saiga::PostProcessingShader {
	public:
		EffectParticleShader();
		virtual ~EffectParticleShader();

		virtual void uploadAdditionalUniforms() override;

	public:
		std::shared_ptr<Saiga::Texture> m_depth;
		std::shared_ptr<Saiga::Texture> m_color;
};

class EffectParticleBundle {
	public:
		EffectParticleBundle();
		virtual ~EffectParticleBundle();

		void render(Saiga::Camera *cam);

		void resize(int width, int height);

		void setVertexData(std::vector<ParticlePositionRadius>& positions);

		std::shared_ptr<EffectParticleShader> createPostProcessingShader();

	public:
		Saiga::Framebuffer m_frameBuffer;
		std::shared_ptr<Saiga::Texture> m_particleDepth;
		std::shared_ptr<Saiga::Texture> m_particleColor;

		Saiga::VertexBuffer<ParticlePositionRadius> m_vertexBuffer;
		vec4 m_color;
};

class EffectParticleBundleManager {
	public:
		int registerEffectParticleBundle(std::shared_ptr<EffectParticleBundle> bundle);
		EffectParticleBundle& getBundle(int index) const;

		void resize(int width, int height);
		void render(Saiga::Camera *cam, Saiga::Shader& shader);

	private:
		friend class PortalRenderer;
		EffectParticleBundleManager();

		EffectParticleBundleManager(EffectParticleBundleManager& o) = delete;
		EffectParticleBundleManager(EffectParticleBundleManager&& o) = delete;

		EffectParticleBundleManager& operator=(const EffectParticleBundleManager& o) = delete;
		EffectParticleBundleManager& operator=(const EffectParticleBundleManager&& o) = delete;

	private:
		std::vector<std::shared_ptr<EffectParticleBundle>> m_bundles;
};
