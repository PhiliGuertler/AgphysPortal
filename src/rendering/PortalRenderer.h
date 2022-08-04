#pragma once
#include "../cuda/logic/portal.h"

#include "saiga/core/camera/all.h"
#include "saiga/opengl/framebuffer.h"

#include "CustomDeferredRenderer.h"

#include "EffectParticles.h"

class PortalRenderer {
	public:
		static PortalRenderer& get();

		static void shutdown();
	public:
		void setOrangePortal(std::shared_ptr<Portal>& portal);

		void setBluePortal(std::shared_ptr<Portal>& portal);

		void renderScene(Camera *cam);

		inline void setMaxRecursionDepth(int num) {
			m_maxRecursionDepth = num;
		}

		void resize(int width, int height);

		void registerRenderingInterface(CustomDeferredRenderingInterface *interface);

		void renderToWindow(int width, int height);

		void clearBuffers();

		void renderOrangePortalContents(Camera *cam);
		void renderBluePortalContents(Camera *cam);

		void updatePortalEffects();

		void renderEffectParticles(Camera *cam);

		void shootEffectParticles(int framesToLive, vec3 direction, vec3 origin, vec4 color);
	private:
		static std::unique_ptr<PortalRenderer> s_instance;

	private:
		// enforce singleton nature
		PortalRenderer();

		PortalRenderer(const PortalRenderer& r) = delete;
		PortalRenderer(const PortalRenderer&& r) = delete;

		PortalRenderer& operator=(const PortalRenderer& r) = delete;
		PortalRenderer& operator=(const PortalRenderer&& r) = delete;

	public:
		std::shared_ptr<CustomDeferredRenderer> m_recursiveRenderer;
		std::shared_ptr<EffectParticleBundleManager> m_effectParticleManager;

		Framebuffer m_orangePortalBuffer[2];
		std::shared_ptr<Texture> m_orangePortalColorTex[2];
		std::shared_ptr<Texture> m_orangePortalDepthTex[2];
		int m_activeOrangePortalBuffer;
		PerspectiveCamera m_orangeCam;

		Framebuffer m_bluePortalBuffer[2];
		std::shared_ptr<Texture> m_bluePortalColorTex[2];
		std::shared_ptr<Texture> m_bluePortalDepthTex[2];
		int m_activeBluePortalBuffer;
		PerspectiveCamera m_blueCam;

	public:
		int m_maxRecursionDepth;
		int m_debugPerspective = 0;

		std::shared_ptr<Portal> m_orangePortal;
		int m_orangePortalEffectsIndex;
		std::shared_ptr<Portal> m_bluePortal;
		int m_bluePortalEffectsIndex;
		std::shared_ptr<Saiga::MVPShader> m_effectParticleShader;

		int m_shootingEffectBundleIndex;
		int m_framesToLive;
		vec3 m_flightDirection;
		vec3 m_flightPosition;
		float m_flightSpeed;

		IndexedVertexBuffer<VertexNT, GLushort> m_quadMesh;
		std::shared_ptr<Shader> m_quadShader;
};
