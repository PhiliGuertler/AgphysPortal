#pragma once

#include "saiga/core/camera/camera.h"
#include "saiga/opengl/assets/simpleAssetObject.h"
#include "saiga/opengl/assets/objAssetLoader.h"

#include "Skybox.h"
#include "CustomDeferredRenderer.h"

#include "../PlayerCamera.h"
#include "../cuda/logic/portal.h"
#include "../Level.h"

// ######################################################################### //
// ### SimpleTextureModel ################################################## //
// ######################################################################### //

class SimpleTextureModel : public TriangleMesh<VertexNT, uint32_t> {
	// nothing to do here
};


// ######################################################################### //
// ### TexturedPortal ###################################################### //
// ######################################################################### //

class TexturedPortal : public BasicAsset<SimpleTextureModel> {
	public:
		static constexpr const char* deferredShaderStr  = "texturedPortal.glsl";
		static constexpr const char* forwardShaderStr   = "texturedPortal.glsl";
		static constexpr const char* depthShaderStr     = "geometry/texturedAsset_depth.glsl";
		static constexpr const char* wireframeShaderStr = "geometry/texturedAsset.glsl";
		void loadDefaultShaders();

 		void fromPortal(const Portal& portal);
		void updateTextureCoords(Camera *cam, const mat4& model);

		virtual void render(Camera *cam, const mat4& model) override;
		virtual void renderDepth(Camera *cam, const mat4& model) override;

	public:
		std::shared_ptr<Texture> portalTex;
};


// ######################################################################### //
// ### Scene ############################################################### //
// ######################################################################### //

class Scene : public Saiga::CustomDeferredRenderingInterface {
	public:
		Scene();
		~Scene();

		void renderScene(Camera *cam);
		void renderImGui();

		void renderPortal(const Saiga::mat4& view, const Saiga::mat4& proj, bool isOrange);

		//void updatePortalPlane(const Portal& portal, std::shared_ptr<Texture> tex, bool isOrange) {
		void updatePortalPlane(const Portal& portal, bool isOrange);

		// --- DeferredRenderingInterface --- //
		virtual void render(Camera *cam) override;
		virtual void renderDepth(Camera *cam) override;
		virtual void renderOverlay(Camera *cam) override;
		virtual void renderFinal(Camera *cam) override;

		// --- Plane Creation Methods --- //
		std::pair<Saiga::Plane, SimpleAssetObject> createPlane(vec3 center, vec3 normal, float width, vec3 tubeCenter);
		void createPlanes(int numPlanes, float planeDistance);

		// --- Fluid Scene Setup Methods --- //
		void setupFluidPlanes(int particleWidth);
		void setupFluidScene1(int particleWidth);
		void setupFluidScene2(int particleWidth);

		// --- Stress Test Setup Methods --- //
		void setupTestScene();

		// --- Imgui Rendering --- //
		void renderGUI();
		void renderSpawningGui();

		void loadLevel(const std::string& filePath);
		void loadWallLevel(const std::string& filePath);

	public:
		// --- Objects to be rendered --- //
		Saiga::ObjAssetLoader m_objAssetLoader;
		// Checkerboard ground plane
		SimpleAssetObject m_groundPlane;
		// wireframe planes around the debug area
		std::vector<SimpleAssetObject> m_planes;
		// level geometry
		std::shared_ptr<Controls::Level> m_level;
		std::shared_ptr<Controls::Level> m_portalWalls;
		
		// Skybox
		Saiga::Skybox2 m_skybox;

		// Orange Portal
		SimpleAssetObject m_orangePortalAsset;
		// Blue Portal
		SimpleAssetObject m_bluePortalAsset;
		// --- /Objects to be rendered --- //

		// light source
		std::shared_ptr<Saiga::DirectionalLight> m_sun;

		// --- Shaders --- //
		std::shared_ptr<Saiga::MVPShader> m_particleShader;
		std::shared_ptr<Saiga::MVPShader> m_particleShaderFast;
		std::shared_ptr<Saiga::MVPShader> m_particleDepthShader;
		std::shared_ptr<Saiga::MVPShader> m_particleDepthShaderFast;
		// --- /Shaders --- //

		// camera that will be used for recursive render calls
		std::unique_ptr<PerspectivePlayerCamera> m_mockupCamera;
		int m_renderingOrange;

		bool m_shouldRenderGUI;
		bool m_showSaigaGUI;
		bool m_renderParticles;
		bool m_renderShadows;

		// sub renderers (might be placed here instead of in a singleton)
};
