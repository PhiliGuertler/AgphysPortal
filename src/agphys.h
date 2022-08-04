#pragma once

#include "saiga/core/sdl/sdl_camera.h"
#include "saiga/core/sdl/sdl_eventhandler.h"
#include "saiga/cuda/interop.h"
#include "saiga/opengl/assets/all.h"
#include "saiga/opengl/assets/objAssetLoader.h"
#include "saiga/opengl/ffmpeg/videoEncoder.h"
#include "saiga/opengl/instancedBuffer.h"
#include "saiga/opengl/rendering/deferredRendering/deferredRendering.h"
#include "saiga/opengl/window/WindowTemplate.h"
#include "saiga/opengl/world/proceduralSkybox.h"
#include "rendering/Skybox.h"
#include "PlayerCamera.h"

#include "saiga/opengl/texture/Texture2D.h"
#include "saiga/core/image/managedImage.h"
#include "saiga/opengl/world/TextureDisplay.h"


#include "glBuffers.h"

#include "imguiOptions.h"

#include "rendering/Scene.h"

// ### namespace Controls includes ### //
#include "Player.h"
#include "Level.h"
// ### /namespace Controls includes ### //

#define LOAD_PLANES 0

using Saiga::AABB;
using Saiga::Camera;
using Saiga::SDL_KeyListener;
using Saiga::SDL_MouseListener;
using Saiga::SimpleAssetObject;

class ParticleSystem;

class Agphys : public StandaloneWindow<WindowManagement::SDL, DeferredRenderer>,
               public SDL_KeyListener,
               public SDL_MouseListener
{
public:
    Agphys();
    ~Agphys();

	// particle system (de)initialization
    void initParticles();
    void destroyParticles();
    void resetParticles();

	void setParticleBuffersToDefaultValues(HostParticles& particles);

	// particle spawning
	void spawnParticles(int numParticles);
    void spawnRigidCubes(int numCubes);
	void spawnRigidMeshes(int numInstances);
	void spawnInterlockedCubes();
	void spawnCloth(int N, int M);
	void spawnFluid(int numParticles);
	void spawnFluidWithPosition(int numParticles,vec3 corner,float length,float width);
	//camera functions and attributes
	struct CameraAttributes{
		// 2 = debug, 1 = firstPerson
		int cameraType = 2;
		
	};
	struct CameraAttributes camAttributes;
	bool toggle = true;
	void updateCamera(int dt);

    // Deferred rendering and update functions (called from Saiga)
    void update(float dt) override;
    void interpolate(float dt, float interpolation) override;
    void render(Camera* cam) override;
    void renderDepth(Camera* cam) override;
    void renderOverlay(Camera* cam) override;
    void renderFinal(Camera* cam) override;
    void parallelUpdate(float dt) override {}

    // Key/Mouse Input (in agphys_io.cpp)
    void pollInputs();
    void keyPressed(SDL_Keysym key) override;
    void keyReleased(SDL_Keysym key) override;
    void mouseMoved(int x, int y) override;
    void mousePressed(int key, int x, int y) override;
    void mouseReleased(int key, int x, int y) override;


	inline int getNumFrames() { return window->mainLoop.numFrames; }

	static Agphys *getInstance() { return s_instance; }

	inline Controls::Player *getPlayer() const { return m_ownplayer.get(); }

	void loadLevel(const std::string& fileName);
	void setupSceneObjects();

private:
	static Agphys *s_instance;

public:
    // Particles
    int numberParticles = 0;
    std::shared_ptr<ParticleSystem> particleSystem;

	std::unique_ptr<Scene> m_scene;
    //HUD
    Texture2D hudTexture;
    TextureDisplay hudDisplay;
    //nonHitHud
    Texture2D hudTexture2;
    TextureDisplay hudDisplay2;
    // ImGUI Stuff
    ImGui::TimeGraph physicsGraph;

    // Other Stuff
    bool pause = false;
    Saiga::VideoEncoder enc;

    // ##################################################################### //
    // ### Own Stuff ####################################################### //
    bool m_frameByFrame = false;
    bool m_rightMouseButtonIsPressed = false;
    vec2 m_lastMousePosition = {0.f,0.f};

	std::unique_ptr<Controls::Player> m_ownplayer;
	std::unique_ptr<Controls::Level> m_level;
    RayTriangleCollision rayTriangleCollision;
	
};
