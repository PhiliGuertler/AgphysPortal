#include "Scene.h"

#include "../agphys.h"
#include "../imguiOptions.h"
#include "../cuda/fluidCreator.h"
#include "../profiling/Profiler.h"
#include "../cuda/rigidBodyManager.h"
#include "../hacky.hack"
#include "../platform/platform.h"

#include "PortalRenderer.h"

void Scene::renderGUI()
{
	if (!m_shouldRenderGUI)
	{
		return;
	}

	ImGuiOptions& options = ImGuiOptions::get();
	auto agphys = Agphys::getInstance();
		
	ImGui::SetMouseCursor(ImGuiMouseCursor_None);

	if(options.resolutionChanged) {
		ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
		ImGui::SetNextWindowSize(ImVec2(400, options.windowHeight/4*3), ImGuiCond_Always);
	}
	ImGui::Begin("Agphys");

	ImGui::InputInt("particleCount", &agphys->numberParticles, 0, 10000);

	if (ImGui::Button("Create Particle System"))
	{
		agphys->destroyParticles();
		agphys->initParticles();
	}
	if(ImGui::CollapsingHeader("Tests")) {
		if(ImGui::Button("Create Test Scene")) {
			options.setUpTestScene();
			setupTestScene();
			agphys->destroyParticles();
			agphys->initParticles();
		}

		ImGui::Separator();
		static int particleWidthForFluidScene = 20;
		ImGui::SliderInt("Particle Width for Fluid Scene", &particleWidthForFluidScene, 2, 30);
		if(ImGui::Button("Create Fluid Scene 1")) {
			setupFluidScene1(particleWidthForFluidScene);
		}
		if(ImGui::Button("Create Fluid Scene 2")) {
			setupFluidScene2(particleWidthForFluidScene);
		}

		ImGui::Separator();
	}

	if(ImGui::CollapsingHeader("Plane Options")) {
		ImGui::InputInt("#planes", &options.numPlanes, 1, 15);
		ImGui::InputFloat("Plane distance from center", &options.planeDistance);
	}
	ImGui::Separator();

	ImGui::Checkbox("renderParticles", &m_renderParticles);
	ImGui::Checkbox("renderShadows", &m_renderShadows);
	ImGui::Checkbox("showSaigaGui", &m_showSaigaGUI);
	ImGui::Checkbox("render Meshes", &options.renderMeshes);
	if(ImGui::CollapsingHeader("Portal Rendering Options")) {
		ImGui::SliderInt("Recursion Depth", &(PortalRenderer::get().m_maxRecursionDepth), 0, 10);
		ImGui::Text("Camera Perspective Values: 0: default Player View, 1: Orange Portal View, 2: Blue Portal View");
		ImGui::SliderInt("Camera Perspective", &(PortalRenderer::get().m_debugPerspective), 0, 2);
	}
	ImGui::Separator();

	ImGui::Checkbox("Go Frame by Frame", &agphys->m_frameByFrame);

	if (ImGui::Button("Pause"))
	{
		agphys->pause = !agphys->pause;
	}

	agphys->physicsGraph.renderImGui();

	ImGui::End();

	{

		if(options.resolutionChanged) {
			ImGui::SetNextWindowPos(ImVec2(0, options.windowHeight/4*3), ImGuiCond_Always);
			ImGui::SetNextWindowSize(ImVec2(400, options.windowHeight/4*3), ImGuiCond_Always);
		}
		ImGui::Begin("Video Encoding");
		agphys->enc.renderGUI();

		ImGui::End();
	}

	renderSpawningGui();

	agphys->particleSystem->renderGUI();
}

void Scene::renderSpawningGui() {
	ImGuiOptions& options = ImGuiOptions::get();
	auto agphys = Agphys::getInstance();

	if(options.resolutionChanged) {
		ImGui::SetNextWindowPos(ImVec2(options.windowWidth - 400, options.windowHeight/2), ImGuiCond_Always);
		ImGui::SetNextWindowSize(ImVec2(400, options.windowHeight/2), ImGuiCond_Always);
	}

	ImGui::Begin("Spawning Menu");
	if(ImGui::CollapsingHeader("Spawn Particles")) {
		ImGui::SliderInt("Num Particles", &options.numParticlesToBeSpawned, 1, 1000);
		if(ImGui::Button("Spawn Particles In Center")) {
			agphys->spawnParticles(options.numParticlesToBeSpawned);
		}
	}

	if(ImGui::CollapsingHeader("Spawn Rigid Bodies")) {
		ImGui::SliderInt("#Rigid Cubes", &options.numRigidBodies, 1, 30);
		ImGui::Checkbox("Spawn Cubes", &options.spawnCubes);
		if(options.spawnCubes) {
			ImGui::SliderInt("Rigid Cube Sidelength X", &options.xParticles, 1, 30);
			ImGui::SliderInt("Rigid Cube Sidelength Y", &options.yParticles, 1, 30);
			ImGui::SliderInt("Rigid Cube Sidelength Z", &options.zParticles, 1, 30);
			if(ImGui::Button("Spawn Rigid Cubes")) {
				agphys->spawnRigidCubes(options.numRigidBodies);
			}
			if(ImGui::Button("Spawn two interlocked Cubes")) {
				agphys->spawnInterlockedCubes();
			}
		} else {
			if(ImGui::Button("Browse")) {
				std::string filePath = Platform::Platform::get().openFileDialog("*.obj");
				if(filePath.size() > 3 && filePath.size() < ImGuiOptions::MAX_FILEPATH) {
					sprintf(options.meshFilePath, "%s", filePath.c_str());
				}
			}
			ImGui::SameLine();
			ImGui::InputText("input filePath", options.meshFilePath, IM_ARRAYSIZE(options.meshFilePath));
			ImGui::InputFloat3("Mesh Scale", options.meshScale.data());
			ImGui::SliderFloat("Mesh Resolution",&options.meshResolution, 0.01f, 3.f, "%.2f");
			if(ImGui::Button("Spawn Rigid Mesh")) {
				agphys->spawnRigidMeshes(options.numRigidBodies);
			}
		}
		ImGui::Separator();
	}

#if 0
	if(ImGui::CollapsingHeader("Spawn Cloths")) {
		ImGui::SliderInt("Cloth N", &options.clothN, 1, 30);
		ImGui::SliderInt("Cloth M", &options.clothM, 1, 30);
		ImGui::InputFloat3("Cloth Center", options.clothCenter.data());
		if(ImGui::InputFloat3("Cloth Normal", options.clothNormal.data())) {
			options.clothNormal.normalize();
		}
		ImGui::SliderFloat("Cloth Angle", &options.clothAngle, 0.f, M_PI * 2.f, "%.2f");
		ImGui::Checkbox("Fixed Cloth Edges", &options.fixedClothEdges);
		if(ImGui::Button("Spawn Cloth")) {
			agphys->spawnCloth(options.clothN, options.clothM);
		}
		ImGui::Separator();
	}
#endif
	if(ImGui::CollapsingHeader("Spawn Fluids")) {
		ImGui::SliderInt("Num Fluid Particles", &options.numFLuidParticles, 1, 10000);
		ImGui::SliderInt("Num Fluid Particles X", &options.numFluidParticlesX, 1, 100);
		ImGui::SliderInt("Num Fluid Particles Z", &options.numFluidParticlesZ, 1, 100);
		ImGui::InputFloat3("Corner Fluid", options.cornerFluid.data());
		if(ImGui::Button("Spawn Fluid")) {
			agphys->spawnFluid(options.numFLuidParticles);
		}
	}

	ImGui::Separator();
	if(ImGui::CollapsingHeader("Profiling Options")) {
		static int numFrames = 10;
		ImGui::SliderInt("Frames to Profile", &numFrames, 1, 300);
		Profiling::Profiler& profiler = Profiling::Profiler::getInstance();
		if(!profiler.sessionIsRunning()) {
			if(ImGui::Button("Start Profile Session")) {
				profiler.profileNextNFrames(numFrames);
			}
		} else {
			ImGui::Button("Profile Session is running...");
		}
	}

	agphys->m_ownplayer->imgui();

	ImGui::End();
}
