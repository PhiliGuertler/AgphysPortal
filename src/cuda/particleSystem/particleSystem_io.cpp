#include "particleSystem.h"

#include "saiga/core/imgui/imgui.h"
#include "saiga/core/math/random.h"
#include "saiga/core/util/assert.h"

#include <thrust/extrema.h>
#include <thrust/sort.h>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "../../rendering/PortalRenderer.h"

void ParticleSystem::renderGravityOptions() {
	ImGuiOptions& options = ImGuiOptions::get();

	if (ImGui::CollapsingHeader("Gravity Options")) {
		ImGui::BeginGroup();
		bool modified = ImGui::VSliderFloat("gravity x", ImVec2(40, 160), &options.gravityDirection.data()[0], -1.f, 1.f, "%.2f");
		ImGui::SameLine();
		modified |= ImGui::VSliderFloat("gravity y", ImVec2(40, 160), &options.gravityDirection.data()[1], -1.f, 1.f, "%.2f");
		ImGui::SameLine();
		modified |= ImGui::VSliderFloat("gravity z", ImVec2(40, 160), &options.gravityDirection.data()[2], -1.f, 1.f, "%.2f");
		ImGui::EndGroup();
		modified |= ImGui::SliderFloat("gravity strength", &options.gravityStrength, 0.f, 100.f, "%.2f");

		if (modified) {
			options.gravityDirection.normalize();
		}

		if(ImGui::Button("Reset Gravity")) {
			options.resetGravity();
		}

		ImGui::Separator();
	}
}

void ParticleSystem::renderSpawnOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	
	if (ImGui::CollapsingHeader("Spawn Parameters")) {
		bool modified = ImGui::InputInt("x", &options.xReset);
		modified |= ImGui::InputInt("z", &options.zReset);
		modified |= ImGui::InputFloat3("Corner", options.cornerReset.data());
		modified |= ImGui::SliderFloat("distance", &options.distanceReset, 0.1f, 10.f, "%.2f");
		modified |= ImGui::SliderFloat("radius", &options.radiusReset, 0.01f, 5.f, "%.2f");
		//dirty 
		options.radiusReset = 0.48;
		if (modified) {
			resetParticles(options.xReset, options.zReset, options.cornerReset, options.distanceReset);
		}

		ImGui::SliderFloat("massinv", &options.massinv, 0.01f, 100.f, "%.2f");
		
		if(ImGui::Button("Reset Spawn Options")) {
			options.resetSpawnOptions();
		}
		
		ImGui::Separator();
	}
}

void ParticleSystem::renderForceBasedOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	if(ImGui::CollapsingHeader("Force Based Dynamics Options")) {
		ImGui::SliderFloat("elastic coeff", &options.elasticCoefficient, 0.f, 5.f, "%.2f");
		ImGui::SliderFloat("spring coeff", &options.springCoefficient, 0.f, 150.f, "%.2f");
		ImGui::SliderFloat("friction coeff", &options.frictionCoefficient, 0.f, 1.f, "%.2f");

		if(ImGui::Button("Reset Force Based Options")) {
			options.resetForceBasedDynamicsOptions();
		}			
	}
}

void ParticleSystem::renderPositionBasedOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	if(ImGui::CollapsingHeader("Position Based Dynamics Options")) {
		ImGui::Checkbox("Enable Plane Resolving", &options.resolvePlanes);
		ImGui::Checkbox("Enable Particle Resolving", &options.resolveParticles);
		ImGui::SliderFloat("Relaxation Parameter", &options.relaxation, 0.01f, 1.f, "%.2f");
		ImGui::SliderInt("Jacobi Iterations", &options.jacobiIterations, 1, 20);

		if(ImGui::Button("Reset Position Based Options")) {
			options.resetPositionBasedDynamicsOptions();
		}
	}
}

void ParticleSystem::renderLinkedCellOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	if(ImGui::CollapsingHeader("Linked Cell Options")) {
		static int numHashBuckets = options.EXP_NUM_HASH_BUCKETS_DEFAULT;
		if(ImGui::SliderInt("2^x HashBuckets", &numHashBuckets, 1, 26)) {
			options.numHashBuckets = 1 << numHashBuckets;
		}
		ImGui::Checkbox("Enable Fixed Cell Length", &options.enableFixedCellLength);
		if(options.enableFixedCellLength) {
			ImGui::SliderFloat("Fixed Cell Length", &options.fixedCellLength, 0.01f, 10.f, "%.2f");
		}
		ImGui::Checkbox("Use Cool hash function", &options.useCoolHash);
		if(ImGui::Button("Reset Linked Cell Options")) {
			options.resetLinkedCellOptions();
		}
	}
}

void ParticleSystem::renderRigidBodyOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	if(ImGui::CollapsingHeader("Rigid Body Options")) {
		ImGui::Checkbox("Use Resolving with SDF", &options.resolveWithSDF);
	}
}

void ParticleSystem::renderClothOptions() {
#if 0
	ImGuiOptions& options = ImGuiOptions::get();
	if(ImGui::CollapsingHeader("Cloth Options")) {
		ImGui::SliderFloat("Cloth Spacing", &options.clothSpacing, 0.0001f, 2.f, "%.4f");
		ImGui::Checkbox("Enable Bending Constraints", &options.resolveBendingConstraints);
		ImGui::SliderFloat("Bend Factor", &options.bendFactor, 0.0001f, 5.f, "%.4f");
		ImGui::Separator();
	}
#endif
}

void ParticleSystem::renderFluidOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	if(ImGui::CollapsingHeader("Fluid Options")) {
		ImGui::SliderFloat("Fluid h", &options.fluidH, 0.01f, 20.f, "%.2f");
		ImGui::SliderFloat("Fluid k", &options.fluidK, 0.01f, 20.f, "%.2f");
		ImGui::SliderFloat("Fluid q", &options.fluidQ, 0.01f, 20.f, "%.2f");
		ImGui::SliderFloat("Fluid epsilon", &options.fluidEpsilon, 1.f, 1000.f, "%.2f");
		ImGui::SliderFloat("Fluid vorticity epsilon", &options.fluidVorticityEpsilon, 1.f, 100.f, "%.2f");
		ImGui::SliderFloat("Fluid ro0", &options.fluidRo0, 0.01f, 20.f, "%.2f");
		ImGui::SliderFloat("Fluid C", &options.fluidC, 0.0001f, 1.f, "%.4f");
		ImGui::SliderInt("Fluid n", &options.fluidN, 1, 10, "%.2f");
		ImGui::Checkbox("Enable Vorticity", &options.enableVorticity);
		ImGui::Checkbox("Enable Viscosity", &options.enableViscosity);
		ImGui::Separator();
	}
}

void ParticleSystem::renderImplementationOptions() {
	ImGuiOptions& options = ImGuiOptions::get();

	if(ImGui::CollapsingHeader("Implementation Options")) {
#if 0
		const char *implementations[] = {
			"Force Based w/ Brute Force",
			"Force Based w/ Collision Resolving",
			"Position Based"
		};
		
		static int currentItem = options.physicsImplementation;
		options.physicsImplementation = (PhysicsImplementation)currentItem;
		if (ImGui::Combo("Implementation", &currentItem, implementations, IM_ARRAYSIZE(implementations))) {
			options.physicsImplementation = (PhysicsImplementation)currentItem;
		}
#endif
		ImGui::Checkbox("Enable Stabilizing", &options.stabilize);
		ImGui::SliderInt("Num Stabilization Steps", &options.numStabilizationSteps, 1, 5);
		ImGui::SliderFloat("Static Friction", &options.staticFriction, 0.001f, 10.f, "%.4f");
		ImGui::SliderFloat("Kinetic Friction", &options.kineticFriction, 0.001f, 10.f, "%.4f");
		ImGui::Checkbox("Enable Friction", &options.enableFriction);
		ImGui::Indent();

		switch(options.physicsImplementation) {
			case PhysicsImplementation::ForceBasedBruteForce:
			case PhysicsImplementation::ForceBasedCollisionResolver:
				renderForceBasedOptions();
				break;
			case PhysicsImplementation::WithRigidBodies:
				renderPositionBasedOptions();
				renderLinkedCellOptions();
				renderRigidBodyOptions();
				renderClothOptions();
				renderFluidOptions();
				break;
			default:
				break;
		}
		ImGui::Unindent();
		ImGui::Separator();
	}
}

void ParticleSystem::renderMouseGameOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	if (ImGui::CollapsingHeader("Mouse Click Games")) {
		const char *items[] = {
			"Colorize Particles",
			"Grow Particles",
			"Shoot Particles",
			"Shoot Particles from Camera",
			"Shoot Rigid Bodies from Camera",
			//"Shoot Cloth from Camera",
			"Shoot Fluid from Camera"};

		static int currentItem = options.mouseGame;
		ImGui::Checkbox("Paint Mode", &options.repeatMouse);
		ImGui::Checkbox("Don't shoot Portals", &options.ballern);
		options.mouseGame = (MouseClickGame)currentItem;
		if (ImGui::Combo("Mouse Game", &currentItem, items, IM_ARRAYSIZE(items))) {
			options.mouseGame = (MouseClickGame)currentItem;
		}
		if (options.mouseGame == MouseClickGame::Colorize) {
			ImGui::Checkbox("Reset other particle highlights", &options.resetHighlights);
			ImGui::ColorPicker4("Highlighted Particle Color", (float *)&options.highlightParticleColor, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoSmallPreview);
		}
		if (options.mouseGame == MouseClickGame::Grow) {
			ImGui::SliderFloat("Growth Factor", &options.growthFactor, 0.01f, 5.f, "%.2f");
			if(ImGui::Button("Reset Growth Factor")) {
				options.resetGrowthFactor();
			}

		}
		if (options.mouseGame == MouseClickGame::Shoot) {
			ImGui::SliderFloat("Shooting Strength", &options.shootStrength, 1.f, 2000.f, "%.2f", 2.f);
			if(ImGui::Button("Reset Shooting Strength")) {
				options.resetShootStrength();
			}
		}
		if(options.mouseGame == MouseClickGame::SpawnParticles) {
			ImGui::SliderFloat("Shooting Strength", &options.shootStrength, 1.f, 2000.f, "%.2f", 2.f);
		}
		if(options.mouseGame == MouseClickGame::SpawnRigidBodies) {
			ImGui::SliderFloat("Shooting Strength", &options.shootStrength, 1.f, 2000.f, "%.2f", 2.f);
		}
#if 0
		if(options.mouseGame == MouseClickGame::SpawnCloths) {
			ImGui::SliderFloat("Shooting Strength", &options.shootStrength, 1.f, 2000.f, "%.2f", 2.f);
		}
#endif
		if(options.mouseGame == MouseClickGame::SpawnFluids) {
			ImGui::SliderFloat("Shooting Strength", &options.shootStrength, 1.f, 2000.f, "%.2f", 2.f);
			ImGui::SliderInt("Fluid Tower Width", &options.fluidWidth, 1, 15);
			ImGui::SliderInt("Fluid Tower Height", &options.fluidHeight, 1, 15);
		}
	}
}

void ParticleSystem::renderFluidRenderingOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	if (ImGui::CollapsingHeader("Fluid Rendering Options")) {
		ImGui::SliderInt("Marching Cubes Steps", &options.enabledMarchingCubesSteps, 0, 11);
		ImGui::SliderFloat("Marching Cubes Grid Size", &options.marchingCubesGridStep, 0.4f, 4.f, "%.2f");
		ImGui::SliderFloat("Marching Cubes Iso Density Value", &options.marchingCubesIsoDensity, 0.f, 70.f, "%.2f");
		ImGui::SliderFloat("Marching Cubes H", &options.marchingCubesH, 0.001f, 10.f, "%.2f");
		ImGui::SliderFloat("Marching Cubes Sigma", &options.marchingCubesSigma, 0.001f, 10.f, "%.2f");
		ImGui::SliderFloat("TessLevelInner", &options.tessLevelInner, 1.f, 8.f, "%.2f");
		ImGui::SliderFloat("TessLevelOuter", &options.tessLevelOuter, 1.f, 8.f, "%.2f");
		ImGui::SliderFloat("TessAlpha", &options.tessAlpha, 0.f, 1.f, "%.2f");
		ImGui::Checkbox("Render Fluid Mesh", &options.renderFluidMesh);
		ImGui::Checkbox("Render Fluid Wireframe", &options.fluidWireframe);
		ImGui::SliderInt("Gauss", &options.gauss, 0, 10);
		ImGui::SliderFloat("Shrink", &options.shrink, 0.f, 3.f, "%.2f");
	}
}

void ParticleSystem::renderFluidRenderingUniformOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	if (ImGui::CollapsingHeader("Fluid Rendering Uniforms")) {
		ImGui::SliderFloat("Specular Factor", &options.specularFactor, 0, 1, "%.2f");
		ImGui::SliderFloat("Diffuse Factor", &options.diffuseFactor, 0, 1, "%.2f");
		ImGui::SliderInt("Specular Power", &options.specularPower, 0, 10);
		ImGui::SliderInt("Fresnel Power", &options.fresnelPower, 0, 10);
		ImGui::SliderFloat("Depth Falloff", &options.depthFalloff, 0.1f, 20.f, "%.2f");
		ImGui::ColorPicker3("Water Diffuse Color", options.diffuseColor.data());
		ImGui::ColorPicker3("Water Specular Color", options.specularColor.data());
		ImGui::Checkbox("Enable \"Foam\"", &options.enableFoam);
	}
}

void ParticleSystem::renderPortalOptions() {
	ImGuiOptions& options = ImGuiOptions::get();
	if(ImGui::CollapsingHeader("Portal Options")) {
		ImGui::SliderFloat("Edge Collision Width", &options.edgeWidth, 0.f, 15.f, "%.2f");
	}
}

void ParticleSystem::renderGUI() {
	ImGuiOptions& options = ImGuiOptions::get();

	if(options.resolutionChanged) {
		ImGui::SetNextWindowPos(ImVec2(options.windowWidth - 400, 0), ImGuiCond_Always);
		ImGui::SetNextWindowSize(ImVec2(400, options.windowHeight/2), ImGuiCond_Always);
	}

	ImGui::Begin("ParticleSystem");


	ImGui::Text("Normal Particles: %d", m_particleCounts.regularParticles);
	ImGui::Text("Rigid Body Particles: %d", m_particleCounts.rigidBodyParticles);
	ImGui::Text("Cloth Particles: %d", m_particleCounts.clothParticles);
	ImGui::Text("Fluid Particles: %d", m_particleCounts.fluidParticles);
	ImGui::Text("Rigid Bodies: %d", m_particleCounts.numRigidBodies);

	ImGui::SliderFloat("Flight Speed", &PortalRenderer::get().m_flightSpeed, 0.1f, 30.f, "%.2f");

	if (ImGui::CollapsingHeader("General Options")) {
		ImGui::Checkbox("Enable Prints", &options.enablePrints);
		ImGui::ColorPicker4("Default Particle Color", (float *)&options.defaultParticleColor, ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoSmallPreview);
		ImGui::Separator();
	}

	renderGravityOptions();

	renderSpawnOptions();

	renderImplementationOptions();

	renderMouseGameOptions();

	renderFluidRenderingOptions();
	
	renderFluidRenderingUniformOptions();

	renderPortalOptions();

	ImGui::End();
}
