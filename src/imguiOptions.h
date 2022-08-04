#pragma once

#include "AgphysCudaConfig.h"

enum MouseClickGame: int {
	Colorize = 0,
	Grow = 1,
	Shoot = 2,
	SpawnParticles = 3,
	SpawnRigidBodies = 4,
	SpawnFluids = 5//,
	//SpawnCloths = 6
};

enum PhysicsImplementation: int {
	ForceBasedBruteForce = 0,
	ForceBasedCollisionResolver = 1,
	WithRigidBodies = 2
};
enum CameraPosition: int {
	DebugCamera = 0,
	ThirdPerson = 1,
	FirstPerson = 2
};
/**
 *	A collection of variables that can be altered by the UI
 *	Also this will be a singleton
 */
struct ImGuiOptions {
	// getter of the singleton instance
	static ImGuiOptions& get() {
		static ImGuiOptions s_instance;
		return s_instance;
	}

	// ##################################################################### //
	// ### Current Implementation ########################################## //
	PhysicsImplementation physicsImplementation = PhysicsImplementation::WithRigidBodies;		
	CameraPosition camPosition = CameraPosition::DebugCamera;
	// ##################################################################### //
	// ### General Options ################################################# //
	bool enablePrints = false;
	vec4 defaultParticleColor = vec4(1.f,0.6f,0.1f,1.f);

	// ##################################################################### //
	// ### Spawn Options ################################################### //
	const int X_RESET_DEFAULT = 50;
	int xReset = X_RESET_DEFAULT;

	const int Z_RESET_DEFAULT = 50;
	int zReset = Z_RESET_DEFAULT;

	const vec3 CORNER_RESET_DEFAULT = {-15.f, 4.f, -15.f};
	vec3 cornerReset = CORNER_RESET_DEFAULT;

	const float DISTANCE_RESET_DEFAULT = 1.f;
	float distanceReset = DISTANCE_RESET_DEFAULT;

	const float MASSINV_DEFAULT = 1.f;
	float massinv = MASSINV_DEFAULT;

	const float RADIUS_DEFAULT = 0.5f;
	float radiusReset = RADIUS_DEFAULT;

	void resetSpawnOptions() {
		xReset = X_RESET_DEFAULT;
		zReset = Z_RESET_DEFAULT;
        cornerReset = vec3(-X_RESET_DEFAULT * 0.5f, DISTANCE_RESET_DEFAULT, -Z_RESET_DEFAULT * 0.5f);
        cornerReset *= DISTANCE_RESET_DEFAULT;
        distanceReset = 2.f * (RADIUS_DEFAULT + 0.1f);
		massinv = MASSINV_DEFAULT;
		radiusReset = RADIUS_DEFAULT;
	}

	// ##################################################################### //
	// ### Gravity ######################################################### //
	// gravityDirection will be saved as a color in [0,1]
	vec3 gravityDirection = {0.f, -1.f, 0.f};
	float gravityStrength = 9.81f;

	// returns gravity with direction and strength combined
	vec3 getGravity() {
		vec3 result = gravityDirection;
		result *= gravityStrength;
		return result;
	}

	void resetGravity() {
		gravityDirection = {0.f, -1.f, 0.f};
		gravityStrength = 9.81f;
	}

	// ##################################################################### //
	// ### Plane Options ################################################### //
	const int NUM_PLANES_DEFAULT = 0;
	int numPlanes = NUM_PLANES_DEFAULT;

	const float PLANE_DISTANCE_DEFAULT = 50.f;
	float planeDistance = PLANE_DISTANCE_DEFAULT;

	void resetPlaneOptions() {
		numPlanes = NUM_PLANES_DEFAULT;
		planeDistance = PLANE_DISTANCE_DEFAULT;
	}

	// ##################################################################### //
	// ### Force Based Dynamics Options #################################### //
	const float ELASTIC_COEFFICIENT_DEFAULT = 0.2f;
	float elasticCoefficient = ELASTIC_COEFFICIENT_DEFAULT;

	const float SPRING_COEFFICIENT_DEFAULT = 80.f;
	float springCoefficient = SPRING_COEFFICIENT_DEFAULT;

	const float FRICTION_COEFFICIENT_DEFAULT = 0.15f;
	float frictionCoefficient = FRICTION_COEFFICIENT_DEFAULT;

	void resetForceBasedDynamicsOptions() {
		elasticCoefficient = ELASTIC_COEFFICIENT_DEFAULT;
		springCoefficient = SPRING_COEFFICIENT_DEFAULT;
		frictionCoefficient = FRICTION_COEFFICIENT_DEFAULT;
	}

	// ##################################################################### //
	// ### Position Based Dynamics Options ################################# //
	bool resolvePlanes = true;
	bool resolveParticles = true;

	const float RELAXATION_DEFAULT = 0.2f;
	float relaxation = RELAXATION_DEFAULT;

	const int JACOBI_ITERATIONS_DEFAULT = 2;
	int jacobiIterations = JACOBI_ITERATIONS_DEFAULT;

	void resetPositionBasedDynamicsOptions() {
		resolvePlanes = true;
		resolveParticles = true;
		relaxation = RELAXATION_DEFAULT;
		jacobiIterations = JACOBI_ITERATIONS_DEFAULT;
	}

	// ##################################################################### //
	// ### Mouse Games ##################################################### //
	MouseClickGame mouseGame = MouseClickGame::SpawnRigidBodies;
	bool repeatMouse = false;

	// ### Colorize ######################################################## //
	vec4 highlightParticleColor = vec4(0.9f, 0.2f, 0.2f, 1.f);
	bool resetHighlights = false;

	// ### Shoot ########################################################### //
	const float SHOOT_STRENGTH_DEFAULT = 80.f;
	float shootStrength = SHOOT_STRENGTH_DEFAULT;

	void resetShootStrength() {
		shootStrength = SHOOT_STRENGTH_DEFAULT;
	}

	// ### Grow ############################################################ //
	const float GROWTH_FACTOR_DEFAULT = 1.1f;
	float growthFactor = GROWTH_FACTOR_DEFAULT;

	void resetGrowthFactor() {
		growthFactor = GROWTH_FACTOR_DEFAULT;
	}

	// ##################################################################### //
	// ### Linked Cell Options ############################################# //
    const int EXP_NUM_HASH_BUCKETS_DEFAULT = 23;
    int numHashBuckets = 1 << EXP_NUM_HASH_BUCKETS_DEFAULT;

	bool enableFixedCellLength = false;

	const float FIXED_CELL_LENGTH_DEFAULT = RADIUS_DEFAULT;
	float fixedCellLength = FIXED_CELL_LENGTH_DEFAULT;

	void resetLinkedCellOptions() {
        numHashBuckets = 1 << EXP_NUM_HASH_BUCKETS_DEFAULT;
		fixedCellLength = FIXED_CELL_LENGTH_DEFAULT;
		enableFixedCellLength = false;
	}

	bool useCoolHash = true;

	void setUpTestScene() {
		resetGravity();
		resetSpawnOptions();
		resetForceBasedDynamicsOptions();
		resetLinkedCellOptions();
		resetGrowthFactor();
		resetShootStrength();
		
		planeDistance = 100.f;
		numPlanes = 4;

		relaxation = 0.3;
		jacobiIterations = 2;

		xReset = 100;
		zReset = 100;
		radiusReset = RADIUS_DEFAULT;
		massinv = MASSINV_DEFAULT;
		distanceReset = 2.f * (RADIUS_DEFAULT + 0.1f);
        cornerReset = vec3(-xReset * 0.5f, DISTANCE_RESET_DEFAULT, -zReset * 0.5f);
		cornerReset *= distanceReset;
	}

	// ##################################################################### //
	// ### Rigid Body Options ############################################## //
	const int NUM_RIGID_BODIES_DEFAULT = 5;
	int numRigidBodies = NUM_RIGID_BODIES_DEFAULT;
	const int SIDE_LENGTH_RIGID_BODIES_DEFAULT = 4;
	int sideLengthRigidBodies = SIDE_LENGTH_RIGID_BODIES_DEFAULT;
	bool resolveWithSDF = true;

	void resetRigidBodyOptions() {
		numRigidBodies = NUM_RIGID_BODIES_DEFAULT;
		sideLengthRigidBodies = SIDE_LENGTH_RIGID_BODIES_DEFAULT;
	}	

	// ##################################################################### //
	// ### Rigid Body Mesh Options ######################################### //
	static const int MAX_FILEPATH = 256;
	char meshFilePath[MAX_FILEPATH] = "./objs/teapot.obj";
	float meshResolution = 1.f;
	vec3 meshScale = vec3(1.f,1.f,1.f);

	// --- Spawning regular particles --- //
	int numParticlesToBeSpawned = 1;

	// --- Spawning Rigid Cubes --- //
	bool spawnCubes = true;
	int xParticles = 3;
	int yParticles = 3;
	int zParticles = 3;

	// --- Spawning Cloths --- //
	int clothN = 12;
	int clothM = 12;


	bool renderMeshes = true;

	// ##################################################################### //
	// ### Cloth Options ################################################### //
	float clothSpacing = 1.f;
	bool resolveBendingConstraints = true;
	vec3 clothCenter = vec3(0,8,0);
	vec3 clothNormal = vec3(1,0,0);
	float clothAngle = M_PI * 0.5f;
	bool fixedClothEdges = true;
	float bendFactor = 0.3f;

	// ##################################################################### //
	// ### Stabilization ################################################### //
	bool stabilize = true;
	float staticFriction = 2.f;
	float kineticFriction = 2.f;
	bool enableFriction = true;
	int numStabilizationSteps = 2;

	// ##################################################################### //
	// ### Fluids ########################################################## //
	int numFLuidParticles = 700;
	int numFluidParticlesX = 10;
	int numFluidParticlesZ = 10;
	vec3 cornerFluid = vec3(0.f,0.f,0.f);

	float fluidH = RADIUS_DEFAULT * 4.f;
	float fluidRo0 = 1.f;
	float fluidEpsilon = 500.f;
	float fluidK = 0.1f;
	float fluidQ = 0.2f;
	int fluidN = 4;
	float fluidVorticityEpsilon = 3.f;
	float fluidC = 0.01f;
	bool enableVorticity = true;
	bool enableViscosity = true;

	// ##################################################################### //
	// ### Fluid Rendering ################################################# //
	float marchingCubesGridStep = 1.3f;
	float marchingCubesIsoDensity = 1.05f;
	float marchingCubesH = RADIUS_DEFAULT * 4.f;
	float marchingCubesSigma = 1.f;
	bool renderFluidMesh = true;
	int fluidWidth = 2;
	int fluidHeight = 2;

	float tessLevelInner = 4.f;
	float tessLevelOuter = 4.f;
	float tessAlpha = 0.2f;
	bool fluidWireframe = false;
	int gauss = 1;

	// Window
	int windowWidth = 1024;
	int windowHeight = 1024;
	bool resolutionChanged = false;
	void setWindowSize(int width, int height) {
		resolutionChanged = (width != windowWidth) || (height != windowHeight);
		windowWidth = width;
		windowHeight = height;
	}

	// ##################################################################### //
	// ### Fluid Rendering Uniforms ######################################## //
	float specularFactor = 0.25f;
	float diffuseFactor = 0.75f;
	int specularPower = 2;
	int fresnelPower = 2;
	vec3 specularColor = vec3(0.1f, 0.2f, 0.95f);
	vec3 diffuseColor = vec3(0.2f, 0.25f, 1.f);
	vec3 lightPosition = vec3(10,10,10);
	vec3 lightDirection = vec3(-1,0,0);
	float depthFalloff = 7.f;
	bool enableFoam = true;
	float shrink = 1.f;

	float nearTweak = 0.5f;

	bool ballern = false;

	// ##################################################################### //
	// ### Portal Options ################################################## //
	float edgeWidth = 5.f;

	//DEBUG
	float cameraRotationAngle = 1.f;
	int enabledMarchingCubesSteps = 11;

	private:
		// private constructor to ensure singleton
		ImGuiOptions() = default;
		// copy and move operator are deleted to prevent invalid usage (like ImGuiOptions o = ImGuiOptions::get();)
		ImGuiOptions& operator=(ImGuiOptions& o) = delete;
		ImGuiOptions& operator=(ImGuiOptions&& o) = delete;
};
