#pragma once

#include "PlayerCamera.h"

#include "AgphysCudaConfig.h"

#include <memory>
#include <array>

#include "cuda/particle.h"

#define insertVec3(a) a.x(), a.y(), a.z()

class ParticleSystem;
class Portal;

namespace Controls {
	enum PlayerMovement : int {
		FRONT = 0x1 << 0,
		LEFT = 0x1 << 1,
		BACK = 0x1 << 2,
		RIGHT = 0x1 << 3,
		JUMP = 0x1 << 4
	};

	class Player {
		public:
			Player(ParticleSystem& ParticleSystem, float aspect);
			virtual ~Player() = default;

			// to be called before the physics step
			void move();

			//reset the player Position
			void resetPlayerPosition();
			// to be called after the physics step
			void updatePlayerCamera(float dt);

			void imgui();

			inline void setMaxSpeed(float maxSpeed) { m_maxSpeed = maxSpeed; }

			// Camera functionalities
			Camera *getCamera() { return m_camera.get(); }
			inline void toggleCaptureMouse() { m_camera->toggleCaptureMouse(); }
			void updateCamera(float dt, const Saiga::SDLWindow& window);
			void interpolateCamera(float dt, float interpolation);
			void transformView(const Portal& portalIn, const Portal& portalOut);

		protected:
			void movePlayerParticles(vec3 direction, float force);

			RigidBodyOrientation copyOrientation();

			void updateOrientation(float dt);

			void internalTeleport(const vec3& posBefore, const vec3& posAfter, bool writeTeleportFlag);

		public:
			ParticleSystem& m_particleSystem;

			float m_offsetToBody;

			float m_maxSpeed;
			float m_movementForce;
			int m_airTimeJumpFrames;
			int m_airTime;
			float m_hoverHeight;

			std::unique_ptr<PerspectivePlayerCamera> m_camera;

			std::array<vec3, 1> m_lastPositions;
			int m_lastIndex;

			vec3 m_localUp;
			quat m_interpolationStart;
			quat m_quatMe;
			float m_interpolationFactor;
			const vec3 c_reference = vec3(0,1,0);

			int m_movementBitmask;

			bool m_capturesCamera;

			bool m_cameraWasTeleported;

		private:
			const int BLOCK_SIZE = 128;
			const float EPSTEIN = 0.001f;
	};
}
