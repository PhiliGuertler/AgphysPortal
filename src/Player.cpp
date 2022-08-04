#include "Player.h"

#include <bitset>

#include "cuda/particleSystem/particleSystem.h"
#include "cuda/rigidBodyManager.h"
#include "cuda/logic/portal.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

#define WELL(a) "[" << a.x() << "," << a.y() << "," << a.z() << "]"

namespace Controls {

	// ######################################################################### //
	// ### Player ############################################################## //
	// ######################################################################### //

	Player::Player(ParticleSystem& particleSystem, float aspect)
		  : m_particleSystem(particleSystem)
		  , m_offsetToBody(5)
		  , m_maxSpeed(30.f)
		  , m_movementForce(1.f)
		  , m_airTimeJumpFrames(10)
		  , m_airTime(0)
		  , m_hoverHeight(0.02f)
		  , m_camera()
		  , m_lastPositions()
		  , m_lastIndex(0)
		  , m_localUp()
		  , m_interpolationFactor(1.f)
		  , m_movementBitmask(0x0)
		  , m_capturesCamera(true)
		  , m_cameraWasTeleported(false)
	{
		m_camera = std::make_unique<PerspectivePlayerCamera>(aspect);

		auto& manager = RigidBodyManager::get();
		manager.createRigidCuboid(particleSystem, 1, 1, 1);
	 	
		auto currentOrientaion = copyOrientation();
		//set new player position
		for(int i = 0; i < m_lastPositions.size(); ++i) {
			m_lastPositions[i] = currentOrientaion.position;
		}

		m_localUp = c_reference;

		vec3 axis = vec3(0,1,0);
		// create the target rotation
		m_quatMe = quat(Eigen::AngleAxis<float>(0.f, axis));
		// create the staring rotation
		m_interpolationStart = m_quatMe;
		// reset the interpolation factor
		m_interpolationFactor = 1.f;


	}

	void Player::updateOrientation(float dt) {
		if(!m_capturesCamera) {
			return;
		}

		m_interpolationFactor += dt;
		m_interpolationFactor = min(1.f, m_interpolationFactor);
	}


	void Player::move() {
		if(!m_capturesCamera) {
			m_movementBitmask = 0x0;
			return;
		}

		// create a movement direction from the bitmask
		vec3 moveDirection = { 0,0,0 };
		if(m_movementBitmask & PlayerMovement::FRONT) {
			moveDirection += vec3(0.f,0.f,1.f);
		}
		if(m_movementBitmask & PlayerMovement::LEFT) {
			moveDirection += vec3(-1.f,0.f,0.f);
		}
		if(m_movementBitmask & PlayerMovement::BACK) {
			moveDirection += vec3(0.f,0.f,-1.f);
		}
		if(m_movementBitmask & PlayerMovement::RIGHT) {
			moveDirection += vec3(1.f,0.f,0.f);
		}

		// rotate movementDirection
		vec4 tmp = m_camera->viewProj * vec4(insertVec3(moveDirection),0);
		moveDirection = vec3(insertVec3(tmp));
		moveDirection.y() = 0.f;
		moveDirection.normalize();
		float airFactor = m_airTime > 0 ? 0.1f : 1.f;
		moveDirection *= airFactor;

		if(m_movementBitmask & PlayerMovement::JUMP) {
			if(m_airTime < m_airTimeJumpFrames) {
				// jumping is possible for a few frames after leaving the ground to adjust the jump height
				float jumpFactor = .3f + (1.f - ((float)m_airTime / (float)m_airTimeJumpFrames));
				moveDirection += vec3(0,1,0) * jumpFactor * jumpFactor;
				m_airTime++;
			}
		}
		m_movementBitmask = 0x0;
		if(moveDirection.norm() < EPSTEIN) return;

		// move particles accordingly
		movePlayerParticles(moveDirection, m_movementForce);
	}


	void Player::imgui() {
		if(ImGui::CollapsingHeader("Camera Options")) {
			m_camera->imgui();
		}
		if(ImGui::CollapsingHeader("Player Options")) {
			ImGui::SliderFloat("Offset to Body", &m_offsetToBody, 0.01f, 30.f, "%.2f");
			ImGui::SliderFloat("Max Speed", &m_maxSpeed, 1.f, 50.f, "%.2f");
			ImGui::SliderFloat("Movement Force", &m_movementForce, 0.1f, 30.f, "%.2f");
			ImGui::SliderInt("Airtime Frames", &m_airTimeJumpFrames, 1, 80);
			ImGui::SliderFloat("Hover Height", &m_hoverHeight, 0.f, 2.f);
			ImGui::Text("Movement Flags: " BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(m_movementBitmask));

			auto& bluePortal = *(m_particleSystem.m_bluePortal);
			auto& orangePortal = *(m_particleSystem.m_orangePortal);
			if(ImGui::Button("Teleport Camera to Blue View")) {
				transformView(orangePortal, bluePortal);
			}
			if(ImGui::Button("Teleport Camera to Orange View")) {
				transformView(bluePortal, orangePortal);
			}
			if(m_particleSystem.getNumParticles(ParticleType::RigidBodyParticle) > 0) {
				ImGui::Checkbox("Player captures Camera", &m_capturesCamera);
			} else {
				ImGui::Text("There is no Rigid Body for the Player!");
			}
		}
	}
			
	void Player::transformView(const Portal& portalIn, const Portal& portalOut) {

		// save the current state of view as the start rotation
		// update camera
		mat4 newView = Portal::transformView(m_camera->view, portalIn, portalOut);
		m_camera->setView(newView);
		//m_camera->proj = m_camera->proj;
		m_camera->recalculateMatrices();
		
		// set the start interpolation to the rotation of the camera as seen from the other portal
		mat3 actualRotation = m_camera->view.block(0,0,3,3);
		m_interpolationStart = quat(actualRotation);

		// reset the interpolation factor
		m_interpolationFactor = 0.f;

		// flush the last positions as the camera just teleported somewhere else
		for(int i = 0; i < m_lastPositions.size(); ++i) {
			m_lastPositions[i] = m_camera->position.head<3>();
		}
	}

	void Player::updatePlayerCamera(float dt) {
		if(!m_capturesCamera) return;

		updateOrientation(dt);

		auto currentOrientaion = copyOrientation();

		// add current position to the last positions
		m_lastPositions[m_lastIndex] = currentOrientaion.position;
		m_lastIndex = (m_lastIndex+1) % m_lastPositions.size();

		// compute average of all positions
		vec3 avgPosition = vec3(0.f,0.f,0.f);
		for(int i = 0; i < m_lastPositions.size(); ++i) {
			avgPosition += m_lastPositions[i];
		}
		avgPosition /= m_lastPositions.size();

		// interpolate the local up vector
		m_localUp = Saiga::slerp(m_interpolationStart, m_quatMe, m_interpolationFactor) * c_reference;

		vec3 newCameraPosition = m_localUp * m_offsetToBody;
		newCameraPosition += avgPosition;

#if 0
		// this is not working as intended :(
		if(!m_cameraWasTeleported) {
			internalTeleport(m_camera->position.head<3>(), newCameraPosition, true);
		}
#endif

		vec3 viewDir = -m_camera->getDirection().head<3>();
		vec3 up = m_localUp;
		vec3 position = newCameraPosition;

		mat4 newView = Saiga::lookAt(position, position+viewDir, up);
		m_camera->setView(newView);

		if(m_particleSystem.m_firstRigidBodyHasHitSomeWalls) {
			m_airTime = 0;
		}
	}
	
	void Player::updateCamera(float dt, const Saiga::SDLWindow& window) {
		vec3 cameraBefore = m_camera->position.head<3>();

		m_camera->update(dt, window); 

		vec3 cameraAfter = m_camera->position.head<3>();

		if(!m_capturesCamera) {
			internalTeleport(cameraBefore, cameraAfter, false);
		}
	}

	void Player::internalTeleport(const vec3& posBefore, const vec3& posAfter, bool writeTeleportFlag) {
		vec3 movementVector = posBefore - posAfter;

		// check if the camera is crossing a portal
		auto& bluePortal = *(m_particleSystem.m_bluePortal);
		auto& orangePortal = *(m_particleSystem.m_orangePortal);

		if(m_particleSystem.m_bluePortal->intersectsPortal(posBefore, movementVector)) {
			// teleport the camera to the orange portal
			std::cout << "Player: Blue" << std::endl;
			transformView(orangePortal, bluePortal);
			m_cameraWasTeleported = true;
		} else if(m_particleSystem.m_orangePortal->intersectsPortal(posBefore, movementVector)) {
			// teleport the camera to the blue portal
			std::cout << "Player: Orange" << std::endl;
			transformView(bluePortal, orangePortal);
			m_cameraWasTeleported = true;
		} else if(writeTeleportFlag) {
			m_cameraWasTeleported = false;
		}


	}
			
	void Player::interpolateCamera(float dt, float interpolation) {
		m_camera->interpolate(dt, interpolation);
	}


}
