#include "saiga/core/imgui/imgui.h"
#include "saiga/opengl/shader/shaderLoader.h"

#include "agphys.h"

#include "platform/platform.h"

#include "profiling/Profiler.h"
#include "rendering/Scene.h"

#include "rendering/PortalRenderer.h"

void Agphys::pollInputs() {
	if(m_rightMouseButtonIsPressed) {
		Saiga::Ray r = window->createPixelRay(m_lastMousePosition);
		// intersect ray with all particles
		particleSystem->intersectRay(r);
	}

	// check movement keys' states
	const Uint8 *keyboardState = SDL_GetKeyboardState(NULL);
	if(keyboardState[SDL_SCANCODE_A]) {
		m_ownplayer->m_movementBitmask |= Controls::LEFT;
	}
	if(keyboardState[SDL_SCANCODE_S]) {
		m_ownplayer->m_movementBitmask |= Controls::BACK;
	}
	if(keyboardState[SDL_SCANCODE_D]) {
		m_ownplayer->m_movementBitmask |= Controls::RIGHT;
	}
	if(keyboardState[SDL_SCANCODE_W]) {
		m_ownplayer->m_movementBitmask |= Controls::FRONT;
	}
	if(keyboardState[SDL_SCANCODE_SPACE]) {
		m_ownplayer->m_movementBitmask |= Controls::JUMP;
	}
}

void Agphys::keyPressed(SDL_Keysym key)
{
	if (ImGui::captureKeyboard()) return;

	ImGuiOptions& options = ImGuiOptions::get();

	switch (key.scancode)
	{
		case SDL_SCANCODE_ESCAPE:
			m_ownplayer->toggleCaptureMouse();
			m_scene->m_shouldRenderGUI = !m_scene->m_shouldRenderGUI;
			break;
		case SDL_SCANCODE_F4:
			window->close();
			break;
		case SDL_SCANCODE_F12:
			window->screenshot("screenshot.png");
			break;
		case SDL_SCANCODE_F3:
			m_scene->m_shouldRenderGUI = !m_scene->m_shouldRenderGUI;
			break;
		case SDL_SCANCODE_PAUSE:
			pause = !pause;
			break;
		case SDL_SCANCODE_F6:
			m_ownplayer->toggleCaptureMouse();
			break;
		case SDL_SCANCODE_E:
			m_ownplayer->m_capturesCamera = !m_ownplayer->m_capturesCamera;
			break;
		case SDL_SCANCODE_F:
			options.ballern = !options.ballern;
			break;
		default:
			break;
	}
}

void Agphys::keyReleased(SDL_Keysym key) {}

void Agphys::mouseMoved(int x, int y) {
	m_lastMousePosition = vec2(x,y);
}

void Agphys::mousePressed(int key, int x, int y)
{
	
	if (ImGui::captureMouse()) return;

	ImGuiOptions& options = ImGuiOptions::get();

	m_lastMousePosition = vec2(x,y);

	//spawn portal calculation
	if(particleSystem->d_tryecke.size() > 0) {
		vec3 newtmp = m_ownplayer->m_camera->getDirection().head<3>();
		ParticlePositionRadius ray;
		ray.position = vec3(insertVec3(m_ownplayer->m_camera->position));
		ray.radius = 0.f;
		vec3 moveVec = newtmp*100000;
		RayTriangleCollision t = particleSystem->getTriangleFromIntersection(ray, moveVec);

		if (key == SDL_BUTTON_LEFT) {
			if(!options.ballern) {
				particleSystem->setOrangePortal(t.normal, t.hitPoint, *m_scene, newtmp);
				PortalRenderer::get().setOrangePortal(particleSystem->m_orangePortal);

				// TODO: shoot a bunch of particles towards the view direction
				PortalRenderer::get().shootEffectParticles(40, -moveVec.normalized(), ray.position, vec4(1.f, .6f, .2f, 1.f));
			}
		}
		if(key == SDL_BUTTON_RIGHT) {
			if(!options.ballern) {
				particleSystem->setBluePortal(t.normal, t.hitPoint, *m_scene, newtmp);
				PortalRenderer::get().setBluePortal(particleSystem->m_bluePortal);

				// TODO: shoot a bunch of particles towards the view direction
				PortalRenderer::get().shootEffectParticles(40, -moveVec.normalized(), ray.position, vec4(.2f, .3f, 1.f, 1.f));
			}
		}
	}

	if(options.ballern) {
		// Mouse Click Games
		if (key == SDL_BUTTON_RIGHT)
		{
			if(options.repeatMouse) {
				m_rightMouseButtonIsPressed = true;
			}

			// intersect ray with all particles
			Saiga::Ray r = window->createPixelRay(m_lastMousePosition);
			particleSystem->intersectRay(r);
		}
	}
}

void Agphys::mouseReleased(int key, int x, int y) {
	if(key == SDL_BUTTON_RIGHT) {
		m_rightMouseButtonIsPressed = false;
	}
}
