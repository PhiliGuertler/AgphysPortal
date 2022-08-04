#pragma once

#include "saiga/core/math/math.h"
#include "saiga/core/util/mouse.h"
#include "saiga/core/camera/camera.h" // Saiga::PerspectiveCamera
#include "saiga/core/camera/controllable_camera.h" // Saiga::CameraController

#include <SDL2/SDL.h> // SDL_ShowCursor, etc
#include "saiga/opengl/window/sdl_window.h" // window.getWidth(), etc

#include "imguiOptions.h"

using namespace Saiga;

// ######################################################################### //
// ### PlayerCamera (See below for a perspective implementation ############ //
// ######################################################################### //

template <typename camera_t>
class PlayerCamera : public CameraController, public camera_t
{
   public:
    // Velocity in units/seconds

    PlayerCamera() {}
    virtual ~PlayerCamera() {}

    void update(float delta, const Saiga::SDLWindow& window);
    void interpolate(float dt, float interpolation);

    void enableInput() { input = true; }
    void disableInput() { input = false; }
    void setInput(bool v) { input = v; }

    void mouseRotate(float dx, float dy);
    void mouseRotateAroundPoint(float dx, float dy);
    void mouseRotateAroundPoint(float dx, float dy, vec3 point);
    void mouseRotateAroundPoint(float dx, float dy, vec3 point, vec3 up);

	// to be called before everything else concerning imgui
	void imguiStartup();
    void imgui();

	void captureMouse(bool capture);
	void toggleCaptureMouse();

    ivec2 lastMousePos;
    int dragState = 0;  // 0 = nothing, 1 = first button drag, 2 = second button drag
	bool m_captureMouse = false;
    std::array<bool, 6> keyPressed{};

	bool m_keyboardIsEnabled = true;
    bool input = true;
    enum Key
    {
        Forward  = 0,
        Backward = 1,
        Left     = 2,
        Right    = 3,
        Fast     = 4,
        Up       = 5,
    };

private:
	void hideMouse(bool hideIt, const Saiga::SDLWindow& window);
};


// ######################################################################### //
// ### PlayerCamera Implementation ######################################### //
// ######################################################################### //

template <class camera_t>
void PlayerCamera<camera_t>::mouseRotate(float dx, float dy)
{
	// TODO: stop the user from turning the camera to high or too low

    if (mouseTurnLocal) {
        this->turnLocal(dx * rotationSpeed, dy * rotationSpeed);
	}
    else {
        this->turn(dx * rotationSpeed, dy * rotationSpeed);
	}
    this->calculateModel();
    this->updateFromModel();
}


template <class camera_t>
void PlayerCamera<camera_t>::mouseRotateAroundPoint(float dx, float dy)
{
    vec3 point;
    if (rotationPoint[0] == std::numeric_limits<float>::infinity())
    {
        vec3 dir = make_vec3(this->getDirection());
        point    = this->getPosition() - 10.0f * dir;
    }
    else
    {
        point = rotationPoint;
    }

    mouseRotateAroundPoint(-dx, -dy, point);
}


template <class camera_t>
void PlayerCamera<camera_t>::mouseRotateAroundPoint(float dx, float dy, vec3 point)
{
    vec2 relMovement(dx, dy);
    float angle = length(relMovement);
    if (angle == 0) return;

    vec4 right = this->getRightVector();
    vec4 up    = this->getUpVector();

    vec3 axis = -normalize(make_vec3(right * relMovement[1] + up * relMovement[0]));

    quat qrot = angleAxis(radians(angle * 0.3f), axis);
    this->rot = qrot * this->rot;
    vec3 p    = qrot * (make_vec3(this->position) - point);

    p += point;
    this->position = make_vec4(p, 1);
    this->calculateModel();
    this->updateFromModel();
}



template <class camera_t>
void PlayerCamera<camera_t>::mouseRotateAroundPoint(float dx, float dy, vec3 point, vec3 up)
{
    vec2 relMovement(dx, dy);
    float angle = length(relMovement);
    if (angle == 0) return;

    vec3 dir = normalize(vec3(point - this->getPosition()));

    vec3 right = normalize(cross(dir, up));
    vec3 axis = -normalize(vec3(right * relMovement[1] + up * relMovement[0]));

    quat qrot      = angleAxis(radians(angle), axis);
    this->rot      = qrot * this->rot;
    this->position = make_vec4(qrot * (this->getPosition() - point), 1);


    this->position = make_vec4(point + this->getPosition(), 1);

    this->calculateModel();
    this->updateFromModel();
}


template <class camera_t>
void PlayerCamera<camera_t>::update(float delta, const Saiga::SDLWindow& window)
{
    hideMouse(m_captureMouse, window);
	//only in firstperson mode
    if (input && m_keyboardIsEnabled)
    {
        int FORWARD =
            keyboard.getMappedKeyState(Forward, keyboardmap) - keyboard.getMappedKeyState(Backward, keyboardmap);
        int RIGHT = keyboard.getMappedKeyState(Right, keyboardmap) - keyboard.getMappedKeyState(Left, keyboardmap);

        float speed;
        if (keyboard.getMappedKeyState(Fast, keyboardmap))
        {
            speed = movementSpeedFast;
        }
        else
        {
            speed = movementSpeed;
        }

        vec3 trans  = delta * speed * FORWARD * vec3(0, 0, -1) + delta * speed * RIGHT * vec3(1, 0, 0);
        vec3 transg = vec3(0, 1, 0) * (delta * speed * keyboard.getMappedKeyState(Up, keyboardmap));
        this->translateLocal(trans);
        this->translateGlobal(transg);
    }
    this->calculateModel();
    this->updateFromModel();
}


template <class camera_t>
void PlayerCamera<camera_t>::interpolate(float dt, float interpolation)
{
    // the camera isn't actually "interpolated"
    // we just use the latest mouse position
    (void)dt;
    (void)interpolation;

    if (!input) return;

    int newDragState = mouse.getMappedKeyState(0, mousemap) ? 1 : mouse.getMappedKeyState(1, mousemap) ? 2 : 0;

    // only do mouse handling here
    ivec2 mousedelta = lastMousePos - mouse.getPosition();
    lastMousePos     = mouse.getPosition();

	vec4 viewDirection = this->getDirection();
	const float MAX_CHARM = 1.f - 0.01f;
	if(viewDirection.y() < -MAX_CHARM && mousedelta.y() > 0) {
		mousedelta.y() = 0;
	} else if(viewDirection.y() > MAX_CHARM && mousedelta.y() < 0) {
		mousedelta.y() = 0;
	}

    if (dragState == 1)
    {
        this->mouseRotate(mousedelta[0], mousedelta[1]);
    }
    else if (dragState == 2)
    {
        this->mouseRotateAroundPoint(mousedelta[0], mousedelta[1]);
    }


    dragState = newDragState;
	if(m_captureMouse) {
		dragState = 1;
	}
}

template <class camera_t>
void PlayerCamera<camera_t>::toggleCaptureMouse() {
	m_captureMouse = !m_captureMouse;
}

template <class camera_t>
void PlayerCamera<camera_t>::captureMouse(bool capture) {
	m_captureMouse = capture;
}

template <class camera_t>
void PlayerCamera<camera_t>::hideMouse(bool hideIt, const Saiga::SDLWindow& window) {
	if(hideIt) {
		lastMousePos = ivec2(window.getWidth()/2, window.getHeight()/2);
		// teleport the mouse to the center of the window
		SDL_WarpMouseInWindow(window.window, lastMousePos.x(), lastMousePos.y());
		// don't allow the mouse to leave the window
		SDL_SetWindowGrab(window.window, SDL_TRUE);
		// hide the mouse cursor
		SDL_ShowCursor(0);
	} else {
		// allow the mouse to leave the window
		SDL_SetWindowGrab(window.window, SDL_FALSE);
		// show the mouse cursor
		SDL_ShowCursor(1);
	}
}

template <class camera_t>
void PlayerCamera<camera_t>::imguiStartup() {
	ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange;
	ImGui::SetMouseCursor(ImGuiMouseCursor_None);

	if(m_captureMouse) {
		SDL_ShowCursor(SDL_FALSE);
	} else {
		SDL_ShowCursor(SDL_TRUE);
	}
}

template <class camera_t>
void PlayerCamera<camera_t>::imgui() {
	ImGuiOptions& options = ImGuiOptions::get();
	ImGui::Checkbox("Enable Keyboard Input", &m_keyboardIsEnabled);
	ImGui::Checkbox("Capture Mouse [F6]", &m_captureMouse);
	ImGui::SliderFloat("Near Plane Tweak", &options.nearTweak, -100.f, 100.f, "%.2f");
	CameraController::imgui();
	camera_t::imgui();
}


// ######################################################################### //
// ### PerspectivePlayerCamera ############################################# //
// ######################################################################### //

class PerspectivePlayerCamera : public PlayerCamera<Saiga::PerspectiveCamera> {
public:
	PerspectivePlayerCamera(float aspect) {
		// default values for [debug] movement if keyboard is enabled
		this->keyboardmap = { SDL_SCANCODE_W, SDL_SCANCODE_S, SDL_SCANCODE_A, SDL_SCANCODE_D, SDL_SCANCODE_LSHIFT, SDL_SCANCODE_SPACE};
        this->mousemap = {SDL_BUTTON_LEFT, SDL_BUTTON_MIDDLE};

		// we wanna do this flexible for debug cam
		m_keyboardIsEnabled = true;

		// default values for view and projection
		movementSpeed     = 20;
		movementSpeedFast = 50;
		setProj(60.0f, aspect, 0.1f, 500.0f);
		setView(vec3(0, 20, 50), vec3(0, 1, 0), vec3(0, 1, 0));
	}
};
