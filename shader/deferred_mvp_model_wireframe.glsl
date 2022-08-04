/**
 * Copyright (c) 2017 Darius Rückert 
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


##GL_VERTEX_SHADER

#version 330
layout(location=0) in vec3 in_position;
layout(location=1) in vec3 in_normal;
layout(location=2) in vec3 in_color;
layout(location=3) in vec3 in_data;

#include "camera.glsl"
uniform mat4 model;

out vec3 normal;
out vec3 color;
out vec3 data;

void main() {
    color = in_color;
    data = in_data;
    normal = normalize(vec3(view*model * vec4( in_normal, 0 )));
    gl_Position = viewProj *model* vec4(in_position,1);
}





##GL_FRAGMENT_SHADER

#version 330

in vec3 normal;
in vec3 color;
in vec3 data;

layout(location=0) out vec3 out_color;

void main() {
    out_color = vec3(0.5f);
}


