##GL_VERTEX_SHADER

#version 430
layout(location=0) in vec3 in_position;
layout(location=1) in vec3 in_normal;
layout(location=2) in vec2 in_tex;
layout(location=3) in vec4 in_data;


#include "camera.glsl"
uniform mat4 model;

out vec3 normal;
out vec4 data;

void main() {
    data = in_data;
    normal = normalize(vec3(view*model * vec4( in_normal, 0 )));
    gl_Position = viewProj *model* vec4(in_position,1);
}





##GL_FRAGMENT_SHADER

#version 430

uniform sampler2D image;
uniform float userData; //blue channel of data texture in gbuffer. Not used in lighting.
uniform vec2 u_screenSize;

in vec3 normal;
in vec4 data;


#include "geometry_helper_fs.glsl"


void main() {
	vec2 tc = gl_FragCoord.xy;
	tc /= u_screenSize.xy;

	vec4 testData = data;
	testData = vec4(1,1,1,1);

    vec4 diffColor = texture(image, tc);
	gl_FragDepth = gl_FragCoord.z-0.0001;
    setGbufferData(vec3(diffColor),normal,testData);
}



