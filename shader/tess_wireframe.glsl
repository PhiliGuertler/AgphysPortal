/**
 * Copyright (c) 2017 Darius Rückert 
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


##GL_VERTEX_SHADER

#version 400
layout(location=0) in vec4 in_position;
layout(location=1) in vec4 in_normal;
layout(location=2) in vec4 in_color;

#include "camera.glsl"
uniform mat4 model;

out vec3 v_normal;
out vec3 v_color;

void main()
{
	v_color = in_color.rgb;
	v_normal = normalize(vec3(view * model * vec4(in_normal.xyz, 0)));
	gl_Position = viewProj * model * vec4(in_position.xyz, 1);
}



##GL_TESS_CONTROL_SHADER
#version 400
layout(vertices = 3) out;

in vec3 v_normal[];
in vec3 v_color[];

out vec3 tcNormal[];
out vec3 tcColor[];

uniform float TessLevelInner;
uniform float TessLevelOuter;


void main()
{
    if (gl_InvocationID == 0) {
        gl_TessLevelInner[0] = TessLevelInner;
        gl_TessLevelOuter[0] = TessLevelOuter;
        gl_TessLevelOuter[1] = TessLevelOuter;
        gl_TessLevelOuter[2] = TessLevelOuter;
    }
	gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
	tcNormal[gl_InvocationID] = v_normal[gl_InvocationID];
	tcColor[gl_InvocationID] = v_color[gl_InvocationID];
}


##GL_TESS_EVALUATION_SHADER
#version 400
layout(triangles, equal_spacing, ccw) in;

in vec3 tcNormal[];
in vec3 tcColor[];

out vec3 teNormal;
out vec3 teColor;

#include "camera.glsl"
uniform mat4 model;

void main()
{
	// bilinear interpolation using barycentric coords
	gl_Position = (gl_TessCoord.x * gl_in[0].gl_Position);
	gl_Position += (gl_TessCoord.y * gl_in[1].gl_Position);
	gl_Position += (gl_TessCoord.z * gl_in[2].gl_Position);

	teNormal = (gl_TessCoord.x * tcNormal[0]);
	teNormal += (gl_TessCoord.y * tcNormal[1]);
	teNormal += (gl_TessCoord.z * tcNormal[2]);

	teColor = (gl_TessCoord.x * tcColor[0]);
	teColor += (gl_TessCoord.y * tcColor[1]);
	teColor += (gl_TessCoord.z * tcColor[2]);
}


##GL_GEOMETRY_SHADER
#version 400

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;
in vec3 teNormal[3];
in vec3 teColor[3];

out vec3 v_normal;
out vec3 v_color;

void main()
{
    gl_Position = gl_in[0].gl_Position;
	v_normal = teNormal[0];
	v_color = teColor[0];
    EmitVertex();

    gl_Position = gl_in[1].gl_Position;
	v_normal = teNormal[1];
	v_color = teColor[1];
    EmitVertex();

    gl_Position = gl_in[2].gl_Position;
	v_normal = teNormal[2];
	v_color = teColor[2];
    EmitVertex();

    EndPrimitive();
}


##GL_FRAGMENT_SHADER

#version 400

in vec3 v_normal;
in vec3 v_color;

#include "geometry_helper_fs.glsl"

void main() {
    setGbufferData(v_color,v_normal,vec4(0));
}


