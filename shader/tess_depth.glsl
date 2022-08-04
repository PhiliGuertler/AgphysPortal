##GL_VERTEX_SHADER

#version 430
layout(location = 0) in vec4 in_position;
layout(location = 1) in vec4 in_normal;

#include "camera.glsl"
uniform mat4 model;

out vec3 v_normal;

void main()
{
	v_normal = in_normal.xyz;
	gl_Position = in_position;
}



##GL_TESS_CONTROL_SHADER
#version 400
layout(vertices = 3) out;

in vec3 v_normal[];

out vec3 tcNormal[];
out vec3 tcPhongPatch[];

uniform float u_tessLevelInner;
uniform float u_tessLevelOuter;
uniform float u_gridSize;

vec3 carsten(vec4 maschmeyer) {
	return maschmeyer.xyz / maschmeyer.w;
}

float PIi(int i, vec3 q) {
	vec3 qMinusP = q - gl_in[i].gl_Position.xyz;
	return q[gl_InvocationID] - dot(qMinusP, v_normal[i]) * v_normal[i][gl_InvocationID];
}

#define SQRT3 1.73205080757

void main()
{
    if (gl_InvocationID == 0) {
        gl_TessLevelInner[0] = u_tessLevelInner;
        gl_TessLevelOuter[0] = u_tessLevelOuter;
        gl_TessLevelOuter[1] = u_tessLevelOuter;
        gl_TessLevelOuter[2] = u_tessLevelOuter;
    }

	float maxLength = length(carsten(gl_in[0].gl_Position) - carsten(gl_in[1].gl_Position));
	maxLength = max(maxLength, length(carsten(gl_in[1].gl_Position) - carsten(gl_in[2].gl_Position)));
	maxLength = max(maxLength, length(carsten(gl_in[2].gl_Position) - carsten(gl_in[0].gl_Position)));

	// 3 is very magic indeed
	if(maxLength > u_gridSize * 1.01 * 3.0) {
		gl_TessLevelInner[0] = 0;
		gl_TessLevelOuter[0] = 0;
		gl_TessLevelOuter[1] = 0;
		gl_TessLevelOuter[2] = 0;
	}

	gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
	tcNormal[gl_InvocationID] = v_normal[gl_InvocationID];

	// compute patch data for phong tessellation
	tcPhongPatch[gl_InvocationID].x = PIi(0,gl_in[1].gl_Position.xyz) + PIi(1, gl_in[0].gl_Position.xyz);
	tcPhongPatch[gl_InvocationID].y = PIi(1,gl_in[2].gl_Position.xyz) + PIi(2, gl_in[1].gl_Position.xyz);
	tcPhongPatch[gl_InvocationID].z = PIi(2,gl_in[0].gl_Position.xyz) + PIi(0, gl_in[2].gl_Position.xyz);
}


##GL_TESS_EVALUATION_SHADER
#version 400
layout(triangles, equal_spacing, ccw) in;

in vec3 tcNormal[];
in vec3 tcPhongPatch[];

#include "camera.glsl"
uniform mat4 model;

uniform float u_tessAlpha;

#define interpolateVal(a) a[0] * gl_TessCoord.x + a[1] * gl_TessCoord.y + a[2] * gl_TessCoord.z

void main()
{
	// interpolate position, normal, color and data
	vec3 interNormal = interpolateVal(tcNormal);

	vec3 interPosition = gl_in[0].gl_Position.xyz * gl_TessCoord.x;
	interPosition += gl_in[1].gl_Position.xyz * gl_TessCoord.y;
	interPosition += gl_in[2].gl_Position.xyz * gl_TessCoord.z;

	vec3 ijTerm = vec3(tcPhongPatch[0].x, tcPhongPatch[1].x, tcPhongPatch[2].x);
	vec3 jkTerm = vec3(tcPhongPatch[0].y, tcPhongPatch[1].y, tcPhongPatch[2].y);
	vec3 ikTerm = vec3(tcPhongPatch[0].z, tcPhongPatch[1].z, tcPhongPatch[2].z);

	vec3 phongPos = gl_TessCoord.x * gl_TessCoord.x * gl_in[0].gl_Position.xyz;
	phongPos += gl_TessCoord.y * gl_TessCoord.y * gl_in[1].gl_Position.xyz;
	phongPos += gl_TessCoord.z * gl_TessCoord.z * gl_in[2].gl_Position.xyz;
	phongPos += gl_TessCoord.x * gl_TessCoord.y * ijTerm;
	phongPos += gl_TessCoord.y * gl_TessCoord.z * jkTerm;
	phongPos += gl_TessCoord.z * gl_TessCoord.x * ikTerm;

	vec3 finalPos = (1.0 - u_tessAlpha)*interPosition + u_tessAlpha*phongPos;
	gl_Position = viewProj * model * vec4(finalPos, 1);
}


##GL_GEOMETRY_SHADER
#version 400

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

void main()
{
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();

    gl_Position = gl_in[1].gl_Position;
    EmitVertex();

    gl_Position = gl_in[2].gl_Position;
    EmitVertex();

    EndPrimitive();
}


##GL_FRAGMENT_SHADER
#version 400

void main() {
}
