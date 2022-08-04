##GL_VERTEX_SHADER

#version 430
layout(location = 0) in vec4 in_position;
layout(location = 1) in vec4 in_normal;
layout(location = 2) in vec4 in_vorticityData;

#include "camera.glsl"
uniform mat4 model;

out vec3 v_normal;
out vec3 v_color;
out float v_data;

void main()
{
	v_color = in_normal.xyz * 0.5f + 0.5f;
	v_normal = in_normal.xyz;
	v_data = in_vorticityData.x;
	gl_Position = in_position;
}



##GL_TESS_CONTROL_SHADER
#version 400
layout(vertices = 3) out;

in vec3 v_normal[];
in vec3 v_color[];
in float v_data[];

out vec3 tcNormal[];
out vec3 tcColor[];
out float tcData[];
out vec3 tcPhongPatch[];

uniform float u_tessLevelInner;
uniform float u_tessLevelOuter;
uniform float u_gridSize;


float PIi(int i, vec3 q) {
	vec3 qMinusP = q - gl_in[i].gl_Position.xyz;
	return q[gl_InvocationID] - dot(qMinusP, v_normal[i]) * v_normal[i][gl_InvocationID];
}

vec3 carsten(vec4 maschmeyer) {
	return maschmeyer.xyz / maschmeyer.w;
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
	tcColor[gl_InvocationID] = v_color[gl_InvocationID];
	tcData[gl_InvocationID] = v_data[gl_InvocationID];

	// compute patch data for phong tessellation
	tcPhongPatch[gl_InvocationID].x = PIi(0,gl_in[1].gl_Position.xyz) + PIi(1, gl_in[0].gl_Position.xyz);
	tcPhongPatch[gl_InvocationID].y = PIi(1,gl_in[2].gl_Position.xyz) + PIi(2, gl_in[1].gl_Position.xyz);
	tcPhongPatch[gl_InvocationID].z = PIi(2,gl_in[0].gl_Position.xyz) + PIi(0, gl_in[2].gl_Position.xyz);
}


##GL_TESS_EVALUATION_SHADER
#version 400
layout(triangles, equal_spacing, ccw) in;

in vec3 tcNormal[];
in vec3 tcColor[];
in float tcData[];
in vec3 tcPhongPatch[];

out vec3 teNormal;
out vec3 teColor;
out float teData;

out vec3 tePosition;
out vec3 transformedPosition;

#include "camera.glsl"
uniform mat4 model;

uniform float u_tessLevelInner;
uniform float u_tessLevelOuter;
uniform float u_tessAlpha;
uniform float u_shrink;

#define interpolateVal(a) a[0] * gl_TessCoord.x + a[1] * gl_TessCoord.y + a[2] * gl_TessCoord.z

void main()
{
	// interpolate position, normal, color and data
	vec3 interNormal = interpolateVal(tcNormal);
	//teNormal = normalize(vec3(viewProj * model * vec4(interNormal, 0)));
	teNormal = normalize(interNormal);
	vec3 interColor = interpolateVal(tcColor);
	teColor = interColor;
	float interData = interpolateVal(tcData);
	teData = interData;

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
	
	finalPos -= u_shrink * teNormal;

	tePosition = finalPos;

	gl_Position = viewProj * model * vec4(finalPos, 1);
	transformedPosition = gl_Position.xyz/gl_Position.w;
}


##GL_GEOMETRY_SHADER
#version 400

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;
in vec3 teNormal[3];
in vec3 teColor[3];
in float teData[3];

in vec3 tePosition[3];
in vec3 transformedPosition[3];

out vec3 v_normal;
out vec3 v_color;
out float v_data;

out vec3 v_position;
out vec3 v_transformedPosition;


void main()
{
	for(int i = 0; i < 3; ++i) {
		gl_Position = gl_in[i].gl_Position;
		v_normal = teNormal[i];
		v_color = teColor[i];
		v_data = teData[i];
		v_position = tePosition[i];
		v_transformedPosition = transformedPosition[i];
		EmitVertex();
	}

    EndPrimitive();
}


##GL_FRAGMENT_SHADER

#version 400

float linearDepth(float depth, float farplane, float nearplane)
{
    return (2 * nearplane) / (farplane + nearplane - depth * (farplane - nearplane));
}

uniform sampler2D gbufferDepth;

in vec3 v_normal;
in vec3 v_color;
in float v_data;

in vec3 v_position;
in vec3 v_transformedPosition;

layout(location = 0) out vec3 o_position;
layout(location = 1) out vec3 o_normal;
layout(location = 2) out float o_vorticity;

void main() {
	o_position = v_transformedPosition * 0.5 + 0.5;
	o_normal = v_normal * 0.5 + 0.5;
	o_vorticity = v_data;

	ivec2 tci = ivec2(gl_FragCoord.xy);
	float readDepth = texelFetch(gbufferDepth, tci, 0).r;
	//readDepth = linearDepth(readDepth, 0.1f, 500.f);

	float depth = gl_FragCoord.z;
	//depth = linearDepth(depth, 0.1f, 500.f);

	if(readDepth < depth) {
		gl_FragDepth = clamp(readDepth-0.0001, 0, 1);
	} else {
		gl_FragDepth = depth;
	}
}
