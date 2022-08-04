##GL_VERTEX_SHADER
#version 430 core

layout(location = 0) in vec4 in_position_radius;

out vec2 v_coord;

void main() {
	v_coord = 0.5 * in_position_radius.xy + 0.5;
	gl_Position = vec4(in_position_radius.xy, 0.0, 1.0);
}


##GL_FRAGMENT_SHADER
#version 430 core

in vec2 v_coord;

uniform vec4 u_color;
uniform sampler2D u_depthMap;
uniform sampler2D u_thicknessMap;
uniform sampler2D u_sceneMap;
uniform mat4 u_proj;
uniform mat4 u_modelView;
uniform vec2 u_invTexScale;

uniform vec3 u_lightDirection;
uniform vec3 u_lightPosition;
uniform float u_shininess;
uniform float u_fresPower;
uniform float u_fresScale;
uniform float u_fresBias;

layout(location = 0) out vec4 o_fragColor;

vec3 uvToEye(vec2 p, float z) {
	vec2 pos = p * 2.0 - 1.0;
	vec4 screenSpacePos = vec4(pos, z, 1.0);
	vec4 viewPos = inverse(u_proj) * screenSpacePos;
	return viewPos.xyz / viewPos.w;
}

void main() {
	vec4 scene = texture(u_sceneMap, v_coord);
	float depth = texture(u_depthMap, v_coord).x;

	if(depth == 0.0) {
		o_fragColor = vec4(0);
		return;
	}

	if(depth == 1.0) {
		o_fragColor = scene;
		return;
	}

	// reconstruct eye space position from depth
	vec3 eyePos = uvToEye(v_coord, depth);

	// finite difference approx for normals.
	// can't take dFdx because the one-sided difference is incorrect at shape boundaries
	vec3 zl = eyePos - uvToEye(v_coord - vec2(u_invTexScale.x, 0.0), texture(u_depthMap, v_coord - vec2(u_invTexScale.x, 0.0)).x);
	vec3 zr = uvToEye(v_coord + vec2(u_invTexScale.x, 0.0), texture(u_depthMap, v_coord + vec2(u_invTexScale.x, 0.0)).x) - eyePos;
	vec3 zt = uvToEye(v_coord + vec2(0.0, u_invTexScale.y), texture(u_depthMap, v_coord + vec2(0.0, u_invTexScale.y)).x) - eyePos;
	vec3 zb = eyePos - uvToEye(v_coord - vec2(0.0, u_invTexScale.y), texture(u_depthMap, v_coord - vec2(0.0, u_invTexScale.y)).x);

	vec3 dx = zl;
	vec3 dy = zt;

	if(abs(zr.z) < abs(zl.z)) {
		dx = zr;
	}

	if(abs(zb.z) < abs(zt.z)) {
		dy = zb;
	}

	vec3 normal = normalize(cross(dx, dy));
    
	vec4 worldPos = inverse(u_modelView) * vec4(eyePos, 1.0);
    
    //Phong specular
	vec3 l = (u_modelView * vec4(u_lightDirection, 0.0)).xyz;
    vec3 viewDir = -normalize(eyePos);
    vec3 halfVec = normalize(viewDir + l);
    float specular = pow(max(0.0f, dot(normal, halfVec)), u_shininess);	

	const vec2 texScale = vec2(0.75, 1.0);
	float refractScale = 1.33 * 0.025;
	refractScale *= smoothstep(0.1, 0.4, worldPos.y);
	vec2 refractCoord = v_coord + normal.xy*refractScale*texScale;

	//float thickness = max(texture(u_thicknessMap, refractCoord).x, 0.3);
	float thickness = max(texture(u_thicknessMap, v_coord).x, 0.3);
	vec3 transmission = exp(-(vec3(1.0)-u_color.xyz)*thickness);
	//vec3 transmission = (1.0-(1.0-u_color.xyz)*thickness*0.8)*u_color.w;

	vec3 refract = texture(u_sceneMap, refractCoord).xyz*transmission;
    
	vec3 lVec = normalize(worldPos.xyz-u_lightPosition);
	float attenuation = max(smoothstep(0.95, 1.0, abs(dot(lVec, -u_lightDirection))), 0.05);

	float ln = dot(l, normal)*attenuation;

    //Fresnel
    float fresnel = u_fresBias + u_fresScale * pow(1.0f - max(dot(normal, viewDir), 0.0), u_fresPower);

	//Diffuse light
	const vec3 magicColor = vec3(0.29, 0.379, 0.59);
	vec3 diffuse = u_color.xyz * mix(magicColor, vec3(1.0), (ln*0.5 + 0.5)) * (1 - u_color.w);
	//vec3 diffuse = u_color.xyz * mix(magicColor, vec3(1.0), (ln*0.5 + 0.5));

	const vec3 skyColor = vec3(0.1, 0.2, 0.4)*1.2;
	const vec3 groundColor = vec3(0.1, 0.1, 0.2);

	vec3 rEye = reflect(viewDir, normal).xyz;
	vec3 rWorld = (inverse(u_modelView)*vec4(rEye, 0.0)).xyz;

	vec3 reflect = vec3(1.0) + mix(groundColor, skyColor, smoothstep(0.15, 0.25, rWorld.y));
    
    //Compositing everything
    vec3 finalColor = diffuse + (mix(refract, reflect, fresnel) + specular) * u_color.w;

	o_fragColor = vec4(finalColor, 1.0);

	gl_FragDepth = depth;
}

##end
