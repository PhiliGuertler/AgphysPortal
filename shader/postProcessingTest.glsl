#include "post_processing_vertex_shader.glsl"

##GL_FRAGMENT_SHADER
#version 430 core

#include "post_processing_helper_fs.glsl"

#include "lighting_helper_fs.glsl"

uniform sampler2D waterPositionTex;
uniform sampler2D waterNormalTex;
uniform sampler2D waterDepthTex;
uniform sampler2D waterVorticityTex;

uniform sampler2D waterPositionTexBack;
uniform sampler2D waterDepthTexBack;

uniform sampler2D foamTexture;
uniform samplerCube u_skybox;

uniform sampler2D coolerGBufferDepth;

uniform float u_specularFactor;
uniform float u_diffuseFactor;
uniform int u_specularPower;
uniform int u_fresnelPower;
uniform vec3 u_specularColor;
uniform vec3 u_diffuseColor;
uniform vec3 u_lightPosition;
uniform vec3 u_lightDirection;
uniform float u_depthFalloff;
uniform bool u_enableFoam;

float fresnelSchlick(vec3 toEye, vec3 toLight, int fresnelPower) {
	vec3 l = normalize(toLight);
	vec3 v = normalize(toEye);
	vec3 h = normalize(l+v);

	float w = pow(1 - clamp(dot(h, v), 0, 1), fresnelPower);
	return w;
}


vec3 blinnPhong(vec3 normal, vec3 toEye, vec3 toLight, vec3 diffuseColor, float diffuseFactor, vec3 specularColor, int specularPower, float specularFactor, int fresnelPower, vec3 lightDirection) {
	vec3 n = normalize(normal);
	vec3 l = normalize(toLight);
	vec3 v = normalize(toEye);
	vec3 h = normalize(l+v);


	float lambertian = clamp(dot(l,n), 0.0, 1.0);
	vec3 reflectDir = reflect(v,n);
	float specAngle = clamp(dot(reflectDir, v), 0.0, 1.0);
	float spec= pow(specAngle, specularPower);

    vec4 skyboxColor = texture(u_skybox, reflectDir);

	vec3 diffuse = lambertian * diffuseFactor * diffuseColor;
	vec3 specular = spec * specularFactor * skyboxColor.xyz;
	//vec3 specular = specularFactor * skyboxColor.xyz;

	float fresnel = fresnelSchlick(toEye, toLight, fresnelPower);

	vec3 specularReflection = 0.03f * specularColor * mix(skyboxColor.xyz, specularColor, fresnel) * pow(clamp(dot(reflect(-lightDirection, normal), toEye), 0, 1), specularPower);

	return diffuse + specular + specularReflection;
	//return v * 0.5 + 0.5;
}

vec3 blinnDiffuse(vec3 normal, vec3 toEye, vec3 toLight, vec3 diffuseColor, float diffuseFactor) {
	vec3 n = normalize(normal);
	vec3 l = normalize(toLight);
	vec3 v = normalize(toEye);
	vec3 h = normalize(l+v);

	float lambertian = clamp(dot(l,n), 0.0, 1.0);
	return lambertian * diffuseFactor * diffuseColor;
}

void main() {
	ivec2 tci = ivec2(gl_FragCoord.xy);

	// texel fetch here
	vec4 imageColor = texelFetch(image, tci, 0);

	vec4 position = texelFetch(waterPositionTex, tci, 0);
	position = position * 2 - 1;

	vec4 waterNormal = texelFetch(waterNormalTex, tci, 0);
	waterNormal.xyz = waterNormal.xyz * 2 - 1;

	float waterDepth = texelFetch(waterDepthTex, tci, 0).r;
	waterDepth = linearDepth(waterDepth, 0.1, 500.f);

	float gDepth = texelFetch(coolerGBufferDepth, tci, 0).r;
	gDepth = linearDepth(gDepth, 0.1, 500.f);

	float waterVorticity = texelFetch(waterVorticityTex, tci, 0).r;
	vec2 waterTC = texelFetch(waterVorticityTex, tci, 0).gb;

	vec2 tcFoam = vec2(gl_FragCoord.xy);
	tcFoam /= screenSize.xy;
	vec2 pictureSize = vec2(302);
	tcFoam *= pictureSize;
	tcFoam *= waterTC;

	vec4 foamTex = texelFetch(foamTexture, ivec2(tcFoam), 0);
	foamTex.w = 1;

	float waterDepthBack = texelFetch(waterDepthTexBack, tci, 0).r;
	waterDepthBack = linearDepth(waterDepthBack, 0.1, 500.f);

	out_color = imageColor;

	// at this position water should be drawn
	float alpha = clamp(waterDepthBack - waterDepth, 0.0, 0.3);
	bool insideWater = waterDepthBack - waterDepth < 0.01 && waterDepth - waterDepthBack < 0.001;

	bool falsePositive = waterDepth < 0.01 && waterDepthBack < 0.01;

	vec4 worldPos = inverse(viewProj) * position;
	vec3 pos = worldPos.xyz/worldPos.w;
	vec3 toEye = pos - camera_position.xyz/camera_position.w;

	// HACK: as this is a directional light, the negative light direction is pointing towards it
	vec3 toLight = -u_lightDirection;
	toLight = normalize(toLight);
	toLight = (view * vec4(toLight, 0)).xyz;

	vec3 blinnPhongResult = blinnPhong(waterNormal.xyz, toEye, toLight, u_diffuseColor, u_diffuseFactor, u_specularColor, u_specularPower, u_specularFactor, u_fresnelPower, u_lightDirection);

	// HACK: ambient term
	blinnPhongResult = max(0.3 * u_diffuseColor, blinnPhongResult);


	if(insideWater && !falsePositive) {
		alpha = max(abs(gDepth-waterDepth), abs(gDepth-waterDepthBack));
		out_color = mix(out_color, vec4(blinnPhongResult, 1), clamp((alpha*0.5)*u_depthFalloff, 0.0, 1.0));

		//out_color = vec4(gDepth, waterDepth, waterDepthBack, 1);
	} else if(abs(waterDepth - waterDepthBack) > 0.99) {
		out_color = out_color;
	} else if(alpha > 0.01) {
		out_color = mix(out_color, vec4(blinnPhongResult, 1), clamp((alpha+0.1)*u_depthFalloff, 0.0, 1.0));
	}

	return;
}
