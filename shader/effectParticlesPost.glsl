#include "post_processing_vertex_shader.glsl"


##GL_FRAGMENT_SHADER
#version 330

#include "post_processing_helper_fs.glsl"

uniform sampler2D particleColor;
uniform sampler2D particleDepth;


//simple pass through shader
void main() {
    ivec2 tci = ivec2(gl_FragCoord.xy);
    vec4 imageColor = texelFetch( image, tci ,0);

	vec4 particleColor = texelFetch(particleColor, tci, 0);

	float depth = texelFetch(particleDepth, tci, 0).r;

	out_color = mix(imageColor, particleColor, particleColor.a);
}
