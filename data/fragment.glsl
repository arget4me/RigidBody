#version 440

in vec3 in_position;
out vec4 frag_colour;
uniform vec3 diffuse_color;

void main () {
  	//frag_colour = vec4(diffuse_color, 1.0);
  	frag_colour = vec4(((vec3(in_position.z) + 1.0) / 4) +diffuse_color/2, 1.0);
}
