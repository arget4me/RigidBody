#version 440

in vec3 in_position;
out vec4 frag_colour;
uniform vec3 diffuse_color;

void main () {
  	//frag_colour = vec4(diffuse_color, 1.0);
	if((abs(in_position.x - floor(in_position.x) - 0.5) <= 0.1 && abs(in_position.y - floor(in_position.y) - 0.5) <= 0.1) || 
	(abs(in_position.y - floor(in_position.y) - 0.5) <= 0.1 && abs(in_position.z - floor(in_position.z) - 0.5) <= 0.1) || 
	(abs(in_position.x - floor(in_position.x) - 0.5) <= 0.1 && abs(in_position.z - floor(in_position.z) - 0.5) <= 0.1))
	{
		frag_colour = vec4(0.4 * diffuse_color, 1.0f);
	}
	else
	{
  		frag_colour = vec4(((vec3(in_position.z) + 1.0) / 4) +diffuse_color/2, 1.0);
	}
}
