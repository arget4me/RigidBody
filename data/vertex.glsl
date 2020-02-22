#version 400

layout(location=0) in vec4 vertex_position; //vertex position

uniform mat4 mvp;

out vec3 in_position;

void main () {
	gl_Position = mvp * vertex_position;
	in_position = vertex_position.xyz;
};
  
