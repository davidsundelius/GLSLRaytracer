#version 130

in vec3		position;

uniform mat4 orthoMatrix; 

void main()
{
	
	gl_Position = orthoMatrix*vec4(position,1);
}