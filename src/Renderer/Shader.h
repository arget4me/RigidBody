#ifndef SHADER_HEADER
#define SHADER_HEADER

#include <GLEW/glew.h>
#include <string>

namespace ArgetRenderer
{
	typedef struct Shader
	{
		GLuint ID = 0;
		std::string VertexPath;
		std::string FragmentPath;
	}ShaderProgram;

	ShaderProgram compileShader(std::string VertexPath, std::string FragmentPath);
	void recompileShader(ShaderProgram& shaderProgram);

	void useShaderProgram(ShaderProgram& shaderProgram);

	void checkShaderCompileError(GLint shaderID);

	void checkShaderProgramError(GLuint shaderProgram);




};



#endif