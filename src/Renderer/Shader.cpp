#include "Shader.h"
#include "Utils/logfile.h"
#include "Utils/readfile.h"

namespace ArgetRenderer
{ 

ShaderProgram compileShader(std::string VertexPath, std::string FragmentPath)
{

	ShaderProgram newProgram = { 0, VertexPath, FragmentPath };

	std::string vertex_shader_str = readFile(VertexPath);
	std::string fragment_shader_str = readFile(FragmentPath);

	const char* vertex_shader_src = vertex_shader_str.c_str();
	const char* fragment_shader_src = fragment_shader_str.c_str();
	DEBUG_LOG(vertex_shader_src);
	DEBUG_LOG(fragment_shader_src);


	GLuint vs = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vs, 1, &vertex_shader_src, NULL);
	glCompileShader(vs);
	checkShaderCompileError(vs);

	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fs, 1, &fragment_shader_src, NULL);
	glCompileShader(fs);
	checkShaderCompileError(fs);

	newProgram.ID = glCreateProgram();

	glAttachShader(newProgram.ID, fs);
	glAttachShader(newProgram.ID, vs);

	glLinkProgram(newProgram.ID);
	checkShaderProgramError(newProgram.ID);

	glDeleteShader(vs);
	glDeleteShader(fs);

	return newProgram;
}


void recompileShader(ShaderProgram &shaderProgram)
{
	std::string vertex_shader_str = readFile(shaderProgram.VertexPath);
	std::string fragment_shader_str = readFile(shaderProgram.FragmentPath);

	const char* vertex_shader_src = vertex_shader_str.c_str();
	const char* fragment_shader_src = fragment_shader_str.c_str();

	GLuint vs = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vs, 1, &vertex_shader_src, NULL);
	glCompileShader(vs);
	checkShaderCompileError(vs);

	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fs, 1, &fragment_shader_src, NULL);
	glCompileShader(fs);
	checkShaderCompileError(fs);

	shaderProgram.ID = glCreateProgram();

	glAttachShader(shaderProgram.ID, fs);
	glAttachShader(shaderProgram.ID, vs);

	glLinkProgram(shaderProgram.ID);
	checkShaderProgramError(shaderProgram.ID);

	glDeleteShader(vs);
	glDeleteShader(fs);
}

GLuint global_active_program = 0;

void useShaderProgram(ShaderProgram& shaderProgram)
{
	if (global_active_program == 0 || global_active_program != shaderProgram.ID)
	{
		DEBUG_LOG("USE Program: "); DEBUG_LOG(shaderProgram.ID); DEBUG_LOG("\n");
		glUseProgram(shaderProgram.ID);
		global_active_program = shaderProgram.ID;
	}
}

void checkShaderCompileError(GLint shaderID)
{
	GLint isCompiled = 0;
	glGetShaderiv(shaderID, GL_COMPILE_STATUS, &isCompiled);

	if (isCompiled == GL_FALSE)
	{
		GLint maxLength = 0;
		glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &maxLength);

		// The maxLength includes the NULL character
		std::string errorLog;
		errorLog.resize(maxLength);
		glGetShaderInfoLog(shaderID, maxLength, &maxLength, &errorLog[0]);

		DEBUG_LOG("shader compilation failed:\n");
		ERROR_LOG(errorLog);
		DEBUG_LOG("\n");
	}
	else
	{
		DEBUG_LOG("shader compilation success!\n");
	}
}

void checkShaderProgramError(GLuint shaderProgram)
{
	GLint isCompiled = 0;
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &isCompiled);

	if (isCompiled == GL_FALSE)
	{
		GLint maxLength = 0;
		glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &maxLength);

		// The maxLength includes the NULL character
		std::string errorLog;
		errorLog.resize(maxLength);
		glGetProgramInfoLog(shaderProgram, maxLength, &maxLength, &errorLog[0]);

		DEBUG_LOG("Shader program compilation failed:\n");
		ERROR_LOG(errorLog);
		DEBUG_LOG("\n");
	}
	else
	{
		DEBUG_LOG("Shader program compilation success!\n");
	}
}

};