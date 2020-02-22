#include "Renderer3D.h"
#include "Shader.h"
#include "Box.h"
#include "Plane.h"

#include <glm/glm.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"

namespace ArgetRenderer
{

unsigned int GLOBAL_BOUND_MODEL = 0;
unsigned int GLOBAL_BOUND_NUM_INDICIES = 0;

ShaderProgram boxShader;

glm::mat4 modelMatrix = glm::mat4(1.0f);
glm::mat4 viewMatrix = glm::mat4(1.0f);
glm::mat4 projectionMatrix;

GLuint mvp_shader_location = 0;
GLuint color_shader_location = 0;

glm::vec4 cameraPosition = glm::vec4(0.0f, -1.0f, -10.0f, 0.0f);

glm::vec3 colors[] = {
	glm::vec3(1.0f, 0.0f, 0.0f),
	glm::vec3(1.0f, 1.0f, 0.0f),
};


void setupRenderer()
{
	boxShader = compileShader("data/vertex.glsl", "data/fragment.glsl");
	useShaderProgram(boxShader);
	useBoxModel();
	usePlaneModel();

	//Setup uniforms
	projectionMatrix = glm::perspective(glm::radians(45.0f), (float)1920 / (float)1080, 1.0f, 100.0f);
	viewMatrix = glm::translate(viewMatrix, glm::vec3(cameraPosition));
	mvp_shader_location = glGetUniformLocation(boxShader.ID, "mvp");
	color_shader_location = glGetUniformLocation(boxShader.ID, "diffuse_color");
}

inline void useModel(unsigned int modelID)
{
	if (modelID == GLOBAL_BOUND_MODEL)return;
	if (modelID == GLOBAL_BOX_ID)
	{
		useBoxModel();
		GLOBAL_BOUND_NUM_INDICIES = GLOBAL_BOX_NUM_INDICIES;
	}
	else if (modelID == GLOBAL_PLANE_ID)
	{
		usePlaneModel();
		GLOBAL_BOUND_NUM_INDICIES = GLOBAL_PLANE_NUM_INDICIES;
	}
}


void rendererDrawIMGUI()
{
	
	{
		ImGui::Text("Box rotation");
		//ImGui::SliderFloat("horizontal", &g_rotation[1], -180.0f, 180.0f);
		//ImGui::SliderFloat("vertical", &g_rotation[0], -180.0f, 180.0f);
		ImGui::Text("Camera controls");
		ImGui::SliderFloat("x - pos", &cameraPosition[0], -50.0f, 50.0f);
		ImGui::SliderFloat("y - pos", &cameraPosition[1], -50.0f, 50.0f);
		ImGui::SliderFloat("z - pos", &cameraPosition[2], -50.0f, 50.0f);
		ImGui::Text("Color Palette");
		ImGui::ColorEdit3("Box colour", (float*) & (colors[GLOBAL_BOX_COLOR])[0]); // Edit 3 floats representing a color
		ImGui::ColorEdit3("Ground colour", (float*) & (colors[GLOBAL_PLANE_COLOR])[0]); // Edit 3 floats representing a color
	}
}

void renderScene(Scene3D& scene)
{
	useShaderProgram(boxShader);



	//Setup camera
	viewMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(cameraPosition));
	glm::mat4 projectionViewMatrix = projectionMatrix * viewMatrix;


	//Render all objects, no ordering
	for (int i = 0; i < scene.positions.size(); i++)
	{
		modelMatrix = glm::mat4(1.0f);
		//glm::mat4 rotation = toMat4(scene.orientations[i]);
		modelMatrix = glm::translate(modelMatrix, scene.positions[i]);
		modelMatrix = modelMatrix * toMat4(scene.orientations[i]);
		modelMatrix = glm::scale(modelMatrix, scene.sizes[i]);

		glm::mat4 modelViewProjectionMatrix = projectionViewMatrix * modelMatrix;

		useModel(scene.modelID[i]);

		glUniformMatrix4fv(mvp_shader_location, 1, GL_FALSE, glm::value_ptr(modelViewProjectionMatrix));
		glUniform3fv(color_shader_location, 1, (GLfloat*)&colors[scene.color_index[i]]);

		glDrawElements(GL_TRIANGLES, GLOBAL_BOUND_NUM_INDICIES, GL_UNSIGNED_INT, 0);
	}


	/*//temporary update start
	glm::vec3 new_falling_box_pos = falling_box_pos + 0.016f * glm::vec3(0.0f, -5.0f, 0.0f);
	if (!checkCollisionNoRotation(new_falling_box_pos, 0.5f, floor_pos))
		falling_box_pos = new_falling_box_pos;
	else
	{
		falling_box_pos = floor_pos;//temporary fix for stationary pos
	}
	//temporary update done
	*/

	/*
{
	modelMatrix = glm::mat4(1.0f);
	modelMatrix = glm::translate(modelMatrix, falling_box_pos);
	//rotx = glm::rotate(glm::mat4(1.0f), glm::radians(g_rotation[0]), glm::vec3(1.0, 0.0, 0.0));
	//roty = glm::rotate(glm::mat4(1.0f), glm::radians(g_rotation[1]), glm::vec3(0.0, 1.0, 0.0));
	modelMatrix = rotx * roty * modelMatrix;
	modelMatrix = glm::scale(modelMatrix, glm::vec3(0.5f));
	glm::mat4 modelViewProjectionMatrix = projectionMatrix * viewMatrix * modelMatrix;
	glUniformMatrix4fv(mvp_shader_location, 1, GL_FALSE, glm::value_ptr(modelViewProjectionMatrix));
	glUniform3fv(color_shader_location, 1, (GLfloat*)&box_color[0]);


	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}


{
	modelMatrix = glm::translate(glm::mat4(1.0f), resting_box_pos);
	modelMatrix = glm::scale(modelMatrix, glm::vec3(0.5f));
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0.0f, +0.5f, 0.0f));

	glm::mat4 modelViewProjectionMatrix = projectionMatrix * viewMatrix * modelMatrix;
	glUniformMatrix4fv(mvp_shader_location, 1, GL_FALSE, glm::value_ptr(modelViewProjectionMatrix));
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}

usePlaneModel();
{
	modelMatrix = glm::mat4(1.0f);
	modelMatrix = glm::translate(modelMatrix, floor_pos);
	modelMatrix = glm::rotate(modelMatrix, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
	modelMatrix = glm::scale(modelMatrix, glm::vec3(4.0f));

	glm::mat4 modelViewProjectionMatrix = projectionMatrix * viewMatrix * modelMatrix;
	glUniformMatrix4fv(mvp_shader_location, 1, GL_FALSE, glm::value_ptr(modelViewProjectionMatrix));
	glUniform3fv(color_shader_location, 1, (GLfloat*)&plane_color[0]);

	glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_INT, 0);
}
*/


	
}



};