#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"

#define GLEW_STATIC
#include <GLEW/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective
#include <glm/gtc/type_ptr.hpp> //glm::value_ptr

#include "Utils/logfile.h"
#include "Game/Scene.h"
#include "Renderer/Renderer3D.h"
#include "Physics/RigidBody.h"

//Globals
int global_height = 720;
int global_width = 1080;

static void error_callback(int error, const char* description)
{
	std::cerr << description;
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if ((key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
}

static void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
	global_width = width;
	global_height = height;
}

int main(int argc, char* argv[])
{
	glfwSetErrorCallback(error_callback);
	if (!glfwInit())
		exit(EXIT_FAILURE);

	GLFWwindow* window = glfwCreateWindow(global_width, global_height, "Box Rigid Body Physics", NULL, NULL);
	//glfwSetKeyCallback(window, key_callback);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	if (!window) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	
	//Activate V-sync
	glfwSwapInterval(1);

	// start GLEW extension handler
	glewExperimental = GL_TRUE;
	glewInit();

	glEnable(GL_DEPTH_TEST); // enable depth-testing
	glDepthFunc(GL_LESS);
	
	glClearColor(0.0f, 0.0f, 0.5f, 1.0f);

	//Setup IMGUI
	ImGui::CreateContext();
	ImGui_ImplGlfwGL3_Init(window, true);

	ImGui::StyleColorsDark();

	
	//Setup Renderer
	ArgetRenderer::setupRenderer();

	//Setup scene
	ArgetRenderer::Scene3D scene;
	ArgetRenderer::setupScene(scene);

#define FPS_TIMED 0
#if FPS_TIMED
	double previousTime = glfwGetTime();
	int frameCount = 0;
#endif

	while (!glfwWindowShouldClose(window))
	{
#if FPS_TIMED
		// Measure speed
		double currentTime = glfwGetTime();
		frameCount++;
		// If a second has passed.
		if (currentTime - previousTime >= 1.0)
		{
			// Display the frame count here any way you want.
			//displayFPS(frameCount);
			DEBUG_LOG(frameCount << "\n");

			frameCount = 0;
			previousTime = currentTime;
		}
#endif

		// update other events like input handling 
		glfwPollEvents();

		// clear the drawing surface
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		//Update
		ArgetRenderer::applyRigidBodyPhysics(scene);

		//Draw
		ImGui_ImplGlfwGL3_NewFrame();
		if (ImGui::Button("Restart Scene"))
		{
			//Setup scene
			ArgetRenderer::Scene3D new_scene;
			ArgetRenderer::setupScene(new_scene);
			scene = new_scene;
		}
		ArgetRenderer::physicsDrawIMGUI();
		ArgetRenderer::rendererDrawIMGUI();
		ImGui::Render();
		
		ArgetRenderer::renderScene(scene);

		ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);

	}
	glfwTerminate();

	return 0;
}