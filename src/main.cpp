#include <Windows.h>
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

#include <cstdlib>
#include "Renderer/Box.h"

//Globals
int global_height = 720;
int global_width = 1280;

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
#if 0
//int main(int argc, char* argv[])
#else
INT WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
	PSTR lpCmdLine, INT nCmdShow)
#endif
{
	glfwSetErrorCallback(error_callback);
	if (!glfwInit())
		exit(EXIT_FAILURE);

	
#if 1
	GLFWwindow* window = glfwCreateWindow(global_width, global_height, "Box Rigid Body Physics", NULL, NULL);
#else
	GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(monitor);
	glfwWindowHint(GLFW_DECORATED, 0);

	GLFWwindow* window = glfwCreateWindow(mode->width, mode->height, "Box Rigid Body Physics", NULL, NULL);
	glViewport(0, 0, mode->width, mode->height);
#endif



	if (!window) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	
	
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

	int FPS = 0;
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
			FPS = frameCount;

			frameCount = 0;
			previousTime = currentTime;
		}
#endif

		// update other events like input handling 
		glfwPollEvents();

		// clear the drawing surface
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		
		//Draw
		ImGui_ImplGlfwGL3_NewFrame();
#if FPS_TIMED
		ImGui::Text("FPS: %d", FPS);
#endif
		static bool apply_physics = false;
		if (!apply_physics) {
			if (ImGui::Button("Start"))
			{
				//Start simulation
				apply_physics = true;
			}
			static glm::vec3 euler_angles_debug = glm::vec3(0.6f, 0.0f, 0.4f);
			ImGui::SliderFloat3("Rotation", &euler_angles_debug[0], -1.0f, 1.0f, "%.1f");
			for (int i = 0; i < scene.orientations.size(); i++)
			{
				if (scene.inverseMasses[i] != -1.0f)
				{
					scene.orientations[i] = glm::quat(euler_angles_debug);

						
				}

			}
		}
		else
		{
			if (ImGui::Button("Reset"))
			{
				//Setup scene
				ArgetRenderer::Scene3D new_scene;
				ArgetRenderer::setupScene(new_scene);
				scene = new_scene;
				apply_physics = false;
			}

			if (ImGui::Button("SpawnBox"))
			{
				//Spawn box at random orientation
				ArgetRenderer::addObject(scene, 
					ArgetRenderer::GLOBAL_BOX_ID, 
					glm::vec3(-2 + (rand() % 4), 2, 0 + (rand() % 4)),
					glm::vec3(1.0f),
					glm::normalize(ArgetRenderer::quatIdentity() + glm::quat(0, rand()%100, rand()%100, rand()%100) * ArgetRenderer::quatIdentity() / 2.0f),
					ArgetRenderer::GLOBAL_BOX_COLOR);
				setRigidBodyBox(scene, scene.positions.size() - 1, 10.0f, 1.0f);
			}


			//Update
			ArgetRenderer::applyRigidBodyPhysics(scene);

		}

		ArgetRenderer::physicsDrawIMGUI();
		//ArgetRenderer::rendererDrawIMGUI();
		ImGui::Render();
		
		ArgetRenderer::renderScene(scene);

		ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);

	}
	glfwTerminate();

	return 0;
}