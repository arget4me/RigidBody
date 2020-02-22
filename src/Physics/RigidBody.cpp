#include "RigidBody.h"
#include <vector>
#include <glm/glm.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Utils/logfile.h"

#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"

namespace ArgetRenderer
{





float fixedTimeStep = 1.0f / 60.0f;
float dampening = 1.00f;

glm::vec3 gravity = glm::vec3(0.0f, 0.0f, 0.0f);

inline void applyFroces(Scene3D& scene, unsigned int index)
{
	//apply gravit
	scene.linearVelocities[index] = scene.linearVelocities[index]*dampening + gravity * fixedTimeStep;
	//velocity -> updated position

	scene.positions[index] += scene.linearVelocities[index] * fixedTimeStep;

	//angular velocity -> updated orientation

}

void calculateCollisions(Scene3D& scene, unsigned int index)
{
	//group in pairs of colliding points

}

void resolveCollisions(Scene3D& scene, unsigned int index)
{

}

void resolveInterpenetration(Scene3D& scene, unsigned int index)
{

}

void applyRigidBodyPhysics(Scene3D& scene, unsigned int index)
{
	//Object at index is a rigid body
	applyFroces(scene, index);
	calculateCollisions(scene, index);
	resolveCollisions(scene, index);
	resolveInterpenetration(scene, index);
}

void applyRigidBodyPhysics(Scene3D& scene)
{
	for (int i = 0; i < scene.inverseMasses.size(); i++)
	{
		if (scene.inverseMasses[i] != -1.0f)
		{
			//If it has an inverse mass then it's a rigid body.
			applyRigidBodyPhysics(scene, i);
		}
	}
}

void physicsDrawIMGUI()
{
	ImGui::Text("Physics");
	ImGui::SliderFloat("Gravity: x", &gravity[0], -10.0f, 10.0f);
	ImGui::SliderFloat("Gravity: y", &gravity[1], -10.0f, 10.0f);
	ImGui::SliderFloat("Gravity: z", &gravity[2], -10.0f, 10.0f);
	ImGui::SliderFloat("Velocity dampening", &dampening, 0.5f, 1.0f);
}



};