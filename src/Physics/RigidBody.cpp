#include "RigidBody.h"
#include <vector>
#include <glm/glm.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include "Utils/logfile.h"

#include "imgui.h"
#include "imgui_impl_glfw_gl3.h"

#include <glm/glm.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "Renderer/Box.h"
#include "Renderer/Plane.h"

namespace ArgetRenderer
{




glm::vec3 box_points[] = {
	{-0.5f, -0.5f, -0.5f},
	{+0.5f, -0.5f, -0.5f},
	{-0.5f, +0.5f, -0.5f},
	{+0.5f, +0.5f, -0.5f},

	{-0.5f, -0.5f, +0.5f},
	{+0.5f, -0.5f, +0.5f},
	{-0.5f, +0.5f, +0.5f},
	{+0.5f, +0.5f, +0.5f},
};


typedef struct {
	int rigidBodyIndex1;
	int rigidBodyIndex2;

	glm::vec3 closingVelocity;
	glm::vec3 contactPosition;
	glm::vec3 contactNormal;
	float penetrationDepth;
}ContactPoint;

#define MAX_NUM_CONTACTS 100
ContactPoint contactPoints[100];
unsigned int numContacts = 0;
int counter = 0;

float restitution = 0.35f;

float fixedTimeStep = 1.0f / 60.0f;
float dampening = 0.99f;

glm::vec3 gravity = glm::vec3(0.0f, 0.0f, 0.0f);


//Test RigidBody ------
glm::vec3 rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f); float rateOfRotation = 0.0f;
//---------------------

inline void applyFroces(Scene3D& scene, unsigned int index)
{
	//apply gravit
	scene.linearVelocities[index] = scene.linearVelocities[index]*dampening + gravity * fixedTimeStep;
	
	

	//angular velocity -> updated orientation
	glm::vec3 angularVelocity;
	if (glm::length(rotationAxis) != 0)angularVelocity = rateOfRotation * glm::normalize(rotationAxis);
	else angularVelocity = glm::vec3(0);

	scene.angularVelocities[index] = glm::quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);

}

void calculateCollisions(Scene3D& scene, unsigned int index)
{
	//group in pairs of colliding points
	//BOX - PLANE collision
	
	counter++;
	//TODO: use primitives for physics instead of model
	if (scene.modelID[index] == GLOBAL_BOX_ID)
	{
		glm::mat4 modelMatrix = glm::mat4(1.0f);
		//glm::mat4 rotation = toMat4(scene.orientations[i]);
		modelMatrix = glm::translate(modelMatrix, scene.positions[index]);
		modelMatrix = modelMatrix * toMat4(scene.orientations[index]);
		modelMatrix = glm::scale(modelMatrix, scene.sizes[index]);

		for (int i = 0; i < sizeof(box_points) / sizeof(glm::vec3); i++)
		{
			glm::vec3 transformedPoint = glm::vec3(modelMatrix * glm::vec4(box_points[i], 1));
			//TODO: do fancier collision detection, don't compare to each other object
			for (int y = 0; y < scene.modelID.size(); y++)
			{
				if (y == index || scene.modelID[y] != GLOBAL_PLANE_ID)continue;

				//check collision with plane
				
				//TODO: represent plane as normal and offset
				glm::vec3 planeNormal = glm::vec3(toMat4(scene.orientations[y]) * glm::vec4(0, 0, -1, 0));
				float planeOffset = glm::dot(scene.positions[y], planeNormal);
				float vertexDistance = glm::dot(transformedPoint, planeNormal);

				/*if (counter % 100 == 0) {
					DEBUG_LOG("Normal: " << planeNormal.x << ", " << planeNormal.y << ", " << planeNormal.z << "\n");
					DEBUG_LOG("Point: " << transformedPoint.x << ", " << transformedPoint.y << ", " << transformedPoint.z << "\n");
					DEBUG_LOG("Plane offset: " << planeOffset << "\n");
					DEBUG_LOG("Point offset: " << vertexDistance << "\n");
				}*/

				if (vertexDistance < planeOffset)
				{
					numContacts++;
					if (numContacts <= MAX_NUM_CONTACTS)
					{
						contactPoints[numContacts - 1].contactNormal = planeNormal;
						contactPoints[numContacts - 1].contactPosition = planeNormal * glm::abs(vertexDistance - planeOffset) + transformedPoint;
						contactPoints[numContacts - 1].penetrationDepth = planeOffset - vertexDistance;
						contactPoints[numContacts - 1].rigidBodyIndex1 = index;
						contactPoints[numContacts - 1].rigidBodyIndex2 = -1; // only one rigid body
						glm::quat& angVelQ = scene.angularVelocities[index];
						glm::vec3 angVel = glm::vec3(angVelQ.x, angVelQ.y, angVelQ.z);
						
						contactPoints[numContacts - 1].closingVelocity = glm::cross(angVel, contactPoints[numContacts - 1].contactPosition - scene.positions[index]) + scene.linearVelocities[index];
					}
				}
			}
		}
	}
}



void resolveCollisions(Scene3D& scene)
{
	for (int i = 0; i < numContacts; i++)
	{
		ContactPoint& p = contactPoints[i];
		glm::vec3 closingVelocity;
		//If only one rigid body in collision
		if (p.rigidBodyIndex2 == -1)
		{
			closingVelocity = glm::dot(p.closingVelocity, p.contactNormal) * p.contactNormal;
		}

		glm::vec3 impulse = -closingVelocity * (1 + restitution);

#define VELOCITY_PER_IMPULSE 1.0f

		glm::vec3 linearImpulse = impulse * scene.inverseMasses[p.rigidBodyIndex1];
		//u = (qrel) X g
		//glm::vec3 angularImpulse = TO_WORLD_SPACE(scene.inverseInertiaTensors[p.rigidBodyIndex1]) * u;
		//DEBUG_LOG("Prev: " << scene.linearVelocities[p.rigidBodyIndex1].x << ", " << scene.linearVelocities[p.rigidBodyIndex1].y << ", " << scene.linearVelocities[p.rigidBodyIndex1].z << "\n");
		scene.linearVelocities[p.rigidBodyIndex1] += linearImpulse;
		//DEBUG_LOG("New: " << scene.linearVelocities[p.rigidBodyIndex1].x << ", " << scene.linearVelocities[p.rigidBodyIndex1].y << ", " << scene.linearVelocities[p.rigidBodyIndex1].z << "\n");
	}
}

void resolveInterpenetration(Scene3D& scene)
{

}

void applyRigidBodyPhysics(Scene3D& scene)
{
	numContacts = 0;
	for (int i = 0; i < scene.inverseMasses.size(); i++)
	{
		if (scene.inverseMasses[i] != -1.0f)
		{
			//If it has an inverse mass then it's a rigid body.
			applyFroces(scene, i);
			calculateCollisions(scene, i);
			//if(numContacts > 0 && counter % 100 == 0)DEBUG_LOG("NUM CONTACTS: " << numContacts);

			resolveCollisions(scene);
			resolveInterpenetration(scene);

			//velocity -> updated position
			scene.positions[i] += scene.linearVelocities[i] * fixedTimeStep;


			//@NOTE: Somehow I need to multiply by TimeStep twise for the velocity to be in degrees per seconds. Looks wrong but this works for now.
			scene.orientations[i] = glm::normalize(scene.orientations[i] + fixedTimeStep * fixedTimeStep / 2.0f * scene.angularVelocities[i] * scene.orientations[i]);

		}
	}
	
}

void physicsDrawIMGUI()
{
	ImGui::Text("Physics");
	ImGui::SliderFloat3("Gravity", &gravity[0], -10.0f, 10.0f, "%.2f");
	//ImGui::SliderFloat("Gravity: y", &gravity[1], -10.0f, 10.0f);
	//ImGui::SliderFloat("Gravity: z", &gravity[2], -10.0f, 10.0f);
	ImGui::SliderFloat("Velocity dampening", &dampening, 0.5f, 1.0f, "%.2f");
	ImGui::SliderFloat3("Axis of rotation", &rotationAxis[0], -1.0f, 1.0f, "%.1f");
	ImGui::SliderFloat("Rate of rotation", &rateOfRotation, 0.0f, 360.0f, "%.1f");
	ImGui::SliderFloat("Restitution", &restitution, 0.1f, 0.5f, "%.2f");

	//glm::vec3 rotationAxis = glm::vec3(0.0f, 0.0f, 1.0f); float rateOfRotation = 36.0f;



}



};