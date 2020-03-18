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
	glm::vec3 desiredVelocityChange;
	float penetrationDepth;
}ContactPoint;

#define MAX_NUM_CONTACTS 100
ContactPoint contactPoints[4000];
ContactPoint resolvePenetrationsPoints[1000];
unsigned int numContacts = 0;
int counter = 0;

float restitution = 0.1f;

float fixedTimeStep = 1.0f / 60.0f;
float dampening = 0.99f;

glm::vec3 gravity = glm::vec3(0.0f, -6.0f, 0.0f);


//Test RigidBody ------
glm::vec3 rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f); float rateOfRotation = 0.0f;
//---------------------

inline void applyFroces(Scene3D& scene, unsigned int index)
{
	//apply gravit
	scene.linearVelocities[index] = scene.linearVelocities[index]*dampening + gravity * fixedTimeStep;
	
	

	//angular velocity -> updated orientation
	//glm::vec3 angularVelocity;
	//if (glm::length(rotationAxis) != 0)angularVelocity = rateOfRotation * glm::normalize(rotationAxis);
	//else angularVelocity = glm::vec3(0);

	scene.angularVelocities[index] *= dampening;
	
	//glm::quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);

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

				/*------------- check collision with plane -------------*/
				
				//TODO: represent plane as normal and offset
				//glm::vec3 planeNormal = glm::vec3(0, 1, 0);//glm::vec3(toMat4(scene.orientations[y]) * glm::vec4(0, 0, -1, 0));
				glm::vec3 planeNormal = glm::vec3(toMat4(scene.orientations[y]) * glm::vec4(0, 0, -1, 0));
				float planeOffset = glm::dot(scene.positions[y], planeNormal);
				float vertexDistance = glm::dot(transformedPoint, planeNormal);

				if (vertexDistance < planeOffset)
				{
					numContacts++;
					if (numContacts <= MAX_NUM_CONTACTS)
					{
						contactPoints[numContacts - 1].contactNormal = planeNormal;
						contactPoints[numContacts - 1].contactPosition = planeNormal * glm::abs(vertexDistance - planeOffset)/2.0f + transformedPoint;
						contactPoints[numContacts - 1].penetrationDepth = planeOffset - vertexDistance;
						contactPoints[numContacts - 1].rigidBodyIndex1 = index;
						contactPoints[numContacts - 1].rigidBodyIndex2 = -1; // only one rigid body
						glm::quat& angVelQ = scene.angularVelocities[index];
						glm::vec3 angVel = glm::vec3(angVelQ.x, angVelQ.y, angVelQ.z);// * fixedTimeStep;
						
						contactPoints[numContacts - 1].closingVelocity = glm::cross(angVel, contactPoints[numContacts - 1].contactPosition - scene.positions[index]) + scene.linearVelocities[index];
						
						
							//contactPoints[numContacts - 1].closingVelocity = scene.linearVelocities[index];//@NOTE: This doesn't account for angular velocity
					}
				}
			}
		}
	}
}



void resolveCollisions(Scene3D& scene)
{
	//@TODO: make this check be rigid body specific, now it only works with one rigid body in the scene.
	//glm::vec3 changeInVelocity = glm::vec3(0);

	for (int i = 0; i < numContacts; i++)
	{
		ContactPoint& p = contactPoints[i];
		glm::vec3 closingVelocity;
		glm::vec3 desiredVelocity;
		
		

		closingVelocity = glm::dot(p.closingVelocity, p.contactNormal) * p.contactNormal;
		desiredVelocity = (-closingVelocity * (1 + restitution));

		//Calculate velocity per unit torque
		glm::vec3 relPoint = p.contactPosition - scene.positions[p.rigidBodyIndex1];
		glm::vec3 torquePerUnit = glm::cross(relPoint, p.contactNormal);
		glm::mat3 rotationMat = glm::toMat3(scene.orientations[p.rigidBodyIndex1]);
		glm::vec3 angVelPerUnit = rotationMat * torquePerUnit;
		angVelPerUnit = scene.inverseInertiaTensors[p.rigidBodyIndex1] * angVelPerUnit;
		angVelPerUnit = glm::inverse(rotationMat) * angVelPerUnit;
		glm::vec3 linearVelPerUnit = glm::cross(angVelPerUnit, relPoint);


		//float linFromAngPerUnitImpulse = glm::max(glm::dot(linearVelPerUnit, p.contactNormal), 0.0f);
		float linFromAngPerUnitImpulse = glm::dot(linearVelPerUnit, p.contactNormal);
		float velPerUnitImpulse = scene.inverseMasses[p.rigidBodyIndex1] + linFromAngPerUnitImpulse;
		
		//If two rigid bodies are colliding
		if (p.rigidBodyIndex2 != -1)
		{
			//Add contribution from other rigid body
		}

	
		//Calculate impulse
		glm::vec3 impulse = desiredVelocity / velPerUnitImpulse;



		//Update Angular velocity
		glm::vec3 impulsiveTorque = glm::cross(relPoint, impulse) ;
		glm::vec3 rotationChange = rotationMat * impulsiveTorque;
		rotationChange = scene.inverseInertiaTensors[p.rigidBodyIndex1] * rotationChange;
		rotationChange = glm::inverse(rotationMat) * rotationChange;
		scene.angularVelocities[p.rigidBodyIndex1] += glm::quat(0, rotationChange.x, rotationChange.y, rotationChange.z);

		//Update linear velocity
		glm::vec3 linearImpulse = scene.inverseMasses[p.rigidBodyIndex1] * impulse;
		
		scene.linearVelocities[p.rigidBodyIndex1] += linearImpulse;
		
	}
}



void resolveInterpenetration(Scene3D& scene)
{
	//@TODO: temporary hack to test if it is the penetration that messes things up

	float maxPenetration = 0.0f;
	for (int i = 0; i < numContacts; i++)
	{
		ContactPoint& p = contactPoints[i];

		if (p.penetrationDepth > glm::length(scene.penetrations[p.rigidBodyIndex1]))
			scene.penetrations[p.rigidBodyIndex1] = p.penetrationDepth * p.contactNormal;

		if (p.rigidBodyIndex2 != -1)
		{
			if (p.penetrationDepth > glm::length(scene.penetrations[p.rigidBodyIndex2]))
				scene.penetrations[p.rigidBodyIndex2] = p.penetrationDepth * p.contactNormal;
		}
	}

	int numObjects = scene.positions.size();
	for (int i = 0; i < numObjects; i++)
	{
		scene.positions[i] += scene.penetrations[i];
	}

}

void applyRigidBodyPhysics(Scene3D& scene)
{
	
	//angular velocity -> updated orientation
	glm::vec3 angularVelocity;
	if (glm::length(rotationAxis) != 0)angularVelocity = rateOfRotation * glm::normalize(rotationAxis);
	else angularVelocity = glm::vec3(0);

	glm::quat debugAngularVelocity = glm::quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);

	numContacts = 0;
	//@Note: temporary hack to test multiple objects, need to resolve each penetration
	scene.penetrations.clear();
	int numObjects = scene.inverseMasses.size();


	for (int i = 0; i < numObjects; i++)
	{
		scene.penetrations.push_back(glm::vec3(0.0f));//same hack as @Note


		if (scene.inverseMasses[i] != -1.0f)
		{
			//If it has an inverse mass then it's a rigid body.
			applyFroces(scene, i);
			calculateCollisions(scene, i);
		}
	}
	resolveCollisions(scene);
	resolveInterpenetration(scene);

	for (int i = 0; i < numObjects; i++)
	{
		if (scene.inverseMasses[i] != -1.0f)
		{

			//velocity -> updated position
			scene.positions[i] += scene.linearVelocities[i] * fixedTimeStep;


			//Update orientation
			scene.orientations[i] = glm::normalize(scene.orientations[i] + fixedTimeStep / 2.0f * (scene.angularVelocities[i] + debugAngularVelocity) * scene.orientations[i]);

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
	//ImGui::SliderFloat3("Axis of rotation", &rotationAxis[0], -1.0f, 1.0f, "%.1f");
	//ImGui::SliderFloat("Rate of rotation", &rateOfRotation, 0.0f, 360.0f, "%.1f");
	ImGui::SliderFloat("Restitution", &restitution, 0.1f, 2.0f, "%.2f");




}



};