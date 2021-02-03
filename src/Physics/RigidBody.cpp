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
	{-0.5f, -0.5f, -0.5f}, //0: bottom - left (back)
	{+0.5f, -0.5f, -0.5f}, //1: bottom - right (back)
	{-0.5f, +0.5f, -0.5f}, //2: top - left (back)
	{+0.5f, +0.5f, -0.5f}, //3: top - right (back)

	{-0.5f, -0.5f, +0.5f}, //4: bottom - left (front)
	{+0.5f, -0.5f, +0.5f}, //5: bottom - right (front)
	{-0.5f, +0.5f, +0.5f}, //6: top - left (front)
	{+0.5f, +0.5f, +0.5f}, //7: top - right (front)
};

glm::ivec2 box_edges_pairs[] = {

	//back edges 
	{0,1},//bottom
	{2,0},//left
	{3,2},//top
	{1,3},//right

	//front edges
	{0 + 4, 1 + 4},//bottom
	{2 + 4, 0 + 4},//left
	{3 + 4, 2 + 4},//top
	{1 + 4, 3 + 4},//right

	//left edges
	{0 + 4, 0},//bottom
	{2 + 4, 2},//top

	//right edges
	{1 + 4, 1},//bottom
	{3 + 4, 3},//top
};

glm::vec3 box_edges[] = {
	
	//back edges 
	{box_points[0] - box_points[1]},//bottom
	{box_points[2] - box_points[0]},//left
	{box_points[3] - box_points[2]},//top
	{box_points[1] - box_points[3]},//right

	//front edges
	{box_points[0 + 4] - box_points[1 + 4]},//bottom
	{box_points[2 + 4] - box_points[0 + 4]},//left
	{box_points[3 + 4] - box_points[2 + 4]},//top
	{box_points[1 + 4] - box_points[3 + 4]},//right

	//left edges
	{box_points[0 + 4] - box_points[0]},//bottom
	{box_points[2 + 4] - box_points[2]},//top

	//right edges
	{box_points[1 + 4] - box_points[1]},//bottom
	{box_points[3 + 4] - box_points[3]},//top
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

float restitution = 0.1f;

float fixedTimeStep = 1.0f / 60.0f;
float dampening = 0.99f;

glm::vec3 gravity = glm::vec3(0.0f, -6.0f, 0.0f);

inline void applyForces(Scene3D& scene, unsigned int index)
{
	//apply gravit
	scene.linearVelocities[index] = scene.linearVelocities[index]*dampening + gravity * fixedTimeStep;
	
	//only dampen angular, no new force is applied here
	scene.angularVelocities[index] *= dampening;
}

void calculateCollisions(Scene3D& scene, unsigned int index)
{
	//TODO: use primitives for physics instead of model
	if (scene.modelID[index] != GLOBAL_BOX_ID) return;

	glm::mat4 modelMatrix = glm::mat4(1.0f);
	modelMatrix = glm::translate(modelMatrix, scene.positions[index]);
	modelMatrix = modelMatrix * toMat4(scene.orientations[index]);
	modelMatrix = glm::scale(modelMatrix, scene.sizes[index]);


	// BOX - PLANE collision
	for (int i = 0; i < sizeof(box_points) / sizeof(glm::vec3); i++)
	{
		glm::vec3 transformedPoint = glm::vec3(modelMatrix * glm::vec4(box_points[i], 1));
		//TODO: do fancier collision detection, don't compare to each other object
		for (int y = 0; y < scene.modelID.size(); y++)
		{
			if (y == index || scene.modelID[y] != GLOBAL_PLANE_ID)continue;

			/*------------- check collision with plane -------------*/
				
			//TODO: represent plane as normal and offset
			glm::vec3 planeNormal = glm::vec3(toMat4(scene.orientations[y]) * glm::vec4(0, 0, -1, 0));
			float planeOffset = glm::dot(scene.positions[y], planeNormal);
			float vertexDistance = glm::dot(transformedPoint, planeNormal);

			if (vertexDistance < planeOffset)
			{
				int newContactIndex = numContacts;
				numContacts++;
				if (numContacts <= MAX_NUM_CONTACTS)
				{
					contactPoints[newContactIndex].contactNormal = planeNormal;
					contactPoints[newContactIndex].contactPosition = planeNormal * glm::abs(vertexDistance - planeOffset)/2.0f + transformedPoint;
					contactPoints[newContactIndex].penetrationDepth = planeOffset - vertexDistance;
					contactPoints[newContactIndex].rigidBodyIndex1 = index;
					contactPoints[newContactIndex].rigidBodyIndex2 = -1; // only one rigid body
					glm::quat& angVelQ = scene.angularVelocities[index];
					glm::vec3 angVel = glm::vec3(angVelQ.x, angVelQ.y, angVelQ.z);
						
					contactPoints[newContactIndex].closingVelocity = glm::cross(angVel, contactPoints[newContactIndex].contactPosition - scene.positions[index]) + scene.linearVelocities[index];
					//contactPoints[numContacts - 1].closingVelocity = scene.linearVelocities[index];//@NOTE: This doesn't account for angular velocity
				}
			}

			break;
		}
	}


	//Box - Box Collision
	for (int y = 0; y < scene.modelID.size(); y++)
	{
		if (y == index || scene.modelID[y] != GLOBAL_BOX_ID)continue;

		if (y < index && scene.active[y])continue;//Only for allowing to awaken previous indices

		float max_axis_length = 0.0f;
		{
			//check dimensions of first cube
			float* s = (float*)&scene.sizes[index];
			max_axis_length = s[0];//0, 0
			if (max_axis_length < s[5])max_axis_length = s[5]; // 1, 1 -> 1*4 + 1 = 5
			if (max_axis_length < s[10])max_axis_length = s[10];// 2, 2 - > 2*4 + 2 = 10

			//Check dimensions of other cube.
			s = (float*)&scene.sizes[y];
			if (max_axis_length < s[0])max_axis_length = s[0];//0, 0
			if (max_axis_length < s[5])max_axis_length = s[5]; // 1, 1 -> 1*4 + 1 = 5
			if (max_axis_length < s[10])max_axis_length = s[10];// 2, 2 - > 2*4 + 2 = 10
		}

		if (glm::length(scene.positions[index] - scene.positions[y])/2 > max_axis_length*max_axis_length * max_axis_length)continue;//Dont check further if distance is already to big

		//Point-face collision: Use Separating axis theorem as an early out
		//if (box_separating_axis_theorem(index, y))
		//Potential collision

		glm::mat4 modelMatrixB = glm::mat4(1.0f);
		modelMatrixB = glm::translate(modelMatrixB, scene.positions[y]);
		modelMatrixB = modelMatrixB * toMat4(scene.orientations[y]);
		modelMatrixB = glm::scale(modelMatrixB, scene.sizes[y]);

		glm::mat4 modelMatrixB_axis = glm::mat4(1.0f);
		modelMatrixB_axis = modelMatrixB_axis * toMat4(scene.orientations[y]);
		modelMatrixB_axis = glm::scale(modelMatrixB_axis, scene.sizes[y]);

		glm::mat4 boxFrameB = glm::mat4(1.0f);
		boxFrameB = glm::translate(boxFrameB, scene.positions[y]);
		boxFrameB = boxFrameB * toMat4(scene.orientations[y]);

		glm::vec3 contactNormal;
		glm::vec3 contactPosition;
		float penetration = 0;
		for (int i = 0; i < sizeof(box_points) / sizeof(glm::vec3); i++)
		{
			glm::vec3 transformedPointA = glm::vec3(modelMatrix * glm::vec4(box_points[i], 1));
			float* mat = (float*) &modelMatrixB_axis;
			glm::vec3 x_axis = glm::vec3(mat[0], mat[1], mat[2]);
			glm::vec3 y_axis = glm::vec3(mat[0 + 4], mat[1 + 4], mat[2 + 4]);
			glm::vec3 z_axis = glm::vec3(mat[0 + 8], mat[1 + 8], mat[2 + 8]);

			glm::vec3 transformedPointA_rel_B = (glm::vec3)(glm::inverse(boxFrameB) * glm::vec4(transformedPointA, 1));
			glm::vec3 normal;

			float min_depth = glm::length(x_axis) * 0.5f - fabs(transformedPointA_rel_B.x);
			if (min_depth < 0) continue;
			normal = glm::normalize(x_axis * ((transformedPointA_rel_B.x < 0) ? -1.0f : 1.0f));

			float depth = glm::length(y_axis) * 0.5f - fabs(transformedPointA_rel_B.y);
			if (depth < 0) continue;
			else if (depth < min_depth)
			{
				min_depth = depth;
				normal = glm::normalize(y_axis * ((transformedPointA_rel_B.y < 0) ? -1.0f : 1.0f));
			}

			depth = glm::length(z_axis) * 0.5f - fabs(transformedPointA_rel_B.z);
			if (depth < 0) continue;
			else if (depth < min_depth)
			{
				min_depth = depth;
				normal = glm::normalize(z_axis * ((transformedPointA_rel_B.z < 0) ? -1.0f : 1.0f));
			}

			if (min_depth > penetration)
			{
				// Compile the contact.
				contactNormal = normal;
				contactPosition = transformedPointA;
				penetration = min_depth;
			}
		}

		bool switch_order = false;
		glm::mat4 modelMatrixA_axis = glm::mat4(1.0f);
		modelMatrixA_axis = modelMatrixA_axis * toMat4(scene.orientations[index]);
		modelMatrixA_axis = glm::scale(modelMatrixA_axis, scene.sizes[index]);

		glm::mat4 boxFrameA = glm::mat4(1.0f);
		boxFrameA = glm::translate(boxFrameA, scene.positions[index]);
		boxFrameA = boxFrameA * toMat4(scene.orientations[index]);
		for (int i = 0; i < sizeof(box_points) / sizeof(glm::vec3); i++)
		{
			glm::vec3 transformedPointB = glm::vec3(modelMatrixB * glm::vec4(box_points[i], 1));
			float* mat = (float*)&modelMatrixA_axis;
			glm::vec3 x_axis = glm::vec3(mat[0], mat[1], mat[2]);
			glm::vec3 y_axis = glm::vec3(mat[0 + 4], mat[1 + 4], mat[2 + 4]);
			glm::vec3 z_axis = glm::vec3(mat[0 + 8], mat[1 + 8], mat[2 + 8]);

			glm::vec3 transformedPointB_rel_A = (glm::vec3)(glm::inverse(boxFrameA) * glm::vec4(transformedPointB, 1));
			glm::vec3 normal;

			float min_depth = glm::length(x_axis)*0.5f - fabs(transformedPointB_rel_A.x);
			if (min_depth < 0) continue;
			normal = glm::normalize(x_axis * ((transformedPointB_rel_A.x < 0) ? -1.0f : 1.0f));

			float depth = glm::length(y_axis) * 0.5f - fabs(transformedPointB_rel_A.y);
			if (depth < 0) continue;
			else if (depth < min_depth)
			{
				min_depth = depth;
				normal = glm::normalize(y_axis * ((transformedPointB_rel_A.y < 0) ? -1.0f : 1.0f));
			}

			depth = glm::length(z_axis) * 0.5f - fabs(transformedPointB_rel_A.z);
			if (depth < 0) continue;
			else if (depth < min_depth)
			{
				min_depth = depth;
				normal = glm::normalize(z_axis * ((transformedPointB_rel_A.z < 0) ? -1.0f : 1.0f));
			}

			if (min_depth > penetration)
			{
				// Compile the contact.
				contactNormal = normal;
				contactPosition = transformedPointB;
				penetration = min_depth;
				switch_order = true;
			}
		}


#if 0		
		//Edge - edge collision
		glm::vec3 edge_normal;
		glm::vec3 edge_point;
		float edge_d = 0.0f;
		for (int i = 0; i < sizeof(box_edges) / sizeof(glm::vec3); i++)
		{
			glm::vec3 edge_best_normal;
			glm::vec3 edge_best_point;
			float edge_best_d = 1000000000.0f; //Shortest distance for each edge is the possible candidate.
			

			glm::vec3 transformedEdgeA = glm::vec3(modelMatrixA_axis * glm::vec4(box_edges[i], 1)); //Line A rotated into worldspace
			glm::vec3 edge_offsetA = glm::vec3(modelMatrixB_axis * glm::vec4(box_points[box_edges_pairs[i].y], 1)); //Line A origin, world space
			for (int j = 0; j < sizeof(box_edges) / sizeof(glm::vec3); j++)
			{
				glm::vec3 transformedEdgeB = glm::vec3(modelMatrixB_axis * glm::vec4(box_edges[j], 1)); //Line B rotated into worldspace
				glm::vec3 edge_offsetB = glm::vec3(modelMatrixB_axis * glm::vec4(box_points[box_edges_pairs[j].y], 1)); //Line B origin world space
				float projB_amount = glm::dot(transformedEdgeA, transformedEdgeB) / glm::length(transformedEdgeB);
				float projA_amount = glm::dot(transformedEdgeA, transformedEdgeB) / glm::length(transformedEdgeA);
				if (projB_amount < 0 || projA_amount < 0 || projA_amount >= glm::length(transformedEdgeA) || projB_amount >= glm::length(transformedEdgeB))continue;
				glm::vec3 projB = scene.positions[y] + edge_offsetB + projB_amount * glm::normalize(transformedEdgeB);
				glm::vec3 projA = scene.positions[index] + edge_offsetA + projA_amount * glm::normalize(transformedEdgeA);
				float dist = glm::length(projA - projB);

				if (glm::length(scene.positions[y] - projA) < glm::length(scene.positions[y] - projB))
				{
					if (edge_best_d > dist)
					{
						edge_best_d = dist;
						edge_best_normal = glm::normalize(projA - projB);
						edge_best_point = projB + edge_best_normal * dist * 0.5f;
					}
				}
			}
			if (edge_best_d != 1000000000.0f && edge_best_d > edge_d)
			{
				edge_d = edge_best_d;
				edge_point = edge_best_point;
				edge_normal = edge_best_normal;
			}
		}

		if (edge_d > penetration)
		{
			penetration = edge_d/2.0f;
			contactNormal = edge_normal;
			contactPosition = edge_point;
		}
#endif		



		if (penetration > 0.0f)
		{
			int newContactIndex = numContacts;
			numContacts++;
			if (numContacts <= MAX_NUM_CONTACTS)
			{
				
				contactPoints[newContactIndex].contactNormal = contactNormal;
				contactPoints[newContactIndex].contactPosition = contactPosition;
				contactPoints[newContactIndex].penetrationDepth = penetration/2.0f;
				if (!switch_order)
				{
					contactPoints[newContactIndex].rigidBodyIndex1 = index;
					contactPoints[newContactIndex].rigidBodyIndex2 = y; // only one rigid body

					//Sum up closing velocities, include rotational induced velocity.
					//The collision response must then resolve the remaining delta velocity.
					{
						glm::quat& angVelQ = scene.angularVelocities[index];
						glm::vec3 angVel = glm::vec3(angVelQ.x, angVelQ.y, angVelQ.z);

						contactPoints[newContactIndex].closingVelocity = glm::cross(angVel, contactPoints[newContactIndex].contactPosition - scene.positions[index]) + scene.linearVelocities[index];
					}

					{
						glm::quat& angVelQ = scene.angularVelocities[y];
						glm::vec3 angVel = glm::vec3(angVelQ.x, angVelQ.y, angVelQ.z);
						contactPoints[newContactIndex].closingVelocity -= glm::cross(angVel, contactPoints[newContactIndex].contactPosition - scene.positions[y]) + scene.linearVelocities[y];
					}
				}
				else
				{
					contactPoints[newContactIndex].rigidBodyIndex2 = index;
					contactPoints[newContactIndex].rigidBodyIndex1 = y; // only one rigid body

					//Sum up closing velocities, include rotational induced velocity.
					//The collision response must then resolve the remaining delta velocity.
					{
						glm::quat& angVelQ = scene.angularVelocities[y];
						glm::vec3 angVel = glm::vec3(angVelQ.x, angVelQ.y, angVelQ.z);
						contactPoints[newContactIndex].closingVelocity = glm::cross(angVel, contactPoints[newContactIndex].contactPosition - scene.positions[y]) + scene.linearVelocities[y];
					}

					{
						glm::quat& angVelQ = scene.angularVelocities[index];
						glm::vec3 angVel = glm::vec3(angVelQ.x, angVelQ.y, angVelQ.z);

						contactPoints[newContactIndex].closingVelocity -= glm::cross(angVel, contactPoints[newContactIndex].contactPosition - scene.positions[index]) + scene.linearVelocities[index];
					}

				}
				scene.active[index] = true;
				scene.active[y] = true;
				
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
		
		if (p.rigidBodyIndex2 != -1)
		{
			//Calculate velocity per unit torque
			glm::vec3 relPoint = p.contactPosition - scene.positions[p.rigidBodyIndex2];
			glm::vec3 torquePerUnit = glm::cross(relPoint, p.contactNormal);
			glm::mat3 rotationMat = glm::toMat3(scene.orientations[p.rigidBodyIndex2]);
			glm::vec3 angVelPerUnit = rotationMat * torquePerUnit;
			angVelPerUnit = scene.inverseInertiaTensors[p.rigidBodyIndex2] * angVelPerUnit;
			angVelPerUnit = glm::inverse(rotationMat) * angVelPerUnit;
			glm::vec3 linearVelPerUnit = glm::cross(angVelPerUnit, relPoint);


			//float linFromAngPerUnitImpulse = glm::max(glm::dot(linearVelPerUnit, p.contactNormal), 0.0f);
			float linFromAngPerUnitImpulse = glm::dot(linearVelPerUnit, p.contactNormal);
			velPerUnitImpulse += scene.inverseMasses[p.rigidBodyIndex2] + linFromAngPerUnitImpulse;
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

		
		//If two rigid bodies are colliding
		if (p.rigidBodyIndex2 != -1)
		{
			//Calculate impulse
			glm::vec3 impulse = -1.0f * desiredVelocity / velPerUnitImpulse;

			//Update Angular velocity
			glm::vec3 relPoint = p.contactPosition - scene.positions[p.rigidBodyIndex2];
			glm::vec3 impulsiveTorque = glm::cross(relPoint, impulse);
			glm::mat3 rotationMat = glm::toMat3(scene.orientations[p.rigidBodyIndex2]);
			glm::vec3 rotationChange = rotationMat * impulsiveTorque;
			rotationChange = scene.inverseInertiaTensors[p.rigidBodyIndex2] * rotationChange;
			rotationChange = glm::inverse(rotationMat) * rotationChange;

			//Update linear velocity
			glm::vec3 linearImpulse = scene.inverseMasses[p.rigidBodyIndex2] * impulse;

			scene.linearVelocities[p.rigidBodyIndex2] += linearImpulse;
			scene.angularVelocities[p.rigidBodyIndex2] += glm::quat(0, rotationChange.x, rotationChange.y, rotationChange.z);

		}
		
	}
}



void resolveInterpenetration(Scene3D& scene)
{
	//@TODO: temporary hack to test if it is the penetration that messes things up: It does

	float maxPenetration = 0.0f;
	for (int i = 0; i < numContacts; i++)
	{
		ContactPoint& p = contactPoints[i];
		if (p.penetrationDepth > glm::length(scene.penetrations[p.rigidBodyIndex1]) && scene.active[p.rigidBodyIndex1])
			scene.penetrations[p.rigidBodyIndex1] = p.penetrationDepth * p.contactNormal * 0.5f;

		if (p.rigidBodyIndex2 != -1)
		{
			if (p.penetrationDepth > glm::length(scene.penetrations[p.rigidBodyIndex2]) && scene.active[p.rigidBodyIndex2])
				scene.penetrations[p.rigidBodyIndex2] = -1.0f * p.penetrationDepth * p.contactNormal * 0.5f;
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
	glm::vec3 angularVelocity = glm::vec3(0);

	numContacts = 0;
	//@Note: hack to test multiple objects, need to resolve each penetration
	scene.penetrations.clear();
	int numObjects = scene.inverseMasses.size();


	for (int i = 0; i < numObjects; i++)
	{
		scene.penetrations.push_back(glm::vec3(0.0f));//same hack as @Note, this is to reset the penetration depth to 0.

		if (scene.inverseMasses[i] != -1.0f)
		{
			//If it has an inverse mass then it's a rigid body.
			if (!scene.active[i])continue;
				
			applyForces(scene, i);

			calculateCollisions(scene, i);
		}
	}

	resolveCollisions(scene);

	resolveInterpenetration(scene);

	//Update objects
	for (int i = 0; i < numObjects; i++)
	{
		if (scene.inverseMasses[i] != -1.0f)
		{

			

			if (glm::length(scene.linearVelocities[i]) < 0.01f && glm::length(fixedTimeStep / 2.0f * scene.angularVelocities[i] * scene.orientations[i]) < 0.005f)
			{
				scene.active[i] = false;
			}

			if(scene.active[i])
			{
				//velocity -> updated position
				scene.positions[i] += scene.linearVelocities[i] * fixedTimeStep;

				//Update orientation
				scene.orientations[i] = glm::normalize(scene.orientations[i] + fixedTimeStep / 2.0f * scene.angularVelocities[i] * scene.orientations[i]);
			}

		}
	}
}

void physicsDrawIMGUI()
{
	ImGui::Text("Physics");
	ImGui::SliderFloat3("Gravity", &gravity[0], -10.0f, 10.0f, "%.2f");
	ImGui::SliderFloat("Velocity dampening", &dampening, 0.5f, 1.0f, "%.2f");
	ImGui::SliderFloat("Restitution", &restitution, 0.1f, 2.0f, "%.2f");
}



};