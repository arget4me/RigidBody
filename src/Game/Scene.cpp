#include "Scene.h"
#include <GLEW/glew.h>
#include "Renderer/Box.h"
#include "Renderer/Plane.h"
#include <glm/glm.hpp>
#include "Utils/logfile.h"
#include "Physics/RigidBody.h"


namespace ArgetRenderer
{




#define FALLING_BOX_POS glm::vec3(0.0f, 2.0f, 0.0f)
#define RESTING_BOX_POS glm::vec3(2.0f, -2.0f, 0.0f)
#define FLOOR_POS glm::vec3(0.0f, -2.0f, 0.0f)


	void setupScene(Scene3D& scene)
	{
		addObject(scene, GLOBAL_BOX_ID, FALLING_BOX_POS, glm::vec3(0.5f), quatIdentity(), 0);
		setRigidBody(scene, scene.positions.size() - 1, 1.0f / 10.0f, glm::mat4(1.0f));
		addObject(scene, GLOBAL_BOX_ID, RESTING_BOX_POS, glm::vec3(0.5f), quatIdentity(), 0);
		addObject(scene, GLOBAL_PLANE_ID, FLOOR_POS, glm::vec3(1000.0f), glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)), 1);
	}

	void addObject(Scene3D& scene, const unsigned int modelID, const glm::vec3 objectPosition, const glm::vec3 const size, glm::quat objectOrientation, const unsigned int color_index)
	{
		//Setub objects
		scene.positions.push_back(objectPosition);
		scene.orientations.push_back(objectOrientation);
		scene.modelID.push_back(modelID);
		scene.sizes.push_back(size);
		scene.color_index.push_back(color_index);
		scene.inverseMasses.push_back(-1.0f);
		scene.angularVelocities.push_back(glm::quat(0.0f, 0.0f, 0.0f, 0.0f));
		scene.linearVelocities.push_back(glm::vec3(0.0f));
		scene.inverseInertiaTensors.push_back(glm::mat4(1.0f));
	}

	void scaleObject(Scene3D& scene, unsigned int ID, const glm::vec3 scale)
	{
		if (ID >= scene.sizes.size())return;
		scene.sizes[ID] = scale;
	}

	void setRigidBody(Scene3D& scene, unsigned int ID, const float inverseMass, const glm::mat4 inverseInertiaTensor)
	{
		if (ID >= scene.sizes.size())return;
		scene.inverseMasses[ID] = inverseMass;
		scene.inverseInertiaTensors[ID] = inverseInertiaTensor;

	}
	//check collision with static plane
	bool checkCollisionNoRotation(glm::vec3& boxPos, float boxSize, glm::vec3& planePos)
	{
		if (glm::length(boxPos - planePos) <= boxSize)
			return true;

		return false;
	}

};