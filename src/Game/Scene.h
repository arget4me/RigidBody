#ifndef SCENE_HEADER
#define SCENE_HEADER
#include <vector>
#include <glm/glm.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace ArgetRenderer
{



	typedef struct Scene {
		std::vector<glm::vec3> positions;
		std::vector<glm::vec3> sizes;
		std::vector<glm::quat> orientations;
		std::vector<unsigned int> modelID;
		std::vector<unsigned int> color_index;
		std::vector<glm::quat> angularVelocities;
		std::vector<glm::vec3> linearVelocities;
		std::vector<glm::mat4> inverseInertiaTensors;
		std::vector<float> inverseMasses;
	}Scene3D;


	inline glm::quat quatIdentity() {
		return { 0, 0, 0, 1 };
	};

	void setupScene(Scene3D& scene);

	void addObject(Scene3D& scene, const unsigned int modelID, const glm::vec3 objectPosition, const glm::vec3 const size, glm::quat objectOrientation, const unsigned int color_index);

	void scaleObject(Scene3D& scene, unsigned int ID, const glm::vec3 scale);

	void setRigidBody(Scene3D& scene, unsigned int ID, const float inverseMass, const glm::mat4 inverseInertiaTensor);

};
#endif 
