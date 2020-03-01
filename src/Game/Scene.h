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
		std::vector<glm::mat3> inverseInertiaTensors;
		std::vector<float> inverseMasses;
		std::vector<glm::vec3> penetrations;
	}Scene3D;


	inline const glm::quat quatIdentity() {
		glm::quat q;
		q.x = 0.0f;
		q.y = 0.0f;
		q.z = 0.0f;
		q.w = 1.0f;
		return q;
	};

	void setupScene(Scene3D& scene);

	void addObject(Scene3D& scene, const unsigned int modelID, const glm::vec3 objectPosition, const glm::vec3 size, glm::quat objectOrientation, const unsigned int color_index);

	void scaleObject(Scene3D& scene, unsigned int ID, const glm::vec3 scale);

	void setRigidBody(Scene3D& scene, unsigned int ID, const float mass, const glm::mat3& inertiaTensor);
	
	void setRigidBodyBox(Scene3D& scene, unsigned int ID, const float cubeMass, const float cubeSize);

};
#endif 
