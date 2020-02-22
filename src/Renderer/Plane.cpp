#include "Plane.h"
#include "Renderer/Renderer3D.h"
#include "glm/glm.hpp"
#include "Utils/logfile.h"
namespace ArgetRenderer
{

	unsigned int GLOBAL_PLANE_ID = 0;

	const unsigned int GLOBAL_PLANE_COLOR = 1;


	const unsigned int GLOBAL_PLANE_NUM_INDICIES = 6;

	void uploadPlane()
	{

		glGenVertexArrays(1, &GLOBAL_PLANE_ID);
		glBindVertexArray(GLOBAL_PLANE_ID);

		unsigned int VBO_position;
		unsigned int VBO_EBO;

		float points[] = {
			// A cube has 8 vertices, but now we have three copies of each one:
			-0.5, +0.5, 0, //0 Top - left
			+0.5, +0.5, 0, //1 Top - Right
			-0.5, -0.5, 0, //2 Bottom - Left
			+0.5, -0.5, 0, //3 Bottom - right
		};

		unsigned int faces[] = {
			 2, 1, 0, //Top left Triangle
			 2, 3, 1, //Bottom right Triangle
		};

		glGenBuffers(1, &VBO_position);
		glBindBuffer(GL_ARRAY_BUFFER, VBO_position);
		glBufferData(GL_ARRAY_BUFFER, sizeof(points), points, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		glGenBuffers(1, &VBO_EBO);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, VBO_EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(faces), faces, GL_STATIC_DRAW);
	}

	void usePlaneModel()
	{
		if (GLOBAL_PLANE_ID == 0)
		{

			DEBUG_LOG("Upload Plane Model!\n\n\n\n");
			uploadPlane();
		}
		else {
			if (GLOBAL_BOUND_MODEL == GLOBAL_PLANE_ID) return;
		}
		//DEBUG_LOG("usePlaneModel!"); DEBUG_LOG(GLOBAL_PLANE_ID); DEBUG_LOG("\n");
		glBindVertexArray(GLOBAL_PLANE_ID);
		GLOBAL_BOUND_MODEL = GLOBAL_PLANE_ID;
	}

};