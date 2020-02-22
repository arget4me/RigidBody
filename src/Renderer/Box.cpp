#include "Box.h"
#include "Renderer/Renderer3D.h"
#include "glm/glm.hpp"
#include "Utils/logfile.h"
namespace ArgetRenderer
{

	unsigned int GLOBAL_BOX_ID = 0;

	const unsigned int GLOBAL_BOX_COLOR = 0;

	const unsigned int GLOBAL_BOX_NUM_INDICIES = 36;

void uploadBox() 
{

	glGenVertexArrays(1, &GLOBAL_BOX_ID);
	glBindVertexArray(GLOBAL_BOX_ID);

	unsigned int VBO_position;
	unsigned int VBO_EBO;

	float points[] = {
		// A cube has 8 vertices, but now we have three copies of each one:
		-0.5, -0.5, -0.5, //0 0
		-0.5, -0.5, -0.5, //0 1
		-0.5, -0.5, -0.5, //0 2
		//
		-0.5, -0.5,  0.5, //1 3
		-0.5, -0.5,  0.5, //1 4
		-0.5, -0.5,  0.5, //1 5
		//
		-0.5,  0.5, -0.5, //2 6
		-0.5,  0.5, -0.5, //2 7
		-0.5,  0.5, -0.5, //2 8
		//
		-0.5,  0.5,  0.5, //3 9
		-0.5,  0.5,  0.5, //3 10
		-0.5,  0.5,  0.5, //3 11
		//
		0.5, -0.5, -0.5, //4 12
		0.5, -0.5, -0.5, //4 13
		0.5, -0.5, -0.5, //4 14
		//
		0.5, -0.5,  0.5, //5 15
		0.5, -0.5,  0.5, //5 16
		0.5, -0.5,  0.5, //5 17
		//
		0.5,  0.5, -0.5, //6 18
		0.5,  0.5, -0.5, //6 19
		0.5,  0.5, -0.5, //6 20
		//
		0.5,  0.5,  0.5, //7 21
		0.5,  0.5,  0.5, //7 22
		0.5,  0.5,  0.5, //7 23
	};

	unsigned int faces[] = {
		// ... and 12 triangular faces, 
		// defined by the following vertex indices:
		0, 9, 6, // 0 3 2
		0, 3, 9, // 0 1 3
		//
		1, 7, 18, // 0 2 6
		1, 18, 12, // 0 6 4
		//
		13, 19, 15, // 4 6 5
		15, 19, 21, // 5 6 7
		//
		16, 22, 10, // 5 7 3
		16, 10, 4, // 5 3 1
		//
		8, 11, 23, // 2 3 7
		8, 23, 20, // 2 7 6
		//
		2, 14, 5, // 0 4 1
		5, 14, 17 // 1 4 5
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

void useBoxModel()
{
	if (GLOBAL_BOX_ID == 0)
	{

		DEBUG_LOG("Upload Box Model!\n\n\n\n");
		uploadBox();
	}
	else {
		if (GLOBAL_BOUND_MODEL == GLOBAL_BOX_ID) return;
	}
	//DEBUG_LOG("useBoxModel!"); DEBUG_LOG(GLOBAL_BOX_ID); DEBUG_LOG("\n");
	glBindVertexArray(GLOBAL_BOX_ID);
	GLOBAL_BOUND_MODEL = GLOBAL_BOX_ID;
}

};