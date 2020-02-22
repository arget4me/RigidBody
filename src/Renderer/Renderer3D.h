#ifndef RENDERER3D_HEADER
#define RENDERER3D_HEADER

#include "Game/Scene.h"
namespace ArgetRenderer
{

extern unsigned int GLOBAL_BOUND_MODEL;
	
void renderScene(Scene3D& scene);

void setupRenderer();

void rendererDrawIMGUI();


};
#endif