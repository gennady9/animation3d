
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

int main(int argc, char* argv[])
{
	Display* disp = new Display(1000, 800, "Wellcome");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer;
	viewer.load_meshes_from_config_file("configuration.txt");
	Init(*disp);
	renderer.init(&viewer);
	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);

	delete disp;
}

//viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse-master/tutorial/data/sphere.obj");
//viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse-master/tutorial/data/cube.obj");
//viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse-master/tutorial/data/bunny.off");