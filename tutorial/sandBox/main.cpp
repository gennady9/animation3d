
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

#define VIEWPORT_WIDTH 1000
#define VIEWPORT_HEIGHT 800

int main(int argc, char* argv[])
{
	Display* disp = new Display(VIEWPORT_WIDTH, VIEWPORT_HEIGHT, "Welcome");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer;
	viewer.load_meshes_from_config_file("configuration.txt");

	viewer.MyTranslate(Vector3f(0, -5, -25)); // Camera initalization
	unsigned int left_view, right_view;

	
	Init(*disp);
	renderer.init(&viewer);
	/*
	viewer.callback_init = [&](igl::opengl::glfw::Viewer&)
	{
		renderer.core().viewport = Eigen::Vector4f(0, 0, 640, 800);
		left_view = renderer.core().id;
		right_view = renderer.append_core(Eigen::Vector4f(0, 0, 200, 300), false);
		return false;
	};

	int firstView = 0;
	int secondView = renderer.append_core(Vector4f(0, 0,200,300), false);
	//std::cout << "second view = " << secondView << std::endl;
	//renderer.ChangeCamera(firstView);
	*/

	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);

	delete disp;
}

//viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse-master/tutorial/data/sphere.obj");
//viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse-master/tutorial/data/cube.obj");
//viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse-master/tutorial/data/bunny.off");