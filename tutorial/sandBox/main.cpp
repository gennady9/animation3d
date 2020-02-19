
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include <Windows.h>
#include <mmsystem.h>
//#include "imgui/imgui.h"

#define VIEWPORT_WIDTH 1000
#define VIEWPORT_HEIGHT 800

int main(int argc, char* argv[])
{

	Display* disp = new Display(VIEWPORT_WIDTH, VIEWPORT_HEIGHT, "Animation project 2020");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer;
	viewer.load_meshes_from_config_file("../../../configuration.txt");

	Init(*disp);
	renderer.init(&viewer);

	disp->SetRenderer(&renderer);
	unsigned int left_view = 0, right_view;
	renderer.core().viewport = Eigen::Vector4f(0, 0, VIEWPORT_WIDTH, VIEWPORT_HEIGHT);
	igl::opengl::glfw::Viewer* scn = renderer.GetScene();
	
	right_view = renderer.append_core(Eigen::Vector4f((VIEWPORT_WIDTH / 4) * 3, VIEWPORT_HEIGHT / 5, VIEWPORT_WIDTH / 4 * 1, VIEWPORT_HEIGHT / 5));
	renderer.core(right_view).camera_center = Vector3f(0, 1, 0);
	renderer.core(right_view).camera_eye = Vector3f(0,0,0);
	renderer.core(right_view).camera_up = Vector3f(0, 0, 1);

	renderer.setSelectedCore(left_view);
	viewer.MyTranslate(Vector3f(0, -5, -25)); // Camera initalization
	viewer.startLevel();
	disp->launch_rendering(true);

	delete disp;
}