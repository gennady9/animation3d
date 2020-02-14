
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include <Windows.h>
#include <mmsystem.h>
//#include "imgui/imgui.h"

#define VIEWPORT_WIDTH 1000
#define VIEWPORT_HEIGHT 800

int main(int argc, char* argv[])
{
	PlaySound(TEXT("../../../sounds/snake_charmer.wav"), NULL, SND_FILENAME | SND_LOOP | SND_ASYNC);

	Display* disp = new Display(VIEWPORT_WIDTH, VIEWPORT_HEIGHT, "Animation project 2020");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer;
	viewer.load_meshes_from_config_file("../../../configuration.txt");

	viewer.MyTranslate(Vector3f(0, -5, -25)); // Camera initalization
	Init(*disp);
	renderer.init(&viewer);

	disp->SetRenderer(&renderer);
	/* temp camera view
	unsigned int left_view = 0, right_view;
	renderer.core().viewport = Eigen::Vector4f(0, 0, VIEWPORT_WIDTH / 2, VIEWPORT_HEIGHT);
	right_view = renderer.append_core(Eigen::Vector4f(VIEWPORT_WIDTH/2 + 20, 0, VIEWPORT_WIDTH/2 - 20, VIEWPORT_HEIGHT));
	
	renderer.core(right_view).camera_eye = Vector3f(0, 0, 1);
	renderer.core(right_view).camera_center = Vector3f(0, 0, 0);

	renderer.setSelectedCore(left_view);
	*/
	//SoundEngine->play2D("ophelia.mp3", GL_TRUE);
	disp->launch_rendering(true);

	delete disp;
}

// code graveyard
/*

	//renderer.core(right_view).init();
	//renderer.core(right_view).align_camera_center(viewer.data().V, viewer.data().F);
	//renderer.core(right_view).toggle(scn->data_list[i].show_faces);
	//renderer.core().camera_up= Vector3f(0, 1, 0);
	//renderer.core().camera_translation = Vector3f(0, 0, 0);
//viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse-master/tutorial/data/sphere.obj");
//viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse-master/tutorial/data/cube.obj");
//viewer.load_mesh_from_file("C:/Dev/EngineForAnimationCourse-master/tutorial/data/bunny.off");


*/