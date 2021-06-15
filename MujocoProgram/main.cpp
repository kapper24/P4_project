
#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "vector"
#include "iostream"
#include <fstream>
#include <string>

// load and compile model
char error[1000] = "Could not load binary model";
char HelloObjectXML[]{ "C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\MujocoProgram\\hello.xml" };
char activateSoftware[]{ "C:\\Users\\Melvin\\Documents\\mujoco Licesne\\mjkey.txt" };
char readFromPCLPath[]{ "C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\MujocoProgram\\ReadfromPCL.txt" };
int n;
float diameter = 5;
float orientation = 90;
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
std::vector<float> handpos = { 0, 0, 0 };
std::vector<float> wineglass = { -2.8, 0, 0, 0.4, 0.53, 0, -0.33, 0.44, 0.30, 0.37, 1.6, 0, 1.6 };
std::vector<float> cup = { -1.63, 0.40, 0.32, 0.4, 0.57, 0, 0, 0.34, 0.27, 1.6, 1.6, 0, 1.6 };
std::vector<float> can = { -1.54, 0, -0.32, 1.55, 0, 0, 0, 0.07, 0.24, 0.37, 0.34, 0, 0.24 };
std::vector<float> Idle = { -1.73,0.05,0.00,1.03,0.43,0.00,0.00,0.00,0.30,0.40,0.34,0.00,0.34 };
std::vector<float> wineClose = { -3.10, 0, 0, 0.90, 0.66, 0.15, -0.37, 0.34, 0.46, 0.53, 1.60, 0, 1.6 };
std::vector<float> triClose = { -1.60, 0, 0, 1.55, 1, 0.79, -0.82, 0, 0.75, 0.88, 1.6, 0, 1.6 };
std::vector<float> triOpen = { -1.60, 0, 0, 1.62, 1, 0.49, -0.82, 0, 0.51, 0.64, 1.6, 0, 1.6 };
std::vector<float> targetpos;
int aMujoco = 1;
bool keychange = 1;
// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
int openHand = 0;
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
	// backspace: reset simulation
	if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
	{
		mj_resetData(m, d);
		mj_forward(m, d);
		handpos = { 0, 0, 0 };
	}
	if (act == GLFW_PRESS && key == GLFW_KEY_A)
	{
		handpos[0] -= 75;
	}
	if (act == GLFW_PRESS && key == GLFW_KEY_D)
	{
		handpos[0] += 75;
	}
	if (act == GLFW_PRESS && key == GLFW_KEY_W)
	{
		handpos[1] += 75;
	}
	if (act == GLFW_PRESS && key == GLFW_KEY_S)
	{
		handpos[1] -= 75;
	}
	if (act == GLFW_PRESS && key == GLFW_KEY_Q)
	{
		handpos[2] -= 75;
	}
	if (act == GLFW_PRESS && key == GLFW_KEY_E)
	{
		handpos[2] += 75;
	}
	if (act == GLFW_PRESS && key == GLFW_KEY_1)
	{
		aMujoco = 1;
		keychange = 1;
	}
	if (act == GLFW_PRESS && key == GLFW_KEY_2)
	{
		aMujoco = 2;
		keychange = 1;
	}
	if (act == GLFW_PRESS && key == GLFW_KEY_3)
	{
		aMujoco = 3;
		keychange = 1;
	}
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
	// update button state
	button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
	button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
	button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

	// update mouse position
	glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
	// no buttons down: nothing to do
	if (!button_left && !button_middle && !button_right)
		return;

	// compute mouse displacement, save
	double dx = xpos - lastx;
	double dy = ypos - lasty;
	lastx = xpos;
	lasty = ypos;

	// get current window size
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// get shift key state
	bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
		glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

	// determine action based on mouse button
	mjtMouse action;
	if (button_right)
		action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	else if (button_left)
		action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	else
		action = mjMOUSE_ZOOM;

	// move camera
	mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	// emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

float calculatefingerjointcylinder(float fingersize, float diameter) {
	float num_fingers_around_cirkle = ((diameter + 2) * 3.14) / fingersize;
	float cornerangles = 180 - (((180 * num_fingers_around_cirkle) - 360) / num_fingers_around_cirkle);
	float angle_radians = cornerangles * 0.017453;
	return angle_radians;
}

std::vector<float> calculate_aperture_closingcylinder(float diameter, std::vector<float> currenthandpos) {
	std::vector<float> newhandpos = currenthandpos;
	newhandpos[8] = calculatefingerjointcylinder(2.6, diameter);
	newhandpos[9] = calculatefingerjointcylinder(3, diameter);
	newhandpos[10] = calculatefingerjointcylinder(2.7, diameter);
	newhandpos[12] = calculatefingerjointcylinder(2, diameter);
	return newhandpos;
}

std::vector<float> calculate_aperture_closing_cup(float cupheight) {
	std::vector<float> newhandpos;
	if (cupheight < 8)
	{
		newhandpos = { -1.60, 0, 0, 0, 0, 0, 0, 0, 0.13, 1.6, 1.6, 0, 1.6 };
	}
	else if (cupheight >= 8 && cupheight <= 12)
	{
		newhandpos = { -1.60, 0, 0, 0, 0, 0, 0, 0, 0.13, 0.18, 1.6, 0, 1.6 };
	}
	else if (cupheight > 12)
	{
		newhandpos = { -1.60, 0, 0, 0, 0, 0, 0, 0, 0.13, 0.18, 0.10, 0, 1.6 };
	}
	return newhandpos;
}

std::vector<float> close_hand_cup(std::vector<float> oldpos) {
	std::vector<float> newhandpos;
	if (oldpos[10] < 1.6)
	{
		newhandpos = { -1.60, 0, 0, 0.52, 1, 0.51, 0, 0, 0.91, 1.02, 0.99, 0, 1.6 };
	}
	else if (oldpos[9] < 1.6)
	{
		newhandpos = { -1.60, 0, 0, 0.52, 1, 0.51, 0, 0, 0.91, 1.02, 1.6, 0, 1.6 };
	}
	else if (oldpos[8] < 1.6)
	{
		newhandpos = { -1.60, 0, 0, 0.52, 1, 0.51, 0, 0, 0.91, 1.6, 1.6, 0, 1.6 };
	}
	return newhandpos;
}

std::vector<float> calculate_aperture_closing_sphere(float diameter, std::vector<float> currenthandpos) {
	std::vector<float> newhandpos = currenthandpos;
	newhandpos[7] = 0.22;
	newhandpos[8] = calculatefingerjointcylinder(2.6, diameter) + 0.15;
	newhandpos[9] = calculatefingerjointcylinder(3, diameter);
	newhandpos[10] = calculatefingerjointcylinder(2.7, diameter);
	newhandpos[11] = 0.22;
	newhandpos[12] = calculatefingerjointcylinder(2, diameter) + 0.15;
	return newhandpos;
}

float fixorient(float zeroorient, float objectorient) {
	objectorient *= 0.017;
	float handorient = zeroorient + objectorient;
	return handorient;
}

int main(int argc, char *argv[])
{
	// activate software
	mj_activate(activateSoftware);
	m = mj_loadXML(HelloObjectXML, 0, error, 1000);
	if (!m)
		mju_error_s("Load model error: %s", error);
	d = mj_makeData(m);
	if (!glfwInit())
		mju_error("Could not initialize GLFW");
	GLFWwindow* Gwindow = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
	glfwMakeContextCurrent(Gwindow);
	glfwSwapInterval(1);

	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

	// create scene and context
	mjv_makeScene(m, &scn, 2000);
	mjr_makeContext(m, &con, mjFONTSCALE_150);
	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(Gwindow, keyboard);
	glfwSetCursorPosCallback(Gwindow, mouse_move);
	glfwSetMouseButtonCallback(Gwindow, mouse_button);
	
	while (!glfwWindowShouldClose(Gwindow))
	{// advance interactive simulation for 1/60 sec
		//  Assuming MuJoCo can simulate faster than real-time, which it usually can,
		//  this loop will finish on time for the next frame to be rendered at 60 fps.
		//  Otherwise add a cpu timer and exit this loop when it is time to render.
		std::string line;
		std::ifstream f(readFromPCLPath);
			while (getline(f, line)) {
				if (line == "Grasp 1") {
					n = 1;
					line = "";
					std::cout << "Grasp " << n << std::endl;
				}
				else if (line == "Grasp 2") {
					n = 2;
					line = "";
					std::cout << "Grasp " << n << std::endl;
				}
				else if (line == "Grasp 3") {
					n = 3;
					line = "";
					std::cout << "Grasp " << n << std::endl;
				}
				else if (line == "Grasp 4") {
					n = 4;
					line = "";
					std::cout << "Grasp " << n << std::endl;
				}
				else if (line == "Grasp 5") {
					n = 5;
					line = "";
					std::cout << "Grasp " << n << std::endl;
				}
				else if (line == "NULL") {
					n = 1;
					line = "";
					std::cout << "Grasp " << n << std::endl;
				}
				else if (line == "Open") {
					std::cout << "Open" << std::endl;
					openHand = 1;
				}
				else if (line == "Close") {
					std::cout << "Close" << std::endl;
					openHand = 0;
				}
				else if (line == "Idle") {
					std::cout << "idle" << std::endl;
					n = 6;
				}
				else if (line.find("diameter") != std::string::npos) {
					std::string number = line.substr(9, line.size());
					diameter = std::stof(number) * 100;
					std::cout << "diameter:" << diameter << "\n";
				}
				else if (line.find("orientation") != std::string::npos) {
					std::string number = line.substr(12, line.size());
					orientation = std::stof(number)-90;
					std::cout << "orientation:" << orientation << "\n";
				}
			};
				//std::cout << line << std::endl;
			
			
		mjtNum simstart = d->time;
		while (d->time - simstart < 1 / 60.0)

			mj_step(m, d);
		if (openHand == 1) {
			switch (n)
			{
			case 1:
				targetpos = calculate_aperture_closingcylinder(diameter, can);
				targetpos[0] = fixorient(can[0], orientation);
				keychange = 0;
				break;
			case 2:
				targetpos = wineglass;
				targetpos[0] = fixorient(wineglass[0], orientation);
				keychange = 0;
				break;
			case 3:
				targetpos = calculate_aperture_closing_cup(diameter);
				targetpos[0] = fixorient(cup[0], orientation);
				keychange = 0;
				break;
			case 4:
				targetpos = calculate_aperture_closing_sphere(diameter, can);
				targetpos[0] = fixorient(can[0], 90);
				keychange = 0;
				break;
			case 5:
				targetpos = triOpen;
				targetpos[0] = fixorient(triOpen[0], orientation);
				keychange = 0;
				break;
			case 6:
				targetpos = Idle;
				keychange = 0;
			default:
				break;
			}
		}
		else if(openHand == 0) {
			switch (n)
			{
			case 1:
			
				targetpos[4] = 1.03;
				targetpos[0] = fixorient(can[0], orientation);
				keychange = 0;
				break;
			case 2:
				targetpos = wineClose;
				targetpos[0] = fixorient(wineClose[0], orientation);
				keychange = 0;
				break;
			case 3:
				targetpos = calculate_aperture_closing_cup(diameter);
				targetpos = close_hand_cup(targetpos);
				targetpos[0] = fixorient(cup[0], orientation);
				keychange = 0;
				break;
			case 4:
				targetpos[4] = 1.03;
				targetpos[0] = fixorient(can[0], 90);
				keychange = 0;
				break;
			case 5:
				targetpos = triClose;
				targetpos[0] = fixorient(triClose[0], orientation);
				keychange = 0;
				break;
			case 6:
				targetpos = Idle;
				keychange = 0;
			default:
				break;
			}
		}

		/*if (keychange == 1) {
			switch (aMujoco)
			{
			case 2:
				targetpos = wineglass;
				keychange = 0;
				break;
			case 3:
				targetpos = cup;
				keychange = 0;
				break;
			case 1:
				targetpos = can;
				keychange = 0;
				break;
			default:
				break;
			}
		}*/
		for (int i = 0; i < 13; i++)
		{

			if (d->ctrl[i] > targetpos[i]) {
				d->ctrl[i] -= 0.01;
			}
			if (d->ctrl[i] < targetpos[i]) {
				d->ctrl[i] += 0.01;
			}
		}

		for (int j = 13; j < 16; j++) {
			if (d->ctrl[j] > handpos[j - 13]) {
				d->ctrl[j] -= 4;
			}
			if (d->ctrl[j] < handpos[j - 13]) {
				d->ctrl[j] += 4;
			}
		}


		// get framebuffer viewport
		mjrRect viewport = { 0, 0, 0, 0 };
		glfwGetFramebufferSize(Gwindow, &viewport.width, &viewport.height);

		// update scene and render
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		// swap OpenGL buffers (blocking call due to v-sync)
		glfwSwapBuffers(Gwindow);

		// process pending GUI events, call GLFW callbacks
		glfwPollEvents();
	}
	if (glfwWindowShouldClose(Gwindow)) {
		//free visualization storage
		mjv_freeScene(&scn);
		mjr_freeContext(&con);

		// free MuJoCo model and data, deactivate
		mj_deleteData(d);
		mj_deleteModel(m);
		mj_deactivate();

		// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
		glfwTerminate();
#endif
	}

    return 0;
}