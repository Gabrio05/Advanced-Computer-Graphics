#include "GEMLoader.h"
#include "Renderer.h"
#include "SceneLoader.h"
#define NOMINMAX
#include "GamesEngineeringBase.h"
#include <unordered_map>

int main(int argc, char *argv[])
{	
	// Initialize default parameters
	std::string sceneName;
	//sceneName = "../cornell-box";
	//sceneName = "../MaterialsScene";  // WARNING OVERSATURED WHITE
	//sceneName = "../MoreScenes/bathroom";  // WARNING ARTEFACTS ON WALLS
	//sceneName = "../MoreScenes/bathroom2";  // WARNING NO LIGHT FROM WINDOW ON MIRROR
	//sceneName = "../MoreScenes/bedroom";
	sceneName = "../MoreScenes/car2";  // ERROR FREEZING
	//sceneName = "../MoreScenes/classroom";  // ERROR FREEZING
	//sceneName = "../MoreScenes/coffee";  // INFO POTENTIAL PROBLEM BLACK SQUARE BELOW
	//sceneName = "../MoreScenes/dining-room";  // ERROR FREEZING
	//sceneName = "../MoreScenes/glass-of-water";  // INFO NO WATER REFLECTION
	//sceneName = "../MoreScenes/house";
	//sceneName = "../MoreScenes/kitchen";
	//sceneName = "../MoreScenes/living-room";  // ERROR FREEZING
	//sceneName = "../MoreScenes/living-room2";  // ERROR BLACK SCREEN
	//sceneName = "../MoreScenes/living-room3";  // ERROR BLACK SCREEN
	//sceneName = "../MoreScenes/materialball";  // ERROR BLACK SCREEN (86 secs render)
	//sceneName = "../MoreScenes/Sibenik";  // ERROR FREEZING
	//sceneName = "../MoreScenes/Sponza";  // ERROR BLACK SCREEN
	//sceneName = "../MoreScenes/staircase";
	//sceneName = "../MoreScenes/staircase2";
	//sceneName = "../MoreScenes/teapot-full";  // ERROR FREEZING
	//sceneName = "../MoreScenes/Terrain";  // ERROR FREEZING
	//sceneName = "../MoreScenes/veach-bidir";
	//sceneName = "../MoreScenes/veach-mis";
	
	std::string filename = "GI.hdr";
	int SPP = 8192;

	if (argc > 1)
	{
		std::unordered_map<std::string, std::string> args;
		for (int i = 1; i < argc; ++i)
		{
			std::string arg = argv[i];
			if (!arg.empty() && arg[0] == '-')
			{
				std::string argName = arg;
				if (i + 1 < argc)
				{
					std::string argValue = argv[++i];
					args[argName] = argValue;
				} else
				{
					std::cerr << "Error: Missing value for argument '" << arg << "'\n";
				}
			} else
			{
				std::cerr << "Warning: Ignoring unexpected argument '" << arg << "'\n";
			}
		}
		for (const auto& pair : args)
		{
			if (pair.first == "-scene")
			{
				sceneName = pair.second;
			}
			if (pair.first == "-outputFilename")
			{
				filename = pair.second;
			}
			if (pair.first == "-SPP")
			{
				SPP = stoi(pair.second);
			}
		}
	}
	Scene* scene = loadScene(sceneName);
	GamesEngineeringBase::Window canvas;
	canvas.create((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, "Tracer", false);
	RayTracer rt;
	rt.init(scene, &canvas);
	bool running = true;
	GamesEngineeringBase::Timer timer;
	while (running)
	{
		canvas.checkInput();
		canvas.clear();
		if (canvas.keyPressed(VK_ESCAPE))
		{
			break;
		}
		if (canvas.keyPressed('W'))
		{
			viewcamera.forward();
			rt.clear();
		}
		if (canvas.keyPressed('S'))
		{
			viewcamera.back();
			rt.clear();
		}
		if (canvas.keyPressed('A'))
		{
			viewcamera.left();
			rt.clear();
		}
		if (canvas.keyPressed('D'))
		{
			viewcamera.right();
			rt.clear();
		}
		if (canvas.keyPressed('E'))
		{
			viewcamera.flyUp();
			rt.clear();
		}
		if (canvas.keyPressed('Q'))
		{
			viewcamera.flyDown();
			rt.clear();
		}
		// Time how long a render call takes
		timer.reset();
		rt.render();
		float t = timer.dt();
		// Write
		std::cout << t << std::endl;
		if (canvas.keyPressed('P'))
		{
			rt.saveHDR(filename);
		}
		if (canvas.keyPressed('L'))
		{
			size_t pos = filename.find_last_of('.');
			std::string ldrFilename = filename.substr(0, pos) + ".png";
			rt.savePNG(ldrFilename);
		}
		if (SPP == rt.getSPP())
		{
			rt.saveHDR(filename);
			break;
		}
		canvas.present();
	}
	return 0;
}