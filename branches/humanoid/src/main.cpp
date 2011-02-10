#include "Scene.h"
#include <boost/property_tree/ini_parser.hpp>
#include <slon/Engine.h>

using namespace math;

std::string graphicsModel;
std::string physicsModel;
std::string configFile;

void ReadConfigFile(const char* fileName, Scene::DESC& sceneDesc)
{
    boost::property_tree::ptree properties;
    boost::property_tree::read_ini(fileName, properties);

    sceneDesc.windowWidth    = properties.get("WindowWidth", 800);
    sceneDesc.windowHeight   = properties.get("WindowHeight", 600);
    sceneDesc.multisample    = properties.get("Multisample", 0);
    sceneDesc.ffp            = properties.get("FixedFuntionPipeline", false);
    sceneDesc.fullscreen     = properties.get("Fullscreen", false);
    sceneDesc.arenaFile      = properties.get("Arena", "");
    graphicsModel            = properties.get("Model", "");
    physicsModel             = properties.get("PhysicsModel", graphicsModel);
    graphicsModel            = properties.get("GraphicsModel", graphicsModel);
    configFile               = properties.get("ConfigFile", "");

    std::string controlType  = properties.get("Control", "Chain");
    if (controlType == "Chain") { 
        sceneDesc.control.type = Scene::CHAIN_CONTROL;
    }
    else if (controlType == "Humanoid") { 
        sceneDesc.control.type = Scene::HUMANOID_CONTROL;
    }
}

void ParseCommandLine(int argc, char** argv, Scene::DESC& desc)
{
    for (int i = 0; i<argc; ++i)
    {
        std::string argStr(argv[i]);
        if (argStr == "-model" && i < argc + 1) {
            graphicsModel = physicsModel = "Data/Models/" + std::string(argv[i+1]);
        }
    }
}

int main(int argc, char** argv)
{
    Scene::DESC desc;
    ReadConfigFile("Data/Config/Scene.ini", desc);

    try
    {
        ParseCommandLine(argc, argv, desc);
    }
    catch(std::exception& err)
    {
        std::cerr << err.what() << std::endl;
        return 1;
    }

    // init engine
    Engine* engine = Engine::Instance();
	engine->init();

    try
    {
		desc.control.graphicsModel = graphicsModel.c_str();
		desc.control.physicsModel  = physicsModel.c_str();
		desc.control.config	       = configFile.c_str();
        desc.control.transform     = math::make_translation(0.0f, 0.0f, 0.0f);

        Scene demoScene(desc);
    }
    catch(std::exception& err)
    {
        std::cerr << err.what() << std::endl;
        return 1;
    }

	return 0;
}
