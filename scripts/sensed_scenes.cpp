/* Author: Constantinos Chamzas */

// Robowflex dataset
#include <motion_bench_maker/parser.h>
#include <motion_bench_maker/octomap_generator.h>
#include <motion_bench_maker/setup.h>

// Robowflex library
#include <robowflex_library/util.h>
#include <robowflex_library/yaml.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/io.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/constants.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    ROS ros(argc, argv, "sensed_scenes");
    ros::NodeHandle node("~");

    std::string dataset, sensor_config;
    bool visualize;

    std::string exec_name = "sensed_scenes";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "visualize", visualize);
    parser::shutdownIfError(exec_name, error);

    auto setup = std::make_shared<Setup>(dataset);

    auto octomap_generator = std::make_shared<OctomapGenerator>(setup->getGenParameters()->sensors);
    auto robot = setup->getRobot();
    auto rviz = std::make_shared<IO::RVIZHelper>(robot);

    for (int i = 1; i <= setup->getNumSamples(); i++)
    {
        ROS_INFO("Generating octomap [%d/%d ]", i, setup->getNumSamples());
        auto scene_geom = std::make_shared<Scene>(robot);
        auto scene_sensed = std::make_shared<Scene>(robot);

        setup->loadGeometricScene(i, scene_geom);
        octomap_generator->geomToSensed(scene_geom, scene_sensed, visualize ? rviz : nullptr);

        if (visualize)
        {
            rviz->updateScene(scene_geom);
            rviz->updateScene(scene_sensed);
            rviz->updateMarkers();
            parser::waitForUser("Wait for enter");
        }

        setup->saveSensedScene(i, scene_sensed);
    }

    return 0;
}
