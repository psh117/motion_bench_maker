/* Author: Carlos Quintero, Constantinos Chamzas */

// Robowflex dataset
#include <motion_bench_maker/octomap_generator.h>
#include <motion_bench_maker/parser.h>
#include <motion_bench_maker/problem_generator.h>
#include <motion_bench_maker/scene_sampler.h>
#include <motion_bench_maker/setup.h>

// Robowflex library
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>
#include <robowflex_library/yaml.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;

struct ModenetDataset
{
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
    Eigen::Tensor<int, 3> grid_map;
    int mode;
    int grid_len {16};
    int succeded {0};

    void write(const std::string & file_name)
    {
        std::ofstream of(file_name, std::fstream::app);
        of << succeded << '\t'
           << pos.x()  << '\t' 
           << pos.y()  << '\t' 
           << pos.z()  << '\t' 
           << quat.x() << '\t' 
           << quat.y() << '\t' 
           << quat.z() << '\t' 
           << quat.w() << '\t';

        for (int i=0; i<grid_len; ++i)
        for (int j=0; j<grid_len; ++j)
        for (int k=0; k<grid_len; ++k)
        {
            of << grid_map(i,j,k) << '\t';
        }
        of << std::endl;
        succeded = 0;
    }

    void print_current()
    {
        std::cout << "succeded: " << succeded  << '\n'
                  << "pos: " << pos.transpose()  << '\n'
                  << "quat: " << quat.coeffs().transpose()  << '\n';
    }


    void publish()
    {
        // visualization_msgs::MarkerArray marker_array;
        // marker_array.markers

        // visualization_msgs::Marker marker_delete;
        // marker_delete.header.frame_id = "map";
        // marker_delete.header.stamp = ros::Time();
        // marker_delete.ns = "cube_display";
        // marker_delete.id = 0;
        // marker_delete.type = visualization_msgs::Marker::CUBE_LIST;
        // marker_delete.action = visualization_msgs::Marker::DELETEALL;
        // cube_pub.publish( marker_delete );
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "cube_display";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        // marker.
        // marker.scale.y = 0.01;
        // marker.scale.z = 0.01;
        double dist = 0.1;
        marker.scale.x = dist;
        marker.scale.y = dist;
        marker.scale.z = dist;
        for (int i=0; i<grid_len; ++i)
        for (int j=0; j<grid_len; ++j)
        for (int k=0; k<grid_len; ++k)
        {
            if (grid_map(i,j,k))
            {
                geometry_msgs::Point point;
                point.x = dist * i;
                point.y = dist * j;
                point.z = dist * k;
                marker.points.push_back(point);
            }
        }
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 1.3;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.a = 0.6; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker_array.markers.push_back(marker);
        cube_pub.publish( marker_array );
    }
    ros::Publisher cube_pub;
};

int main(int argc, char **argv)
{
    ROS ros(argc, argv, "generate_learning_dataset");
    ros::NodeHandle node("~");

    std::string dataset, config, planner_name;
    bool solve, visualize, sensed, pointcloud, visualize_sensed, tuning_mode, filled;
    double fill_length;
    int start, end;

    std::string exec_name = "generate_learning_dataset";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "config", config);
    error += !parser::get(exec_name, node, "sensed", sensed);
    error += !parser::get(exec_name, node, "planner_name", planner_name);
    error += !parser::get(exec_name, node, "pointcloud", pointcloud);
    error += !parser::get(exec_name, node, "visualize", visualize);
    error += !parser::get(exec_name, node, "visualize_sensed", visualize_sensed);
    error += !parser::get(exec_name, node, "tuning_mode", tuning_mode);
    error += !parser::get(exec_name, node, "start", start);
    error += !parser::get(exec_name, node, "end", end);
    error += !parser::get(exec_name, node, "filled", filled);
    error += !parser::get(exec_name, node, "fill_length", fill_length);
    parser::shutdownIfError(exec_name, error);

    auto setup = std::make_shared<Setup>(config, dataset);
    auto robot = setup->getRobot();
    auto group = setup->getGroup();

    // Disable hybridization
    auto settings = OMPL::Settings();
    settings.hybridize_solutions = false;
    settings.interpolate_solutions = false;

    // Create planner
    auto planner = setup->createPlanner("planner", settings);
    auto gp = setup->getGenParameters();

    // Nominal scene.
    auto nominal_yaml = std::make_shared<Scene>(robot);
    auto nominal_urdf = std::make_shared<Robot>("scene_robot");

    if (setup->isSceneYaml())
        setup->loadSceneYaml(nominal_yaml);
    else
        setup->loadSceneUrdf(nominal_urdf);

    // Create problem generator.
    auto problem_generator = std::make_shared<ProblemGenerator>(gp->queries);

    // If single tip
    if (gp->tips.empty())
        problem_generator->setParameters(robot, group, gp->ee_offset[0]);
    else
        problem_generator->setParameters(robot, group, gp->ee_offset, gp->tips, gp->ee_dependency);

    // Create an octomap generator.
    auto octomap_generator = std::make_shared<OctomapGenerator>(gp->sensors);
    auto local_occupancy_generator = octomap_generator->getLocalOccupancy();
    local_occupancy_generator->setObjectBoundingCoordinates(Eigen::Vector3d{-0.03,-0.03,-0.06}, Eigen::Vector3d{0.03,0.03,0.06});
    // local_occupancy_generator->setObjectBoundingCoordinates(Eigen::Vector3d{-0.135,-0.323,-0.02}, Eigen::Vector3d{0.135,0.025,0.02});

    // Create scene_sampler
    auto scene_sampler = std::make_shared<SceneSampler>(gp->variation, gp->base_offset);

    // Create default start_state
    auto start_state = std::make_shared<robot_state::RobotState>(*robot->getScratchStateConst());

    // Create RVIZ helper.
    auto rviz = std::make_shared<IO::RVIZHelper>(setup->getRobot());

    int index = start;
    int numSamples = (end > 1) ? end : setup->getNumSamples();

    ModenetDataset modenet_dataset;
    modenet_dataset.cube_pub = node.advertise<visualization_msgs::MarkerArray>("/cube_marker", 0 );

    std::string target_obj_name = "Can1";
    while (index <= numSamples)
    {
        ROS_INFO("Attempting [%d/%d] ....", index, numSamples);

        auto scene_geom = std::make_shared<Scene>(robot);

        // check if the scene is of yaml format, and load it appropriately.
        if (setup->isSceneYaml())
            scene_geom = scene_sampler->sample(nominal_yaml);
        else
            scene_geom = scene_sampler->sample(nominal_urdf, robot);

        auto scene = scene_geom->deepCopy();
        
        if (sensed)
            octomap_generator->geomToSensed(scene_geom, scene, visualize_sensed ? rviz : nullptr, filled, fill_length);

        Eigen::Isometry3d robot_pose_w, obj_pose_w, obj_pose_r;
        obj_pose_w = scene->getObjectPose(target_obj_name);
        robot_pose_w = scene->getFramePose("panda_link0");
        
        obj_pose_r = robot_pose_w.inverse() * obj_pose_w;
        modenet_dataset.grid_map = local_occupancy_generator->getGridMap();
        modenet_dataset.pos = obj_pose_r.translation();
        modenet_dataset.quat = Eigen::Quaterniond(obj_pose_r.linear());

        modenet_dataset.print_current();
        problem_generator->updateScene(scene);

        // If a start query is not provided use the default start_state.
        auto result = problem_generator->getNumberOfStartObjectQueries() == 0 ?
                          problem_generator->createRandomRequestWithStartState(start_state) :
                          problem_generator->createRandomRequest();
        std::cout << "res: " << result.second << std::endl;

        modenet_dataset.succeded = result.second;
        std::cout << "writing...\n" ;
        modenet_dataset.print_current();
        modenet_dataset.write("test_file.tsv");
        modenet_dataset.publish();
        rviz->updateScene(scene);
        if(result.second)
            rviz->visualizeState(result.first->getGoalConfiguration());
        else
            rviz->visualizeState(result.first->getStartConfiguration());
        rviz->updateScene(scene);

                parser::waitForUser("Displaying initial state!");

        continue;
        if (result.second)
        {
            // Try to solve with a planner to verify feasibility
            const auto &request = result.first;
            // Add the planner so we know that the correct config exists
            request->setPlanner(planner);
            request->setAllowedPlanningTime(60);
            request->setNumPlanningAttempts(2);

            if (!request->setConfig(planner_name))
                ROS_ERROR("Did not find planner %s", planner_name.c_str());

            // scene->getScene()->setCurrentState()

            const auto &res = planner->plan(scene, request->getRequestConst());

            if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                modenet_dataset.succeded = 1;
                auto trajectory = std::make_shared<Trajectory>(*res.trajectory_);

                rviz->updateScene(scene);
                // Visualize start state.
                rviz->visualizeState(request->getStartConfiguration());
                parser::waitForUser("Displaying initial state!");

                // Visualize goal state.
                rviz->visualizeState(request->getGoalConfiguration());
                parser::waitForUser("Displaying goal state!");

                // Visualize the trajectory.
                rviz->updateTrajectory(trajectory->getTrajectory());
                parser::waitForUser("Displaying The trajectory!");

                // save request, scene,  trajectory
                setup->saveRequest(index, request);
                setup->saveTrajectory(index, trajectory);
                setup->saveGeometricScene(index, scene_geom);
                // This saveas attached correctly but does not work with the octomap
                // setup->saveGeometricScene(index, scene);

                // Remove all the geometric objects from the scene.
                if (sensed)
                {
                    for (const auto &obj : scene->getCollisionObjects())
                        if (obj != "<octomap>")
                            scene->removeCollisionObject(obj);
                    setup->saveSensedScene(index, scene);
                    // Save the pointcloud as well
                    if (pointcloud)
                        setup->savePCDScene(index, octomap_generator->getLastPointCloud());
                }

                index++;
            }
            else
            {
                modenet_dataset.succeded = 0;
            }
        }
        if (tuning_mode)
        {
            rviz->updateScene(scene);
            rviz->visualizeState(robot->getScratchStateConst());
            rviz->removeAllMarkers();
            for (int i = 0; i < problem_generator->getLastQueryPose().size(); i++)
                rviz->addTransformMarker("start" + std::to_string(i), "map",
                                         problem_generator->getLastQueryPose()[i]);
            rviz->updateMarkers();
            parser::waitForUser("Displaying the last queried IK pose !");
        }
    }

    setup->saveConfigToDataset();
    return 0;
}
