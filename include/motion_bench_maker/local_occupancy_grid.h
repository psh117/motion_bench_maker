/* Author: Suhan Park */

#ifndef ROBOWFLEX_DATASET_LOCAL_OCCUPANCY_GRID_
#define ROBOWFLEX_DATASET_LOCAL_OCCUPANCY_GRID_

// C++
#include <cstring>

// Robowflex
#include <robowflex_library/class_forward.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/planning.h>
#include <moveit/robot_state/robot_state.h>
#include <pcl_ros/point_cloud.h>

// Yaml
#include <yaml-cpp/yaml.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>

#include <unsupported/Eigen/CXX11/Tensor>


namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(LocalOccupancyGrid);
    /** \endcond */

    class LocalOccupancyGrid
    {
    public:

        typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
        typedef std::shared_ptr<CloudXYZ> CloudXYZPtr;
        void generateFromPointCloud(const CloudXYZPtr cloud, 
                                    const RobotPose &cam_pose, 
                                    const RobotPose &obj_pose, 
                                    bool filled, 
                                    double fill_legth);
        Eigen::Vector3d convertIndexToPos(int i, int j, int k);
        void setNearDistance(double near_distance);
        void setGridNumber(double n_grid);
        void setObjectBoundingCoordinates(const Eigen::Vector3d & start, const Eigen::Vector3d & end);

        const Eigen::Tensor<int, 3> & getGridMap();

        int getGridNum() { return n_grid_; };
        double* getLength() { return length_arr_; };
        double* getResolution() { return resolutions_; };
        Eigen::Vector3d getVoxelPos(int i, int j, int k);

    private:
        std::string name;        ///< Name of the grid
        RobotPose cam_pose_;
        RobotPose object_pose_;     ///< Object pose with respect to global frame.
        std::pair<Eigen::Vector3d,Eigen::Vector3d> obj_bounding_coord; ///< bounding coordinates to remove object in grid
        std::pair<Eigen::Vector3d,Eigen::Vector3d> local_occu_bounding_coord; ///< bounding coordinates to remove object in grid
        double near_distance_ {0.2};      ///< default is 20 cm
        int n_grid_ {16};
        double length_arr_[3];
        double resolutions_[3];
        Eigen::Tensor<int, 3> grid_map_;
    };
}

#endif
