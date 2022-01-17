/* Author: Suhan Park */


#include <motion_bench_maker/local_occupancy_grid.h>

using namespace robowflex;

Eigen::Vector3d LocalOccupancyGrid::convertIndexToPos(int i, int j, int k)
{
    Eigen::Vector3d pos;
    pos << i * resolutions_[0] - length_arr_[0]/2,
           j * resolutions_[1] - length_arr_[1]/2,
           k * resolutions_[2] - length_arr_[2]/2;
    return pos;
}
void LocalOccupancyGrid::generateFromPointCloud(const CloudXYZPtr cloud, 
                            const RobotPose &cam_pose, 
                            const RobotPose &obj_pose, 
                            bool filled, 
                            double fill_legth)
{
    object_pose_ = obj_pose;
    cam_pose_ = cam_pose;

    for (int i=0; i<3; ++i)
    {
        length_arr_[i] = abs(obj_bounding_coord.first[i] - obj_bounding_coord.second[i]) + near_distance_ * 2;
        resolutions_[i] = length_arr_[i] / n_grid_;
    }
    Eigen::Vector3d local_occu_bounding_coord_start, local_occu_bounding_coord_end;
    local_occu_bounding_coord_start << -length_arr_[0]/2, -length_arr_[1]/2, -length_arr_[2]/2;
    local_occu_bounding_coord_end   <<  length_arr_[0]/2,  length_arr_[1]/2,  length_arr_[2]/2;
    grid_map_.resize(n_grid_,n_grid_,n_grid_);
    grid_map_.setZero();
    local_occu_bounding_coord.first = local_occu_bounding_coord_start;
    local_occu_bounding_coord.second = local_occu_bounding_coord_end;
    
    int indices_start[3];
    int indices_end[3];
    for (int i=0; i<3; ++i)
    {
        indices_start[i] = (obj_bounding_coord.first[i] + length_arr_[i]/2) * n_grid_ / length_arr_[i];
        indices_end[i] = (obj_bounding_coord.second[i] + length_arr_[i]/2) * n_grid_ / length_arr_[i];
        assert(indices_start[i] < n_grid_);
        assert(indices_start[i] >= 0);
        assert(indices_end[i] < n_grid_);
        assert(indices_end[i] >= 0);
    }
    
    std::cout << "obj_pose.matrix()" << std::endl;
    std::cout << obj_pose.matrix() << std::endl;
    // std::cout << "local_occu_bounding_coord_start" << local_occu_bounding_coord_start.transpose() << std::endl;
    // std::cout << "local_occu_bounding_coord_end" << local_occu_bounding_coord_end.transpose() << std::endl;
    
    // local_occu_bounding_coord_start = obj_pose.linear() * local_occu_bounding_coord_start + obj_pose.translation();
    // local_occu_bounding_coord_end = obj_pose.linear() * local_occu_bounding_coord_end + obj_pose.translation();
    
    // for (int i=0; i<3; ++i)
    // {
    //     local_occu_bounding_coord.first(i) = min(local_occu_bounding_coord_start(i), local_occu_bounding_coord_end(i));
    //     local_occu_bounding_coord.second(i) = max(local_occu_bounding_coord_start(i), local_occu_bounding_coord_end(i));
    // }

    for (const auto &p : *cloud)
    {
        /* check for NaN */
        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
            continue;

        Eigen::Vector3d global_point = cam_pose * Eigen::Vector3d{p.x, p.y, p.z};
        Eigen::Vector3d obj_camera_point = obj_pose.inverse() * cam_pose.translation();
        Eigen::Vector3d obj_point = obj_pose.inverse() * global_point;
        
        bool out_of_bound = false;
        for (int i=0; i<3; ++i)
        {
            if (obj_point(i) >= local_occu_bounding_coord_end(i) || 
                obj_point(i) <= local_occu_bounding_coord_start(i))
            {
                out_of_bound = true;
                break;
            }
        }
        if (out_of_bound) continue;

        // std::cin.ignore();
        // std::cout << "obj_point" << obj_point.transpose() << std::endl;
        // std::cout << "inbound" << std::endl;
        // if(obj_point.x() < local_occu_bounding_coord.first(0) ||
        //    p.x > local_occu_bounding_coord.second(0) ||
        //    p.y < local_occu_bounding_coord.first(1) ||
        //    p.y > local_occu_bounding_coord.second(1) ||
        //    p.z < local_occu_bounding_coord.first(2) ||
        //    p.z > local_occu_bounding_coord.second(2))
        //    continue;

        // int i,j,k;
        int indices[3];
        for (int i=0; i<3; ++i)
        {
            indices[i] = (obj_point(i) + length_arr_[i]/2) * n_grid_ / length_arr_[i];
            assert(indices[i] < n_grid_);
            assert(indices[i] >= 0);
        }

        // std::cout << indices[0] << ", " << indices[1] << ", " << indices[2] << std::endl;
        grid_map_(indices[0],indices[1],indices[2]) = 1;

        // transform to camera frame
        // auto point = cam_pose * Eigen::Vector3d{p.x, p.y, p.z};
        // occupied_cells.insert(tree_->coordToKey(point.x(), point.y(), point.z()));
        // fullCloud_->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));

        if (filled)
        {
            // std::cout << "raycast" << std::endl;
            Eigen::Vector3d direction = obj_point - obj_camera_point;
            auto origin = obj_point;
        //     auto point_vec = point - cam_pose.translation();
        //     auto point2 = point + point_vec * fill_legth;
        //     octomap::point3d target_start(point.x(), point.y(), point.z());
        //     octomap::point3d target_end(point2.x(), point2.y(), point2.z());
            // Initialization phase -------------------------------------------------------

            // point3d direction = (end - origin);
            // double length = (double) direction.norm();
            double length = fill_legth;
            direction = direction.normalized(); // normalize vector
            int    current_key[3];
            int    step[3];
            double tMax[3];
            double tDelta[3];

            // OcTreeKey current_key = key_origin; 

            for(unsigned int i=0; i < 3; ++i) {
                current_key[i] = indices[i];
                // compute step direction
                if (direction(i) > 0.0) step[i] =  1;
                else if (direction(i) < 0.0)   step[i] = -1;
                else step[i] = 0;

                // compute tMax, tDelta
                if (step[i] != 0) {
                    // corner point of voxel (in direction of ray)
                    double voxelBorder = (indices[i] * resolutions_[i]) - length_arr_[i]/2;
                    voxelBorder += (float) (step[i] * resolutions_[i] * 0.5);

                    tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
                    tDelta[i] = resolutions_[i] / fabs( direction(i) );
                }
                else {
                    tMax[i] =  std::numeric_limits<double>::max( );
                    tDelta[i] = std::numeric_limits<double>::max( );
                }
            }

            // Incremental phase  ---------------------------------------------------------

            bool done = false;
            while (!done) {
                unsigned int dim;

                // find minimum tMax:
                if (tMax[0] < tMax[1]){
                    if (tMax[0] < tMax[2]) dim = 0;
                    else                   dim = 2;
                }
                else {
                    if (tMax[1] < tMax[2]) dim = 1;
                    else                   dim = 2;
                }

                // advance in direction "dim"
                current_key[dim] += step[dim];
                tMax[dim] += tDelta[dim];

                // assert (current_key[dim] < 2*this->tree_max_val);

                // reached endpoint, key equv?
                if (current_key[dim] == n_grid_ || current_key[dim] == 0) {
                    done = true;
                    break;
                }
                else {

                    // reached endpoint world coords?
                    // dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
                    double dist_from_origin = std::min(std::min(tMax[0], tMax[1]), tMax[2]);
                    // if this is longer than the expected ray length, we should have already hit the voxel containing the end point with the code above (key_end).
                    // However, we did not hit it due to accumulating discretization errors, so this is the point here to stop the ray as we would never reach the voxel key_end
                    if (dist_from_origin > length) {
                        done = true;
                        break;
                    }
                    
                    else {  // continue to add freespace cells
                        // std::cout << current_key[0] << ", "<< current_key[1] << ", "<< current_key[2] << ", " << std::endl;
                        // if (current_key[0] )
                        grid_map_(current_key[0],current_key[1],current_key[2]) = 2;
                        // std::cout << current_key[0] << ", " << current_key[1] << ", " << current_key[2] << std::endl;
                    // ray.addKey(current_key);
                    }
                }                    
            } // end while

        }

        // if (filled)
        // {
        //     // this fills unvisible area
        //     auto point_vec = point - cam_pose.translation();
        //     auto point2 = point + point_vec * fill_legth;
        //     octomap::point3d target_start(point.x(), point.y(), point.z());
        //     octomap::point3d target_end(point2.x(), point2.y(), point2.z());
        // if (tree_->computeRayKeys(target_start, target_end, key_ray_))
        //         occupied_cells.insert(key_ray_.begin(), key_ray_.end());
        // }
    }

    for (int i=indices_start[0]; i<=indices_end[0] + 1; i++)
    for (int j=indices_start[1]; j<=indices_end[1] + 1; j++)
    for (int k=indices_start[2]; k<=indices_end[2] + 1; k++)
    {
        grid_map_(i,j,k) = 0;
    }

    // voxel_grid_.setInputCloud(cloud);
    // voxel_grid_.setLeafSize
}

void LocalOccupancyGrid::setNearDistance(double near_distance) { near_distance_ = near_distance; } 

void LocalOccupancyGrid::setGridNumber(double n_grid) { n_grid_ = n_grid; } 

void LocalOccupancyGrid::setObjectBoundingCoordinates(const Eigen::Vector3d & start, const Eigen::Vector3d & end)
{
    obj_bounding_coord.first = start;
    obj_bounding_coord.second = end;
}

const Eigen::Tensor<int, 3> & LocalOccupancyGrid::getGridMap() { return grid_map_; }

Eigen::Vector3d LocalOccupancyGrid::getVoxelPos(int i, int j, int k)
{
    Eigen::Vector3d rel_pos, pos;
    rel_pos = convertIndexToPos(i,j,k);
    pos = object_pose_ * rel_pos;
    return pos;
}