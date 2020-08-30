/*********************************************************************
 *
 * Author: Roman Fedorenko
 *
 *********************************************************************/
#include <voronoi_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(voronoi_planner::VoronoiPlanner, nav_core::BaseGlobalPlanner)

namespace voronoi_planner {


    void visualize(const char *filename, DynamicVoronoi* voronoi, bool** map,
                   std::vector<std::pair<float, float> > *path) {
        // write pgm files

        FILE* F = fopen(filename, "w");
        if (!F) {
            std::cerr << "visualize: could not open file for writing!\n";
            return;
        }
        fprintf(F, "P6\n");
        fprintf(F, "%d %d 255\n", voronoi->getSizeX(), voronoi->getSizeY());



        for(int y = voronoi->getSizeY()-1; y >=0; y--){
            for(int x = 0; x<voronoi->getSizeX(); x++){
                unsigned char c = 0;
                if (voronoi->isVoronoi(x,y) && map[x][y] != 1) {
                    fputc( 255, F );
                    fputc( 0, F );
                    fputc( 0, F );
                } else if (map[x][y] == 1 ) {
                    fputc( 255, F );
                    fputc( 255, F );
                    fputc( 255, F );
                }
                else
                {
                    fputc( 0, F );
                    fputc( 0, F );
                    fputc( 0, F );
                }
            }
        }


        fclose(F);
    }



void VoronoiPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

VoronoiPlanner::VoronoiPlanner() :
        costmap_(NULL), initialized_(false), publish_voronoi_grid_(true),
        smooth_path_ (true), weight_data_ (0.5), weight_smooth_ (0.3) {
}

VoronoiPlanner::VoronoiPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), publish_voronoi_grid_(true),
        smooth_path_ (true), weight_data_ (0.5), weight_smooth_ (0.3)
{
    //initialize the planner
    initialize(name, costmap, frame_id);
}

VoronoiPlanner::~VoronoiPlanner() {

}

void VoronoiPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void VoronoiPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();


        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        voronoi_grid_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("voronoi_grid", 1);


        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &VoronoiPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<voronoi_planner::VoronoiPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<voronoi_planner::VoronoiPlannerConfig>::CallbackType cb = boost::bind(
                &VoronoiPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);


        ros::Subscriber costmapUpdateSubscriber = private_nh.subscribe("/move_base/global_costmap/costmap_updates", 10, &VoronoiPlanner::costmapUpdateCallback, this);



        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void VoronoiPlanner::costmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
    ROS_INFO("Map update. x %d; y %d; w %d; h %d", msg->x, msg->y, msg->width, msg->height);
}


void VoronoiPlanner::reconfigureCB(voronoi_planner::VoronoiPlannerConfig& config, uint32_t level) {
    weight_data_ = config.weight_data;
    weight_smooth_ = config.weight_smooth;

    publish_voronoi_grid_ = config.publish_voronoi_grid;
    smooth_path_ = config.smooth_path;
}

void VoronoiPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool VoronoiPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void VoronoiPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    float convert_offset_ = 0;
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool VoronoiPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    float convert_offset_ = 0;
    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool VoronoiPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool VoronoiPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    worldToMap(wx, wy, start_x, start_y);


    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    worldToMap(wx, wy, goal_x, goal_y);


    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);





    bool **map=NULL;
    int sizeX, sizeY;

    sizeX = costmap_->getSizeInCellsX();
    sizeY = costmap_->getSizeInCellsY();


    map = new bool*[sizeX];

//    ROS_INFO("Map size is %d %d", sizeX, sizeY);

    ros::Time t = ros::Time::now();
    ros::Time t_b = ros::Time::now();


    for (int x=0; x<sizeX; x++) {
        (map)[x] = new bool[sizeY];
    }

    for (int y=sizeY-1; y>=0; y--) {
        for (int x=0; x<sizeX; x++) {
            unsigned char c = costmap_->getCost(x,y);

            if ( c == costmap_2d::FREE_SPACE || c == costmap_2d::NO_INFORMATION )
                (map)[x][y] = false; // cell is free
            else (map)[x][y] = true;// cell is occupied
        }
    }

    ROS_INFO("Time (for map convert): %f sec", (ros::Time::now() - t).toSec());
    t = ros::Time::now();


    bool doPrune = true;


    // initialize voronoi object it with the map

    ROS_INFO("voronoi.initializeMap");
    voronoi_.initializeMap(sizeX, sizeY, map);
    ROS_INFO("Time (for initializeMap): %f sec", (ros::Time::now() - t).toSec());
    t = ros::Time::now();



    ROS_INFO("voronoi.update");
    voronoi_.update(); // update distance map and Voronoi diagram
    ROS_INFO("Time (for update): %f sec", (ros::Time::now() - t).toSec());
    t = ros::Time::now();



    ROS_INFO("voronoi.prune");
    if (doPrune) voronoi_.prune();  // prune the Voronoi
    ROS_INFO("Time (for prune): %f sec", (ros::Time::now() - t).toSec());
    t = ros::Time::now();



//    ROS_INFO("voronoi.visualize");
//    voronoi_.visualize("/tmp/initial.ppm");
//    ROS_INFO("Time (for visualize): %f sec", (ros::Time::now() - t).toSec());


    std::cerr << "Generated initial frame.\n";



    std::vector<std::pair<float, float> > path1;
    std::vector<std::pair<float, float> > path2;
    std::vector<std::pair<float, float> > path3;

//    start_x = 10;
//    start_y = 100;
//    goal_x = 300;
//    goal_y = 330;


    std::cout << "start_x,start_y " << start_x <<
              " " << start_y << std::endl;
    std::cout << "goal_x,goal_y " << goal_x <<
              " " << goal_y << std::endl;


    bool res1 = false, res2 = false, res3 = false;

    if( !voronoi_.isVoronoi(goal_x,goal_y) )
    {
        //        path3 = findPath( goal, init, A, 0, 1 );
        res3 = findPath( &path3, goal_x, goal_y, start_x, start_y, &voronoi_, 0, 1 );
        std::cout << "findPath 3 res " << res3 << std::endl;
        //        goal = path3(end,:);
        goal_x = std::get<0>( path3[path3.size()-1] );
        goal_y = std::get<1>( path3[path3.size()-1] );

        std::cout << "voronoi.isVoronoi(goal_x,goal_y) " << voronoi_.isVoronoi(goal_x,goal_y) << std::endl;


        //        path3 = flipud(path3);
        std::reverse(path3.begin(), path3.end());
    }

    if( !voronoi_.isVoronoi(start_x,start_y) )
    {
        res1 = findPath( &path1, start_x, start_y, goal_x, goal_y, &voronoi_, 0, 1 );
        std::cout << "findPath 1 res " << res1 << std::endl;
        start_x = std::get<0>( path1[path1.size()-1] );
        start_y = std::get<1>( path1[path1.size()-1] );

        std::cout << "voronoi.isVoronoi(start_x,start_y) " << voronoi_.isVoronoi(start_x,start_y) << std::endl;
    }

    res2 = findPath( &path2, start_x, start_y, goal_x, goal_y, &voronoi_, 1, 0 );
    std::cout << "findPath 2 res " << res2 << std::endl;


    if(!(res1 && res2 && res3))
    {
        ROS_INFO("Failed to find full path");
    }
//    else
//    {

//    path = [path1;path2;path3];
    path1.insert( path1.end(), path2.begin(), path2.end() );
    path1.insert( path1.end(), path3.begin(), path3.end() );


    for(int i = 0; i < path1.size(); i++)
    {
        int x = std::get<0>(path1[i]);
        int y = std::get<1>(path1[i]);


//        std::cout << "[" << x << "; " <<
//                     y << "]" << std::endl;

        if(x > 0 && y > 0)
            map[x][y] = 1;
    }

//    if(smooth_path_){
//        smoothPath(&path1);
//    }

//    visualize("/tmp/plan.ppm", &voronoi_, map, &path1);


    for(int i = 0; i < path1.size(); i++)
    {

        geometry_msgs::PoseStamped new_goal = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

        new_goal.pose.position.x = std::get<0>(path1[i]);
        new_goal.pose.position.y = std::get<1>(path1[i]);

        mapToWorld(new_goal.pose.position.x, new_goal.pose.position.y,
                   new_goal.pose.position.x, new_goal.pose.position.y);

//        std::cout << "[" << new_goal.pose.position.x << "; " <<
//                     new_goal.pose.position.y << "]" << std::endl;



        new_goal.pose.orientation.x = goal_quat.x();
        new_goal.pose.orientation.y = goal_quat.y();
        new_goal.pose.orientation.z = goal_quat.z();
        new_goal.pose.orientation.w = goal_quat.w();
        plan.push_back( new_goal );
    }

    // add orientations if needed
//    orientation_filter_->processPath(start, plan);

//    }

    ROS_ERROR("\nTime to get plan: %f sec\n", (ros::Time::now() - t_b).toSec());


    //publish the plan for visualization purposes
    publishPlan(plan);

    if(publish_voronoi_grid_){
        publishVoronoiGrid(&voronoi_);
    }

//    delete potential_array_;

    for (int x = 0; x < sizeX; x++) {
        delete[] map[x];
    }
    delete[] map;

    return !plan.empty();
}

bool VoronoiPlanner::findPath(std::vector<std::pair<float, float> > *path,
              int init_x, int init_y,
              int goal_x, int goal_y,
              DynamicVoronoi *voronoi,
              bool check_is_voronoi_cell,
              bool stop_at_voronoi )
{
//    ROS_INFO("init_x %d, init_y %d, goal_x %d, goal_y %d, check_is_voronoi_cell %d, stop_at_voronoi %d", init_x, init_y, goal_x, goal_y, check_is_voronoi_cell, stop_at_voronoi);
//    ROS_INFO("isVoronoi(init) %d; isVoronoi(goal) %d", voronoi->isVoronoi(init_x, init_y), voronoi->isVoronoi(goal_x, goal_y) );
    // available movements (actions) of the robot on the grid
    std::vector<std::pair<int, int> > delta;
    delta.push_back( {-1, 0} );     // go up
    delta.push_back( {0, -1} );     // go left
    delta.push_back( {1, 0} );      // go down
    delta.push_back( {0, 1} );      // go right
    delta.push_back( {-1, -1} );    // up and left
    delta.push_back( {-1, 1} );     // up and right
    delta.push_back( {1, -1} );     // down and left
    delta.push_back( {1,  1} );     // down and right

    // cost of movement
    float cost = 1;

    // grid size
    unsigned int sizeX = voronoi->getSizeX();
    unsigned int sizeY = voronoi->getSizeY();

    // closed cells grid (same size as map grid)
    bool **closed=NULL;
    closed = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) {
        (closed)[x] = new bool[sizeY];
    }

    for (int y=sizeY-1; y>=0; y--) {
        for (int x=0; x<sizeX; x++) {
            (closed)[x][y] = false;
        }
    }

    //heuristic = zeros(szA(1), szA(2));
    //for(i=1:szA(1))
    //    for(j=1:szA(2))
    //        heuristic(i,j) = norm( [i - goal(1); j - goal(2)] );
    //    end
    //end

    // actions (number of delta's row) cells grid (same size as map grid)
    int **action=NULL;
    action = new int*[sizeX];
    for (int x=0; x<sizeX; x++) {
        (action)[x] = new int[sizeY];
    }
    for (int y=sizeY-1; y>=0; y--) {
        for (int x=0; x<sizeX; x++) {
            (action)[x][y] = -1;
        }
    }

    // set current cell
    int x = init_x;
    int y = init_y;

    // set cost
    float g = 0;

    //f = heuristic(x,y) + g;

    // vector of open (for possible expansion) nodes
    std::vector<std::tuple<float, int, int> > open;
    open.push_back( std::make_tuple( g, x, y ) );

    // path found flag
    bool found = false;
    // no solution could be found flag
    bool resign = false;

    while( !found && !resign )
    {
        if (open.size() == 0)
        {
            resign = true;
            path->empty();
            return false;
        }
        else
        {
            // sort open by cost
            sort(open.begin(), open.end());
            reverse(open.begin(), open.end());
            // get node with lowest cost
            std::tuple<float, int, int> next = open[open.size()-1];
            open.pop_back();
            g = std::get<0>(next);
            x = std::get<1>(next);
            y = std::get<2>(next);

            // check, whether the solution is found (we are at the goal)
            if(stop_at_voronoi)
            {
                // if stop_at_voronoi is set, we stop, when get path to any voronoi cell
                if(voronoi->isVoronoi(x,y))
                {
                    found = true;
                    goal_x = x;
                    goal_y = y;
                    continue;
                }
            }
            else
            {
                if ( x == goal_x && y == goal_y )
                {
                    found = true;
                    continue;
                }
            }
            for( int i=0; i < delta.size(); i++ )
            {
                // expansion
                int x2 = x + std::get<0>(delta[i]);
                int y2 = y + std::get<1>(delta[i]);

                // check new node to be in grid bounds
                if ( x2 >= 0 && x2 < sizeX && y2 >= 0 && y2 < sizeY )
                {
                    // check new node not to be in obstacle
                    if(voronoi->isOccupied(x2,y2))
                    {
                        continue;
                    }
                    // check new node was not early visited
                    if ( closed[x2][y2] ){
                        continue;
                    }

                    // check new node is on Voronoi diagram
                    if (!voronoi->isVoronoi(x2,y2) && check_is_voronoi_cell){
                        continue;
                    }

                    float g2 = g + cost;
                    //                        f2 = heuristic(x2,y2) + g2;
                    open.push_back( std::make_tuple( g2, x2, y2 ) );
                    closed[x2][y2] = true;
                    action[x2][y2] = i;
                }
            }
        }
    }

    // Make reverse steps from goal to init to write path
    x = goal_x;
    y = goal_y;

    int i = 0;
    path->clear();

    while( x != init_x || y != init_y )
    {
        path->push_back({x,y});
        i++;

        int x2 = x - std::get<0>( delta[ action[x][y] ] );
        int y2 = y - std::get<1>( delta[ action[x][y] ] );


        x = x2;
        y = y2;
    }

    reverse(path->begin(), path->end());

    for (int x = 0; x < sizeX; x++) {
        delete[] closed[x];
    }
    delete[] closed;

    for (int x = 0; x < sizeX; x++) {
        delete[] action[x];
    }
    delete[] action;

    return true;
}




void VoronoiPlanner::smoothPath(std::vector<std::pair<float, float> > *path)
{
    // Make a deep copy of path into newpath
    std::vector<std::pair<float, float> > newpath = *path;

    float tolerance = 0.00001;
    float change = tolerance;

    if(path->size() < 2)
        return;

    while (change >= tolerance)
    {

        change = 0.0;
        for(int i = 1; i < path->size() - 1; i++)
        {
            float aux_x = newpath[i].first;
            float aux_y = newpath[i].second;


            float newpath_x = newpath[i].first + weight_data_ * ( path->at(i).first - newpath[i].first);
            float newpath_y = newpath[i].second + weight_data_ * ( path->at(i).second - newpath[i].second);


            newpath_x = newpath_x + weight_smooth_ *
                    (newpath[i-1].first + newpath[i+1].first - (2.0 * newpath_x));
            newpath_y = newpath_y + weight_smooth_ *
                    (newpath[i-1].second + newpath[i+1].second - (2.0 * newpath_y));

            change = change + fabs(aux_x - newpath_x);
            change = change + fabs(aux_y - newpath_y);

            newpath[i] = std::make_pair(newpath_x,newpath_y);
        }
    }
    *path = newpath;
}


void VoronoiPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()) {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = ros::Time::now();
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

void VoronoiPlanner::publishVoronoiGrid(DynamicVoronoi *voronoi)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    ROS_WARN("costmap sx = %d,sy = %d, voronoi sx = %d, sy = %d", nx, ny,
             voronoi->getSizeX(), voronoi->getSizeY());

    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    for (unsigned int x = 0; x < nx; x++)
    {
        for (unsigned int y = 0; y < ny; y++)
        {
            if(voronoi->isVoronoi(x,y))
                grid.data[x + y*nx] = 128;
            else
                grid.data[x + y*nx] = 0;
        }
    }
    voronoi_grid_pub_.publish(grid);
}



} //end namespace voronoi_planner

