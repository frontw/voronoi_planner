#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
/*********************************************************************
 *
 *
 *
 *********************************************************************/
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>

#include <tuple>
#include <vector>
#include "dynamicvoronoi.h"

#include <voronoi_planner/VoronoiPlannerConfig.h>

namespace voronoi_planner {

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the voronoi_planner planner on a costmap.
 */

class VoronoiPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         */
        VoronoiPlanner();

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        VoronoiPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~VoronoiPlanner();

        /**
         * @brief  Initialization function for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        bool findPath(std::vector<std::pair<float, float> > *path,
                      int init_x, int init_y,
                      int goal_x, int goal_y,
                      DynamicVoronoi *voronoi,
                      bool check_is_voronoi_cell,
                      bool stop_at_voronoi );

        void smoothPath(std::vector<std::pair<float, float> > *path);


        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

        void publishVoronoiGrid(DynamicVoronoi *voronoi);

    protected:

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        bool initialized_;

        bool publish_voronoi_grid_;
        ros::Publisher voronoi_grid_pub_;

        bool smooth_path_;
        float weight_data_;
        float weight_smooth_;



    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);

        double planner_window_x_, planner_window_y_, default_tolerance_;
        std::string tf_prefix_;
        boost::mutex mutex_;
        ros::ServiceServer make_plan_srv_;

        DynamicVoronoi voronoi_;

        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
        unsigned char* cost_array_;
        unsigned int start_x_, start_y_, end_x_, end_y_;


        dynamic_reconfigure::Server<voronoi_planner::VoronoiPlannerConfig> *dsrv_;
        void reconfigureCB(voronoi_planner::VoronoiPlannerConfig &config, uint32_t level);
        void costmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);

};

} //end namespace voronoi_planner

#endif
