#ifndef RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
#define RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_

#include <random>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Graph.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <memory>
#include <random>
#include <cmath>

namespace rrt_planner
{

/**
 * Main class which implements the RRT algorithm
 */
class RRTPlanner
{
public:
  explicit RRTPlanner(ros::NodeHandle *);

  ~RRTPlanner() = default;

  // virtual std::vector<Point2D> getPlan() = 0;

  std::vector<Point2D> plan();

  void mapCallback(const nav_msgs::OccupancyGrid::Ptr &);
  void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

  // functions added for RRT algorithm
  Point2D findClosestPoint(Point2D point);

  bool findInPoints(std::vector<Point2D> &newPoints, Point2D point);

  std::vector<Point2D> connectPoints(Point2D a, Point2D b);

  void addToGraph(std::vector<Point2D>& points, Point2D point);

  std::vector<Point2D> getPlan();

  std::vector<Point2D> getPath(Point2D src, Point2D dest);  

  double distance(Point2D& a, Point2D& b);

private:

  // functions defined in the skeleton
  void publishPath();

  bool isPointUnoccupied(const Point2D & p);

  void buildMapImage();

  void displayMapImage(int delay = 1);

  void drawGoalInitPose();

  void drawCircle(Point2D & p, int radius, const cv::Scalar & color);

  void drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness = 1);

  inline geometry_msgs::PoseStamped pointToPose(const Point2D &);

  inline void poseToPoint(Point2D &, const geometry_msgs::Pose &);

  inline int toIndex(int, int);

// params defined in the skeleton
  ros::NodeHandle * nh_;
  ros::NodeHandle private_nh_;

  bool map_received_;
  std::unique_ptr<cv::Mat> map_;
  nav_msgs::OccupancyGrid::Ptr map_grid_;

  bool init_pose_received_;
  Point2D init_pose_;

  bool goal_received_;
  Point2D goal_;

  ros::Subscriber map_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;

// params defined for RRT algorithm
  std::string map_topic_;
  std::string goal_topic_;
  std::string pose_topic_;

  bool pose_set_;

  std::shared_ptr<Graph> graph_;

  int max_vertices_;
  int step_;
};

}

#endif  // RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
