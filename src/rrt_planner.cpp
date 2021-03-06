#include "rrt_planner/rrt_planner.h"
#include "Graph.h"

namespace rrt_planner
{

RRTPlanner::RRTPlanner(ros::NodeHandle * node)
: nh_(node),
  private_nh_("~"),
  map_received_(false),
  init_pose_received_(false),
  goal_received_(false)
{
  // Get map and path topics from parameter server
  std::string map_topic, path_topic;
  private_nh_.param<std::string>("map_topic", map_topic, "/map");
  private_nh_.param<std::string>("path_topic", path_topic, "/path");

  // some other params
  private_nh_.param("max_vertices", max_vertices_, 1500);
  private_nh_.param("step_size", step_, 20);

  // Subscribe to map topic
  map_sub_ = nh_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
    map_topic, 1, &RRTPlanner::mapCallback, this);

  // Subscribe to initial pose topic that is published by RViz
  init_pose_sub_ = nh_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
    "/initialpose", 1, &RRTPlanner::initPoseCallback, this);

  // Subscribe to goal topic that is published by RViz
  goal_sub_ = nh_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
    "/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

  // Advertise topic where calculated path is going to be published
  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

  ROS_INFO("Planner node initialized");

  // This loops until the node is running, will exit when the node is killed
  while (ros::ok()) {
    // if map, initial pose, and goal have been received
    // build the map image, draw initial pose and goal, and plan
    if (map_received_ && init_pose_received_ && goal_received_) {
      buildMapImage();
      drawGoalInitPose();
      ROS_INFO("Poses drawn, enter plan function!!!");
      plan();
    } else {
      if (map_received_) {
        displayMapImage();
      }
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }
}

void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr & msg)
{
  map_grid_ = msg;

  // Build and display the map image
  buildMapImage();
  displayMapImage();

  // Reset these values for a new planning iteration
  map_received_ = true;
  init_pose_received_ = false;
  goal_received_ = false;

  ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
}

void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
  if (init_pose_received_) {
    buildMapImage();
  }

  // Convert mas to Point2D
  poseToPoint(init_pose_, msg->pose.pose);

  // Reject the initial pose if the given point is occupied in the map
  if (!isPointUnoccupied(init_pose_)) {
    init_pose_received_ = false;
    ROS_INFO("%d", init_pose_.x());
    ROS_INFO("%d", init_pose_.y());
    ROS_INFO("Grid value: %d", map_grid_->data[init_pose_.x() + (init_pose_.y() - 1)*500]);
    ROS_WARN(
      "The initial pose specified is on or too close to an obstacle please specify another point");
  } else {
    init_pose_received_ = true;
    drawGoalInitPose();
    ROS_INFO("x = %d", init_pose_.x());
    ROS_INFO("y = %d", init_pose_.y());
    ROS_INFO("Grid value: %d", map_grid_->data[init_pose_.x() + (init_pose_.y() - 1)*500]);
    ROS_INFO("Initial pose obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
  if (goal_received_) {
    buildMapImage();
  }

  // Convert msg to Point2D
  poseToPoint(goal_, msg->pose);

  // Reject the goal pose if the given point is occupied in the map
  if (!isPointUnoccupied(goal_)) {
    goal_received_ = false;
    ROS_INFO("%d", goal_.x());
    ROS_INFO("%d", goal_.y());
    ROS_INFO("Grid value: %d", map_grid_->data[goal_.x() + (goal_.y() - 1)*500]);
    ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
  } else {
    goal_received_ = true;
    drawGoalInitPose();
    ROS_INFO("x = %d", goal_.x());
    ROS_INFO("y = %d", goal_.y());
    ROS_INFO("Grid value: %d", map_grid_->data[goal_.x() + (goal_.y() - 1)*500]);
    ROS_INFO("Goal obtained successfully.");
  }

  displayMapImage();
}

void RRTPlanner::drawGoalInitPose()
{
  if (goal_received_) {
    drawCircle(goal_, 3, cv::Scalar(12, 255, 43));
  }
  if (init_pose_received_) {
    drawCircle(init_pose_, 3, cv::Scalar(255, 200, 0));
  }
}

std::vector<Point2D> RRTPlanner::plan()
{
  // Reset these values so planning only happens once for a
  // given pair of initial pose and goal points
  goal_received_ = false;
  init_pose_received_ = false;

  ROS_INFO("*********** Inside plan() function ************");
  std::vector<Point2D> newPoints;

  // Check if goal is reachable from starting position
  newPoints = connectPoints(goal_, init_pose_);
  if (findInPoints(newPoints, goal_)) {
      ROS_INFO("*********** findInPoints == True ************");
    for (int i = 0; i < newPoints.size() - 1; i++) {
      drawLine(newPoints[i], newPoints[i + 1], cv::Scalar(0, 0, 255), 1);
      if (i != 0)
        drawCircle(newPoints[i], 1, cv::Scalar(0, 255, 255));
    }
    cv::imshow("Output", *map_);
    cv::waitKey(1);
    ROS_INFO("*********** Before returning newPoints ************");
    return newPoints;
  }

  graph_.reset(new Graph());
  graph_->add(init_pose_);

  // random integer generator
  std::random_device device;
  std::mt19937 eng(device());

  std::uniform_int_distribution<int> distX(0, map_->cols - 1);
  std::uniform_int_distribution<int> distY(0, map_->rows - 1);

  bool occupied;

  int totalPoints = 0;

  Point2D randomPoint;
  Point2D closestPoint;

  while (graph_->find(goal_) == nullptr && graph_->getSize() < max_vertices_ && totalPoints < 30000) {

    if (totalPoints % 100 == 0)
      ROS_INFO("Random points generated: %d", totalPoints);

    // keep sampling till sampled point is not occupied by an obstacle
    occupied = true;
    while (occupied) {
      randomPoint.x(distX(eng));
      randomPoint.y(distY(eng));
      if (isPointUnoccupied(randomPoint) && graph_->find(randomPoint) == nullptr)
        occupied = false;
    }

    if (totalPoints == 500) {
      randomPoint.x(10);
      randomPoint.y(10);
    }

    // find closest point in the graph to the sampled point
    closestPoint = findClosestPoint(randomPoint);
    newPoints = connectPoints(randomPoint, closestPoint);
    addToGraph(newPoints, randomPoint);

    totalPoints += 1;

    closestPoint = findClosestPoint(goal_);
    newPoints = connectPoints(goal_, closestPoint);
    addToGraph(newPoints, goal_);

    cv::imshow("Output", *map_);
    cv::waitKey(1);
  }

  std::vector<Point2D> path;
  if (graph_->find(goal_) != nullptr) {
    ROS_INFO("Goal found!");
    path = getPath(init_pose_, goal_);
    for (int i = 0; i < path.size() - 1; i++)
      drawLine(path[i], path[i+1], cv::Scalar(0, 0, 255), 1);
  } else
    ROS_INFO("Could not reach goal");

  ROS_INFO("Total Point2D in graph: %d", graph_->getSize());

  cv::imshow("Output", *map_);
  cv::waitKey(100);
  ROS_INFO("Just about to return path");
  return path;
}

Point2D RRTPlanner::findClosestPoint(Point2D point) {
  auto minDist = DBL_MAX;
  double dist;
  Point2D closest;

  for (auto &node : graph_->getNodes()) {

    auto root = node->getRoot();

    if (root == point)
      continue;

    dist = distance(root, point);

    if (dist < minDist) {
      minDist = dist;
      closest = root;
    }
  }

  return closest;
}

std::vector<Point2D> RRTPlanner::connectPoints(Point2D a, Point2D b) {
  std::vector<Point2D> newPoints;
  newPoints.emplace_back(b);

  cv::LineIterator linePoints(*map_, cv::Point(b.x(), b.y()), cv::Point(a.x(), a.y()));
  cv::Point point;
  Point2D v;
  bool blocked = false;
  ROS_INFO("*********** Inside connectPoints function ************");
  ROS_INFO("linePoints.count = %d", linePoints.count);
  for (int i = 0; i < linePoints.count; i++, linePoints++) {
    point = linePoints.pos();

    v.x(point.x);
    v.y(point.y);
    ROS_INFO("X = %d",v.x());
    ROS_INFO("Y = %d",v.y());
    if (!isPointUnoccupied(v)) {
      ROS_INFO("Point in between is occupied!");
      blocked = true;
      break;
    }

    if (distance(v, newPoints[newPoints.size() - 1]) > step_)
      newPoints.emplace_back(v);
  }

  if (!blocked) {
    if (newPoints[newPoints.size() - 1] != a)
      newPoints.emplace_back(a);
  }

  return newPoints;
}

bool RRTPlanner::findInPoints(std::vector<Point2D> &points, Point2D point) {
  for (auto &p : points) {
    if (p == point)
      return true;
  }
  return false;
}

void RRTPlanner::addToGraph(std::vector<Point2D> &newPoints, Point2D point) {
  if (newPoints.size() > 1) {
    int p = 0;
    for (p = 0; p < newPoints.size() - 1; p++) {
      graph_->add(newPoints[p], newPoints[p + 1]);

      if (p != 0)
        drawCircle(newPoints[p], 1, cv::Scalar(0, 255, 255));

      drawLine(newPoints[p], newPoints[p + 1], cv::Scalar(255, 0, 0),1);
    }

    if (findInPoints(newPoints, point))
      drawCircle(point, 1, cv::Scalar(0, 255, 0));
    else
      drawCircle(newPoints[p + 1], 1, cv::Scalar(0, 255, 0));
  }
}

std::vector<Point2D> RRTPlanner::getPath(Point2D src, Point2D dest) {
  auto srcNode = graph_->find(src);
  auto destNode = graph_->find(dest);
  std::vector<Point2D> path;
  graph_->getPath(srcNode, destNode, path);
  std::reverse(path.begin(), path.end());
  return path;
}

double RRTPlanner::distance (Point2D &a, Point2D &b) {
  return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2));
}


bool RRTPlanner::isPointUnoccupied(const Point2D & p)
{
  // we have /init_pose which is Point2D and /map which is nav_msgs::OccupancyGrid
  // understand the 'row-major-ordering' for msg_grid_

  if(map_grid_->data[p.x() * 500 + p.y()] == 100){
    return false;
  } else {
    return true;
  }
}

void RRTPlanner::buildMapImage()
{
  // Create a new opencv matrix with the same height and width as the received map
  map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                              map_grid_->info.width,
                                              CV_8UC3,
                                              cv::Scalar::all(255)));

  // Fill the opencv matrix pixels with the map points
  for (int i = 0; i < map_grid_->info.height; i++) {
    for (int j = 0; j < map_grid_->info.width; j++) {
      if (map_grid_->data[toIndex(i, j)]) {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
      } else {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
}

void RRTPlanner::displayMapImage(int delay)
{
  cv::imshow("Output", *map_);
  cv::waitKey(delay);
}

void RRTPlanner::drawCircle(Point2D & p, int radius, const cv::Scalar & color)
{
  cv::circle(
    *map_,
    cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
    radius,
    color,
    -1);
}

void RRTPlanner::drawLine(Point2D & p1, Point2D & p2, const cv::Scalar & color, int thickness)
{
  cv::line(
    *map_,
    cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
    cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
    color,
    thickness);
}

inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D & p)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = p.y() * map_grid_->info.resolution;
  pose.pose.position.y = p.x() * map_grid_->info.resolution;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_grid_->header.frame_id;
  return pose;
}

inline void RRTPlanner::poseToPoint(Point2D & p, const geometry_msgs::Pose & pose)
{
  p.x(pose.position.y / map_grid_->info.resolution);
  p.y(pose.position.x / map_grid_->info.resolution);
}

inline int RRTPlanner::toIndex(int x, int y)
{
  return x * map_grid_->info.width + y;
}


}  // namespace rrt_planner
