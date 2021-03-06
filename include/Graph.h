#ifndef PLANNER_GRAPH_H
#define PLANNER_GRAPH_H

#include <vector>
#include <memory>


class Point2D
{
public:
  Point2D(): x_(0), y_(0) {}
  Point2D(int x, int y): x_(x), y_(y) {}

  int x() const
  {
    return x_;
  }

  int y() const
  {
    return y_;
  }

  void x(int x)
  {
    x_ = x;
  }

  void y(int y)
  {
    y_ = y;
  }

  bool operator==(const Point2D &rhs) {
    return (rhs.x() == x_ && rhs.y() == y_);
  }

  bool operator!=(const Point2D &rhs) {
    return !(*this == rhs);
  }

private:
  int x_;
  int y_;
};

class Node {

 public:

  Node() {
    root_ = Point2D(0, 0);
    links_.resize(0);
  }

  explicit Node(const Point2D &root) {
    root_ = Point2D(root);
    links_.resize(0);
  }

  std::shared_ptr<Node> add(const Point2D &link) {
    auto node = find(link);
    if (node == nullptr) {
      node.reset(new Node(link));
      links_.push_back(node);
    }
    return node;
  }

  std::shared_ptr<Node> add(const std::shared_ptr<Node> &link) {
    auto node = find(link->getRoot());
    if (node == nullptr) {
      node = link;
      links_.push_back(node);
    }
    return node;
  }

  std::shared_ptr<Node> find(const Point2D &vertex) {
    for (auto &node : links_) {
      if (node->getRoot() == vertex)
        return node;
    }
    return nullptr;
  }

  Point2D getRoot() {
    return root_;
  }

  std::vector<std::shared_ptr<Node>> getLinks() {
    return links_;
  }

  bool operator==(const Node &rhs) {
    return (this->root_ == rhs.root_);
  }

 private:

  Point2D root_;
  std::vector<std::shared_ptr<Node>> links_;

};

class Graph {

 public:

  Graph();

  std::shared_ptr<Node> add(const Point2D &vertex);

  void add(const Point2D &src, const Point2D &dest);

  std::shared_ptr<Node> find(const Point2D &vertex);

  std::shared_ptr<Node> find(const std::shared_ptr<Node> &node);

  int getSize();

  std::vector<std::shared_ptr<Node>> getNodes();

  bool getPath(std::shared_ptr<Node> src, std::shared_ptr<Node> dest, std::vector<Point2D>& path);

 private:

  std::vector<std::shared_ptr<Node>> nodes_;

  int size_;

};

#endif //PLANNER_GRAPH_H
