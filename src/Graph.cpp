#include "Graph.h"

Graph::Graph() {
  size_ = 0;
  nodes_.clear();
}

std::shared_ptr<Node> Graph::add(const Point2D &vertex) {
  auto node = find(vertex);
  if(node == nullptr) {
    node.reset(new Node(vertex));
    nodes_.push_back(node);
    size_++;
  }
  return node;
}

void Graph::add(const Point2D &src, const Point2D &dest) {
  auto srcNode = add(src);
  auto destNode = add(dest);
  srcNode->add(destNode);
}

std::shared_ptr<Node> Graph::find(const Point2D &vertex) {
  for(auto& node : nodes_) {
    if(node->getRoot() == vertex)
      return node;
  }
  return nullptr;
}

std::shared_ptr<Node> Graph::find(const std::shared_ptr<Node> &node) {
  for(auto& n : nodes_) {
    if(n->getRoot() == node->getRoot())
      return n;
  }
  return nullptr;
}

int Graph::getSize() {
  return size_;
}

std::vector<std::shared_ptr<Node>> Graph::getNodes() {
  return nodes_;
}

bool Graph::getPath(std::shared_ptr<Node> src, std::shared_ptr<Node> dest, std::vector<Point2D>& path) {
  if(src == nullptr || dest == nullptr)
    return false;
  if (src->getRoot() == dest->getRoot()) {
    path.emplace_back(src->getRoot());
    return true;
  }
  if (!src->getLinks().empty()) {
    for (auto &link : src->getLinks()) {
      if (getPath(link, dest, path)) {
        path.emplace_back(src->getRoot());
        return true;
      }
    }
  }
  return false;
}
