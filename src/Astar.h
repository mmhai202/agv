// AStar.h
#pragma once
#include <vector>
#include <map>
#include <queue>

struct Node {
  int id;
  int x, y;
  int gCost;
  int hCost;
  int fCost() { return gCost + hCost; }
  Node* parent;
  bool isBlocked = false;
};

struct CompareNode {
  bool operator()(Node* a, Node* b) {
    return a->fCost() > b->fCost();  // So sánh fCost của các node
  }
};

class AStar {
public:
  Node* start = nullptr;
  Node* goal = nullptr;
  void begin();
  void setMission(int Start, int Goal);
  void setBlocked(int id);
  void clearAllBlocked();
  std::vector<Node*> findPath(Node* start, Node* goal);

private:
  int _heuristic(Node* current, Node* goal);
  std::vector<Node*> _grid;
};
