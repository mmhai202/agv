// AStar.cpp
#include "AStar.h"
#include <algorithm>

void AStar::begin() {
  // Tạo danh sách các node ArUco (ID 0 đến 11 với vị trí x, y)
  for (int i = 0; i < 12; i++) {
    Node* node = new Node();
    node->id = i;
    node->x = i % 4;  // 4 cột
    node->y = i / 4;  // 3 hàng
    node->gCost = 100;  // Chi phí gCost khởi tạo lớn
    node->hCost = 0;
    node->parent = nullptr;
    _grid.push_back(node);
  }
}

void AStar::reset() {
  for (Node* n : _grid) {
    n->gCost    = 100;
    n->hCost    = 0;
    n->parent   = nullptr;
  }
}

void AStar::setMission(int Start, int Goal) {
  start = _grid[Start]; 
  goal = _grid[Goal];
}
void AStar::setBlocked(int id) {_grid[id]->isBlocked = true;}

void AStar::clearAllBlocked() {for (Node* node : _grid) node->isBlocked = false;}

int AStar::_heuristic(Node* current, Node* goal) {return abs(current->x - goal->x) + abs(current->y - goal->y);}

std::vector<Node*> AStar::findPath(Node* start, Node* goal) {
  std::priority_queue<Node*, std::vector<Node*>, CompareNode> openList;
  std::map<int, bool> closedList;

  start->gCost = 0;
  start->hCost = _heuristic(start, goal);
  openList.push(start);

  while (!openList.empty()) {
    Node* current = openList.top();
    openList.pop();

    if (current == goal) {
      std::vector<Node*> path;
      while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    closedList[current->id] = true;

    // Duyệt qua các node lân cận (lên, xuống, trái, phải)
    std::vector<std::pair<int, int>> neighbors = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};  // Các hướng di chuyển
    for (auto& dir : neighbors) {
      int newX = current->x + dir.first;
      int newY = current->y + dir.second;

      // Kiểm tra nếu node mới hợp lệ (trong phạm vi bản đồ và không phải vật cản)
      if (newX >= 0 && newX < 4 && newY >= 0 && newY < 3) {
        Node* neighbor = _grid[newX + newY * 4];  // Lấy node lân cận

        if (neighbor->isBlocked || closedList[neighbor->id]) continue;  // Nếu node là vật cản hoặc đã thăm, bỏ qua

        int tentativeGCost = current->gCost + 1;  // Chi phí tạm thời từ node hiện tại đến neighbor
        if (tentativeGCost < neighbor->gCost || !closedList[neighbor->id]) {
          neighbor->parent = current;
          neighbor->gCost = tentativeGCost;
          neighbor->hCost = _heuristic(neighbor, goal);
          openList.push(neighbor);  // Thêm neighbor vào openList
        }
      }
    }
  }

  return {};  // Không tìm thấy đường đi
}
