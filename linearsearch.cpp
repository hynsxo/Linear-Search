#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <utility>
#include <algorithm>

using namespace std;

// 노드를 표현하는 구조체
struct Node {
    int x_index;       // 현재 x 인덱스
    double y_value;    // 현재 y 값
    double cost;       // 현재까지 누적 비용
    bool operator>(const Node& other) const {
        return cost > other.cost;  // 비용 오름차순 정렬
    }
};

// 다익스트라 알고리즘 함수
vector<pair<double, double>> dijkstra(const vector<double>& x_values, 
                                      const vector<pair<double, double>>& y_ranges, 
                                      double start_y) {
    int n = x_values.size();  // x 값의 개수

    // 최소 비용 및 이전 노드 저장
    vector<vector<double>> dist(n, vector<double>(1000, numeric_limits<double>::infinity()));
    vector<vector<pair<int, double>>> prev(n, vector<pair<int, double>>(1000, {-1, -1.0}));

    // 초기값 유효성 확인
    if (start_y < y_ranges[0].first || start_y > y_ranges[0].second) {
        cout << "초기값 " << start_y << "가 유효하지 않습니다. 시작 범위는 ["
             << y_ranges[0].first << ", " << y_ranges[0].second << "] 입니다." << endl;
        return {};
    }

    // 시작 노드 추가
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push({0, start_y, 0.0});  // (x 인덱스, y 값, 비용)

    // dist 초기화
    dist[0][0] = 0.0;  // 시작 비용은 0

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        int x_idx = current.x_index;
        double y_val = current.y_value;

        if (x_idx == n - 1) break;  // 마지막 x 값에 도달

        // 다음 x 값의 범위
        double next_x = x_values[x_idx + 1];
        double y_min_next = y_ranges[x_idx + 1].first;
        double y_max_next = y_ranges[x_idx + 1].second;

        // 가능한 y 값 탐색 (0.01 간격)
        for (double next_y = y_min_next; next_y <= y_max_next; next_y += 0.01) {
            double cost = abs(next_y - y_val);

            if (dist[x_idx + 1][(int)(next_y * 100)] > current.cost + cost) {
                dist[x_idx + 1][(int)(next_y * 100)] = current.cost + cost;
                pq.push({x_idx + 1, next_y, dist[x_idx + 1][(int)(next_y * 100)]});
                prev[x_idx + 1][(int)(next_y * 100)] = {x_idx, y_val};
            }
        }
    }

    // 최적 경로 계산
    vector<pair<double, double>> path;
    double min_cost = numeric_limits<double>::infinity();
    double best_y;

    for (double y = y_ranges[n - 1].first; y <= y_ranges[n - 1].second; y += 0.01) {
        if (dist[n - 1][(int)(y * 100)] < min_cost) {
            min_cost = dist[n - 1][(int)(y * 100)];
            best_y = y;
        }
    }

    // 경로 역추적
    int current_x = n - 1;
    double current_y = best_y;
    while (current_x != -1) {
        path.push_back({x_values[current_x], current_y});
        auto prev_node = prev[current_x][(int)(current_y * 100)];
        current_x = prev_node.first;
        current_y = prev_node.second;
    }

    reverse(path.begin(), path.end());
    return path;
}

int main() {
    vector<double> x_values = {0.5, 1.5, 2.5, 3.5, 4.5, 5.5};
    vector<pair<double, double>> y_ranges = {{0.5, 1.0}, {1.0, 1.5}, {0.5, 1.0}, {1.0, 1.5}, {5.0, 5.5}, {0.5, 1.0}};
    double start_y;

    cout << "초기값 y를 입력하세요 (범위: " << y_ranges[0].first << "~" << y_ranges[0].second << "): ";
    cin >> start_y;

    vector<pair<double, double>> path = dijkstra(x_values, y_ranges, start_y);

    if (!path.empty()) {
        cout << "최적 경로:" << endl;
        for (const auto& p : path) {
            cout << "(" << p.first << ", " << p.second << ")" << endl;
        }
    } else {
        cout << "유효한 경로를 찾을 수 없습니다." << endl;
    }

    return 0;
}
