#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <numeric> // iota 함수 사용

using namespace std;

struct Node {
    int x_idx;
    double y_val;
    double cost;
    bool operator>(const Node &other) const {
        return cost > other.cost;
    }
};

// y 값을 인덱스로 변환하는 함수
int y_to_index(double y, double global_y_min, double step_size) {
    return static_cast<int>(round((y - global_y_min) / step_size));
}

// 다음 y 범위에서 상위 10% 거리와 중앙값 근처의 y 값을 선택하는 함수
double select_top10_with_median(const vector<double>& y_vals, double current_y, double y_min, double y_max) {
    vector<double> distances;
    for (double y : y_vals) {
        distances.push_back(abs(y - current_y));
    }

    // 거리 정렬 및 인덱스 추적
    vector<int> sorted_idx(y_vals.size());
    iota(sorted_idx.begin(), sorted_idx.end(), 0);
    sort(sorted_idx.begin(), sorted_idx.end(), [&](int i, int j) {
        return distances[i] < distances[j];
    });

    // 상위 10% 거리 추출
    int top_10_limit = max(1, static_cast<int>(ceil(sorted_idx.size() * 0.1)));
    vector<double> top_10_y_vals;
    for (int i = 0; i < top_10_limit; ++i) {
        top_10_y_vals.push_back(y_vals[sorted_idx[i]]);
    }

    // y 범위 중앙값 계산
    double y_mid = (y_min + y_max) / 2;

    // 중앙값과 가장 가까운 값을 선택
    auto closest = min_element(top_10_y_vals.begin(), top_10_y_vals.end(), [&](double a, double b) {
        return abs(a - y_mid) < abs(b - y_mid);
    });

    return *closest;
}

// 다익스트라 알고리즘 실행
vector<pair<double, double>> dijkstra_top10_with_median(const vector<double>& x_values, const vector<pair<double, double>>& y_ranges, double start_y) {
    int n = x_values.size(); // x 값의 개수
    double step_size = 0.01; // y 값 간격

    // y 범위의 전역 최소 및 최대값
    double global_y_min = y_ranges[0].first;
    double global_y_max = y_ranges[0].second;
    for (const auto& range : y_ranges) {
        global_y_min = min(global_y_min, range.first);
        global_y_max = max(global_y_max, range.second);
    }

    int max_steps = ceil((global_y_max - global_y_min) / step_size) + 1;

    // 초기값 유효성 확인
    if (start_y < y_ranges[0].first || start_y > y_ranges[0].second) {
        throw runtime_error("초기값이 유효하지 않습니다. 시작 범위는 [" + to_string(y_ranges[0].first) + ", " + to_string(y_ranges[0].second) + "]입니다.");
    }

    // 거리 및 이전 노드 저장
    vector<vector<double>> dist(n, vector<double>(max_steps, INFINITY));
    vector<vector<pair<int, double>>> prev(n, vector<pair<int, double>>(max_steps, {-1, -1}));

    // 시작 노드 초기화
    dist[0][y_to_index(start_y, global_y_min, step_size)] = 0;

    // 우선순위 큐
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push(Node{0, start_y, 0});

    // 다익스트라 알고리즘 실행
    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        int x_idx = current.x_idx;
        double y_val = current.y_val;
        double current_cost = current.cost;

        if (x_idx == n - 1) {
            continue; // 마지막 x 값에서는 경로 갱신만 수행
        }

        // 다음 x 값의 y 범위 확인
        int next_x_idx = x_idx + 1;
        double y_min_next = y_ranges[next_x_idx].first;
        double y_max_next = y_ranges[next_x_idx].second;

        vector<double> next_y_vals;
        for (double next_y = y_min_next; next_y <= y_max_next; next_y += step_size) {
            next_y_vals.push_back(next_y);
        }

        // 상위 10% 거리와 중앙값 근처의 y 값을 선택
        double selected_y = select_top10_with_median(next_y_vals, y_val, y_min_next, y_max_next);
        double next_cost = current_cost + abs(selected_y - y_val);
        int next_y_idx = y_to_index(selected_y, global_y_min, step_size);

        if (dist[next_x_idx][next_y_idx] > next_cost) {
            dist[next_x_idx][next_y_idx] = next_cost;
            prev[next_x_idx][next_y_idx] = {x_idx, y_val};
            pq.push(Node{next_x_idx, selected_y, next_cost});
        }
    }

    // 최적 경로 역추적
    vector<pair<double, double>> optimal_path;
    int best_y_idx = min_element(dist[n - 1].begin(), dist[n - 1].end()) - dist[n - 1].begin();
    double current_y = global_y_min + (best_y_idx * step_size);
    int current_x = n - 1;

    while (current_x >= 0) {
        optimal_path.push_back({x_values[current_x], current_y});
        auto prev_node = prev[current_x][y_to_index(current_y, global_y_min, step_size)];
        if (prev_node.first == -1) {
            break;
        }
        current_x = prev_node.first;
        current_y = prev_node.second;
    }

    reverse(optimal_path.begin(), optimal_path.end());
    return optimal_path;
}

int main() {
    vector<double> x_values = {0.5, 1.5, 2.5, 3.5, 4.5, 5.5};
    vector<pair<double, double>> y_ranges = {{0.5, 1.0}, {1.0, 1.5}, {0.5, 1.0}, {1.0, 1.5}, {5.0, 5.5}, {0.5, 1.0}};

    double start_y;
    cout << "초기값 y를 입력하세요 (범위: " << y_ranges[0].first << "~" << y_ranges[0].second << "): ";
    cin >> start_y;

    if (!cin || cin.fail()) {
        cerr << "입력이 유효하지 않습니다. 프로그램을 종료합니다." << endl;
        return 1;
    }

    try {
        vector<pair<double, double>> optimal_path = dijkstra_top10_with_median(x_values, y_ranges, start_y);
        cout << "최적 경로 (상위 10% 중 중앙값 근처):" << endl;
        for (const auto& p : optimal_path) {
            cout << "(" << p.first << ", " << p.second << ")" << endl;
        }
    } catch (const exception& e) {
        cerr << e.what() << endl;
    }

    return 0;
}
