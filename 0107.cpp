#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense> // VectorXd 사용

using namespace std;
using namespace Eigen;

struct Node {
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

// 상위 10% 거리와 중앙값 근처의 y 값을 선택하는 함수
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
vector<double> dijkstra_top10_with_median(const VectorXd& y_min, const VectorXd& y_max, double start_y) {
    double step_size = 0.01; // y 값 간격

    // 전역 최소 및 최대값 계산
    double global_y_min = y_min.minCoeff();
    double global_y_max = y_max.maxCoeff();

    int max_steps = ceil((global_y_max - global_y_min) / step_size) + 1;

    // 초기값 유효성 확인
    if (start_y < global_y_min || start_y > global_y_max) {
        throw runtime_error("초기값이 유효하지 않습니다. 시작 범위는 [" + to_string(global_y_min) + ", " + to_string(global_y_max) + "]입니다.");
    }

    // 거리 및 이전 노드 저장
    vector<double> dist(max_steps, INFINITY);
    vector<int> prev(max_steps, -1);

    // 시작 노드 초기화
    dist[y_to_index(start_y, global_y_min, step_size)] = 0;

    // 우선순위 큐
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push(Node{start_y, 0});

    // 다익스트라 알고리즘 실행
    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        double y_val = current.y_val;
        double current_cost = current.cost;

        // 다음 y 값의 범위 생성
        vector<double> next_y_vals;
        for (double next_y = global_y_min; next_y <= global_y_max; next_y += step_size) {
            next_y_vals.push_back(next_y);
        }

        // 상위 10% 거리와 중앙값 근처의 y 값을 선택
        double selected_y = select_top10_with_median(next_y_vals, y_val, global_y_min, global_y_max);
        double next_cost = current_cost + abs(selected_y - y_val);
        int next_y_idx = y_to_index(selected_y, global_y_min, step_size);

        if (dist[next_y_idx] > next_cost) {
            dist[next_y_idx] = next_cost;
            prev[next_y_idx] = y_to_index(y_val, global_y_min, step_size);
            pq.push(Node{selected_y, next_cost});
        }
    }

    // 최적 경로 역추적
    vector<double> optimal_path;
    int best_y_idx = min_element(dist.begin(), dist.end()) - dist.begin();
    double current_y = global_y_min + (best_y_idx * step_size);

    while (best_y_idx != -1) {
        optimal_path.push_back(current_y);
        best_y_idx = prev[best_y_idx];
        if (best_y_idx != -1) {
            current_y = global_y_min + (best_y_idx * step_size);
        }
    }

    reverse(optimal_path.begin(), optimal_path.end());
    return optimal_path;
}

int main() {
    VectorXd y_min(1), y_max(1); // 예시로 하나의 값만 입력받음
    double start_y;

    // y_min, y_max 값 입력 받기
    cout << "y_min 벡터 값을 입력하세요 (예: 0.5): ";
    cin >> y_min[0];

    cout << "y_max 벡터 값을 입력하세요 (예: 1.5): ";
    cin >> y_max[0];

    cout << "start_y 값을 입력하세요: ";
    cin >> start_y;

    if (!cin || cin.fail()) {
        cerr << "입력이 유효하지 않습니다. 프로그램을 종료합니다." << endl;
        return 1;
    }

    try {
        vector<double> optimal_path = dijkstra_top10_with_median(y_min, y_max, start_y);
        cout << "최적 경로 (상위 10% 중 중앙값 근처):" << endl;
        for (double y : optimal_path) {
            cout << y << endl;
        }
    } catch (const exception& e) {
        cerr << e.what() << endl;
    }

    return 0;
}
