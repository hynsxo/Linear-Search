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

// Cubic Spline 보간에 필요한 구조체
struct SplineSet {
    double a, b, c, d, x;
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

// Cubic Spline 보간법 함수
vector<SplineSet> cubic_spline(const vector<double>& x, const vector<double>& y) {
    int n = x.size() - 1;
    vector<double> a(y.begin(), y.end());
    vector<double> b(n), d(n), h(n), alpha(n + 1), c(n + 1), l(n + 1), mu(n + 1), z(n + 1);

    for (int i = 0; i < n; ++i)
        h[i] = x[i + 1] - x[i];

    for (int i = 1; i < n; ++i)
        alpha[i] = 3 * (a[i + 1] - a[i]) / h[i] - 3 * (a[i] - a[i - 1]) / h[i - 1];

    l[0] = 1;
    mu[0] = z[0] = 0;

    for (int i = 1; i < n; ++i) {
        l[i] = 2 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n] = 1;
    z[n] = c[n] = 0;

    for (int j = n - 1; j >= 0; --j) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2 * c[j]) / 3;
        d[j] = (c[j + 1] - c[j]) / (3 * h[j]);
    }

    vector<SplineSet> spline_coeffs(n);
    for (int i = 0; i < n; ++i) {
        spline_coeffs[i] = {a[i], b[i], c[i], d[i], x[i]};
    }
    return spline_coeffs;
}

// 다익스트라 알고리즘 실행
vector<pair<double, double>> dijkstra_top10_with_median(const vector<double>& x_values, const vector<pair<double, double>>& y_ranges, double start_y) {
    int n = x_values.size();
    double step_size = 0.01;
    double global_y_min = y_ranges[0].first, global_y_max = y_ranges[0].second;

    for (const auto& range : y_ranges) {
        global_y_min = min(global_y_min, range.first);
        global_y_max = max(global_y_max, range.second);
    }

    int max_steps = ceil((global_y_max - global_y_min) / step_size) + 1;
    vector<vector<double>> dist(n, vector<double>(max_steps, INFINITY));
    vector<vector<pair<int, double>>> prev(n, vector<pair<int, double>>(max_steps, {-1, -1}));
    dist[0][y_to_index(start_y, global_y_min, step_size)] = 0;

    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push(Node{0, start_y, 0});

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        int x_idx = current.x_idx;
        double y_val = current.y_val;

        if (x_idx == n - 1) continue;

        int next_x_idx = x_idx + 1;
        double y_min_next = y_ranges[next_x_idx].first;
        double y_max_next = y_ranges[next_x_idx].second;

        vector<double> next_y_vals;
        for (double next_y = y_min_next; next_y <= y_max_next; next_y += step_size) {
            next_y_vals.push_back(next_y);
        }

        double selected_y = select_top10_with_median(next_y_vals, y_val, y_min_next, y_max_next);
        double next_cost = current.cost + abs(selected_y - y_val);
        int next_y_idx = y_to_index(selected_y, global_y_min, step_size);

        if (dist[next_x_idx][next_y_idx] > next_cost) {
            dist[next_x_idx][next_y_idx] = next_cost;
            prev[next_x_idx][next_y_idx] = {x_idx, y_val};
            pq.push(Node{next_x_idx, selected_y, next_cost});
        }
    }

    vector<pair<double, double>> optimal_path;
    int best_y_idx = min_element(dist[n - 1].begin(), dist[n - 1].end()) - dist[n - 1].begin();
    double current_y = global_y_min + (best_y_idx * step_size);
    int current_x = n - 1;

    while (current_x >= 0) {
        optimal_path.push_back({x_values[current_x], current_y});
        auto prev_node = prev[current_x][y_to_index(current_y, global_y_min, step_size)];
        if (prev_node.first == -1) break;
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

    try {
        auto optimal_path = dijkstra_top10_with_median(x_values, y_ranges, start_y);
        cout << "최적 경로:" << endl;
        for (const auto& p : optimal_path) {
            cout << "(" << p.first << ", " << p.second << ")" << endl;
        }

        vector<double> path_x, path_y;
        for (const auto& p : optimal_path) {
            path_x.push_back(p.first);
            path_y.push_back(p.second);
        }

        auto spline_coeffs = cubic_spline(path_x, path_y);
        cout << "Cubic Spline Coefficients:" << endl;
        for (const auto& coeff : spline_coeffs) {
            cout << "S(x) = " << coeff.a << " + " << coeff.b << "(x - " << coeff.x << ") + " << coeff.c << "(x - " << coeff.x << ")^2 + " << coeff.d << "(x - " << coeff.x << ")^3" << endl;
        }
    } catch (const exception& e) {
        cerr << e.what() << endl;
    }

    return 0;
}

