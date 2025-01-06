function optimal_path = dijkstra_top10_with_median(x_values, y_ranges, start_y)
    % x_values: x 좌표 벡터
    % y_ranges: Nx2 행렬로 구성된 y 범위
    % start_y: 초기 y 값
    % optimal_path: 최적 경로 (x, y 쌍의 행렬)

    step_size = 0.01; % y 값 간격
    n = length(x_values); % x 값의 개수

    % y 범위의 전역 최소 및 최대값
    global_y_min = min(y_ranges(:, 1));
    global_y_max = max(y_ranges(:, 2));
    max_steps = ceil((global_y_max - global_y_min) / step_size) + 1;

    % 초기값 유효성 확인
    if start_y < y_ranges(1, 1) || start_y > y_ranges(1, 2)
        error('초기값 %.2f가 유효하지 않습니다. 시작 범위는 [%.2f, %.2f]입니다.', start_y, y_ranges(1, 1), y_ranges(1, 2));
    end

    % 거리 및 이전 노드 저장
    dist = inf(n, max_steps);
    prev = cell(n, max_steps);

    % y 값을 전역 인덱스로 변환하는 함수
    y_to_index = @(y) round((y - global_y_min) / step_size) + 1;

    % 시작 노드 초기화
    dist(1, y_to_index(start_y)) = 0;

    % 우선순위 큐 (MATLAB의 구조체 배열 사용)
    pq = struct('x_idx', 1, 'y_val', start_y, 'cost', 0);

    % 다익스트라 알고리즘 실행
    while ~isempty(pq)
        % 큐에서 최저 비용 노드 추출
        [~, idx] = min([pq.cost]);
        current = pq(idx);
        pq(idx) = []; % 큐에서 제거

        x_idx = current.x_idx;
        y_val = current.y_val;
        current_cost = current.cost;

        if x_idx == n
            continue; % 마지막 x 값에서는 경로 갱신만 수행
        end

        % 다음 x 값의 y 범위 확인
        next_x_idx = x_idx + 1;
        y_min_next = y_ranges(next_x_idx, 1);
        y_max_next = y_ranges(next_x_idx, 2);

        % 다음 y 값 범위 생성
        next_y_vals = y_min_next:step_size:y_max_next;

        % 상위 10% 거리와 중앙값 근처의 y 값 선택
        selected_y = select_top10_with_median(next_y_vals, y_val, y_min_next, y_max_next);
        next_cost = current_cost + abs(selected_y - y_val);
        next_y_idx = y_to_index(selected_y);

        if dist(next_x_idx, next_y_idx) > next_cost
            dist(next_x_idx, next_y_idx) = next_cost;
            prev{next_x_idx, next_y_idx} = [x_idx, y_val];
            pq(end + 1) = struct('x_idx', next_x_idx, 'y_val', selected_y, 'cost', next_cost);
        end
    end

    % 최적 경로 역추적
    optimal_path = [];
    [~, best_y_idx] = min(dist(n, :)); % 마지막 x 값에서 최소 비용 y 인덱스
    current_y = global_y_min + (best_y_idx - 1) * step_size;
    current_x = n;

    while current_x > 0
        optimal_path = [[x_values(current_x), current_y]; optimal_path];
        prev_node = prev{current_x, y_to_index(current_y)};
        if isempty(prev_node)
            break;
        end
        current_x = prev_node(1);
        current_y = prev_node(2);
    end
end

function selected_y = select_top10_with_median(y_vals, current_y, y_min, y_max)
    % y_vals: 가능한 y 값들
    % current_y: 현재 y 값
    % y_min, y_max: y 범위의 최소 및 최대값

    % 현재 y 값과의 거리 계산
    distances = abs(y_vals - current_y);

    % 거리 정렬
    [~, sorted_idx] = sort(distances);
    sorted_y_vals = y_vals(sorted_idx);

    % 상위 10% 거리 추출
    top_10_limit = max(1, ceil(length(sorted_y_vals) * 0.1));
    top_10_y_vals = sorted_y_vals(1:top_10_limit);

    % y 범위 중앙값 계산
    y_mid = (y_min + y_max) / 2;

    % 중앙값과 가장 가까운 값 선택
    [~, closest_idx] = min(abs(top_10_y_vals - y_mid));
    selected_y = top_10_y_vals(closest_idx);
end

% 실행 코드
x_values = [0.5, 1.5, 2.5, 3.5, 4.5, 5.5];
y_ranges = [0.5, 1.0; 1.0, 1.5; 0.5, 1.0; 1.0, 1.5; 5.0, 5.5; 0.5, 1.0];

start_y = input(sprintf('초기값 y를 입력하세요 (범위: %.2f~%.2f): ', y_ranges(1, 1), y_ranges(1, 2)));

try
    optimal_path = dijkstra_top10_with_median(x_values, y_ranges, start_y);
    disp('최적 경로 (상위 10% 중 중앙값 근처):');
    disp(optimal_path);
catch ME
    disp(ME.message);
end

