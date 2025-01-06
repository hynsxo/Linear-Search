function optimal_path = dijkstra_top10(x_values, y_ranges, start_y)
    % x_values: x 좌표 벡터
    % y_ranges: Nx2 행렬로 구성된 y 범위
    % start_y: 초기 y 값
    % optimal_path: 최적 경로 (x, y 쌍의 행렬)

    n = length(x_values); % x 값의 개수
    step_size = 0.01; % y 값 간격

    % 각 x 값에서의 최대 y 범위 확인
    global_y_min = min(y_ranges(:, 1));
    global_y_max = max(y_ranges(:, 2));
    max_steps = ceil((global_y_max - global_y_min) / step_size) + 1;

    % 초기값 유효성 확인
    if start_y < y_ranges(1, 1) || start_y > y_ranges(1, 2)
        error('초기값 %.2f가 유효하지 않습니다. 시작 범위는 [%.2f, %.2f]입니다.', ...
              start_y, y_ranges(1, 1), y_ranges(1, 2));
    end

    % 최소 비용 및 이전 노드 저장
    dist = inf(n, max_steps);
    prev = cell(n, max_steps); % 이전 노드 정보

    % y 값을 전역 인덱스로 변환하는 함수
    y_indices = @(y) round((y - global_y_min) / step_size) + 1;

    % 시작 노드 초기화
    dist(1, y_indices(start_y)) = 0;

    % 우선순위 큐 구현
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

        % 중앙값 계산
        y_mid_next = (y_min_next + y_max_next) / 2;
        
        % 가능한 y 값 탐색
        next_y_vals = y_min_next:step_size:y_max_next;
        next_costs = abs(next_y_vals - y_mid_next); % 중앙값과의 차이

        % 거리 기준으로 상위 10% 선택
        [~, sorted_idx] = sort(next_costs);
        top_10_idx = sorted_idx(1:min(ceil(length(sorted_idx) * 0.1), length(sorted_idx))); % 상위 10개 인덱스

        for idx = top_10_idx
            next_y = next_y_vals(idx);
            next_cost = current_cost + abs(next_y - y_val);
            next_y_idx = y_indices(next_y);

            if dist(next_x_idx, next_y_idx) > next_cost
                dist(next_x_idx, next_y_idx) = next_cost;
                prev{next_x_idx, next_y_idx} = [x_idx, y_val];
                pq(end + 1) = struct('x_idx', next_x_idx, 'y_val', next_y, 'cost', next_cost);
            end
        end
    end

    % 최종 비용 정렬 및 상위 10% 임계값 계산
    final_costs = dist(n, ~isinf(dist(n, :))); % 마지막 x 값의 모든 유효 비용
    sorted_costs = sort(final_costs); % 비용 정렬
    top10_threshold = sorted_costs(ceil(length(sorted_costs) * 0.1)); % 상위 10% 임계값

    % 상위 10%에 해당하는 비용 중 가장 작은 값을 선택
    optimal_path = [];
    for y_idx = 1:max_steps
        if dist(n, y_idx) <= top10_threshold
            % 상위 10% 조건을 만족하는 경로 중 하나를 역추적
            current_x = n;
            current_y = (y_idx - 1) * step_size + global_y_min;
            path = [];
            while current_x > 0
                path = [x_values(current_x), current_y; path];
                prev_node = prev{current_x, y_indices(current_y)};
                if isempty(prev_node)
                    break;
                end
                current_x = prev_node(1);
                current_y = prev_node(2);
            end
            optimal_path = path; % 첫 번째 유효 경로를 선택
            break; % 하나의 경로만 반환
        end
    end
end

% 실행 코드
x_values = [0.5, 1.5, 2.5, 3.5, 4.5, 5.5];
y_ranges = [0.5, 1.0; 1.0, 1.5; 0.5, 1.0; 1.0, 1.5; 5.0, 5.5; 0.5, 1.0];
start_y = input(sprintf('초기값 y를 입력하세요 (범위: %.2f~%.2f): ', y_ranges(1, 1), y_ranges(1, 2)));

try
    optimal_path = dijkstra_top10(x_values, y_ranges, start_y);
    disp('최적 경로 (상위 10% 중 하나):');
    disp(optimal_path);
catch ME
    disp(ME.message);
end

