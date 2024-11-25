import heapq

# 儲存所有可能最短路徑的 Dijkstra 實現
def enhanced_dijkstra(graph, start, end):
    queue = [(0, start, [])]  # (累積距離, 當前節點, 路徑)
    min_cost = float('inf')
    shortest_paths = []  # 儲存所有可能最短路徑
    visited = {}  # 節點 -> 最短距離

    while queue:
        cost, node, path = heapq.heappop(queue)

        # 如果超過當前已知最短距離，跳過
        if cost > min_cost:
            continue

        # 如果是新節點，或找到更短路徑，更新訪問紀錄
        if node not in visited or cost <= visited[node]:
            visited[node] = cost

            # 如果到達終點
            if node == end:
                if cost < min_cost:
                    min_cost = cost
                    shortest_paths = [(cost, path)]
                elif cost == min_cost:
                    shortest_paths.append((cost, path))
                continue

            # 將鄰居加入隊列
            for next_node, weight, path_name in graph.get(node, []):
                heapq.heappush(queue, (cost + weight, next_node, path + [(node, next_node, path_name)]))

    return shortest_paths


# 尋找所有最短路徑（支持必經點）
def find_all_shortest_paths(graph, start, end, must_pass_node=None):
    if must_pass_node:
        # 起點 -> 必經節點的所有最短路徑
        to_mid_paths = enhanced_dijkstra(graph, start, must_pass_node)
        # 必經節點 -> 終點的所有最短路徑
        from_mid_paths = enhanced_dijkstra(graph, must_pass_node, end)

        # 合併所有可能的路徑
        all_paths = []
        for to_cost, to_path in to_mid_paths:
            for from_cost, from_path in from_mid_paths:
                combined_cost = to_cost + from_cost
                combined_path = to_path + from_path
                all_paths.append((combined_cost, combined_path))

        # 找出所有最短路徑
        min_cost = min(path[0] for path in all_paths)
        return [path for path in all_paths if path[0] == min_cost]
    else:
        # 無需必經點，直接尋找
        return enhanced_dijkstra(graph, start, end)


# 測試範例的圖
graph = {
    'A': [('E', 4, 'a1'), ('B', 2, 'a2'), ('C', 2, 'a3')],
    'B': [('A', 2, 'b1'), ('C', 1, 'b2'), ('K', 7, 'b3'), ('F', 2, 'b4'), ('G', 7, 'b5'), ('E', 4, 'b6')],
    'C': [('A', 2, 'c1'), ('J', 6, 'c2'), ('H', 5, 'c3'), ('B', 1, 'c4')],
    'D': [('E', 3, 'd1'), ('F', 4, 'd2'), ('G', 6, 'd3')],
    'E': [('A', 4, 'e1'), ('B', 4, 'e2'), ('G', 6, 'e3'), ('D', 3, 'e4')],
    'F': [('B', 2, 'f1'), ('K', 4, 'f2'), ('H', 3, 'f3'), ('D', 4, 'f4')],
    'G': [('E', 6, 'g1'), ('B', 7, 'g2'), ('J', 3, 'g3'), ('H', 4, 'g4'), ('I', 5, 'g5'), ('D', 6, 'g6')],
    'H': [('F', 3, 'h1'), ('C', 5, 'h2'), ('J', 3, 'h3'), ('K', 2, 'h4'), ('I', 2, 'h5'), ('G', 4, 'h6')],
    'I': [('G', 5, 'i1'), ('H', 2, 'i2'), ('K', 5, 'i3'), ('J', 7, 'i4')],
    'J': [('C', 6, 'j1'), ('K', 2, 'j2'), ('I', 7, 'j3'), ('H', 3, 'j4'), ('G', 3, 'j5')],
    'K': [('B', 7, 'k1'), ('F', 4, 'k2'), ('H', 2, 'k3'), ('I', 5, 'k4'), ('J', 2, 'k5')]
}

# 用戶交互
while True:
    print("\n請輸入必須經過的節點（例如 H），或輸入 'exit' 結束程式：")
    user_input = input("必須經過的節點: ").strip().upper()

    if user_input == 'EXIT':
        print("程式結束，感謝使用！")
        break

    if user_input and user_input not in graph:
        print(f"節點 {user_input} 不存在於圖結構中，請重新輸入！")
        continue

    # 計算最短路徑
    start_node = 'A'
    end_node = 'K'
    must_pass_node = user_input if user_input else None

    shortest_paths = find_all_shortest_paths(graph, start_node, end_node, must_pass_node)

    # 輸出結果
    print(f"\n從 {start_node} 到 {end_node} 的最短路徑 (經過 {must_pass_node if must_pass_node else '無特殊節點'})：")
    for i, (cost, path) in enumerate(shortest_paths, start=1):
        print(f"\n路徑選擇 {i}:")
        print(f"  最短距離: {cost}")
        print("  路徑詳情:")
        for edge in path:
            print(f"    {edge[0]} -> {edge[1]} via {edge[2]}")
