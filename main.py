import heapq

# 節點圖結構 (已填寫詳細內容)
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

# 檢查圖結構對稱性
def check_graph_symmetry(graph):
    errors = []
    for node, edges in graph.items():
        for neighbor, distance, path_name in edges:
            reverse_edges = graph.get(neighbor, [])
            reverse_edge = next((e for e in reverse_edges if e[0] == node), None)
            if not reverse_edge or reverse_edge[1] != distance:
                errors.append((node, neighbor, distance))
    return errors

errors = check_graph_symmetry(graph)
if errors:
    print("圖結構檢查發現以下對稱性問題：")
    for error in errors:
        print(f"  節點 {error[0]} 到節點 {error[1]} 的距離 {error[2]} 不對稱！")
else:
    print("圖結構對稱性檢查通過！")


# Dijkstra 演算法實現
# 找最短路徑的函數已經在你的程式中完成，我們只需要確保清楚列出所有可能的最短路徑。
def dijkstra_with_paths(graph, start, end, must_pass_node=None):
    def find_paths(graph, start, end):
        queue = [(0, start, [])]  # (累積距離, 當前節點, 路徑)
        seen = set()
        shortest_paths = []
        min_cost = float('inf')

        while queue:
            (cost, node, path) = heapq.heappop(queue)

            # 如果已超過目前最小距離，略過
            if cost > min_cost:
                continue

            # 如果節點已被處理過，略過
            if node in seen:
                continue
            seen.add(node)

            # 如果抵達終點
            if node == end:
                if cost < min_cost:
                    min_cost = cost
                    shortest_paths = [(cost, path)]  # 找到更短的路徑，清空並添加
                elif cost == min_cost:
                    shortest_paths.append((cost, path))  # 同樣是最短路徑，添加到列表
                continue

            # 探索鄰居節點
            for next_node, weight, path_name in graph.get(node, []):
                if next_node not in seen:
                    heapq.heappush(queue, (cost + weight, next_node, path + [(node, next_node, path_name)]))

        return shortest_paths

    # 如果有必須經過的節點
    if must_pass_node:
        to_mid_paths = find_paths(graph, start, must_pass_node)
        from_mid_paths = find_paths(graph, must_pass_node, end)
        all_paths = []

        for to_cost, to_path in to_mid_paths:
            for from_cost, from_path in from_mid_paths:
                combined_cost = to_cost + from_cost
                combined_path = to_path + from_path
                all_paths.append((combined_cost, combined_path))

        # 篩選最短的路徑
        min_cost = min(path[0] for path in all_paths)
        shortest_paths = [path for path in all_paths if path[0] == min_cost]
        return shortest_paths

    # 否則直接尋找最短路徑
    return find_paths(graph, start, end)


# 無限執行等待使用者輸入
while True:
    print("\n請輸入必須經過的節點（例如 E），或輸入 'exit' 結束程式：")
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

    shortest_paths = dijkstra_with_paths(graph, start_node, end_node, must_pass_node)

    # 輸出結果
    print(f"\n從 {start_node} 到 {end_node} 的最短路徑 (經過 {must_pass_node if must_pass_node else '無特殊節點'})：")
    for i, (cost, path) in enumerate(shortest_paths, start=1):
        print(f"\n路徑選擇 {i}:")
        print(f"  最短距離: {cost}")
        print("  路徑詳情:")
        for edge in path:
            print(f"    {edge[0]} -> {edge[1]} via {edge[2]}")
