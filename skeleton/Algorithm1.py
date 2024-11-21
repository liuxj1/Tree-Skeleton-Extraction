def Ring_dfs(node, parent, visited, path, graph):
    visited[node] = True
    path.append(node)

    for neighbour in graph[node]:
        if not visited[neighbour]:
            cycle = Ring_dfs(neighbour, node, visited, path.copy(), graph)
            if cycle:  # If a closed loop is found, return immediately
                return cycle
        elif neighbour != parent and neighbour in path:
            # Find the closed loop and return the path to the closed loop
            cycle_start_index = path.index(neighbour)
            return path[cycle_start_index:]

    path.pop()  # Backtrack, remove the current node in the path
    visited[node] = False  # Mark the current node as unvisited
    return False

def Ring_Structure_Detection_and_Filtering(first_pair_info):
    while True:
        # Construct an adjacency list of the graph to represent the connection relationship between different skeleton groups Si
        skeleton_graph = {}
        for skeleton, _ in first_pair_info.items():
            S1, S2 = skeleton
            if S1 not in skeleton_graph:
                skeleton_graph[S1] = []
            if S2 not in skeleton_graph:
                skeleton_graph[S2] = []
            skeleton_graph[S1].append(S2)
            skeleton_graph[S2].append(S1)

        visited = {s: False for s in skeleton_graph}

        for vertex in skeleton_graph:
            if not visited[vertex]:
                cycle = Ring_dfs(vertex, None, visited, [],
                            skeleton_graph)  # DFS searches for which connected skeleton groups form a ring structure
                if cycle:
                    break
        # If there is a ring structure, enter the filter, otherwise, exit the loop
        if cycle:
            cycle_edge = [(cycle[i], cycle[i + 1]) for i in range(len(cycle) - 1)] + [
                (cycle[-1], cycle[0])]  # Convert skeleton groups with ring structures into edge connection form
            cycle_with_data = [(tuple(sorted(edge)), first_pair_info[tuple(sorted(edge))]) for edge in
                               cycle_edge]  # Obtain connection pair information between connected skeleton groups

            pairs_distance_sorted = sorted(cycle_with_data, key=lambda x: x[1][
                2])  # Sort the connection pairs forming a ring structure from small to large according to distance
            pairs_angle_123_sorted = sorted(cycle_with_data, key=lambda x: x[1][
                3])  # Sort the connection pairs forming a ring structure from small to large according to angle
            target_distance = pairs_distance_sorted[-1][1][2]  # The connection pair with the largest distance

            # Filter the connection pairs with the smallest angle and the largest distance
            close_pairs = [pair for pair in pairs_angle_123_sorted if abs(pair[1][2] - target_distance) <= 0.02]
            # Remove filtered join pairs in x dictionary
            del first_pair_info[close_pairs[0][0]]

        else:
            break
    return first_pair_info
