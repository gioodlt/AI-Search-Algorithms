from collections import deque
import heapq


class Node:
    def __init__(self, state, parent=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.path_cost = path_cost


def read_input():
    with open("input.txt", "r") as file:
        lines = [line.strip() for line in file if line.strip()]

    algorithm = lines[0]
    start_state = lines[1]
    goal_state = lines[2]

    number_of_edges = int(lines[3])
    graph = {}
    index = 4

    for i in range(number_of_edges):
        state1, state2, cost = lines[index].split()
        cost = int(cost)

        if state1 not in graph:
            graph[state1] = []

        graph[state1].append((state2, cost))
        index += 1

    number_of_heuristics = int(lines[index])
    index += 1

    heuristic = {}

    for i in range(number_of_heuristics):
        state, estimate = lines[index].split()
        heuristic[state] = int(estimate)
        index += 1

    return algorithm, start_state, goal_state, graph, heuristic


def build_path(node):
    path = []

    while node is not None:
        path.append((node.state, node.path_cost))
        node = node.parent

    path.reverse()
    return path


def write_output(path):
    with open("output.txt", "w") as file:
        for state, cost in path:
            file.write(state + " " + str(cost) + "\n")


def bfs(start_state, goal_state, graph):
    start_node = Node(start_state, None, 0)

    frontier = deque([start_node])
    frontier_states = {start_state}
    explored = set()

    while frontier:
        node = frontier.popleft()
        frontier_states.remove(node.state)

        if node.state == goal_state:
            return build_path(node)

        explored.add(node.state)

        for child_state, cost in graph.get(node.state, []):
            if child_state not in explored and child_state not in frontier_states:
                child_node = Node(child_state, node, node.path_cost + cost)
                frontier.append(child_node)
                frontier_states.add(child_state)

    return []


def dfs(start_state, goal_state, graph):
    start_node = Node(start_state, None, 0)

    frontier = [start_node]
    frontier_states = {start_state}
    explored = set()

    while frontier:
        node = frontier.pop()
        frontier_states.remove(node.state)

        if node.state == goal_state:
            return build_path(node)

        explored.add(node.state)

        children = graph.get(node.state, [])

        for child_state, cost in reversed(children):
            if child_state not in explored and child_state not in frontier_states:
                child_node = Node(child_state, node, node.path_cost + cost)
                frontier.append(child_node)
                frontier_states.add(child_state)

    return []


def ucs(start_state, goal_state, graph):
    start_node = Node(start_state, None, 0)

    frontier = []
    counter = 0

    heapq.heappush(frontier, (0, counter, start_node))

    best_cost = {start_state: 0}
    explored = set()

    while frontier:
        current_cost, order, node = heapq.heappop(frontier)

        if node.state in explored:
            continue

        if current_cost != best_cost[node.state]:
            continue

        if node.state == goal_state:
            return build_path(node)

        explored.add(node.state)

        for child_state, cost in graph.get(node.state, []):
            new_cost = node.path_cost + cost

            if child_state not in explored:
                if child_state not in best_cost or new_cost < best_cost[child_state]:
                    child_node = Node(child_state, node, new_cost)
                    best_cost[child_state] = new_cost

                    counter += 1
                    heapq.heappush(frontier, (new_cost, counter, child_node))

    return []


def astar(start_state, goal_state, graph, heuristic):
    start_node = Node(start_state, None, 0)

    frontier = []
    counter = 0

    start_priority = heuristic.get(start_state, 0)
    heapq.heappush(frontier, (start_priority, counter, start_node))

    best_cost = {start_state: 0}
    explored = set()

    while frontier:
        priority, order, node = heapq.heappop(frontier)

        if node.state in explored:
            continue

        if node.path_cost != best_cost[node.state]:
            continue

        if node.state == goal_state:
            return build_path(node)

        explored.add(node.state)

        for child_state, cost in graph.get(node.state, []):
            new_cost = node.path_cost + cost

            if child_state not in explored:
                if child_state not in best_cost or new_cost < best_cost[child_state]:
                    child_node = Node(child_state, node, new_cost)
                    best_cost[child_state] = new_cost

                    counter += 1
                    child_priority = new_cost + heuristic.get(child_state, 0)

                    heapq.heappush(frontier, (child_priority, counter, child_node))

    return []


def main():
    algorithm, start_state, goal_state, graph, heuristic = read_input()

    if algorithm == "BFS":
        path = bfs(start_state, goal_state, graph)
    elif algorithm == "DFS":
        path = dfs(start_state, goal_state, graph)
    elif algorithm == "UCS":
        path = ucs(start_state, goal_state, graph)
    elif algorithm == "A*":
        path = astar(start_state, goal_state, graph, heuristic)
    else:
        path = []

    write_output(path)


if __name__ == "__main__":
    main()