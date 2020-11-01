import pygame
import graphUI
import time
import math
from queue import PriorityQueue
from node_color import white, yellow, black, red, blue, purple, orange, green, grey

"""
Feel free print graph, edges to console to get more understand input.
Do not change input parameters
Create new function/file if necessary
"""
def BFS(graph, edges, edge_id, start, goal):
    """
    BFS search
    """
    # TODO: your code
    print("Implement BFS algorithm.")
    discovered = []; # Mảng đánh dấu các đỉnh đã duyệt qua
    shortestPath = []; # Mảng lưu đường đi
    queue = [(start, [start])]; # Hàng đợi lưu các đỉnh đang xét
    isGoal = 0; # cờ hiệu để kiểm tra xem đã đến đích chưa: 1 = rồi, 0 = chưa

    while queue:
        (vertex, path) = queue.pop(0); # đỉnh 0 là đỉnh bắt đầu đi, bỏ đỉnh 0 vào hàng đợi
        graph[vertex][3] = yellow; # đỉnh hiện tại đang xét tô màu vàng
        graphUI.updateUI();
        time.sleep(0.5);

        for next in graph[vertex][1]: # xét các đỉnh kề
            if next == goal: # nếu tìm thấy đỉnh đích
                isGoal = 1; # đánh dấu là đã tìm được đích
                shortestPath = path + [next]; # thêm đỉnh vừa xét vào mảng đường đi
            else:
                queue.append((next,path + [next])); # thêm đỉnh vừa xét vào hàng đợi

            if next not in discovered: # nếu đỉnh kề chưa được duyệt qua
                graph[next][3] = red;   # thì tô màu đỏ
                graphUI.updateUI();
                discovered.append(next);# sau đó thêm vào mảng discovered[]
                edges[edge_id(vertex,next)][1] = white; # tô màu cạnh nối 2 đỉnh màu trắng
                time.sleep(0.1);
        if isGoal == 1: # nếu đã tìm được đích
            break; # dừng vòng while
        graph[vertex][3] = blue; # đánh dấu dỉnh đã duyệt qua
        graphUI.updateUI();
        time.sleep(0.5);

    graph[start][3] = orange; # đánh dấu đỉnh bắt đầu
    graphUI.updateUI();
    for i in range(len(shortestPath) - 1): # với mỗi cạnh trong mảng đường đi
        edges[edge_id(shortestPath[i],shortestPath[i+1])][1] = green; # tô màu xanh lá
    graph[goal][3] = purple; # đánh dấu đỉnh đích
    graphUI.updateUI();
    time.sleep(0.1);

    print('Path found: {}'.format(shortestPath))
    pass

def dfs_path(graph, edges, edge_id, current, goal, visited): #hàm hỗ trợ để tìm đường đi
    if current == goal: # nếu đỉnh đang xét là đích
        graph[goal][3] = purple # tô màu
        graphUI.updateUI()
        time.sleep(0.5);
        return [current]
    else:
        graph[current][3] = yellow # tô màu đỉnh hiện tại
        graphUI.updateUI()
        time.sleep(0.5);
        for neighbor in graph[current][1]:
            graph[neighbor][3] = blue
            edges[edge_id(current, neighbor)][1] = white
            graphUI.updateUI()
            time.sleep(0.1);
            if neighbor not in visited: # nếu đỉnh kề chưa được xét
                graph[neighbor][3] = red # tô màu
                graphUI.updateUI()
                time.sleep(0.5);
                visited.add(neighbor) # thêm vào tập đã duyệt
                path = dfs_path(graph, edges, edge_id, neighbor, goal, visited)

                if path is not None: # nếu tập đường đi khác null
                    path.insert(0, current) # path[0] = current
                    return path # trả về đường đi

def DFS(graph, edges, edge_id, start, goal):
    """
    DFS search
    """
    # TODO: your code
    print("Implement DFS algorithm.")
    discovered = set()
    discovered.add(start)

    if start == goal:
        graph[goal][3] = purple;
        graphUI.updateUI()
        time.sleep(0.5);
    else:
        path = dfs_path(graph, edges, edge_id, start, goal, discovered)
        graph[start][3] = orange
        graphUI.updateUI()
        time.sleep(0.5);
        for i in range(len(path) - 1):
            edges[edge_id(path[i], path[i + 1])][1] = green
            graph[goal][3] = purple
            graphUI.updateUI()
            time.sleep(0.1);

    print('Path found: {}'.format(path))
    pass

def getWeight(from_node, to_node):
    weight = math.sqrt((to_node[0] - from_node[0])**2 + (to_node[1] - from_node[1])**2)
    return weight

def UCS(graph, edges, edge_id, start, goal):
    """
    Uniform Cost Search search
    """
    # TODO: your code
    print("Implement Uniform Cost Search algorithm.")
    discovered = set();
    frontier = PriorityQueue();
    previous = set();
    frontier.put((-1, 0, start));

    while frontier:
        prev, weight, curr = frontier.get();
        if curr not in discovered:
            graph[curr][3] = yellow;
            graphUI.updateUI();
            time.sleep(0.5);
            previous.add((prev,curr));
            discovered.add(curr);
            if curr == goal:
                path = [curr]
                while 1:
                    for i in previous:
                        if i[1] == curr:
                            prev = i[0]
                    path.append(prev)
                    curr = prev
                    if curr == start:
                        break
                path.reverse()
                print('Path found: {}'.format(path))
                graph[start][3] = orange
                for i in range(len(path) - 1):
                    edges[edge_id(path[i], path[i + 1])][1] = green
                graph[goal][3] = purple
                graphUI.updateUI()
                time.sleep(0.1);
                return
            for neighbor in graph[curr][1]:
                totalW = weight + getWeight(graph[curr][0], graph[neighbor][0])
                if neighbor not in frontier.queue:
                    graph[neighbor][3] = red
                    edges[edge_id(curr, neighbor)][1] = white
                    graphUI.updateUI()
                    time.sleep(0.1);
                    frontier.put((curr, totalW, neighbor))
            graph[curr][3] = blue
            graphUI.updateUI()
            time.sleep(0.5);
    pass

def euclidean_distance(current_x, current_y, goal_x, goal_y):
    distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
    return distance

def manhattan_distance(current_x, current_y, goal_x, goal_y):
    return abs(current_x - goal_x) + abs(current_y - goal_y)

def AStar(graph, edges, edge_id, start, goal):
    """
    A star search
    """
    # TODO: your code
    open = set([start]) # tập mở
    closed = set() # tập đóng
    g = {}  # g chứa khoảng cách tới đỉnh bắt đầu
    parents = {}  # tập đỉnh kề của các đỉnh

    g[start] = 0 # khoảng cách từ đỉnh bắt đầu tới chính nó bằng 0
      # start là đỉnh bắt đầu nên không có parent nên start là parent của chính nó
    parents[start] = start

    while len(open) > 0:
            pygame.event.get()
            n = None
            # f(n) = g(n) + h(n)
            # đỉnh có f(n) nhỏ nhất được xét
            for v in open:
                if n == None or g[v] + manhattan_distance(graph[v][0][0], graph[v][0][1], graph[goal][0][0], graph[goal][0][1]) < g[n] + manhattan_distance(graph[n][0][0], graph[n][0][1], graph[goal][0][0], graph[goal][0][1]):
                    n = v

            if n == goal or graph[n][1] == None:
                pass
            else:
                lst = list()
                graph[n][3] = yellow
                graphUI.updateUI()
                time.sleep(0.5);
                for neighbor in graph[n][1]:
                    graph[neighbor][3] = red
                    edges[edge_id(n, neighbor)][1] = white
                    graphUI.updateUI()
                    time.sleep(0.1);
                    lst.append((neighbor, euclidean_distance(graph[neighbor][0][0], graph[neighbor][0][1], graph[n][0][0], graph[n][0][1])))
                for (m, weight) in lst:
                    # đỉnh m không nằm trong tập mở và tập đóng sẽ được thêm vào tập đóng
                    if m not in open and m not in closed:
                        graph[m][3] = blue
                        graphUI.updateUI()
                        time.sleep(0.5);
                        open.add(m)
                        parents[m] = n
                        g[m] = g[n] + weight

                    # với mỗi đỉnh m,so sánh khoảng cách của đỉnh m so với n
                    else:
                        if g[m] > g[n] + weight:
                            # update g(m)
                            g[m] = g[n] + weight
                            # thay parent của m thành n
                            parents[m] = n

                            # nếu m nằm trong tập đóng, lấy ra khỏi tập đóng rồi thêm vào tập mở
                            if m in closed:
                                closed.remove(m)
                                open.add(m)

            if n == None:
                print('Path does not exist!')
                return None

            # nếu đỉnh đang duyệt là đỉnh đích
            # thi xây dựng đường đi từ đỉnh bắt đầu tới nó
            if n == goal:
                graph[start][3] = orange
                graph[goal][3] = purple
                graphUI.updateUI()
                time.sleep(0.5);
                path = []

                while parents[n] != n:
                    path.append(n)
                    n = parents[n]

                path.append(start)

                path.reverse()

                print('Path found: {}'.format(path))
                for i in range(len(path) - 1):
                    edges[edge_id(path[i], path[i + 1])][1] = green
                graphUI.updateUI()
                time.sleep(0.1);
                return path

            # lấy n ra khỏi tập m rồi thêm vào tập đóng
            # vì tất cả đỉnh kề đã được duyệt
            graph[n][3] = yellow
            graphUI.updateUI()
            time.sleep(0.5);
            open.remove(n)
            closed.add(n)
    print('Path does not exist!')
    return None
    print("Implement AStar algorithm.")
    pass


def example_func(graph, edges, edge_id, start, goal):
    """
    This function is just show some basic feature that you can use your project.
    @param graph: list - contain information of graph (same value as global_graph)
                    list of object:
                     [0] : (x,y) coordinate in UI
                     [1] : adjacent node indexes
                     [2] : node edge color
                     [3] : node fill color
                Ex: graph = [
                                [
                                    (139, 140),             # position of node when draw on UI
                                    [1, 2],                 # list of adjacent node
                                    (100, 100, 100),        # grey - node edged color
                                    (0, 0, 0)               # black - node fill color
                                ],
                                [(312, 224), [0, 4, 2, 3], (100, 100, 100), (0, 0, 0)],
                                ...
                            ]
                It means this graph has Node 0 links to Node 1 and Node 2.
                Node 1 links to Node 0,2,3 and 4.
    @param edges: dict - dictionary of edge_id: [(n1,n2), color]. Ex: edges[edge_id(0,1)] = [(0,1), (0,0,0)] : set color
                    of edge from Node 0 to Node 1 is black.
    @param edge_id: id of each edge between two nodes. Ex: edge_id(0, 1) : id edge of two Node 0 and Node 1
    @param start: int - start vertices/node
    @param goal: int - vertices/node to search
    @return:
    """

    # Ex1: Set all edge from Node 1 to Adjacency node of Node 1 is green edges.
    node_1 = graph[1]
    for adjacency_node in node_1[1]:
        edges[edge_id(1, adjacency_node)][1] = green
    graphUI.updateUI()

    # Ex2: Set color of Node 2 is Red
    graph[2][3] = red
    graphUI.updateUI()

    # Ex3: Set all edge between node in a array.
    path = [4, 7, 9]  # -> set edge from 4-7, 7-9 is blue
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = blue
    graphUI.updateUI()
