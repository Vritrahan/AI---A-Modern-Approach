import heapq    #Min Heap Data Structure

graph= {'A' : {'B' : 2, 'C' : 1},
        'B' : {'D' : 3},
        'C' : {'E' : 2},
        'D' : {},
        'E' : {'D' : 1, 'F' : 1, 'G' : 1},
        'F' : {'H' : 1},
        'G' : {'H' : 1},
        'H' : {}}

def reconstructPath(cameFrom, start, goal):
    current = goal
    path = [current]
    while current != start:
        current=cameFrom[current]
        path.extend(current)
    path.reverse()
    return path

def UCS(graph, start, goal):    #Uniform Cost Search
    PriorityQueue = []
    heapq.heappush(PriorityQueue, (0, start))
    cameFrom = {start : None}
    costSoFar = {start : 0}
    while PriorityQueue:
        current = heapq.heappop(PriorityQueue)[1]
        if current == goal:
            break
        for next in graph[current]:
            newCost = costSoFar[current] + graph[current][next]
            if next not in costSoFar or newCost < costSoFar[next]:
                costSoFar[next] = newCost
                priority = newCost
                heapq.heappush(PriorityQueue, (priority, next))
                cameFrom[next] = current
    return reconstructPath(cameFrom, start, goal)

def heuristic(a, b):    #Shitty heuristic function
    return ord(a) - ord(b)  #ord -> ASCII value

def GBFS(graph, start, goal):   #Greedy Best-First Search
    PriorityQueue = []
    heapq.heappush(PriorityQueue, (0, start))
    cameFrom = {start : None}
    costSoFar = {start : 0}
    while PriorityQueue:
        current = heapq.heappop(PriorityQueue)[1]
        if current == goal:
            break
        for next in graph[current]:
            newCost = costSoFar[current] + graph[current][next]
            if next not in costSoFar or newCost < costSoFar[next]:
                costSoFar[next] = newCost
                priority = heuristic(goal, next)    #only difference from UCS
                heapq.heappush(PriorityQueue, (priority, next))
                cameFrom[next] = current
    return reconstructPath(cameFrom, start, goal)

def aStar(graph, start, goal):  #a* search 
    PriorityQueue = []
    heapq.heappush(PriorityQueue, (0, start))
    cameFrom = {start : None}
    costSoFar = {start : 0}
    while PriorityQueue:
        current = heapq.heappop(PriorityQueue)[1]
        if current == goal:
            break
        for next in graph[current]:
            newCost = costSoFar[current] + graph[current][next]
            if next not in costSoFar or newCost < costSoFar[next]:
                costSoFar[next] = newCost
                priority = newCost + heuristic(goal, next)  #only difference from UCS and GBFS
                heapq.heappush(PriorityQueue, (priority, next))
                cameFrom[next] = current
    return reconstructPath(cameFrom, start, goal)

print(UCS(graph, 'A', 'H'))
print(GBFS(graph, 'A', 'H'))
print(aStar(graph, 'A', 'H'))