#Using an adjacency list
graph= {'A': ['B', 'C'],
        'B': ['C', 'D'],
        'C': ['D'],
        'D': ['C'],
        'E': ['F'],
        'F': ['C']}

def findPath(graph, start, goal, path=[]):
    path.extend(start)
    if start==goal:
        return path
    if start not in graph:
        return None
    for node in graph[start]:
        if node not in path:
            newPath=findPath(graph,node,goal,path)
            if newPath:
                return newPath
    return None

def findAllPaths(graph, start, goal, path=[]):
    path.extend(start)
    if start==goal:
        return [path]
    if start not in graph:
        return []
    paths=[]
    ogPath=path[:]
    for node in graph[start]:
        path=ogPath[:]
        if node not in path:
            newPaths=findAllPaths(graph,node,goal,path)
            for newPath in newPaths:
                paths.append(newPath)
    return paths

def findShortestPath(graph, start, goal, path=[]):
        path.extend(start)
        if start == goal:
            return path
        if start not in graph:
            return None
        shortest = None
        ogPath=path[:]
        for node in graph[start]:
            path=ogPath[:]
            if node not in path:
                newPath = findShortestPath(graph, node, goal, path)
                if newPath:
                    if not shortest or len(newPath) < len(shortest):
                        shortest = newPath
        return shortest

def DFS(graph, start, visited=None):
    if visited is None:
        visited=[]
    visited.extend(start)
    for next in graph[start]:
        if next not in visited:
            DFS(graph, next, visited)
    return visited

def iDFS(graph, start): #iterative
    visited=[]
    stack=[start]
    while stack:
        vertex=stack.pop()
        if vertex not in visited:
            visited.extend(vertex)
            stack.extend(node for node in graph[vertex] if node not in visited)
    return visited

def BFS(graph, start):
    visited=[]
    queue=[start]
    while queue:
        vertex=queue.pop(0)
        if vertex not in visited:
            visited.extend(vertex)
            queue.extend(node for node in graph[vertex] if node not in visited)
    return visited

def DLS(graph, start, depth, visited=None): #Depth Limited Search
    if visited is None:
        visited=[]
    if depth <=0:
        return
    visited.extend(start)
    for next in graph[start]:
        if next not in visited:
            DLS(graph, next, depth-1, visited)
    return visited

def iDeep(graph, start, maxDepth):  #Iterative Deepening
    for i in range(maxDepth):
        print(DLS(graph, start, i+1), end=' ')
    print()

def BDS(graph, start, goal):    #Bi-Directional Search
    visited1=[]
    visited2=[]
    queue1=[start,[]]
    queue2=[goal,[]]
    newParents1=[]
    newParents2=[]
    while queue1 and queue2:
        if queue1:
            vertex=queue1.pop(0)
            parents=queue1.pop(0)
            newParents1=parents[:]
            newParents1.extend(vertex)
            if vertex==goal or vertex in visited2:
                break
            if vertex not in visited1:
                visited1.extend(vertex)
                for node in graph[vertex]:
                    if node not in visited1:
                        queue1.extend(node)
                        queue1.append(newParents1)
        if queue2:
            vertex=queue2.pop(0)
            parents=queue2.pop(0)
            newParents2=parents[:]
            newParents2.extend(vertex)
            if vertex==start or vertex in visited1:
                break
            if vertex not in visited2:
                visited2.extend(vertex)
                for node in graph:
                    if vertex in graph[node] and node not in visited2:
                        queue2.extend(node)
                        queue2.append(newParents2)
    newParents2.reverse()
    newParents1.extend(newParents2[1:])
    return newParents1

print(findPath(graph,'A','D'))
print(findAllPaths(graph,'A','D'))
print(findShortestPath(graph,'A','D'))
print(DFS(graph,'A'))
print(iDFS(graph,'A'))
print(BFS(graph,'A'))
print(DLS(graph,'A', 2))
iDeep(graph,'A', 4)
print(BDS(graph, 'A', 'D'))