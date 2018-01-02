from PathPlanner.adjGraph import Graph, Vertex
from PathPlanner.priorityQueue import PriorityQueue

def dijkstra(aGraph,start,goal):
    pq = PriorityQueue()
    start.setDistance(0)
    pq.buildHeap([(v.getDistance(),v) for v in aGraph])
    while not pq.isEmpty():
        currentVert = pq.delMin()
        if currentVert == goal:
            path = getPath(goal)
            return path # end algorithm as soon as goal is reached
        for nextVert in currentVert.getConnections():
            newDist = currentVert.getDistance() \
                    + currentVert.getWeight(nextVert)
            if newDist < nextVert.getDistance():
                nextVert.setDistance( newDist )
                nextVert.setPred(currentVert)
                pq.decreaseKey(nextVert,newDist)

def getPath(goal):
    pred = goal.getPred()
    path = [goal.pos]
    while not pred == None:
        path.append(pred.pos)
        pred = pred.getPred()
    return list(reversed(path))
