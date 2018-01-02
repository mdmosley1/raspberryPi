from PathPlanner.adjGraph import Graph, Vertex
import numpy as np

def occMap2Graph(gMap,x_spacing,y_spacing):
    # input is an occupancy grid map
    rows,cols = gMap.shape
    myGraph = Graph()
    for row in range(rows):
        for col in range(cols):
            if gMap[row,col]==0:
                nodeId = posToNodeId(row,col,cols)
                pos0 = (col+.5)*x_spacing,(row+.5)*y_spacing
                myGraph.addVertex(nodeId,pos0)
                newPositions = genLegalMoves(row,col,rows,cols,gMap)
                for r,c in newPositions:
                    nid = posToNodeId(r,c,cols)                    
                    pos1 = (c+.5)*x_spacing,(r+.5)*y_spacing                    
                    myGraph.addVertex(nid,pos1)
                    dist = getDistBtwnNodes(pos0,pos1)
                    myGraph.addEdge(nodeId,nid,dist)
    return myGraph

def genLegalMoves(row,col,rows,cols,gMap):
    newMoves = []
    moveOffsets = [(-1,0),(1,0),(0,-1),(0,1),   # up,right,left,down
                   (-1,-1),(-1,1),(1,1),(1,-1), # allow for diagonal movement
                   (1,2),(-1,2),(1,-2),(-1,-2),  # allow nonadjacent nodes to be connected
                   (-2,1),(2,1),(-2,-1),(2,-1)]  
    for i in moveOffsets:
        newRow = row + i[0]
        newCol = col + i[1]
        if legalCoord(newRow,newCol,rows,cols,gMap):
            newMoves.append((newRow,newCol))
    return newMoves

def legalCoord(row,col,rows,cols,gMap):
    if col >= 0 and col < cols \
       and row >=0 and row < rows\
       and gMap[row,col]==0:
        return True
    else:
        return False

def posToNodeId(row, column, cols):
    return (row * cols) + column

def getDistBtwnNodes(pos0,pos1):
    dist = np.sqrt((pos1[1]-pos0[1])**2 + (pos1[0]-pos0[0])**2)
    return dist
