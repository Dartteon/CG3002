from urllib.request import urlopen
import json 
import heapq

base_url = "http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=%s&Level=%s"

class NODE:
    '''Used instead of map data for ease of use'''
    def __init__(self, nodeId, x, y, nodeName, linkTo):
        '''Initialise node'''
        self.nodeId = int(nodeId)
        self.x = int(x)
        self.y = int(y)
        self.nodeName = nodeName
        self.linkTo = linkTo
        self._f = 0
        self._g = 0
        self._h = 0
        self.parent = None
        
    def __lt__(self, other):
        '''Return true if self node has smaller f than other node'''
        return self.f < other.f

    def __eq__(self,other):
        '''Return true if both nodes have the same node ID'''
        return self.nodeId == other.nodeId
    
    @property
    def f(self):
        '''Returns private f value'''
        return self._f
    
    @property
    def g(self):
        '''Returns private g value'''
        return self._g
    
    @g.setter
    def g(self, cost):
        '''Sets private g value and update private f value'''
        self._g = cost
        self._f = cost + self.h
        
    @property
    def h(self):
        '''Returns private h value'''
        return self._h
    
    @h.setter
    def h(self, heuristic):
        '''Sets private h value and update private f value'''
        self._h = heuristic
        self._f = self.g + heuristic
        
def get_json(buildingName, floorNumber):
    '''Returns map data from building name and floor number'''
    url = base_url % (buildingName, floorNumber)
    response = urlopen(url)
    data = json.loads(response.read())
    return data

def heuristic(goalNode):
    '''Returns Euclidean Distance between all nodes and goal node'''
    h=[]
    for node in nodeList:
        dx = goalNode.x - node.x
        dy = goalNode.y - node.y
        h.append((dx**2 + dy**2)**0.5)
    return h

def get_nodes():
    nodeList=[]
    for node in jsonmap['map']:
        nodeList.append(NODE(node['nodeId'],node['x'],node['y'],node['nodeName'],node['linkTo']))
    return nodeList

def distance_between(node1, node2):
    dx = node1.x - node2.x
    dy = node1.y - node2.y
    distance = ((dx**2 + dy**2)**0.5)
    return distance

def get_route():
    currentNode = goalNode
    route = [goalNode.nodeId]
    while currentNode.parent is not None:
        route.append(currentNode.parent)
        currentNode = nodeList[currentNode.parent-1]
    route.reverse()
    return route

def newline():
    print('')

##Choose map to download
# buildingNameList = ['DemoBuilding', 'COM1', 'COM2']
# floorNumberList = {'DemoBuilding':[1, 2, 3], 'COM1':[1, 2], 'COM2':[2, 3]}
# number = -1
# floorNumber= -1

# while (number - 1) not in range(len(buildingNameList)):
#     print('Building?\n1: DemoBuilding\n2: COM1\n3: COM2')
#     number = int(input())
# buildingName = buildingNameList[number-1]

# while floorNumber not in floorNumberList[buildingName]:
#     newline()
#     print('Floor number?')
#     print(floorNumberList[buildingName])
#     floorNumber = int(input())

buildingName = input('Building name:')
floorNumber = input('Floor number:')

jsonmap = get_json(buildingName, floorNumber)
info = jsonmap['info']
wifi = jsonmap['wifi']
nodeList = get_nodes()

newline()
print(buildingName, 'Floor', floorNumber, 'has node IDs from 1 to', len(nodeList))
startNode = nodeList[int(input('Start node ID: '))-1]
goalNode = nodeList[int(input('Goal node ID: '))-1]
newline()

hList = heuristic(goalNode)

for index in range(len(hList)):
    nodeList[index].h = hList[index]

openList = []
closedList = []
orderList = []

heapq.heappush(openList, startNode)

while openList:
    currentNode = heapq.heappop(openList)
    orderList.append(currentNode.nodeId)
    if currentNode == goalNode:
        openList = []
    else:
        adjacentNodeId = [int(x) for x in currentNode.linkTo.split(',')]
        for nodeId in adjacentNodeId:
            gTemp = currentNode.g + distance_between(nodeList[nodeId-1], currentNode)
            if nodeList[nodeId-1] in openList and nodeList[nodeId-1].g <= gTemp:
                pass
            elif nodeList[nodeId-1] in closedList and nodeList[nodeId-1].g <= gTemp:
                pass
            else:
                nodeList[nodeId-1].parent = currentNode.nodeId
                nodeList[nodeId-1].g = gTemp
                heapq.heappush(openList, nodeList[nodeId-1])
        
    closedList.append(currentNode)

route = get_route()
print('Order of visited nodes: ', orderList)
print('Route is: ', route)