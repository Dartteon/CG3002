from urllib.request import urlopen
from gtts import gTTS
import json 
import heapq
import math

# blabla = ("Spoken text")
# tts = gTTS(text=blabla, lang='en')
# tts.save("C:/test.mp3")

base_url = "http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=%s&Level=%s"

def main():
    # Integer input mode for fixed list of maps
    buildingName = int_to_buildingName()
    floorNumber = int_to_floodNumber(buildingName)

    # Text input mode for new maps
    # buildingName = input('Building name: ')
    # floorNumber = input('Floor number: ')

    jsonmap = get_json(buildingName, floorNumber)
    info = jsonmap['info']
    wifi = jsonmap['wifi']
    northAt = int(info['northAt'])
    nodeList = get_nodes(jsonmap)

    newline()
    print('Name of the first node at', buildingName, 'Floor', floorNumber, 'is', nodeList[0].nodeName)
    print(buildingName, 'Floor', floorNumber, 'has node IDs from 1 to', len(nodeList))
    print('North is at', info['northAt'], 'degrees')
    startNode = nodeList[int(input('Start node ID: '))-1]
    goalNode = nodeList[int(input('Goal node ID: '))-1]
    newline()

    hList = heuristic(goalNode, nodeList)

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

    route = get_route(goalNode, nodeList)
    print('Order of visited nodes: ', orderList)
    print('Route is: ', route)

    path_to_goal(nodeList, route, northAt)
    
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

def heuristic(goalNode, nodeList):
    '''Returns Euclidean Distance between all nodes and goal node'''
    h=[]
    for node in nodeList:
        dx = goalNode.x - node.x
        dy = goalNode.y - node.y
        h.append((dx**2 + dy**2)**0.5)
    return h

def get_nodes(jsonmap):
    nodeList=[]
    for node in jsonmap['map']:
        nodeList.append(NODE(node['nodeId'],node['x'],node['y'],node['nodeName'],node['linkTo']))
    return nodeList

def distance_between(node1, node2):
    dx = node1.x - node2.x
    dy = node1.y - node2.y
    distance = ((dx**2 + dy**2)**0.5)
    return distance

def get_route(goalNode, nodeList):
    currentNode = goalNode
    route = [goalNode.nodeId]
    while currentNode.parent is not None:
        route.append(currentNode.parent)
        currentNode = nodeList[currentNode.parent-1]
    route.reverse()
    return route

def displacement_from_position(position, node, northAt):
    dx = node.x - position['x']
    dy = node.y - position['y']
    distance = ((dx**2 + dy**2)**0.5)
    if distance == 0.0:
        turnAngle = 0
    else:
        bearing = ((90 - math.degrees(math.atan2(dy, dx))) - northAt) % 360
        turnAngle = bearing - position['heading']
    if turnAngle > 180:
        turnAngle -= 360
    displacement = {'distance':int(distance), 'turnAngle':int(turnAngle)}
    return displacement

def path_to_goal(nodeList, route, northAt):
    index = 0
    previousNode = nodeList[route[index]-1]
    position = {'x':nodeList[route[index]-1].x, 'y':nodeList[route[index]-1].y, 'heading':0}
    while previousNode is not nodeList[route[len(route)-1]-1]:
        index += 1
        nextNode = nodeList[route[index]-1]
        displacement = displacement_from_position(position, nextNode, northAt)
        while displacement['distance'] > 10:
            print('To reach the next node (node ID ', nextNode.nodeId, ') turn ', displacement['turnAngle'], ' degrees and walk ', displacement['distance'], ' cm', sep='')
            position['x'] = int(input('Current x: '))
            position['y'] = int(input('Current y: '))
            position['heading'] = int(input('Current heading: '))
            displacement = displacement_from_position(position, nextNode, northAt)
        previousNode = nextNode

def newline():
    print('')

def int_to_buildingName():
    buildingNameList = ['DemoBuilding', 'COM1', 'COM2']
    number = 0        
    while (number - 1) not in range(len(buildingNameList)):
        try:
            print('Building?\n1: DemoBuilding\n2: COM1\n3: COM2')
            number = int(input())
            if number - 1 not in range(len(buildingNameList)):
                print('Please enter an integer building index number from the given list.')
        except ValueError:
            print('That was not an integer.\nPlease enter an integer building index number from the given list.')
        newline()
    return buildingNameList[number-1]

def int_to_floodNumber(buildingName):
    floorNumberList = {'DemoBuilding':[1, 2, 3], 'COM1':[1, 2], 'COM2':[2, 3]}
    floorNumber= 0    
    while floorNumber not in floorNumberList[buildingName]:
        try:
            print('Floor number?')
            print(floorNumberList[buildingName])
            floorNumber = int(input())
            if floorNumber not in floorNumberList[buildingName]:
                print('Please enter an integer floor number from the given list.')
        except ValueError:
            print('That was not an integer.\nPlease enter an integer floor number from the given list.')
        newline()
    return floorNumber

main()