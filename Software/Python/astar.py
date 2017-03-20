from urllib2 import urlopen
from Firmware_Receiver import SerialCommunicator
from Firmware_Receiver import Arduino
# from Firmware_Dummy import SerialCommunicator
# from Firmware_Dummy import Arduino
import pyttsx
import json 
import heapq
import math
from stepCounter import read_step_counter
import time

# Base URL for map info download
base_url = "http://showmyway.comp.nus.edu.sg/getMapInfo.php?Building=%s&Level=%s"
# Initialising Text-to-Speech
engine = pyttsx.init()
serial = SerialCommunicator()
arduino = Arduino()

def main():
    # Integer input mode for fixed list of maps
    # buildingName = int_to_buildingName()
    # floorNumber = int_to_floorNumber(buildingName)
    # Text input mode for new maps
    info = None
    while info is None:
        buildingNameOrNumber = str(raw_input('Building name or number: '))
        floorNumber = raw_input('Floor number: ')

        jsonmap = get_json(buildingNameOrNumber, floorNumber)
        info = jsonmap['info']
        if info is None:
            print('No map available for this building name and floor number combination.')
    wifi = jsonmap['wifi']
    northAt = int(info['northAt'])
    nodeList = get_nodes(jsonmap)

    try:
        buildingName = int(buildingNameOrNumber)
        buildingName = 'Building ' + str(buildingName)
    except ValueError:
        buildingName = buildingNameOrNumber

    # newline()
    print 'Name of the first node at ' + buildingName + ' Floor ' + str(floorNumber) + ' is ' + nodeList[0].nodeName
    print 'North is at ' + info['northAt'] + ' degrees'
    while True:
        try:
            print buildingName + ' Floor ' + str(floorNumber) + ' has node IDs from 1 to ' + str(len(nodeList))
            startNode = nodeList[int(raw_input('Start node ID: '))-1]
            goalNode = nodeList[int(raw_input('Goal node ID: '))-1]
        except IndexError:
            print('One or more node IDs are out of range')
            newline()
            pass
        else:
            break


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
    print 'Order of visited nodes: ' + str(orderList)
    print 'Route is: ' + str(route)

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
    try:
        fileName = str(buildingName) + '-' + str(floorNumber) + '.json'
        with open(fileName, 'r') as file:
            data = json.load(file)
    except Exception as e:
        url = base_url % (buildingName, floorNumber)
        response = urlopen(url)
        data = json.loads(response.read())
        info = data['info']
        if info is not None:
            fileName = str(buildingName) + '-' + str(floorNumber) + '.json'
            with open(fileName, 'w') as file:
                json.dump(data, file)
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

# def displacement_from_position(position, node, northAt):
#     dx = node.x - position['x']
#     dy = node.y - position['y']
#     distance = ((dx**2 + dy**2)**0.5)
#     if distance == 0.0:
#         turnAngle = 0
#     else:
#         bearing = ((90 - math.degrees(math.atan2(dy, dx))) - northAt) % 360
#         turnAngle = bearing - position['heading']
#     if turnAngle > 180:
#         turnAngle -= 360
#     elif turnAngle <= -180:
#         turnAngle += 360
#     displacement = {'distance':int(distance), 'turnAngle':int(turnAngle)}
#     return displacement

def path_to_goal(nodeList, route, northAt):
    index = 0
    totalNodeDistance = 0
    prevTotalDistance = 0
    previousNode = nodeList[route[index]-1]
    goalNode = nodeList[route[len(route)-1]-1]
    position = {'x':nodeList[route[index]-1].x, 'y':nodeList[route[index]-1].y, 'heading':0}
    arduino.handshakeWithArduino()
    instructionTimeStamp = 0.0
    while previousNode is not goalNode:
        index += 1
        nextNode = nodeList[route[index]-1]
        # displacement = displacement_from_position(position, nextNode, northAt)
        nodeToNode = path_to_node(nextNode, previousNode, northAt)
        # If the user is within 20cm to the next node, we will take it as they have reached that node
        
        # while distanceToNode['distance'] > 20:
        instruction = ''
        while True:
            data = received_data_from_arduino(position)
            if data['distance'] < prevTotalDistance:
                to_user(instruction, instructionTimeStamp)
                continue
            else:
                prevTotalDistance = data['distance']
            distanceToNode = nodeToNode['distance'] - (data['distance'] - totalNodeDistance)
            if distanceToNode <= 0:
                break
            turnAngle = nodeToNode['nodeBearing'] - data['direction']
            if turnAngle > 180:
                turnAngle -= 360
            elif turnAngle <= -180:
                turnAngle += 360
            instruction =  'Turn ' + str(turnAngle) + ' degrees and walk ' + str(distanceToNode) + ' cm'
            to_user(instruction, instructionTimeStamp)
            
                # text_to_speech(instruction)
            
            # direction = direction_for_user(displacement['turnAngle'])
            # instruction = instruction_for_user(direction, displacement['distance'], nextNode.nodeId)
            
            # print instruction
            # if readInstruction == 1:
            #     text_to_speech(instruction)
            

            # position['x'] = int(raw_input('Current x: '))
            # position['y'] = int(raw_input('Current y: '))
            # position['heading'] = int(raw_input('Current heading: '))
            # displacement = displacement_from_position(position, nextNode, northAt)
        reached_message = 'You have reached node ID ' + str(nextNode.nodeId)
        totalNodeDistance += nodeToNode['distance']
        print reached_message
        text_to_speech(reached_message)
        previousNode = nextNode
    text_to_speech('You have reached the final node')

def to_user(instruction, instructionTimeStamp):
    if instruction == '':
        return
    print(instruction)
    if time.time() - instructionTimeStamp > 3.0:
        instructionTimeStamp = time.time()
        print('audio')

def path_to_node(nextNode, previousNode, northAt):
    dx = nextNode.x - previousNode.x
    dy = nextNode.y - previousNode.y
    distance = ((dx**2 + dy**2)**0.5)
    if distance == 0.0:
        nodeBearing = 0
    else:
        nodeBearing = ((90 - math.degrees(math.atan2(dy, dx))) - northAt) % 360
    # if nodeBearing > 180:
    #     nodeBearing -= 360
    # elif nodeBearing <= -180:
    #     nodeBearing += 360
    nodeToNode = {'distance':int(distance), 'nodeBearing':int(nodeBearing)}
    return nodeToNode


def received_data_from_arduino(position):
    dataReceived = serial.serialRead()
    # distance = dataReceived['distance']
    # direction = dataReceived['direction']

    # response = {}
    # for i in range(len(dataReceived['dir'])):
    #     response =  read_step_counter(dataReceived['accelx'][i], dataReceived['accelz'][i], long(dataReceived['timestamp']))
    #     if response['status'] == 1: # Step taken
    #         position['x'] += 75 * (math.sin(math.radians(int(dataReceived['dir']))))
    #         position['y'] += 75 * (math.cos(math.radians(int(dataReceived['dir']))))
    #     position['heading'] = int(dataReceived['dir'])

    # print 'direction: ' + str(direction) + ' acceleration X, Y, Z: ' + str(meanAccelX) + ', ' + str(meanAccelY) + ', ' + str(meanAccelZ)
    return dataReceived

# def direction_for_user(turnAngle):
#     # If angle is between -10 and 10 degrees, ignore it and let the user walks straight
#     if turnAngle >= -20 and turnAngle <= 20:
#         direction = 'straight'

#     # Slight turn to the right and left
#     elif turnAngle > 21 and turnAngle <=65:
#         direction = 'slight right turn'
#     elif turnAngle < -21 and turnAngle >= -65:
#         direction = 'slight left turn'

#     # Right angle turn to the right and left
#     elif turnAngle > 66 and turnAngle <= 125:
#         direction = 'right turn'
#     elif turnAngle < -66 and turnAngle >= -125:
#         direction = 'left turn'

#     # Major turn to the right and left
#     elif turnAngle > 126 and turnAngle <= 160:
#         direction = 'right and further slight right turn'
#     elif turnAngle < - 126 and turnAngle >= -160:
#         direction = 'left and further slight left turn'
#     else:
#         direction = 'uturn'
#     return direction

# def instruction_for_user(direction, distance, nextNode):
#     if direction is 'straight':
#         instruction = 'To reach the next node (node ID ' + str(nextNode) + ') walk straight ' + str(distance) + ' cm.'
#     else:
#         instruction = 'To reach the next node (node ID ' + str(nextNode) + ') make a ' + direction + ' and walk ' + str(distance) + ' cm.'
#     return instruction

# def int_to_buildingName():
#     buildingNameList = ['DemoBuilding', 'COM1', 'COM2']
#     number = 0        
#     while (number - 1) not in range(len(buildingNameList)):
#         try:
#             print('Building?\n1: DemoBuilding\n2: COM1\n3: COM2')
#             number = int(raw_input())
#             if number - 1 not in range(len(buildingNameList)):
#                 print 'Please enter an integer building index number from the given list.'
#         except ValueError:
#             print 'That was not an integer.\nPlease enter an integer building index number from the given list.'
#         newline()
#     return buildingNameList[number-1]

# def int_to_floorNumber(buildingName):
#     floorNumberList = {'DemoBuilding':[1, 2, 3], 'COM1':[1, 2], 'COM2':[2, 3]}
#     floorNumber= 0    
#     while floorNumber not in floorNumberList[buildingName]:
#         try:
#             print 'Floor number?'
#             print str(floorNumberList[buildingName])
#             floorNumber = int(raw_input())
#             if floorNumber not in floorNumberList[buildingName]:
#                 print 'Please enter an integer floor number from the given list.'
#         except ValueError:
#             print 'That was not an integer.\nPlease enter an integer floor number from the given list.'
#         newline()
#     return floorNumber

def text_to_speech(text):
    # For testing on Chris' mac
    # print 'Text-to-speech goes here'
    engine.say(text)
    engine.runAndWait()

def newline():
    print('')

# def mean(list):
#     return float(sum(list)) / max(len(list), 1)

text_to_speech('R pi started and program initialising')
main()