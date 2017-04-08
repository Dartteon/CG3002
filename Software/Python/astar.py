from urllib2 import urlopen
from Firmware_Receiver import SerialCommunicator
from Firmware_Receiver import Arduino
# from Firmware_Dummy import SerialCommunicator
# from Firmware_Dummy import Arduino
# import pyttsx
import json
import heapq
import math
from stepCounter import read_step_counter
import time
import os
import threading
import messages
import constants
from voiceThread import VoiceHandler
from voiceThread import INSTRUCTION
from gpio import Keypad
# from initialisation import get_confirmation
import re

TTS_DELAY = 3.5 # Text to speech delay in seconds

# Initialising Text-to-Speech
# engine = pyttsx.init()
serial = SerialCommunicator()
# arduino = Arduino()
voiceOutput = VoiceHandler()
voiceThread = threading.Thread(target=voiceOutput.voiceLoop)
voiceThread.start()
keypad = Keypad()
totalNodeDistance = 0
distanceOffset = 0

def main():
    # Integer input mode for fixed list of maps
    # buildingName = int_to_buildingName()
    # floorNumber = int_to_floorNumber(buildingName)
    # Text input mode for new maps
    # info = None
    # while info is None:
    # Get start map
    # loop until user confirms the inputs
    # Get start details
    buildingMode = 1
    while True:
        # Get building
        voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_BUILDING_NUMBER.format(type = 'starting'), constants.HIGH_PRIORITY))
        print('Starting building name or number: ')
        buildingNameOrNumber = str(keypad.getKeysInput())
        voiceOutput.addToQueue(INSTRUCTION(buildingNameOrNumber, constants.HIGH_PRIORITY))
        time.sleep(1)

        # Get floor
        voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_BUILDING_LEVEL.format(type = 'starting'), constants.HIGH_PRIORITY))
        print('Starting floor number: ')
        floorNumber = str(keypad.getKeysInput())
        voiceOutput.addToQueue(INSTRUCTION(floorNumber, constants.HIGH_PRIORITY))
        time.sleep(1)

        # Get start node
        voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_START_NODE, constants.HIGH_PRIORITY))
        print('Start node ID: ')
        startNodeRaw = int(keypad.getKeysInput())
        voiceOutput.addToQueue(INSTRUCTION(str(startNodeRaw), constants.HIGH_PRIORITY))
        time.sleep(1)

        # Get confirmation
        confirmation = get_confirmation(buildingNameOrNumber, floorNumber, startNodeRaw)
        if confirmation == '1':
            break

    # Get end details
    while True:
        # Get building
        voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_BUILDING_NUMBER.format(type = 'destination'), constants.HIGH_PRIORITY))
        print('Destination building name or number: ')
        destinationBuildingNameOrNumber = str(keypad.getKeysInput())
        voiceOutput.addToQueue(INSTRUCTION(destinationBuildingNameOrNumber, constants.HIGH_PRIORITY))
        time.sleep(1)

        # Get floor
        voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_BUILDING_LEVEL.format(type = 'destination'), constants.HIGH_PRIORITY))
        print('Destination floor number: ')
        destinationFloorNumber = str(keypad.getKeysInput())
        voiceOutput.addToQueue(INSTRUCTION(destinationFloorNumber, constants.HIGH_PRIORITY))
        time.sleep(1)

        # Get end node
        voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_END_NODE, constants.HIGH_PRIORITY))
        print('Goal node ID: ')
        goalNodeRaw = int(keypad.getKeysInput())
        voiceOutput.addToQueue(INSTRUCTION(str(goalNodeRaw), constants.HIGH_PRIORITY))
        time.sleep(1)

        # Get confirmation
        confirmation = get_confirmation(destinationBuildingNameOrNumber, destinationFloorNumber, goalNodeRaw)
        if confirmation == '1':
            break

    if (buildingNameOrNumber != destinationBuildingNameOrNumber) or (floorNumber != destinationFloorNumber):
        mapList = []
        get_map_list('0', buildingNameOrNumber, floorNumber, destinationBuildingNameOrNumber, destinationFloorNumber, mapList)
        buildingMode = 2
    else:
        mapList = ['{building}-{floor}'.format(building = buildingNameOrNumber, floor = floorNumber)]

    if not constants.IS_DEBUG_MODE:
        serial.serialFlush()
        serial.handshakeWithArduino()

    if buildingMode == 1:
        voiceOutput.addToQueue(INSTRUCTION('getting map', constants.HIGH_PRIORITY))
        jsonmap = get_json(buildingNameOrNumber, floorNumber)
        info = jsonmap['info']
        northAt = int(info['northAt'])
        nodeList = get_nodes(jsonmap)

        startNode = nodeList[int(startNodeRaw)-1]
        goalNode = nodeList[int(goalNodeRaw)-1]

        print 'Name of the first node at building' + buildingNameOrNumber + ' Floor ' + str(floorNumber) + ' is ' + nodeList[0].nodeName
        print 'North is at ' + info['northAt'] + ' degrees'

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
    elif buildingMode == 2:
        # Prepare data

        #Loop
        while mapList:
            print mapList
            if len(mapList) == 1:
                currMap = mapList.pop(0)
                voiceOutput.addToQueue(INSTRUCTION('getting map', constants.HIGH_PRIORITY))
                jsonmap = get_json(currMap.split('-')[0], currMap.split('-')[1])
                info = jsonmap['info']
                northAt = int(info['northAt'])
                nodeList = get_nodes(jsonmap)
                print int(startNodeRaw)
                startNode = nodeList[int(startNodeRaw)-1]
                goalNode = nodeList[int(goalNodeRaw)-1]

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

            else:
                currMap = mapList.pop(0)
                voiceOutput.addToQueue(INSTRUCTION('getting map', constants.HIGH_PRIORITY))
                print currMap
                jsonmap = get_json(currMap.split('-')[0], currMap.split('-')[1])
                info = jsonmap['info']
                print info
                northAt = int(info['northAt'])
                nodeList = get_nodes(jsonmap)
                print jsonmap
                for node in nodeList:
                    print node.nodeName
                    goalName = 'TO '+mapList[0]
                    if goalName in node.nodeName:
                        goalNode = node
                        nextStartNodeRaw = node.nodeName.split('-').pop()
                        break
                startNode = nodeList[int(startNodeRaw)-1]
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
                startNodeRaw = nextStartNodeRaw

    voiceOutput.addToQueue(INSTRUCTION(messages.DESTINATION_REACHED, constants.HIGH_PRIORITY))

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

def get_confirmation(buildingNameOrNumber, floorNumber, nodeRaw):
    while True:
        voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_CONFIRMATION.format(building = buildingNameOrNumber, floor = floorNumber, node = nodeRaw), constants.HIGH_PRIORITY))
        print('1 to confirm, 2 to try again: ')
        confirmation = str(keypad.getKeysInput())
        voiceOutput.addToQueue(INSTRUCTION(confirmation, constants.HIGH_PRIORITY))
        time.sleep(1)
        if confirmation == '1' or confirmation == '2':
            return confirmation
            # pass
        else:
            voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_OUT_OF_RANGE, constants.HIGH_PRIORITY))

def get_map_list(prevNode, buildingStart, floorStart, buildingEnd, floorEnd, mapList):
    # If same building and level
    if (buildingStart == buildingEnd) and (floorStart == floorEnd):
        mapList.append('{building}-{floor}'.format(building = buildingStart, floor = floorStart))
        return
    # Else process starting map
    startMap = get_json(buildingStart, floorStart)
    newNode = False
    hasConnector = False
    try:
        for node in startMap['map']:
            if re.match(r'TO \w+-\d+-\d+', node['nodeName']):
                hasConnector = True
                connector = re.findall(r'\d+', node['nodeName'][2:])
                connectingMap = connector[0] + '-' + connector[1]
                connectingNodeID = connector[2]
                if prevNode != connectingNodeID:
                    newNode = True
                    mapList.append('{building}-{floor}'.format(building = buildingStart, floor = floorStart))
                    # print buildingStart+'-'+floorStart
                    get_map_list(node['nodeId'], connector[0], connector[1], buildingEnd, floorEnd, mapList)
    except Exception as e:
        print e
        debug_print('No map {building}-{floor}'.format(building = buildingStart, floor = floorStart))
        mapList.pop()
        return
    if hasConnector:
        if not newNode:
            debug_print('No new connector node in {building}-{floor}'.format(building = buildingStart, floor = floorStart))
            mapList.pop()
    else:
        debug_print('No connector node in {building}-{floor}'.format(building = buildingStart, floor = floorStart))

def get_json(buildingName, floorNumber):
    '''Returns map data from building name and floor number'''
    try:
        voiceOutput.addToQueue(INSTRUCTION('trying cache', constants.HIGH_PRIORITY))
        if constants.IS_DEBUG_MODE:
            fileName = ''
        else:
            fileName = '/home/pi/CG3002/Software/Python/'
        fileName += str(buildingName) + '-' + str(floorNumber) + '.json'
        with open(fileName, 'r') as file:
            data = json.load(file)
            debug_print(data)
        return data
    except Exception as e:
        voiceOutput.addToQueue(INSTRUCTION('cache failed', constants.HIGH_PRIORITY))
        print e
        try:
            voiceOutput.addToQueue(INSTRUCTION('trying wifi', constants.HIGH_PRIORITY))
            info = None
            url = constants.BASE_URL.format(building = buildingName, level = floorNumber)
            response = urlopen(url)
            data = json.loads(response.read())
            debug_print(data)
            info = data['info']
            if info != None:
                fileName = str(buildingName) + '-' + str(floorNumber) + '.json'
                with open(fileName, 'w') as file:
                    json.dump(data, file)
                    debug_print(data)
                return data
        except Exception as e:
            voiceOutput.addToQueue(INSTRUCTION('wifi failed', constants.HIGH_PRIORITY))
            print e

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
        if re.match(r'TO \d-\d-\d',node['nodeName']):
            connector = re.findall(r'\d+', node['nodeName'])
            connectingMap = connector[0] + '-' + connector[1]
            connectingNodeID = connector[2]
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

def path_to_goal(nodeList, route, northAt):
    global totalNodeDistance
    index = 0
    prevTotalDistance = 0
    counter=0
    counterx=0
    previousNode = nodeList[route[index]-1]
    goalNode = nodeList[route[len(route)-1]-1]
    position = {'x':nodeList[route[index]-1].x, 'y':nodeList[route[index]-1].y, 'heading':0}
    while previousNode != goalNode:
        isNextNodeReached = False
        index += 1
        nextNode = nodeList[route[index]-1]
        # displacement = displacement_from_position(position, nextNode, northAt)
        nodeToNode = path_to_node(nextNode, previousNode, northAt)
        # print 'calculate node to node'
        # If the user is within 20cm to the next node, we will take it as they have reached that node

        # while distanceToNode['distance'] > 20:
        instruction = ''
        instructionTimeStamp = 0.0
        distanceTimeStamp = 0.0
        while not isNextNodeReached:
            audio = False
            step = False
            distanceAudio = False
            step = False
            data = request_data_from_arduino(prevTotalDistance)
            if data['distance'] > prevTotalDistance:
                step = True
                msg = str(data['distance']) + ' steps'
                prevStepsTaken = data['distance']
                voiceOutput.addToQueue(INSTRUCTION(msg, constants.MED_PRIORITY))
                print(data['distance'], ' steps')
                prevTotalDistance = data['distance']
            else:
                print(data['distance'], ' steps')
                msg = str(data['distance']) + ' steps'

            if time.time() - instructionTimeStamp > TTS_DELAY:
                instructionTimeStamp = time.time()
                audio = True
            if time.time() - distanceTimeStamp > 2 * TTS_DELAY:
                if not step:
                    if not audio:
                        distanceTimeStamp = time.time()
                        distanceAudio = True

            distanceToNode = nodeToNode['distance'] - (data['distance'] * constants.STRIDE_LENGTH - totalNodeDistance)

            if distanceToNode < constants.STRIDE_LENGTH:
                print str(float(distanceToNode/constants.STRIDE_LENGTH))
                stepsToNode = round(float(distanceToNode/constants.STRIDE_LENGTH), 1)
            else:
                stepsToNode = int(distanceToNode/constants.STRIDE_LENGTH)

            if distanceToNode <= 0:
                # break
                isNextNodeReached = True
            turnAngle = nodeToNode['nodeBearing'] - data['direction'] + 20 #offset
            if turnAngle > 180:
                turnAngle -= 360
            elif turnAngle <= -180:
                turnAngle += 360

            if abs(turnAngle) > 10:
                instruction = 'Turn ' + str(turnAngle)
                print instruction
                if not counter:
                    voiceOutput.addToQueue(INSTRUCTION(instruction, constants.LOW_PRIORITY))
                    counter += 1
                else:
                    counter = (counter + 1) % 4
            else:
                instruction = 'Walk ' + str(stepsToNode)
                print str(distanceToNode)
                print instruction
                if not counterx:
                    voiceOutput.addToQueue(INSTRUCTION(instruction, constants.LOWEST_PRIORITY))
                    counterx += 1
                else:
                    counterx = (counterx + 1) % 4
        reached_message = messages.REACHED_NEXT_NODE.format(id = str(nextNode.nodeId))
        totalNodeDistance += nodeToNode['distance']
        print reached_message
        voiceOutput.addToQueue(INSTRUCTION(reached_message, constants.HIGH_PRIORITY))
        previousNode = nextNode
        isNextNodeReached = False

def to_user(instruction, audio):
    if instruction == '':
        return
    print(instruction)
    if audio:
        print('audio')
        text_to_speech(instruction)
    newline()

def path_to_node(nextNode, previousNode, northAt):
    dx = nextNode.x - previousNode.x
    dy = nextNode.y - previousNode.y
    distance = ((dx**2 + dy**2)**0.5)
    if distance == 0.0:
        nodeBearing = 0
    else:
        nodeBearing = ((90 - math.degrees(math.atan2(dy, dx))) - northAt) % 360
    nodeToNode = {'distance':int(distance), 'nodeBearing':int(nodeBearing)}
    return nodeToNode

def request_data_from_arduino(prevTotalDistance):
    global distanceOffset

    dataRequested = serial.serialRead()
    debug_print(dataRequested)
    if dataRequested['distance'] < prevTotalDistance - distanceOffset:
        distanceOffset = prevTotalDistance
    dataRequested['distance'] += distanceOffset
    debug_print(dataRequested)
    return dataRequested

def text_to_speech(text):
    os.system("espeak -s 200 -v en+f3 '{msg}' 2>/dev/null".format(msg = text))

def debug_print(input):
    if constants.IS_DEBUG_MODE:
        print(input)

def newline():
    print('')

# def mean(list):
#     return float(sum(list)) / max(len(list), 1)

text_to_speech(messages.PROGRAM_INIT)
# voiceOutput.addToQueue(INSTRUCTION(messages.PROGRAM_INIT, 0))
main()
