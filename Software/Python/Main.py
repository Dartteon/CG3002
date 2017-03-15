#import
import Node

def main():
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

    print 'Name of the first node at ' + buildingName + ' Floor ' + str(floorNumber) + ' is ' + nodeList[0].nodeName
    print buildingName + ' Floor ' + str(floorNumber) + ' has node IDs from 1 to ' + str(len(nodeList))
    print 'North is at ' + info['northAt'] + ' degrees'
    startNode = nodeList[int(raw_input('Start node ID: '))-1]
    goalNode = nodeList[int(raw_input('Goal node ID: '))-1]
    newline()

    hList = heuristic(goalNode, nodeList)

