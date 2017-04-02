# from gpio import Keypad
# keypad = Keypad()
def initialise():
    pass

def get_confirmation(buildingNameOrNumber, floorNumber, nodeRaw):
    while True:
        voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_CONFIRMATION.format(building = buildingNameOrNumber, floor = floorNumber, node = nodeRaw), constants.HIGH_PRIORITY))
        print('1 to confirm, 2 to try again: ')
        confirmation = str(keypad.getKeysInput())
        voiceOutput.addToQueue(INSTRUCTION(confirmation, constants.HIGH_PRIORITY))
        if confirmation == '1' or confirmation == '2':
            return confirmation
            # pass
        else:
            voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_OUT_OF_RANGE, constants.HIGH_PRIORITY))
