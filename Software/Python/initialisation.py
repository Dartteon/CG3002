def initialise():
    pass

def get_confirmation(buildingNameOrNumber, floorNumber):
    while True:
        voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_CONFIRMATION.format(building = buildingNameOrNumber, floor = floorNumber), constants.HIGH_PRIORITY))
        print('1 to confirm, 2 to try again: ')
        confirmation = str(keypad.getKeysInput())
        voiceOutput.addToQueue(INSTRUCTION(confirmation, constants.HIGH_PRIORITY))
        if confirmation not ('1' or '2'):
            voiceOutput.addToQueue(INSTRUCTION(messages.INPUT_OUT_OF_RANGE, constants.HIGH_PRIORITY))
        else:
            return confirmation
