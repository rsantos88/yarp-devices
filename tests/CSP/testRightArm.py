import csv
import math
import sys
import yarp

# usage: python3 testRightArm.py /teoSim test/test_rightArm.csv test/test_encs.csv

DELAY = 0.05

robotPrefix = sys.argv[1]   # prefix
csvInPath = sys.argv[2]     # csv IN csvFile
csvOutPath = sys.argv[3]    # csv OUT csvFile


playerPrefix = '/player' + robotPrefix


if robotPrefix == '/teo':
    isReal = True
elif robotPrefix == '/teoSim':
    isReal = False
else:
    print('error: illegal prefix parameter, choose "/teo" or "/teoSim"')
    quit()

yarp.Network.init()

if not yarp.Network.checkNetwork():
    print('error: please try running yarp server')
    quit()

options = yarp.Property()
options.put('device', 'remote_controlboard')

options.put('remote', robotPrefix + '/rightArm')
options.put('local', playerPrefix + '/rightArm')
rightArmDevice = yarp.PolyDriver(options)
rightArmEnc = rightArmDevice.viewIEncoders()
rightArmMode = rightArmDevice.viewIControlMode()
rightArmPosd = rightArmDevice.viewIPositionDirect()


rightArmAxes = rightArmEnc.getAxes()
rightArmMode.setControlModes(yarp.IVector(rightArmAxes, yarp.VOCAB_CM_POSITION_DIRECT))

with open(csvInPath, 'r') as csvInFile:
    start = yarp.now()
    i = 1
    csvreader = csv.reader(csvInFile)
    with open(csvOutPath, 'w', newline='') as csvOutfile:
        csvwriter = csv.writer(csvOutfile)
        csvwriter.writerow(['timestamp', 'setPosition J0', 'setPosition J1', 'setPosition J2', 'setPosition J3', 'setPosition J4', 'setPosition J5', 'getEncoder J0', 'getEncoder J1', 'getEncoder J2', 'getEncoder J3', 'getEncoder J4', 'getEncoder J5'])
        for row in csvreader:
            if True:
                print('reading >> ', list(map(float, row[6:12])))
                rightArmPosd.setPositions(yarp.DVector(list(map(float, row[6:12]))))
                csvwriter.writerow([yarp.now(), row[6], row[7], row[8], row[9], row[10], row[11], rightArmEnc.getEncoder(0), rightArmEnc.getEncoder(1), rightArmEnc.getEncoder(2), rightArmEnc.getEncoder(3), rightArmEnc.getEncoder(4), rightArmEnc.getEncoder(5)])
                delay = DELAY * i - (yarp.now() - start)
                yarp.delay(delay)
                i = i + 1

rightArmDevice.close()

yarp.Network.fini()
