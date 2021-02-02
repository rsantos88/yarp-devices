import csv
import math
import sys
import yarp

# usage: python3 testSingleJoint.py /teoSim 3 test/test_leftArm.csv  test/test_encs.csv

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

options.put('remote', robotPrefix + '/leftArm')
options.put('local', playerPrefix + '/leftArm')
leftArmDevice = yarp.PolyDriver(options)
leftArmEnc = leftArmDevice.viewIEncoders()
leftArmMode = leftArmDevice.viewIControlMode()
leftArmPosd = leftArmDevice.viewIPositionDirect()


leftArmAxes = leftArmEnc.getAxes()
leftArmMode.setControlModes(yarp.IVector(leftArmAxes, yarp.VOCAB_CM_POSITION_DIRECT))

with open(csvInPath, 'r') as csvInFile:
    start = yarp.now()
    i = 1
    csvreader = csv.reader(csvInFile)
    with open(csvOutPath, 'w', newline='') as csvOutfile:
        csvwriter = csv.writer(csvOutfile, delimiter=',')
        csvwriter.writerow(['timestamp', 'setPosition J0', 'setPosition J1', 'setPosition J2', 'setPosition J3', 'setPosition J4', 'setPosition J5', 'getEncoder J0', 'getEncoder J1', 'getEncoder J2', 'getEncoder J3', 'getEncoder J4', 'getEncoder J5'])
        for row in csvreader:
            if True:
                print('reading >> ', list(map(float, row[:6])))
                leftArmPosd.setPositions(yarp.DVector(list(map(float, row[:6]))))
                csvwriter.writerow([yarp.now(), row[0], row[1], row[2], row[3], row[4], row[5], leftArmEnc.getEncoder(0), leftArmEnc.getEncoder(1), leftArmEnc.getEncoder(2), leftArmEnc.getEncoder(3), leftArmEnc.getEncoder(4), leftArmEnc.getEncoder(5)])
                delay = DELAY * i - (yarp.now() - start)
                yarp.delay(delay)
                i = i + 1

leftArmDevice.close()

yarp.Network.fini()
