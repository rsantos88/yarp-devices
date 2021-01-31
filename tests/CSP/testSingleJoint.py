import csv
import math
import sys
import yarp

# usage: python3 testSingleJoint.py /teoSim 3 test/test.csv  test/test_encs.csv

DELAY = 0.05

robotPrefix = sys.argv[1]   # prefix
joint = int( sys.argv[2])   # joint
csvInPath = sys.argv[3]     # csv IN csvFile
csvOutPath = sys.argv[4]    # csv OUT csvFile


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

options.put('remote', robotPrefix + '/rightArm')
options.put('local', playerPrefix + '/rightArm')
rightArmDevice = yarp.PolyDriver(options)
rightArmEnc = rightArmDevice.viewIEncoders()
rightArmMode = rightArmDevice.viewIControlMode()
rightArmPosd = rightArmDevice.viewIPositionDirect()

leftArmAxes = leftArmEnc.getAxes()
rightArmAxes = rightArmEnc.getAxes()

# read encoders: save values in leftEncs
leftEncs=yarp.Vector(leftArmAxes)


# single joint
leftArmMode.setControlMode(joint, yarp.VOCAB_CM_POSITION_DIRECT)
#rightArmMode.setControlMode(joint, yarp.VOCAB_CM_POSITION_DIRECT)

with open(csvInPath, 'r') as csvInFile:
    start = yarp.now()
    i = 1
    csvreader = csv.reader(csvInFile)
    with open(csvOutPath, 'w', newline='') as csvOutfile:
        csvwriter = csv.writer(csvOutfile, delimiter=',')
        csvwriter.writerow(['timestamp', 'value'])
        for row in csvreader:
            if True:
                print('reading >> ', row[3])
                leftArmPosd.setPosition(joint, float(row[3])) # set position
                print('encoder >> ', leftArmEnc.getEncoder(joint))
                csvwriter.writerow([yarp.now(), leftArmEnc.getEncoder(joint)])
                delay = DELAY * i - (yarp.now() - start)
                yarp.delay(delay)
                i = i + 1

leftArmDevice.close()
rightArmDevice.close()

yarp.Network.fini()
