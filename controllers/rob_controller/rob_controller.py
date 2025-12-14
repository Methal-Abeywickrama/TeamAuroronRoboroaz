"""rob_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor

TIME_STEP = 64
MAX_SPEED = 6.28
# create the Robot instance.
robot = Robot()

ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


while robot.step(TIME_STEP) != -1:
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    leftSpeed  = MAX_SPEED
    rightSpeed = MAX_SPEED
    # detect obstacles
    if psValues[0] > 80 and psValues[7] > 80:
        leftSpeed = 0.0
        rightSpeed = 0.0
        print('Stopped')
        right_check = psValues[1]
        left_check = psValues[6]
        if right_check > left_check:
            pass
    # write actuators inputs
    print()
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    pass

# Enter here exit cleanup code.
