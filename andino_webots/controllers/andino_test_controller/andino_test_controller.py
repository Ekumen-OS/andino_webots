"""andino_test_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
m1= robot.getDevice('left_wheel_joint')
m2 = robot.getDevice('right_wheel_joint')


m1.setPosition(float('inf'))
m2.setPosition(float('inf'))
i = 0
while robot.step(timestep) != -1:

    if i < 200:
        m1.setVelocity(1)
        m2.setVelocity(1)
    elif i < 300:
        m1.setVelocity(-0.5)
        m2.setVelocity(0.5)
    elif i < 400:
        m1.setVelocity(-1)
        m2.setVelocity(-1)
    elif i > 550:
        i = 0
    i += 1
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
