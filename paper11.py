'''
  Q1a
  ------------------
  This question is similar to Q4a in the 2012 paper.
'''
def navigateToWaypoints(WaypointX, WaypointY):
  currentPos = (0, 0, 0)
  for i in range(len(WaypointY)):
    deltaX = WaypointX[i] - currentPos[0]
    deltaY = WaypointY[i] - currentPos[1]

    # rotate robot towards new point
    angle = atan2(deltaY, deltaX) - currentPos[2]
    if angle > pi:
      angle -= 2*pi
    if angle < -pi:
      angle += 2*pi
    Rotate(angle)

    # move to new point
    dist = sqrt(deltaX ** 2 + deltaY ** 2)
    DriveForward(dist)

    Beep()

navigateToWaypoints(WaypointX, WaypointY) # call our function


'''
  Q1b
  ------------------
  In the experiment, the proportions of the map stored by the robot did not
  accurately match the proportions of the actual environment. When the robot
  recognises waypoint N-1 as waypoint 0, it detected a loop closure, meaning
  that the two waypoints actually align in the environment.

  Once a loop closure is detected, the robot can apply a pose graph
  optimisation algorithm (relaxation). This will recompute each node to match
  is maximally probable value given the new information the robot has on its
  environment. As a result, the robot will obtain a more accurate map of the
  environment.
'''


'''
  Q2a
  ------------------
  The constants should be evaluated as follows:
  First decide at which velocity the robot should move and set POWER
  accordingly. Then adjust the FORWARD_INTERVAL and TURN_INTERVAL in a
  way that the robot performs the desired movements precisely.
'''
POWER = 1 # constant defining the power value the motor is set to
FORWARD_INTERVAL = 10 # constant (in sec) defining time interval to complete
                      # forward move of 1m
TURN_INTERVAL = 2 # constant defining time interval (in sec) to complete
                  # 90° turn on the spot.
def moveSquare():
  for i in range(4):
    SetLeftMotorPower(POWER)
    SetRightMotorPower(POWER)
    wait(FORWARD_INTERVAL)
    SetLeftMotorPower(-POWER)
    SetRightMotorPower(POWER)
    wait(TURN_INTERVAL)

moveSquare() # call our function


'''
  Q2bi
  ------------------
  delta_theta = (v_R - v_L) * delta_t / W
'''


'''
  Q2bii
  ------------------
  R = W*(v_R + v_L) / 2*(v_R - v_L)
'''


'''
  Q3a
  ------------------
  We take measuremets within certain intervals (e.g. 1cm) over the entire
  range of the sensor. We keep track of the ground truth value v and the
  measurement z reported by the sensor.

  Using both values, we can evaluate the amount of garbage readings K (in %)
  and the variance (sigma^2) of the sensor measurements.

  p(z|v) will then be the following expression:
    p(z|v) = exp(-(z-v)**2 / 2*variance) + K

  A bias would be present if the mean of P(z|v) would be a value other than
  z=v. This could be the case for certain values for v, or consistently for
  all v's.
'''


'''
  Q3b
  ------------------
  This question is similar to Q3b in the 2012 paper.

  The result will look somewhat like this:

  m = 100
    |                                  ...
    |                                 .   .
    |                                ..   ..
    |                               ..     ..
    |                               .|      |---- standard deviation 2cm
    |                              . |      |.
    |                             .. |      |..
    |                            ..  |      | ..
    |                          ..    |      |   ..
    |     .....................      |      |    ......................
    |     |  ~~~~~~~~~~~~~~~~~~~~  Garbage area  ~~~~~~~~~~~~~~~~~~~  |
    +-----|-----------------------------|-----------------------------|-----
        v=10cm                        v=100                        v=200cm
'''


'''
  Q3d
  ------------------
  Motion:
    The robots evaluates the movement it performs based on the current
    estimate of its position, which is the average of its current
    particles.

  Prediction:
    Based on the movement it performs, the robot estimates its new position.
    This is done by updating every particle with the values of the movement.
    For every particle, error terms are included when updating the
    particles. The error terms are drawn from a Gaussian distribution with
    zero mean.

  Measurement Update:
    After obtaining a new measurement, the weight of each particle is set
    to the value of p(z|m) which is the likelihood of obtaining the
    measurement z given that the actual value of the distance would be m.
    m is calculated based on the postion of the particle and the
    information of the enviroment (walls) the robot has.

  Normalisation:
    The weights are adjusted proportionally to another in a way that their
    sum is 1.

  Resampling:
    We draw a new set of particles from the current set of particles. The
    likelihood of a particle to be drawn is proportional to its weight.
    We then set the weight of all drawn particles to the same value in such
    way that all weights add up to 1.
'''


'''
  Q4ai
  ------------------
  This question is similar to Q2bi in the 2014 paper.
  This solution works if the wall is steady.
'''
K = 30 # gain constant, needs to be calibrated

def ReadSonar():
  # median filter for sonar readings
  return sorted([GetSonarDepth() for i in range(5)])[3]

def NavigateToDistance(d):
  z = ReadSonar()
  error = z - d
  # while absolute error above some threshold (5cm)
  while abs(error) > 1:
    # motor speed between -100 and 100
    SetMotorSpeed(max(min(K * error, 100), -100))
    z = ReadSonar()
    error = z - d
    timer.sleep(0.05)
  # set speed to 0 once destination is reached
  SetMotorSpeed(0)

# call our script
NavigateToDistance(30)


'''
  Q4aii
  ------------------
  The robot would stay too far behind the target. At some point, before
  it gets close enough, the forward speed calculated will be too low to
  generate any actual movement.

  We can account for this by using the PID method instead. Using this,
  the robot will regard the integral of the error as well, and increase
  the speed if the error term grows over time, this eventually correcting
  itself to be within the right distance.
'''
K_p = 30 # gain constant, needs to be calibrated
K_i =
K_d =
INTERVAL = 1/20

def ReadSonar():
  # median filter for sonar readings
  return sorted([GetSonarDepth() for i in range(5)])[3]

def NavigateToDistance(d):
  z = ReadSonar()
  integral = 0
  prevErr = 0
  error = z - d

  while true:
    integral += error * INTERVAL
    derivative = (error - prevErr) / INTERVAL
    prevErr = error
    speed = K_p * error + K_i * integral + K_d * derivative

    # motor speed between -100 and 100
    SetMotorSpeed(max(min(speed, 100), -100))
    z = ReadSonar()
    error = z - d
    timer.sleep(INTERVAL)

# call our script
NavigateToDistance(30)


'''
  4b
  ------------------
  This question is similar to Q2c in the 2012 paper.

  Using the rotating sonar, we can measure the distance in every direction
  in certain intervals (e.g. 1°). The values are then stored as "signature".
  By collecting signatures for different location, a robot could recognise
  its location by measuring the signature at its current position and
  comparing it to its stored signatures.

  Using depth histograms, the robot can evaluate its position more
  efficiently regardless of its rotation. We rearrange the signatures to
  form a histogram of the frequency of depth measurements in the signature.
  We can then directly compare these without having to compare signatures
  at every possible rotation of the robot.
'''
