'''
  Q1a
  ------------------
  This question is similar to Q1a in the 2014 paper.
'''
def initialiseParticleSet():
  return [((0.1, 0.1, 0.0), 1.0/100)] * 100


'''
  Q1b
  ------------------
  This question is similar to Q1b in the 2014 paper.
'''
SIGMA_E = 1 # standard deviation for e, needs to be calibrated
SIGMA_F = 1 # standard deviation for f, needs to be calibrated
SIGMA_G = 1 # standard deviation for g, needs to be calibrated

def motionPrediction(D, alpha):
  for particle in particleSet:
    if D != 0:
      e = sampleGaussian(0, SIGMA_E)
      f = sampleGaussian(0, SIGMA_F)
      particle[0][0] += cos(particle[0][2]) * (D + e)
      particle[0][1] += sin(particle[0][2]) * (D + e)
      particle[0][2] += f
    else:
      g = sampleGaussian(0, SIGMA_G)
      particle[0][2] += alpha + g


'''
  Q1c
  ------------------
  This question is similar to Q1c in the 2014 paper.
'''
G = 0.2 # percentage of garbage readings
SIGMA = 10 # standard deviation of readings in cm

def measurementUpdate(z):
  for part in particleSet:
    m = getDistanceToClosestWall(part[0][0], part[0][1], part[0][2])
    err = z - m
    part[1] = exp(-(err ** 2) / (2 * (SIGMA ** 2))) + G


'''
  Q1d
  ------------------
  This question is similar to Q1d in the 2014 paper.
'''
def normaliseParticleSet():
  weightSum = sum(map(lambda part: part[1], particleSet))
  map(lambda part: (part[0], part[1]/weightSum), particleSet)


'''
  Q1e
  ------------------
  This question is similar to Q1e in the 2014 paper.
'''
particleSet # is defined outside the functions

def resampleParticleSet():
  cumArray = cumulativeArray()
  return [(drawSample(cumArray), 1.0/100), for i in range(100)]

def cumulativeArray(particleSet):
  cumArray = [0] * 100
  acc = 0
  for i in enumerate(particleSet):
    acc += particleSet[i]
    cumArray[i] = acc
  return cumArray

# Returns sample of the form (x, y, theta)
def drawSample(cumArray):
  rand = sampleUniform(1)
  for i in range(1, cumArray):
    if (rand - cumArray[i]) < 0:
      return particleSet[i-1][0]


'''
  Q2ai
  ------------------
  This question is similar to Q2ai in the 2013 paper.
'''
k_p = 1.0 # needs to be calibrated
REF_POWER = 30 # choose
def FollowWall(D):
  while True:
    z = getSonarDepth()
    gain = (D - z) * k_p
    gain = min(max(gain, -GAIN_LIMIT), GAIN_LIMIT)
    SetLeftPower(REF_POWER - gain)
    SetRightPower(REF_PWER + gain)
    timer.sleep(0.05)


'''
  Q2aii
  ------------------
  This question is similar to Q2aii in the 2013 paper.

  The gain value (k_p) defines how we adjust the speed of the robot's
  wheels in proportion to the error. We set it to 1.0, however, in practice
  this value needs to be calibrated. The value will need to be adjusted
  with regard to the speed at which the robot is moving forward.

  A value too high will result in the robot taking too sudden turns, and
  hence either instantly driving into the wall or suddenly turning away
  from the wall whenever the measured distance is too high or too low.

  A value too low will result in the robot not being able to adjust soon
  enough when it detects a distance higher or lower than desired. The
  robot would then slowly move away or towards the wall whenever it is
  not perfectly aligned at the correct distance.
'''


'''
  Q2b
  ------------------
  Extending Q2ai as follows:
'''
TURN_THRESHOLD = 50 # threshold at which we assume that the wall
                    # turns to the left
def FollowWall(D):
  while True:
    z = getSonarDepth()
    bump = readBumpSensor()
    if !bump and z < TURN_THRESHOLD:
      gain = (D - z) * k_p
      gain = min(max(gain, -GAIN_LIMIT), GAIN_LIMIT)
      SetLeftPower(REF_POWER - gain)
      SetRightPower(REF_PWER + gain)
      timer.sleep(0.05)
    elif !bump:
      # wall turns left, turn robot to the left
      SetLeftPower(-REF_POWER)
      SetRightPower(REF_POWER)
      timer.sleep(0.05) # wait as long as it takes to perform right turn
    else:
      # bump detected, turn right (away from wall to the left)
      SetLeftPower(REF_POWER)
      SetRightPower(-REF_POWER)
      timer.sleep(0.05) # wait as long as it takes to perform right turn


'''
  Q2c
  ------------------
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


'''
  Q3ai
  ------------------
  p(z|v) is the likelihood of obtaining the depth measurement z if the
  ground truth value for the distance is v. p(z|v) depends on the variance
  of the depth measurements and the amount of garbage readings the sensor
  produces.
'''


'''
  Q3aii
  ------------------
  We take measuremets within certain intervals (e.g. 1cm) over the entire
  range of the sensor. We keep track of the ground truth value v and the
  measurement z reported by the sensor.

  Using both values, we can evaluate the amount of garbage readings K (in %)
  and the variance (sigma^2) of the sensor measurements.

  p(z|v) will then be the following expression:
    p(z|v) = exp(-(z-v)**2 / 2*variance) + K
'''


'''
  Q3iii
  ------------------
  This question is tricky because the diagram given is pretty small.

  It's very clear that the standard deviation grows proportionally to the
  ground truth distance measured.

  It seems like the plot is also drifting in some direction, indicating
  biased measurements (reporting values too high) with growing ground
  truth values.
'''


'''
  Q3iv
  ------------------
  We need to draw a plot similar as the one given, but with two differences:
  1.: The bell curves do not become flatter with growing ground truth
      distance.
  2.: All curves are moved relative to the coordinate system by 5cm units
      in the direction of v.
'''


'''
  Q3b
  ------------------
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
         v=5cm                         v=100                       v=200cm

'''


'''
  Q3c
  ------------------
  For each cell, we store the likelihood of occupancy using the odds notation,
  which is the ratio of P(O_i) and P(E_i), which we abbreviate as o(O_i).
  Given a new measurement Z, we can evaluate the posterior likelihoods P(Z|O_i)
  and P(Z|E_i). In order to update the odds value in the cell, we then
  multiply the existing o(O_i) by the ratio of P(Z|O_i) and P(Z|E_i).

  This can be done more efficiently using log odds, since
  log(o(O_i) * P(Z|O_i)/P(Z|E_i)) = log(o(O_i) + log(P(Z|O_i)/P(Z|E_i)).

  For a cell that is equally likely to be occupied as unoccupied, the log odds
  value is log(0.5/0.5)=0. We can then update the value additively, adding 0.5
  if the cell is occupied, and subtracting 1 if the cell is empty. This is
  because we have higher confidence in a cell being empty opposed to it being
  occupied (since the spread is wider towards the end of the sonar beam).
'''


'''
  Q4a
  ------------------
  Given:
    WaypointX[]
    WaypointY[]
'''
def navigateToWaypoints(WaypointX, WaypointY)
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

    #move to new point
    dist = sqrt(deltaX ** 2 + deltaY ** 2)
    DriveForward(dist)

    Beep()


'''
  Q4bi
  ------------------
  The pattern we produce here should look similar to the pattern from the
  first pratical (draw square). At the starting position, the robot is
  pretty accurately in the same position at every run. With every movement
  it then performs, we get a higher degree of uncertainty, thus a higher
  spread in the possible paths it takes.
'''


'''
  Q4bii
  ------------------
  We get a higher degree of uncertainty (more spread). Furthermore, the
  wheels are more likely to slip on the carpet. Hence we probably more
  less than the desired distance on average and turns will not be executed
  fully.
'''
