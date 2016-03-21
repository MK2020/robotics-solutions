'''
  Q1a
  ------------------
'''
def InitialiseParticleSet():
  return [((0.1, 0.1, 0.0), 1.0/100)] * 100


'''
  Q1b
  ------------------
'''
e_dev = 0.1 # standard deviation for e, to be calibrated
f_dev = 0.6 # standard deviation for f, to be calibrated
g_dev = 0.1 # standard deviation for g, to be calibrated

def MotionPrediction(D, alpha):
  for particle in particleSet:
    if D != 0:
      e = random.gauss(0, e_dev)
      f = random.gauss(0, f_dev)
      particle[0][0] += cos(alpha) * (D + e)
      particle[0][1] += sin(alpha) * (D + e)
      particle[0][2] += f
    else:
      g = random.gauss(0, g_dev)
      particle[0][2] += alpha + g


'''
  Q1c
  ------------------
  ST_DEV reporesents the standard deviation of the sonar measurements
  K represents the percentage of garbage readings returned by sensor
'''
K = 0.15 # assume 15% garbage readings
ST_DEV = 10 # in cm
def MeasurementUpdate(z):
  for part in particleSet:
    dist = GetDistanceToClosestWall(part[0][0], part[0][1], part[0][2])
    err = z - dist
    part[1] = math.exp(-(z**2) / (2 * (ST_DEV**2))) + K


'''
  Q1d
  ------------------
'''
def NormaliseParticleSet():
  weightSum = sum(map(lambda part: part[1], particleSet))
  map(lambda part: (part[0], part[1]/weightSum), particleSet)


'''
  Q1e
  ------------------
'''
def ResampleParticleSet():
  cumArray = cumArray()
  particleSet = [(DrawSample(), 1.0/100) for i in range(100)]

def CumulativeArray():
  cumArray = [0] * 100
  weight = 0
  for i in enumerate(cumArray):
    cumArray[i] = particleSet[i] + weight
    weight += particleSet[i]
  return cumArray

# Returns sample of the form (x, y, theta)
def DrawSample(cumArray):
  rand = random.random()
  for i in range(1, len(cumArray))
    if (rand - cumArray[i]) < 0:
      return particleSet[i-1][0]


'''
  Q2a
  ------------------
    m = 1m
    - standard deviation = 10cm
    - 5% garbage readings

    We get a narrow Gaussian band, plus constant additive accounting
    for garbage readings

    |                                  ...
    |                                 .   .
    |                                ..   ..
    |                               ..     ..
    |                               |       |---- standard deviation 10cm
    |                              .|       |.
    |                             ..|       |..
    |   Garbage area             .. |       | ..
    |         |                ..   |       |   ..
    |.........|................     |       |    ...........................
    +-----------------------------------------------------------------------
                                                                           z

    m = 2m
    - standard deviation 20cm
    - 10% garbage readings

    We get a wider Gaussian band and a smaller garbage area.

    |                                 .....
    |                               ...   ...
    |                             ..         ..
    |                            ..           ..
    |                           .|             |---- standard deviation 20cm
    |                         .. |             | ..
    |                        .   |             |   .
    |                     ...    |             |    ...
    |                 ....       |             |       ....
    |................            |             |            ................
    |    ~~~~~~~~~~~~~~~~~~~~~~~~  Garbage area  ~~~~~~~~~~~~~~~~~~~~~~~~
    |                            |             |
    +-----------------------------------------------------------------------
                                                                           z

    m = 3m
    - standard deviation 30cm
    - 15% garbage readings

    We get an even wider Gaussian band and an even smaller garbage area.

    |
    |                               .........
    |                            ...         ...
    |                          ...             ...
    |                        ..                    |---- standard
    |                     ...|                     |...  deviation 30cm
    |                  ...   |                     |   ...
    |             .....      |                     |      .....
    |        ......          |                     |          ......
    |........                |                     |                ........
    |                        |                     |
    |    ~~~~~~~~~~~~~~~~~~~~~~~~  Garbage area  ~~~~~~~~~~~~~~~~~~~~~~~~
    |                        |                     |
    +-----------------------------------------------------------------------
                                                                           z
'''


'''
  Q2bi
  ------------------
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
  Q2bii
  ------------------
  The robot would stay too far behind the target. At some point, before
  it gets close enough, the forward speed calculated will be too low to
  generate any actual movement.

  We can account for this by using the PID method instead. Using this,
  the robot will regard the integral of the error as well, and increase
  the speed if the error term grows over time, this eventually correcting
  itself to be within the right distance.
'''


'''
  Q3a
  ------------------
  The provided function will store the array signature in a way that the
  position in the array does not correlate with the actual angle of the
  sonar. This doesn't matter, as long as we keep that in mind.
'''
def StoreSignature():
  Signature = [None] * 360
  for i in range(0, 360):
    SetSonarAngle(i - 179)
    Signature[i] = GetSonarDepth()
  return Signature


'''
  Q3b
  ------------------
'''
THRESHOLD = 10 # threshold needed to recognise signature

def RecognisePlace(SavedSignatures):
  # get signature of current place using prev. defined function
  signature = StoreSignature()
  bestSignature = -1
  bestDiff = inf
  for i, savedSignature in enumerate(SavedSignatures):
    diff = sum[abs(x - y) for x, y in zip(list1, list2)]
    if (diff < bestDiff && diff < THRESHOLD):
      bestSignature = i
      bestDiff = diff
  return bestSignature


'''
  Q3c
  ------------------
'''
def ConvertSingatureToHistogram(Signature):
  histogram = [0] * 200/5
  for depth in Signature:
    depth = min(depth, 199)
    histogram[depth / 5] += 1
  return histogram


'''
  Q4a
  ------------------
  I assume we have to use PID in this case.
  (not finished yet!)
'''
MOTOR_RATE = 20 # Hz
k_p = (100, 100)
k_i = (100, 100)
k_d = (100, 100)

def MoveForward(dist):
  aBase, bBase = GetMotorEncoder(A), GetMotorEncoder(B)
  aMoved, bMoved = 0, 0
  integralA, integralB = 0, 0
  priorErrA, priorErrB = 0, 0
  while :
    errA = dist - (GetMotorEncoder(A) - aBase)
    outputA, integralA = CalcPID(errA, 0, integralA, priorErrA)
    priorErrA = errA
    errB = dist - (GetMotorEncoder(B) - bBase)
    outputB, integralB = CalcPID(errB, 1, integralB, priorErrB)
    priorErrB = errB
    SetMotorPower()

def CalcPID(error,
            motor,
            distanceIntegral,
            distanceErrorPrior):
  distanceIntegral = distanceIntegral + (error * (1.0/MOTOR_RATE))
  derivative = (error – distanceErrorPrior) / (1.0/MOTOR_RATE)
  output = k_p[motor] * error + k_i[motor] * distanceIntegral
           + k_d[motor] * derivative
  return output, distanceIntegral


'''
  Q4bi
  ------------------
  Uniformly distribute robot positions across grid.
'''


'''
  Q4bii
  ------------------
  Keep every position where the sonar faces a wall at 25cm to 35cm.
'''


'''
  Q4biii
  ------------------
  Keep every position from previous iteration where depth at 90° is
  15cm to 25cm.
'''
