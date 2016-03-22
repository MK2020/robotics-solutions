'''
  Q1a
  ------------------
  Given:
    light sensor (bright/dark)
    temperature sensor (hot/cold)

    kitchen: bright (80%) / cold (90%) / 12sqm
    bathroom: dark (70%) / hot (60%) / 8sqm
    living room: bright (60%) / hot (60%) / 20sqm

  Prior:
    P(kitchen) =     12/40 = 3/10
    P(bathroom) =     8/40 = 1/5
    P(living room) = 20/40 = 1/2

  Marginal likelihood:
    P(bright) = 3/10 * 0.8 + 1/5 * 0.3 + 1/2 * 0.6 = 3/5

  Posterior (P(X|bright)):
    P(kitchen | bright)     = P(bright | kitchen) * P(kitchen) / P(bright)
                            = 4/5 * 3/10 / 3/5 = 2/5
    P(bathroom | bright)    = 3/10 * 1/5 / 3/5 = 1/10
    P(living room | bright) = 3/5 * 1/2 / 3/5  = 1/2
'''


'''
  Q1b
  ------------------
  P(bright, cold | kitchen)     = 2/5 * 9/10 = 18/25
  P(bright, cold | bathroom)    = 3/10 * 2/5 = 3/25
  P(bright, cold | living room) = 3/5 * 2/5  = 6/25

  P(bright, cold) = 3/10 * 18/25 + 1/5 * 3/25 + 1/2 * 6/25 = 9/25

  P(kitchen|bright, cold) = 18/25 * 3/10 / 9/25 = 3/5
  P(bathroom|bright, cold) = 3/25 * 1/5 / 9/25 = 1/15
  P(living room|bright, cold) = 6/25 * 1/2 / 9/25 = 1/3
'''


'''
  Q1c
  ------------------
  In order to estimate a position using a descrete probabilistic approach,
  the robot would divide the space in which it is moving into cells and
  keep track of the probability for every single cell and every possible
  rotation. Using measurements taken with any kind of sensor, it could then
  evaluate the likelihood of obtaining the measurement given that it is the
  cell with the rotation for every possible cell and rotation.

  With every measurement, these values will be updated on the basis of the
  values obtained from the previous measurement.

  Using a particle filter in order to do this is more feasable than using
  a probabilistic histogram since it drastically reduces the amount of
  probabilities the robot needs to keep track of. Furthermore, once the
  robot obtained some measurements, the likelihood the vast majority of
  possible positions and rotations will be close to 0. Hence a limited
  number of particles (~100) is sufficient to evaluate the robot's position
  with reasonable precision.
'''


'''
  Q2ai
  ------------------
  This question is similar to Q2ai in the 2012 paper.
'''
def FollowWall(zDesired):
  speed = 10.0 # motor speed
  k_p = 1.0 # proportional gain constant (needs to be calibrated)

  while True:
    error = zDesired - GetSonarDepth()
    gain = k_p * error / 2
    # upper and lower limit for speed
    gain = min(max(gain, -GAIN_LIMIT), GAIN_LIMIT)

    SetLeftVelocity(speed + gain)
    SetRightVelocity(speed - gain)
    time.sleep(0.05)

FollowWall(30) # Trigger our script


'''
  Q2aii
  ------------------
  This question is similar to Q2aii in the 2012 paper.

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
  The material for this question (TR programming) was not covered in the
  course 2015/16.
'''


'''
  Q3a
  ------------------
  This is the same as question Q2a in the 2014 paper.

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
    +-----------------------------------|-----------------------------------
                                       z=m

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
    +-----------------------------------|-----------------------------------
                                       z=m

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
    +-----------------------------------|-----------------------------------
                                       z=m
'''


'''
  Q3b
  ------------------
  This question is similar to Q3a in the 2014 paper.

  The provided function will store the array signature in a way that the
  position in the array does not correlate with the actual angle of the
  sonar. This doesn't matter, as long as we keep that in mind.
'''
def StoreSignature():
  Signature = [None] * 360
  for i in range(0, 360):
    SetSensorAngle(i - 179)
    Signature[i] = GetSensorDepth()
  return Signature


'''
  Q3c
  ------------------
  This question is similar to Q3c in the 2014 paper.

  The advantage of using a depth histogram over a signature is that we can
  efficiently recognise places regardless of the orientation of the robot.
  If we were using signatures, we had to compare each signature at every
  possible angle to find the best match, whereas when using a depth
  histogram, we only need to compare each signature once.
'''
def ConvertSingatureToHistogram(Signature):
  histogram = [0] * 200/5
  for depth in Signature:
    depth = min(depth, 199)
    histogram[depth / 5] += 1
  return histogram


'''
  Q4ai
  ------------------
  Left wheel distance moved:  R - W/2
  Right wheel distance moved: R + W/2
  Both wheel arcs subtend the same angle delta_theta

  We get:
  delta_theta = (v_L * delta_t) / (R - W/2)
  delta_theta = (v_R * delta_t) / (R + W/2)
  delta_theta = (v_R - v_L) * delta_t / W
'''


'''
  Q4aii
  ------------------
  We get:
  R = (W * (v_R + v_L)) / (2 * (v_R - v_L))
'''


'''
  Q4b
  ------------------
  Given:
    W = 20cm
    delta_t = 1s
    float vLarray[10]
    float vRarray[10]
'''
def MoveRobot(vLarray, vRarray):
  x, y, theta = 0, 0, 0
  for i in range(len(vLarray)):
    SetLeftSpeed(vLarray[i])
    SetRightSpeed(vRarray[i])
    Wait(1.0)
    delta_theta = (v_R - v_L) * delta_t / W
    R = (W * (v_R + v_L)) / (2 * (v_R - v_L))
    x += R * (sin(delta_theta + theta) - sin(theta))
    y -= R * (cos(delta_theta + theta) - cos(theta))
    theta += delta_theta


'''
  Q4c
  ------------------
  In order to adjust the formula for uncertainty in motion, we introduce
  error terms e and g. Each of them are drawn at every calculation
  from a Gaussian distribution with zero mean and a standard deviation
  depending on the precision of the robot.

  We obtain the following expression:

  x_new = x + (R + e) * (sin(delta_theta + theta) - sin(theta))
  y_new = y - (R + e) * (cos(delta_theta + theta) - cos(theta))
  theta_new = theta + delta_theta + g
'''
