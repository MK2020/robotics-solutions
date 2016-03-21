'''
  Q1a
  ------------------
  Combining the formulas to get omega_l and omega_r:

  omega_i = v_i / r_i

  v_r = thetadot * W + v_l
  v_l = 2v - v_r
  v_r = thetadot * W + 2v - v_r
  v_r = thetadot * W / 2 + v
  omega_r = (thetadot * W / 2 + v) / r_r

  v_l = v_r - thetadot * W
  v_r = 2v - v_l
  v_l = 2v - v_l - thetadot * W
  v_l = v - thetadot * W / 2
  omega_l = (v - thetadot * W / 2) / r_l
'''
def ComputeWheelRotations(v, thetadot):
  omega_r = (v + thetadot * W/2) / r_r
  omega_l = (v - thetadot * W/2) / r_l
  return (omega_r, omega_l)


'''
  Q1b
  ------------------
  i:  Ensure that alpha is between pi and -pi,
        then apply servoing technique (lecture 3)
  ii: Servoing (lecture 3, slide 19):
        set demand proportional to negative error

  Possibly use different k_p values in each function
'''
def ComputeAngleControllerOutput(alpha):
  if alpha > pi:
    alpha -= 2pi
  return k_p * alpha

def ComputeDistanceControllerOutput(D, r_D):
  return k_p * (r_D - D)


'''
  Q1c
  ------------------

            D          r_D                             alpha
            |           |                                |
            |           |                                |
  -----------------------------------     --------------------------------
  | ComputeDistanceControllerOutput |     | ComputeAngleControllerOutput |
  -----------------------------------     --------------------------------
                            v \                 / thetadot
                               \               /
                          -------------------------
                          | ComputeWheelRotations |
                          -------------------------
                                      | omega_l, omega_r
                                      |
'''


'''
  Q1d
  ------------------
  k_i and k_d initially 0
  k_p will be set to some value, here 100

  we can then use the Ziegler-Nichols method to tune the parameters

  good explanation for everything here:
  http://robotsforroboticists.com/pid-control/
'''
k_p = 100
k_i = 0
k_d = 0

iteration_time = 1/20

angle_integral = 0
angle_error_prior = 0
def ComputeAngleControllerOutput(alpha):
  if alpha > pi:
    alpha -= 2pi
  angle_integral = angle_integral + (alpha * iteration_time)
  derivative = (alpha – angle_error_prior) / iteration_time
  angle_error_prior = alpha
  return k_p * alpha + k_i * angle_integral + k_d * derivative


distance_integral = 0
distance_error_prior = 0
def ComputeDistanceControllerOutput(D, r_D):
  error = r_D – D
  distance_integral = distance_integral + (error * iteration_time)
  derivative = (error – distance_error_prior) / iteration_time
  distance_error_prior = error
  return k_p * error + k_i * distance_integral + k_d * derivative


'''
  Q2a
  ------------------
  i.    e doubled in magnitude:
        The spread of the particles becomes larger, but the basic shape
        remains the same.

  ii.   f doubled in magnitude:
        Particles spred wider orthogonally to direction of movement.

  iii.  g doubled in magnitude:
        Particle spread wider orthogonally to direction of movement
        after every turn.
'''


'''
  Q2b
  ------------------
  Given:
    DriveForward(D)
    Rotate(alpha)
    Beep()
    [Waypoint] waypointList

  Task:
    starting state: (0, 0, 0)
    navigate to each point as efficiently as possible, in steps of max. 20cm
    call Beep() after reaching waypoint

  Finding the optimal order of traversal is actually a very complex
    problem (Travelling Salesman problem). Since this is not the main
    focus of the exercise, we'll use a greedy nearest-neighbour strategy.
'''

def NavigateToAll(waypointList):
  waypointList = OptimiseOrder(Waypoint(0, 0), waypointList)
  currentPoint = Waypoint(0, 0)
  currentAngle = 0

  for waypoint in waypointList:
    d_x = waypoint.x - currentPoint.x
    d_y = waypoint.y - currentPoint.y

    # rotate to waypoint
    angle = atan2(d_y, d_x) - currentAngle
    if angle > pi:
      angle -= 2*pi
    if angle <= -pi:
      angle += 2*pi
    Rotate(angle)

    # move to waypoint
    dist = sqrt((currentPoint.x - waypointList[i].x) ** 2
                + (currentPoint.y - waypointList[i].y) ** 2)
    destReached = False
    while dist:
      # move in steps of 20cm
      distStep = min(20, dist)
      DriveForward(distStep)
      dist -= distStep

    currentPoint = waypoint
    Beep()

def OptimiseOrder(currentPoint, waypointList):
  # assert: len(waypointList) > 0
  order = []
  while len(waypointList) > 1:
    bestIndex = 0
    bestDist = inf
    for i in range(0, waypointList.length):
      dist = sqrt((currentPoint.x - waypointList[i].x) ** 2
                  + (currentPoint.y - waypointList[i].y) ** 2)
      if dist < bestDist
        bestIndex = i
        bestDist = dist
    currentPoint = waypointList.pop(bestIndex)
    order.append(currentPoint)
  # don't bother checking distance when only one element left in list
  order.append(waypointList[0])
  return order


'''
  Q3a
  ------------------
  UpdateOccupancyMap:
    Update occupancy map in response to sonar measurement z when robot
    is at (x, y, theta) and sonar at angle alpha.

  Given:
    occupancyMap[500][500]
    cell in occupancyMap represents log odds
    half width of sonar beam: 5°
    standard deviation of depth measurement z = 4cm

  Assumptions:
    Robot somewhere in middle of grid, max. sonar range less than distance
    to grid borders.
    For simplicity, we say that any cell "lies within the beam" if any part
    of the beam goes through the cell.
'''

LOW_LIMIT = -4 # lower limit for log odds value of a cell
UP_LIMIT = 4 # upper limit for log odds value of a cell
U = 1 # update value for log odds

def UpdateOccupancyMap(x, y, theta, z, alpha):
  # normalise angles
  theta = radians(theta)
  alpha = radians(alpha)
  abs_angle = NormaliseAngle(alpha + theta)

  for i in range(0, len(occupancyMap)):
    for j in range(0, len(occupancyMap[0])):
      if WithinBeam(i, j, x, y, abs_angle):
        distance = sqrt((i-x)**2 + (j-y)**2)
        if distance <= z - 4:
          occupancyMap[i][j] = max(occupancyMap[i][j] - U, LOW_LIMIT)
        elif distance <= z + 4:
          occupancyMap[i][j] = min(occupancyMap[i][j] + U, UP_LIMIT)

def NormaliseAngle(angle):
  angle = angle % (2 * pi)
  if angle <= -math.pi:
    angle += 2 * pi
  elif angle > math.pi:
    angle -= 2 * pi
  return angle

def WithinBeam(i, j, x, y, angle):
  # angle of i,j from robot position:
  # add 0.5 to cell boundries since coordinates correspond
  # to bottom left point in cell, but we actually want to know if cell
  # is more than 50% covered by beam
  pointAngle = atan2(j + 0.5 - y, i + 0.5 - x)
  angleOffset = pointAngle - angle
  return angleOffset <= radians(5) and angleOffset >= radians(-5)


'''
  Q3b
  ------------------
  The robot can use the occupancy map in order to calculate the location
  signature for any position within the bounds of the map (with the
  restriction that it will not be able to get values in sutuations where
  there are no bounds within the map).

  For a given set of angles (e.g. 0°, 1°, 2°..) and a position (x, y), the
  signature is obtained as follows:

  For each angle, find the closest cell in that direction from the starting
  position that has a likelyhood of occupancy of higher than some threshold
  (0.5 makes sense, i.e. log odds greater than 0). The distance between that
  cell and the starting position will then be saved for that particular
  angle. The values for all angles together form the signature.
'''


'''
  Q4a
  ------------------
    m = 1m
    - close range
    - precise, but not very robust
    - standard deviation = 4cm
    - ~12% garbage readings

    We get a narrow Gaussian band, plus constant additive accounting
    for garbage readings

    |                                  ...
    |                                 .   .
    |                                ..   ..
    |                               ..     ..
    |                               |       |---- standard deviation 4cm
    |                              .|       |.
    |                             ..|       |..
    |                            .. |       | ..
    |                          ..   |       |   ..
    |..........................     |       |    ..........................
    |                               |       |
    |    ~~~~~~~~~~~~~~~~~~~~~~~~  Garbage area  ~~~~~~~~~~~~~~~~~~~~~~~~
    |                               |       |
    +-----------------------------------------------------------------------
                                                                           z

    m = 2m
    - medium range
    - less precise, but more robust
    - standard deviation 16cm
    - ~6% garbage readings

    We get a wider Gaussian band and a smaller garbage area.

    |                                 .....
    |                               ...   ...
    |                             ...       ...
    |                            ..           ..
    |                           .|             |---- standard deviation 16cm
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
    - long range
    - unprecise
    - standard deviation 36cm
    - ~3% garbage readings

    We get an even wider Gaussian band and an even smaller garbage area.

    |
    |                               .........
    |                            ...         ...
    |                          ...             ...
    |                        ..                    |---- standard
    |                     ...|                     |...  deviation 36cm
    |                  ...   |                     |   ...
    |             .....      |                     |      .....
    |        ......          |                     |          ......
    |........                |                     |                ........
    +-----------------------------------------------------------------------
                                                                           z
'''


'''
  Q4b
  ------------------
'''

def GetLikelihood(particle, z, walls):
  closestDist = inf
  for wall in walls:
    dist = ((wall.By - wall.Ay) * (wall[0][0] - particle.x)
             - (wall.Bx - wall[0][0]) * (wall.Ay - particle.y))
           / ((wall.By - wall.Ay) * cos(particle.theta)
              - (wall.Bx - wall[0][0]) * sin(particle.theta))
    (x, y) = particle.x + m*cos(theta), particle.y + m*sin(theta)
    # if distance closer and point of intersection within wall bounds
    if dist < closestDist and WithinWall(wall, x, y)
      closestDist = dist

  error = z - closestDist
  return math.exp( -(error**2) / (2 * (GetStD(closestDist) ** 2)))
         + GetGarbageK(closestDist)

def WithinWall(wall, x, y):
  # since x and y are on line which extends wall, we only need to check
  # one dimension
  return (x >= wall.Ax and x <= wall.Bx) or (x <= wall.Ax and x >= wall.Bx)

def GetGarbageK(m):
  return (1/m) * (0.24*50)

def GetStD(m):
  return (m ** 2) * (1 / (50 ** 2))
