from constants import *
import time


def is_wall(x, y):
    return x == 0 or x == 16 or y == 0 or y == 21


def get_filled_list(value, count):
    # noinspection PyUnusedLocal
    return [value for x in range(count)]


def get_filled_grid(value):
    # noinspection PyUnusedLocal
    return [[value for y in range(22)] for x in range(17)]


def get_initial_grids():
    map_grid = get_filled_grid(O)
    obs_grid = get_filled_grid(O)

    for x in range(17):
        for y in range(22):
            if not is_wall(x, y):
                map_grid[x][y] = U
                obs_grid[x][y] = C
    for x in range(1, 4):
        for y in range(1, 4):
            map_grid[x][y] = C
    for x in range(13, 16):
        for y in range(18, 21):
            map_grid[x][y] = C

    return map_grid, obs_grid


ENCODE_MAP = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f']
DECODE_MAP = {key: index for key, index in zip(ENCODE_MAP, range(100))}


def encode_grid(grid):
    global ENCODE_MAP
    grid_no_wall = [col[1:-1] for col in grid[1:-1]]

    encoded_str = ""
    for i in range(150):  # 150 hex characters
        code = 0
        for j in range(2):  # first and second
            index = i * 2 + j
            x = index / 20
            y = index % 20
            inner_code = grid_no_wall[x][y]
            if j == 1:
                inner_code *= 4
            code += inner_code
        encoded_str += ENCODE_MAP[code]
    return encoded_str


def decode_grid(encoded_str):
    global DECODE_MAP
    grid = get_filled_grid(O)
    for i in range(150):
        code = DECODE_MAP[encoded_str[i]]
        for j in range(2):
            index = i * 2 + j
            x = index / 20
            y = index % 20
            grid[x + 1][y + 1] = code % 4
            code /= 4
    return grid


# headings
EAST = 0
WEST = 1
NORTH = 2
SOUTH = 3

# arranged in anti-clockwise order
NATURAL_HEADING = [EAST, NORTH, WEST, SOUTH]

HEADING_STR = {EAST: "EAST", WEST:"WEST", NORTH:"NORTH", SOUTH:"SOUTH"}

# directions of movement
FORWARD = 0
BACKWARD = 1
LEFT = 2
RIGHT = 3

# arranged in anti-clockwise order
NATURAL_DIRECTION = [FORWARD, LEFT, BACKWARD, RIGHT]

# translates direction to command
DIRECTION_TO_COMMAND = {LEFT: CM_LEFT, RIGHT: CM_RIGHT, BACKWARD: CM_BACK}

# cost of making a particular turn
TURNING_COST = {FORWARD: 0, LEFT: 2, RIGHT: 2, BACKWARD:4}

# The 9 relative cell names (L|F|R)(0|1|2)
R0 = 0
R1 = 1
R2 = 2
F0 = 3
F1 = 4
F2 = 5
L0 = 6
L1 = 7
L2 = 8
L0L = 9  # left of L0
L0LL = 10  # left left of L0
R0R = 11 # right of R0

SHIFT_CONSTANTS_FORWARD = {
    NORTH: [0, +1],
    SOUTH: [0, -1],
    EAST:  [+1, 0],
    WEST:  [-1, 0],
}

SHIFT_CONSTANTS_LEFT = {
    NORTH: [-1, 0],
    SOUTH: [+1, 0],
    EAST:  [0, +1],
    WEST:  [0, -1],
}

ALPHA_CODE = {
    R0: [ 1, -2],
    R1: [ 0, -2],
    R2: [-1, -2],
    F0: [ 2,  1],
    F1: [ 2,  0],
    F2: [ 2, -1],
    L0: [ 1,  2],
    L1: [ 0,  2],
    L2: [-1,  2],
    L0L: [1,  3],
    L0LL:[1,  4],
    R0R: [1, -3],
}


def apply_shift(coord, heading, direction, count=1):
    """
    shift a coordinate with heading, in a specified direction
    :param coord: (x, y) coordinate to be shifted
    :param heading: heading of robot
    :param direction: direction to shift towards
    :param count: number of shifts to perform
    :return: (x, y) shifted coordinate
    """
    if direction in [RIGHT, BACKWARD]:
        direction -= 1  # change right to left, backward to forward
        count *= -1
    dict_chosen = {FORWARD: SHIFT_CONSTANTS_FORWARD, LEFT: SHIFT_CONSTANTS_LEFT}[direction]
    unit_shift = dict_chosen[heading]
    x = coord[0] + count * unit_shift[0]
    y = coord[1] + count * unit_shift[1]
    return x, y


def get_alpha_coord(robot_coord, robot_heading, alpha):
    step1 = apply_shift(robot_coord, robot_heading, FORWARD, ALPHA_CODE[alpha][0])
    step2 = apply_shift(step1,       robot_heading, LEFT,    ALPHA_CODE[alpha][1])
    return step2


def get_heading_after_direction(existing_heading, direction):
    natural_heading_index = NATURAL_HEADING.index(existing_heading)
    natural_direction_index = NATURAL_DIRECTION.index(direction)
    new_natural_heading_index = (natural_heading_index + natural_direction_index) % 4
    return NATURAL_HEADING[new_natural_heading_index]


def get_align_time_index(heading):
    return 0 if heading in [EAST, WEST] else 1


def is_coord_valid((x,y)):
    x = int(x)
    y = int(y)
    return 0 <= x < 17 and 0 <= y < 22


class ArenaMap(object):

    def reset_location_to_home_south(self):
        self.robot_coord = (2.5, 2.5)  # centre x of robot
        self.robot_degree = 270.0  # orientation of robot

    def __init__(self, physical_robot=None, g=None):
        """
        map is 17 by 22. Coordinate is centered at the home corner
        """
        self.map_grid, self.obs_grid = get_initial_grids()
        self.robot_coord = (2.5, 2.5)  # centre x of robot
        self.robot_degree = 270.0  # orientation of robot
        self.reset_location_to_home_south()

        self.robot_state = ST_READY  # robot main state

        self.algo_states = {}  # generator objects for algorithm functions
        self.algo_substate = 0  # each algo function uses this substate differently
        self.algo_exp1_step = 0
        self.algo_super_state = 0
        self.algo_step = 0  # number of times step_algorithm() is called
        self.algo_movement_step = 0  # number of times robot moves [forward, back, left, right]
        self.algo_align_time = [0, 0]  # algo_movement_step when last aligned (HORIZONTAL, VERTICAL)
        self.coord_heading_reading = {}  # a map of (coord, heading) -> sensor_reading_now
        self.algo_sequence_sent = False

        self.permission_balance  = simulator_mode  # in case of physical robot, use this as condition to start balancing
        self.permission_align    = simulator_mode  # in case of physical robot, use this as condition to start aligning
        self.permission_explore  = simulator_mode  # in case of physical, use this as condition to start exploration
        self.permission_shortest = simulator_mode  # in case of physical, use this as condition to start shortest path
        self.w_number_grid = []  # for debugging
        self.w_detail_grid = []  # for debugging

        self.time_max = 5 * 60
        self.time_explore_begin = 0
        self.time_explore_end = 0
        self.time_home_align_done = 0
        self.time_shortest_begin = 0
        self.time_shortest_end = 0
        self.max_exploration_percentage = 100

        #  the PhysicalRobot object
        self.physical_robot = physical_robot
        """:type : PhysicalRobot"""
        self.g = g
        """:type : GlobalState"""

    def reset_grids(self):
        self.map_grid, self.obs_grid = get_initial_grids()

    def get_robot_coord(self):
        return self.robot_coord

    def get_robot_degree(self):
        return self.robot_degree

    def get_robot_heading(self):
        degree = self.robot_degree
        if degree <= 45 or degree > 315:
            return EAST
        if 45 < degree <= 135:
            return NORTH
        if 135 < degree <= 225:
            return WEST
        if 225 < degree <= 315:
            return SOUTH
        raise Exception("Bad degree: {}".format(degree))

    def get_direction_for_target_heading(self, target_heading):
        ds = [LEFT, RIGHT, FORWARD, BACKWARD]
        for d in ds:
            if get_heading_after_direction(self.get_robot_heading(), d) == target_heading:
                return d

    def get_map_at(self, (x, y)):
        """
        only used by gav() and get_r_vals()
        """
        x = int(x)
        y = int(y)
        if not is_coord_valid((x,y)):
            return INVALID
        return self.map_grid[x][y]

    def get_time_explore(self):
        if self.time_explore_begin == 0:
            return 0
        if self.time_explore_end == 0:
            return time.time() - self.time_explore_begin
        return self.time_explore_end - self.time_explore_begin

    def get_time_shortest(self):
        if self.time_shortest_begin == 0:
            return 0
        if self.time_shortest_end == 0:
            return time.time() - self.time_shortest_begin
        return self.time_shortest_end - self.time_shortest_begin

    def get_num_unexplored(self):
        # return sum([sum([self.get_map_at(x,y) for x in range(17)]) for y in range(22)])
        # line below preferred since it's more readable
        count = 0
        for x in range(1,16):
            for y in range(1,21):
                if self.map_grid[x][y] == U:
                    count += 1
        return count

    def should_stop_explore(self):
        cond1 = self.get_time_explore() > self.time_max
        cond2 = float(300 - self.get_num_unexplored()) / 3 > self.max_exploration_percentage
        return cond1 or cond2

    def set_map_at(self, (x, y), val):
        """
        only used by do_detection()
        """
        x = int(x)
        y = int(y)
        if x in [0, 16] or y in [0, 21]:
            return
        self.map_grid[x][y] = val

    def get_obs_at(self, (x,y)):
        x = int(x)
        y = int(y)
        if is_coord_valid((x,y)):
            return self.obs_grid[x][y]
        else:
            return INVALID

    def gac(self, alpha):
        """
        Get Alpha Coordinate: returns the coordinate of the corresponding alpha point
        :param alpha: one of R0,R1,R2 and so on for L and F
        :return: the coordinate of the corresponding alpha
        """
        coord = self.get_robot_coord()
        heading = self.get_robot_heading()
        return get_alpha_coord(coord, heading, alpha)

    """
    only those who use L0L can potentially get INVALID. Who use L0L?
    - do_detection (error-proof)
    - path planning
      - computing a1
      - computing a2
    """

    def gav(self, alpha):
        """
        Get value of alpha
        :param alpha: one of R0,R1,R2 and so on for L and F
        :return: C/U/O of corresponding alpha point
        """
        return self.get_map_at(self.gac(alpha))

    def get_r_vals(self):
        """
        Gets the map value at R0,R1,R2
        :return: map values of R0,R1,R2 in a list
        """
        return [self.get_map_at(self.gac(alpha)) for alpha in [R0, R1, R2]]

    def do_detection(self):
        """
        Looks at F0, F1, F2 for obstacles. When they contain obstacle,
        :return: if F0, F1, F2 are all clean, return True, else False
        """
        targets = [F0, F1, F2, L0, L0L, L0LL]
        coords = [self.gac(t) for t in targets]

        if self.physical_robot is None:
            true_values = [self.get_obs_at(c) for c in coords]
        else:
            true_values = self.physical_robot.get_ahead_vals()
            # record reading at present (coord, heading)
            x, y = self.get_robot_coord()
            x = int(x)
            y = int(y)
            robo_heading = self.get_robot_heading()
            self.coord_heading_reading[((x,y), robo_heading)] = self.physical_robot.sensor_reading_now

        if true_values[3] == O:  # L0 blocks L0L and L0LL
            true_values[4] = U
            true_values[5] = U
        if true_values[4] == O:  # L0L blocks L0LL
            true_values[5] = U

        if not use_L0LL:
            true_values[5] = U

        # set map only when map is not U or INVALID
        for coord, value in zip(coords, true_values):
            if value not in [U, INVALID]:
                self.set_map_at(coord, value)
        return all([tv == C for tv in true_values[:3]])

    def get_map_u_count(self):
        return sum([sum([v == U for v in column]) for column in self.map_grid])

    def need_align_ahead(self):
        align_index = get_align_time_index(self.get_robot_heading())
        return self.algo_movement_step - self.algo_align_time[align_index] > 5

    def need_align_side(self):
        heading_ahead = self.get_robot_heading()
        heading_side = get_heading_after_direction(heading_ahead, LEFT)
        align_index = get_align_time_index(heading_side)
        return self.algo_movement_step - self.algo_align_time[align_index] > 5

    def shall_align_ahead(self):
        needed = self.need_align_ahead()
        vals = [self.gav(a) for a in [F0, F1, F2]]
        result = needed and all([self.gav(a) == O for a in [F0, F1, F2]])
        return result

    def shall_align_left(self):
        needed = self.need_align_side()
        vals = [self.gav(a) for a in [L0, L1, L2]]
        result = needed and all([self.gav(a) == O for a in [L0, L1, L2]])
        return result

    def shall_align_right(self):
        needed = self.need_align_side()
        vals = [self.gav(a) for a in [R0, R1, R2]]
        result = needed and all([self.gav(a) == O for a in [R0, R1, R2]])
        return result

    def command_movement(self, command):
        if command in [CM_FORWARD]:  # previously also had left, right and back
            self.algo_movement_step += 1

        if command == CM_FORWARD:
            new_coord = apply_shift(self.robot_coord, self.get_robot_heading(), FORWARD)
            self.robot_coord = new_coord
        elif command == CM_BACK:
            new_degree = (self.robot_degree + 180) % 360
            self.robot_degree = new_degree
        elif command == CM_LEFT:
            new_degree = (self.robot_degree + 90) % 360
            self.robot_degree = new_degree
        elif command == CM_RIGHT:
            new_degree = (self.robot_degree + 270) % 360
            self.robot_degree = new_degree
        elif command == CM_ALIGN:
            heading = self.get_robot_heading()
            align_index = get_align_time_index(heading)
            self.algo_align_time[align_index] = self.algo_movement_step
        elif command in [CM_INIT_ALIGN, CM_INIT_BALANCE]:
            pass
        elif CM_MULTI_FORWARD < command < CM_MULTI_FORWARD + 31:
            num_steps = command - CM_MULTI_FORWARD
            new_coord = apply_shift(self.robot_coord, self.get_robot_heading(), FORWARD, num_steps)
            self.robot_coord = new_coord

        else:
            assert False, "command not accepted, command={}".format(command)

        if self.g is not None:
            dp("physical command: " + str(command))
            self.g.command_algo = command

    def get_encoded_map(self):
        return encode_grid(self.map_grid)

    def get_encoded_obs(self):
        return encode_grid(self.obs_grid)

    def set_map_to_file(self, filename):
        f = open(filename, 'r')
        map_str = f.read().strip()
        self.map_grid = decode_grid(map_str)
        f.close()

    def set_obs_to_file(self, filename):
        f = open(filename, 'r')
        map_str = f.read().strip()
        self.obs_grid = decode_grid(map_str)
        f.close()

    def get_w_grid_and_list(self, include_u=False):
        w_grid = get_filled_grid(None)
        w_list = []
        for x in range(1,16):
            for y in range(1,21):
                vs0 = self.map_grid[x - 1][y - 1:y + 2]
                vs1 = self.map_grid[x    ][y - 1:y + 2]
                vs2 = self.map_grid[x + 1][y - 1:y + 2]
                # if U is included, then as long as there's no O in the 3x3 grid, it's walkable
                cond1 = include_u and all([O not in vs for vs in [vs0, vs1, vs2]])
                # if don't include U, then all 3x3 must be C for it to be walkable
                cond2 = not include_u and all([[C,C,C] == vs for vs in [vs0, vs1, vs2]])
                if cond1 or cond2:
                    w_grid[x][y] = W((x,y))
                    w_list.append(w_grid[x][y])
                else:
                    w_grid[x][y] = None
        return w_grid, w_list

    def update_w_number_grid(self, w_grid):
        self.w_number_grid = [
            [None if not w_grid[x][y] else w_grid[x][y].distance for y in range(22)] for x in range(17)]

        def get_w_detail(w):
            return {"a2": w.a2, "t2": w.t2, "sd": w.second_direction}
        self.w_detail_grid = [
            [None if not w_grid[x][y] else get_w_detail(w_grid[x][y]) for y in range(22)] for x in range(17)]


class W:
    def __init__(self, (x, y)):
        self.coord = (x, y)
        self.predecessor = None
        self.first_direction = None
        self.heading = None
        self.distance = DISTANCE_INFINITY
        self.a1 = 0
        self.a1_set = set()
        self.a2_list = [0, 0, 0, 0]
        self.t2_list = [DISTANCE_INFINITY, DISTANCE_INFINITY, DISTANCE_INFINITY, DISTANCE_INFINITY]
        self.a2 = 0
        self.t2 = 999
        self.second_direction = None
        self.second_heading = None

    def get_backtrace(self):
        backtrace = [self]
        current = self.predecessor
        while current is not None:
            backtrace.append(current)
            current = current.predecessor
        return backtrace

    def get_trace(self):
        backtrace = self.get_backtrace()
        backtrace.reverse()
        return backtrace
