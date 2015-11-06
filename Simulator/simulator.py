
ST_READY          ="ready"
ST_INIT_BALANCING ="init_balancing"
ST_INIT_BALANCED  ='init_balanced'
ST_INIT_ALIGNING  ='init_aligning'
ST_INIT_ALIGNED   ='init_aligned'
ST_EXP1_ING       ='exp1_ing'
ST_EXP1_DONE      ='exp1_done'
ST_EXP2_ING       ='exp2_ing'
ST_EXP2_DONE      ='exp2_done'
ST_HOMING         ='homing'
ST_HOMED          ='homed'
ST_ALIGNING2      ='aligning2'
ST_ALIGNED2       ='aligned2'
ST_SHORTESTING    ='shortesting'
ST_GOAL           ='goal'

ST_ERROR_ARDUINO  ='error_arduino'
ST_ERROR_RPI      ='error_pi'
ST_ERROR_ABORT    ='error_abort'


"""
Below is a list of commands.
1. Pi gives a command to arduino when necessary
2. arduino constantly report its "currently executing command"
CM_ABORT is used by pi to abort operation in arduino
CM_ERROR is used only by arduino to indicate arduino has somehow failed
"""

CM_FORWARD = 1  # go forward 1 cell
CM_LEFT = 2  # turn left
CM_RIGHT = 3  # turn right
CM_BACK = 4  # turn back
CM_INIT_BALANCE = 8  # initial 2-wheel balancing sequence
CM_INIT_ALIGN = 9  # initial distance & orientation alignment sequence
CM_ALIGN = 10  # perform distance and orientation alignment
CM_WAIT = 11  # wait indefinitely for new commands, equivalent to RESET
CM_STAY = 12  # stay for 0.25 seconds, then switch state to Next
CM_ABORT = 16  # give up, just stop
CM_ERROR = 17  # this is not a command but a status report by Arduino itself

C = 0  # clean
U = 1  # unknown
O = 2  # obstacle


def is_wall(x, y):
    return x == 0 or x == 16 or y == 0 or y == 21


def get_initial_grids():
    map_grid = [[O for y in range(22)] for x in range(17)]
    obs_grid = [[O for y in range(22)] for x in range(17)]

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
    grid = [[O for y in range(22)] for x in range(17)]
    for i in range(150):
        code = DECODE_MAP[encoded_str[i]]
        for j in range(2):
            index = i * 2 + j
            x = index / 20
            y = index % 20
            grid[x + 1][y + 1] = code % 4
            code /= 4
    return grid


EAST = 0
WEST = 1
NORTH = 2
SOUTH = 3

NATURAL_HEADING = [EAST, NORTH, WEST, SOUTH]

FORWARD = 0
BACKWARD = 1
LEFT = 2
RIGHT = 3

NATURAL_DIRECTION = [FORWARD, LEFT, BACKWARD, RIGHT]

DIRECTION_TO_COMMAND = {LEFT: CM_LEFT, RIGHT: CM_RIGHT, BACKWARD: CM_BACK}

TURNING_COST = {FORWARD: 0, LEFT: 2, RIGHT: 2, BACKWARD:4}

R0 = 0
R1 = 1
R2 = 2
F0 = 3
F1 = 4
F2 = 5
L0 = 6
L1 = 7
L2 = 8

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
        direction -= 1 # change right to left, backward to forward
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


class ArenaMap(object):

    def reset_grids(self):
        self.map_grid, self.obs_grid = get_initial_grids()

    def __init__(self):
        """
        map is 17 by 22. Coordinate is centered at the home corner
        """
        self.map_grid, self.obs_grid = get_initial_grids()
        self.robot_coord = (2.5, 2.5)  # centre x of robot
        self.robot_degree = 0.0  # orientation of robot
        self.exploration_heading = EAST  # exploration heading, different from robot heading

        self.robot_state = ST_READY  # robot main state

        self.algo_states = {}  # generator objects for algorithm
        self.algo_substate = 0
        self.w_number_grid = []
        self.w_detail_grid = []

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

    def get_map_at(self, (x, y)):
        return self.map_grid[int(x)][int(y)]

    def set_map_at(self, (x, y), val):
        self.map_grid[int(x)][int(y)] = val

    def get_obs_at(self, (x,y)):
        return self.obs_grid[int(x)][int(y)]

    def gac(self, alpha):
        """
        Get Alpha Coordinate: returns the coordinate of the corresponding alpha point
        :param alpha: one of R0,R1,R2 and so on for L and F
        :return: the coordinate of the corresponding alpha
        """
        coord = self.get_robot_coord()
        heading = self.get_robot_heading()
        return get_alpha_coord(coord, heading, alpha)

    def get_Rs(self):
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
        targets = [F0, F1, F2]
        coords = [self.gac(t) for t in targets]
        true_values = [self.get_obs_at(c) for c in coords]
        [self.set_map_at(c, tv) for c, tv in zip(coords, true_values)]
        return all([self.get_map_at(c) == C for c in coords])

    def get_map_u_count(self):
        return sum([sum([v == U for v in column]) for column in self.map_grid])

    def command_movement(self, command):
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
        else:
            assert False, "only forward,back,left and right accepted. command={}".format(command)

    def update_exploration_heading(self):
        self.exploration_heading = self.get_robot_heading()

    def get_encoded_map(self):
        return encode_grid(self.map_grid)

    def get_encoded_obs(self):
        return encode_grid(self.obs_grid)

    def get_w_grid_and_list(self):
        w_grid = [[None for y in range(22)] for x in range(17)]
        w_list = []
        for x in range(1,16):
            for y in range(1,21):
                if (    [C,C,C] == self.map_grid[x-1][y-1:y+2] and
                        [C,C,C] == self.map_grid[x  ][y-1:y+2] and
                        [C,C,C] == self.map_grid[x+1][y-1:y+2]):
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
        self.distance = 10000000
        self.a1 = 0
        self.a2_list = [0, 0, 0, 0]
        self.t2_list = [999, 999, 999, 999]
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
