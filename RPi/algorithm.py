from simulator import *
from constants import *
from globalVars import GlobalState
import time

autonomous_permission = False

def proceed_with_algo(arena, algo_function):
    """
    Execute a step in algo_function. Returns False when the algo function has completed
    :param algo_function:
    :return:
    """
    try:
        if algo_function not in arena.algo_states:
            algo_object = algo_function(arena)
            arena.algo_states[algo_function] = algo_object
        algo_object = arena.algo_states[algo_function]
        algo_object.next()
        return True
    except StopIteration:
        return False


def is_at_home((x, y)):
    return 2 <= x < 3 and 2 <= y < 3


def step_algorithm(arena):
    """
    update the state of arena based on algorithm
    :param ArenaMap arena: the arena containing the robot's state
    :return: nothing
    """
    global autonomous_permission
    arena.algo_step += 1

    if not simulator_mode:
        autonomous_permission = (arena.g.autonomous and (time.time() - arena.time_home_align_done) >= 30)
    else:
        autonomous_permission = False

    # when algo_super_state is not NONE, we have a higher-priority task at hand that we would perform first.
    if arena.algo_super_state is not SS_NONE:
        if arena.algo_super_state == SS_ALIGN_LEFT:
            if not proceed_with_algo(arena, algo_align_left):
                arena.algo_super_state = SS_NONE
            return
        elif arena.algo_super_state == SS_ALIGN_RIGHT:
            if not proceed_with_algo(arena, algo_align_right):
                arena.algo_super_state = SS_NONE
            return
        return

    # READY
    if arena.robot_state == ST_READY and arena.permission_balance:
        arena.robot_state = ST_INIT_BALANCING

        if bypass_exploration:
            arena.set_map_to_file("map_str")
            arena.set_obs_to_file("map_str")
            arena.robot_state = ST_HOME_ALIGN

        return

    # INIT_BALANCING
    if arena.robot_state == ST_INIT_BALANCING:
        if not proceed_with_algo(arena, algo_init_balancing):
            arena.robot_state = ST_INIT_BALANCED
        return

    # INIT_BALANCED
    if arena.robot_state == ST_INIT_BALANCED and arena.permission_align:
        arena.robot_state = ST_INIT_ALIGNING
        return

    # INIT_ALIGNING
    if arena.robot_state == ST_INIT_ALIGNING:
        if simulator_mode:
            arena.robot_state = ST_INIT_ALIGNED
        if not proceed_with_algo(arena, algo_init_aligning):
            arena.robot_state = ST_INIT_ALIGNED
        return

    # INIT_ALIGNED
    if arena.robot_state == ST_INIT_ALIGNED and arena.permission_explore:
        arena.robot_state = ST_EXP1_ING
        arena.algo_substate = arena.algo_step
        return

    # EXP1_ING
    if arena.robot_state == ST_EXP1_ING:

        if arena.should_stop_explore():
            arena.robot_state = ST_HOMING
            return

        arena.algo_exp1_step += 1
        coord = arena.get_robot_coord()

        # check if shall perform alignment
        if arena.shall_align_ahead():
            arena.command_movement(CM_ALIGN)
            return
        elif arena.shall_align_left():
            arena.algo_states[algo_align_left] = algo_align_left(arena)
            arena.algo_super_state = SS_ALIGN_LEFT
            return
        elif arena.shall_align_right():
            arena.algo_states[algo_align_right] = algo_align_right(arena)
            arena.algo_super_state = SS_ALIGN_RIGHT
            return

        # check if reached home again
        if is_at_home(coord) and arena.algo_exp1_step > 20:
            arena.robot_state = ST_EXP1_DONE
            return
        # algo_exp1_ing never ends on its own.
        if not proceed_with_algo(arena, algo_exp1_ing):
            arena.robot_state = ST_EXP1_DONE
        guess_remaining_unknown(arena)

    # EXP1_DONE
    elif arena.robot_state == ST_EXP1_DONE:
        if not proceed_with_algo(arena, algo_exp1_done):
            guess_remaining_unknown(arena)
            if use_exp2:
                arena.robot_state = ST_EXP2_ING
            else:
                arena.robot_state = ST_EXP2_DONE

    # EXP2_ING
    elif arena.robot_state == ST_EXP2_ING:

        if arena.should_stop_explore():
            arena.robot_state = ST_HOMING
            return

        # check if shall perform alignment
        if arena.shall_align_ahead():
            arena.command_movement(CM_ALIGN)
            return
        elif arena.shall_align_left():
            arena.algo_states[algo_align_left] = algo_align_left(arena)
            arena.algo_super_state = SS_ALIGN_LEFT
            return
        elif arena.shall_align_right():
            arena.algo_states[algo_align_right] = algo_align_right(arena)
            arena.algo_super_state = SS_ALIGN_RIGHT
            return
        # proceed with normal algo
        if not proceed_with_algo(arena, algo_exp2_ing):
            arena.robot_state = ST_EXP2_DONE
        guess_remaining_unknown(arena)

    # EXP2_DONE
    elif arena.robot_state == ST_EXP2_DONE:
        arena.robot_state = ST_HOMING
        guess_remaining_unknown(arena)

    # HOMING
    elif arena.robot_state == ST_HOMING:
        # check if shall perform alignment
        if arena.shall_align_ahead():
            arena.command_movement(CM_ALIGN)
            return
        elif arena.shall_align_left():
            arena.algo_states[algo_align_left] = algo_align_left(arena)
            arena.algo_super_state = SS_ALIGN_LEFT
            return
        elif arena.shall_align_right():
            arena.algo_states[algo_align_right] = algo_align_right(arena)
            arena.algo_super_state = SS_ALIGN_RIGHT
            return

        if not proceed_with_algo(arena, algo_homing):
            arena.robot_state = ST_HOME_ALIGN
            arena.time_explore_end = time.time()

    # HOME_ALIGN
    elif arena.robot_state == ST_HOME_ALIGN:
        if not proceed_with_algo(arena, algo_home_align):
            arena.robot_state = ST_HOMED
            arena.time_home_align_done = time.time()

    # HOMED
    elif arena.robot_state == ST_HOMED and (arena.permission_shortest or autonomous_permission):
        arena.robot_state = ST_SHORTESTING
        guess_remaining_unknown(arena)

    # SHORTESTING
    elif arena.robot_state == ST_SHORTESTING:
        if not simulator_mode and use_064:
            if arena.algo_sequence_sent:
                if arena.physical_robot.command_done:
                    arena.robot_state = ST_GOAL
                    arena.time_shortest_end = time.time()
                else:
                    return

            else:
                command_sequence = []
                while proceed_with_algo(arena, algo_shortesting):
                    if arena.g.command_algo is not None:
                        command_sequence.append(arena.g.command_algo)
                arena.g.command_sequence = command_sequence
                arena.g.command_algo = None

                arena.algo_sequence_sent = True

        else:
            if not proceed_with_algo(arena, algo_shortesting):
                arena.robot_state = ST_GOAL
                arena.time_shortest_end = time.time()

    # GOAL
    elif arena.robot_state == ST_GOAL:
        # guess_remaining_unknown(arena)
        return


def algo_init_balancing(arena):
    """
    Performs initial wheel balancing via 3 actions
    :param ArenaMap arena: ll
    :return:
    """
    yield
    return
    """
    arena.command_movement(CM_INIT_BALANCE)
    yield
    arena.command_movement(CM_BACK)
    yield
    arena.command_movement(CM_INIT_BALANCE)
    yield
    arena.reset_location_to_home_south()
    """


def algo_init_aligning(arena):
    """
    read ideal values
    1. send the CM_INIT_ALIGN
    2. wait 2 seconds for reading to come back and do average
    3. move on
    :param ArenaMap arena: lol
    """
    # 1
    arena.command_movement(CM_INIT_ALIGN)
    yield

    # 2
    time_begin = time.time()
    time_last = None
    readings = []

    while True:
        if time_last == arena.physical_robot.reading_time:
            yield
        time_last = arena.physical_robot.reading_time
        readings.append(arena.physical_robot.sensor_reading_now)
        time_now = time.time()
        yield
        if time_now - time_begin > 3:
            break

    sums = [0, 0, 0, 0, 0, 0]
    for reading in readings:
        for i in range(6):
            sums[i] += reading[i]
    for i in range(6):
        sums[i] = float(sums[i]) / len(readings)

    arena.physical_robot.sensor_reading_ideal = sums
    print "number of samples: ", len(readings)
    print "ideal_readings: ", arena.physical_robot.sensor_reading_ideal


def condition_alpha(arena):
    """
    Returns if alpha conditoin is satisfied
    Alpha condition =
        R2==U OR
        R0,R1,R2 are all in [U,C]
    :param ArenaMap arena: the arena containing the robot's state
    :return:
    """
    need_align = arena.need_align_side()
    r_values = arena.get_r_vals()
    cond_for_align = need_align and C not in r_values and U in r_values
    return r_values[2] == U or all([v in [U, C] for v in r_values]) or cond_for_align


def algo_exp1_ing(arena):
    """
    If condition Alpha is satisfied, check Right
        if R0,R1,R2 all C, set heading to Right
        else recover heading and proceed down the list
    IF no O detected right ahead, mark F0,F1,F2 as C, go ahead 1 cell
    ELSE, mark corresponding block as O
        left-checking sequence
            before check left, check right first if any of R0,R1,R2 is U
            check left
                IF L0,L1,L2 all are C, set to Left
                ELSE, set to Back

    :param ArenaMap arena: and here's another comment
    """
    arena.time_explore_begin = time.time()

    while True:
        arena.do_detection()
        if condition_alpha(arena):
            arena.command_movement(CM_RIGHT)
            yield
            all_clean = arena.do_detection()
            if all_clean:
                pass
            else:
                arena.command_movement(CM_LEFT)
                yield
        all_clean = arena.do_detection()
        if all_clean:
            arena.command_movement(CM_FORWARD)
            yield
        else:
            if U in arena.get_r_vals():
                arena.command_movement(CM_RIGHT)
                yield
                arena.do_detection()
                arena.command_movement(CM_LEFT)
                yield
            arena.command_movement(CM_LEFT)
            yield
            all_clean = arena.do_detection()
            if all_clean:
                pass
            else:
                arena.command_movement(CM_LEFT)
                yield


def algo_exp1_done(arena):
    """
    1. align with both walls (not implemented)
    2. turn to EAST
    3. start exp2
    :param ArenaMap arena: arena
    """
    # align not implemented
    # yield
    heading = arena.get_robot_heading()
    if heading == WEST:
        arena.command_movement(CM_BACK)
        yield
    elif heading == NORTH:
        arena.command_movement(CM_RIGHT)
        yield
    elif heading == SOUTH:
        arena.command_movement(CM_LEFT)
        yield


def dijkstra_update_cost(w_predecessor, direction, w_grid):
    next_coord = apply_shift(w_predecessor.coord, w_predecessor.heading, direction)
    w = w_grid[next_coord[0]][next_coord[1]]
    if w is None:
        return
    cost = TURNING_COST[direction] + 1
    new_distance = w_predecessor.distance + cost
    if new_distance < w.distance:
        w.predecessor = w_predecessor
        w.distance = new_distance
        w.heading = get_heading_after_direction(w_predecessor.heading, direction)
        w.first_direction = direction


def dijkstra((x, y), heading, w_grid, w_list):
    todo = list(w_list)
    done = []
    w_robot = w_grid[int(x)][int(y)]
    w_robot.distance = 0
    w_robot.heading = heading
    w_robot.first_direction = FORWARD
    while len(todo) > 0:
        min_w = min(todo, key=lambda w: w.distance)
        if min_w.distance == DISTANCE_INFINITY:
            break
        todo.remove(min_w)
        done.append(min_w)
        dijkstra_update_cost(min_w, FORWARD, w_grid)
        dijkstra_update_cost(min_w, BACKWARD, w_grid)
        dijkstra_update_cost(min_w, LEFT, w_grid)
        dijkstra_update_cost(min_w, RIGHT, w_grid)


def fdivide(a, b):
    return float(a) / (b + 0.00000001)


def compute_reward_per_time(a1, a2, t1, t2):
    return fdivide(a1 + a2, t1 + t2)


def get_a2_from(coord, heading, a2_store, arena):
    """

    :param coord:
    :param heading:
    :param a2_store: dict of (a2, t2, a2_set)
    :param arena:
    :return:
    """
    key = (coord, heading)
    if key in a2_store:
        return a2_store[key]
    ahead_coords = [get_alpha_coord(coord, heading, a) for a in [F0, F1, F2, L0]]
    ahead_values = [arena.get_map_at(c) for c in ahead_coords]
    a2 = sum([v == U for v in ahead_values])
    t2 = 0
    can_forward = all([v != O for v in ahead_values])
    forward_coord = apply_shift(coord, heading, FORWARD)
    if can_forward:
        a2_inc, t2_inc = get_a2_from(forward_coord, heading, a2_store, arena)
        if a2_inc is not 0:
            a2 += a2_inc
            t2 += t2_inc + 1

    a2_store[key] = (a2, t2)
    return a2, t2


def w_look_4_directions(arena, w, a2_store):
    """
    After W's distance has been computed, we look to EAST, WEST, NORTH, SOUTH
    for a2 and t2, and determine the maximum
    :param ArenaMap arena: robot's state
    :param W w: the w for which we do the looking
    :parem a2_store: dict of (a2, t2_inc, a2_set)
    :return: nothing. w's state is mutated
    """
    directions = [FORWARD, BACKWARD, LEFT, RIGHT]
    # coord = w.coord
    for d in directions:
        t2 = TURNING_COST[d]
        new_heading = get_heading_after_direction(w.heading, d)
        a2, t2_inc = get_a2_from(w.coord, new_heading, a2_store, arena)
        t2 += t2_inc
        w.a2_list[d] = a2
        w.t2_list[d] = t2
    max_reward = 0
    for d in directions:
        reward_per_time = compute_reward_per_time(w.a1, w.a2_list[d], w.distance, w.t2_list[d])
        if reward_per_time > max_reward:
            w.second_direction = d
            w.second_heading = get_heading_after_direction(w.heading, d)
            w.a2 = w.a2_list[d]
            w.t2 = w.t2_list[d]


def trace_a1(w_list, arena):
    """
    trace a w, compute the number of Us it clears during its first path
    :param [W] w_list: a list of w to be traced
    :param ArenaMap arena: arena
    :return:
    """
    w_list_sorted = list(w_list)
    sorted(w_list_sorted, key=lambda xw:xw.distance)
    for w in w_list_sorted:
        trace = w.get_trace()
        if trace is None:  # robot's current location
            continue
        predecessor = w.predecessor
        w.a1_set = set(predecessor.a1_set)  # copy predecessor's a1_set
        # ignore the ones cleared by predecessor, consider:
        # 1. consequence of turning
        # 2. consequence of forwarding
        alpha_list = [F0,F1,F2,L0,L0L]
        coords = []
        if w.first_direction != FORWARD:  # consequence turning
            coord = predecessor.coord
            heading = w.heading
            coords = [get_alpha_coord(coord, heading, a) for a in alpha_list]
        # consequence of forwarding
        coords += [get_alpha_coord(w.coord, w.heading, a) for a in alpha_list]
        for c in coords:
            if is_coord_valid(c) and arena.get_map_at(c) == U:
                w.a1_set.add(c)
        w.a1 = len(w.a1_set)


def algo_exp2_ing(arena):
    """
    1. find all W and their distance via Dijkstra's algo
    2. compute a1 for all W
    3. compute 4 a2,t2 for all W, preserve best among 4
    4. pick best W with maximal (a1+a2)/(t1+t2)
    5. walk to W
    6. walk second path
    :param ArenaMap arena: and here's another comment
    """
    while True:
        problem_encountered = False  # if, during intended walk, an obstacle is found. if yes, rerun the loop

        if arena.get_map_u_count() == 0:
            return
        # 1, find all walkable, assuming U is clear
        include_u = False
        w_grid, w_list = arena.get_w_grid_and_list(include_u=include_u)
        # yield  # debug yield
        coord = arena.get_robot_coord()
        heading = arena.get_robot_heading()
        dijkstra(coord, heading, w_grid, w_list)
        # yield  # debug yield
        # 2 compute a1 for all W
        if include_u:
            trace_a1(w_list, arena)

        # 3 compute 4 a2,t2 for all W
        a2_store = {}
        for w in w_list:
            if w.distance == DISTANCE_INFINITY:  # it is possible to have unreachable Ws
                continue
            w_look_4_directions(arena, w, a2_store)
        arena.update_w_number_grid(w_grid)
        # yield  # debug yield
        # 4, compute reward_per_time for each w
        w_rpt_list = [compute_reward_per_time(w.a1, w.a2, w.distance, w.t2) for w in w_list]
        argmax = w_rpt_list.index(max(w_rpt_list))
        w_max = w_list[argmax]
        if max(w_rpt_list) in [0, 0.0]:
            print "Exploration cannot proceed."
            raise StopIteration()
        # yield  # debug yield
        # 5 walk to w_max
        trace = w_max.get_trace()
        arena.do_detection()
        for w_togo in trace[1:]:
            direction = w_togo.first_direction
            if direction != FORWARD:
                arena.do_detection()
                arena.command_movement(DIRECTION_TO_COMMAND[direction])
                yield
            all_clean = arena.do_detection()
            if not all_clean:
                problem_encountered = True
                break
            arena.command_movement(CM_FORWARD)
            yield
        if problem_encountered:
            break
        # 6 walk second path
        t2_already = 0
        if w_max.second_direction != FORWARD:
            arena.do_detection()
            arena.command_movement(DIRECTION_TO_COMMAND[w_max.second_direction])
            yield
            arena.do_detection()
            t2_already += TURNING_COST[w_max.second_direction]
        while t2_already < w_max.t2:
            all_clean = arena.do_detection()
            if not all_clean:
                break
            arena.command_movement(CM_FORWARD)
            yield
            arena.do_detection()
            t2_already += 1
        pass


def algo_homing(arena):
    """
    reaching into home position
    1. compute shortest path everywhere
    2. go home
    :param ArenaMap arena: and here's another comment
    """
    while True:
        # 1 compute shortest path everywhere
        w_grid, w_list = arena.get_w_grid_and_list()
        coord = arena.get_robot_coord()
        heading = arena.get_robot_heading()
        dijkstra(coord, heading, w_grid, w_list)
        arena.update_w_number_grid(w_grid)

        # 2 go home
        w_home = w_grid[2][2]
        got_problem = False
        trace = w_home.get_trace()
        for w_togo in trace[1:]:
            direction = w_togo.first_direction
            if direction != FORWARD:
                arena.command_movement(DIRECTION_TO_COMMAND[direction])
                yield
            all_clean = arena.do_detection()
            # check for mis-detection during homing
            if not all_clean:
                got_problem = True
                break
            arena.command_movement(CM_FORWARD)
            yield
        if not got_problem:
            break

shortest_w_grid = None

def algo_home_align(arena):
    """
    perform alignment at home

    1. perform alignment sequence against corner
    2. compute both-way shortest distance and choose the shorter one
    :param ArenaMap arena: and here's another comment
    """

    global shortest_w_grid
    if not bypass_exploration:
        # 1 alignment sequence, west, then south
        if arena.get_robot_heading() != WEST:
            d1 = arena.get_direction_for_target_heading(WEST)
            arena.command_movement(DIRECTION_TO_COMMAND[d1])
            yield
        arena.command_movement(CM_ALIGN)
        yield

        d2 = arena.get_direction_for_target_heading(SOUTH)
        arena.command_movement(DIRECTION_TO_COMMAND[d2])
        yield
        arena.command_movement(CM_ALIGN)
        yield

    # 2 compute both-way shortest distance

    # compute east-distance
    east_w_grid, w_list = arena.get_w_grid_and_list()
    coord = arena.get_robot_coord()
    dijkstra(coord, EAST, east_w_grid, w_list)
    w_goal = east_w_grid[14][19]
    east_distance = w_goal.distance

    # compute north-distance
    north_w_grid, w_list = arena.get_w_grid_and_list()
    coord = arena.get_robot_coord()
    dijkstra(coord, NORTH, north_w_grid, w_list)
    w_goal = north_w_grid[14][19]
    north_distance = w_goal.distance

    target_heading = EAST if east_distance <= north_distance else NORTH
    shortest_w_grid = east_w_grid if east_distance <= north_distance else north_w_grid

    if target_heading != arena.get_robot_heading():
        d = arena.get_direction_for_target_heading(target_heading)
        arena.command_movement(DIRECTION_TO_COMMAND[d])
        yield

    arena.time_home_align_done = time.time()


def algo_shortesting(arena):
    """
    reaching into goal position
    1. compute shortest path everywhere
    2. go goal
    :param ArenaMap arena: and here's another comment
    """
    global autonomous_permission, shortest_w_grid
    arena.time_shortest_begin = time.time()
    # 1 compute shortest path everywhere

    """
    w_grid, w_list = arena.get_w_grid_and_list()
    coord = arena.get_robot_coord()
    heading = arena.get_robot_heading()
    dijkstra(coord, heading, w_grid, w_list)
    arena.update_w_number_grid(w_grid)
    """

    # 2 go goal
    w_goal = shortest_w_grid[14][19]
    trace = w_goal.get_trace()

    # in continuous straight tracks if possible
    forward_count = 0
    for w_togo in trace[1:]:
        direction = w_togo.first_direction
        if direction != FORWARD:
            if forward_count == 1:
                arena.command_movement(CM_FORWARD)
                yield
            elif forward_count > 1:
                arena.command_movement(CM_MULTI_FORWARD + forward_count)
                yield
            forward_count = 0
            arena.command_movement(DIRECTION_TO_COMMAND[direction])
            forward_count += 1
            yield
        else:
            forward_count += 1
    if forward_count == 1:
        arena.command_movement(CM_FORWARD)
        yield
    elif forward_count > 1:
        arena.command_movement(CM_MULTI_FORWARD + forward_count)
        yield

    """
    # go step by step
    for w_togo in trace[1:]:
        direction = w_togo.first_direction
        if direction != FORWARD:
            arena.command_movement(DIRECTION_TO_COMMAND[direction])
            yield
        arena.command_movement(CM_FORWARD)
        yield
    """
    return


CORNERS = [(2, 2), (14, 2), (14, 19), (2, 19)]  # from home, anti-clockwise
WALLS = [(WEST, SOUTH), (EAST, SOUTH), (EAST, NORTH), (WEST, NORTH)]


def is_at_corner((x, y)):
    x = int(x)
    y = int(y)
    return (x, y) in CORNERS


def algo_align_left(arena):
    """
    turn left, align, turn right
    :param ArenaMap arena: lol
    """
    arena.command_movement(CM_LEFT)
    yield
    arena.command_movement(CM_ALIGN)
    yield
    arena.command_movement(CM_RIGHT)
    yield


def algo_align_right(arena):
    """
    turn right, align, turn left
    :param ArenaMap arena: lol
    """
    arena.command_movement(CM_RIGHT)
    yield
    arena.command_movement(CM_ALIGN)
    yield
    arena.command_movement(CM_LEFT)
    yield


def guess_remaining_unknown(arena):
    """
    use arena.coord_heading_reading to finish guessing the remaining Us
    Our specific objective is to clear unknown cells that have been reached by the robot's R0R sensing.
    There are 2 possibilities:
    1. R0R is U, and R0 is C
    2. R0R is U, and R0 is O: nothing can be done
    3. R0R is U, and R0 is U: nothing can be done

    :param ArenaMap arena: lol
    """
    for (coord,heading), readings in arena.coord_heading_reading.iteritems():
        alphas = [R0, R0R]
        coords = [get_alpha_coord(coord, heading, a) for a in alphas]
        values = [arena.get_map_at(c) for c in coords]
        # only in case 1 do we proceed
        if not (values[0] == C and values[1] == U):
            continue
        """
        Now that we have confirmed R0R is U and R0 is C, we know we could have made a frontal look at R0
        But that only works if we made a look at R0 with heading of heading_right

        xxxrr
        xcx
        xxx

        the frontal look could have been made from 3 positions:
        coord.turn_right            sensor: F0
        coord.forward1.turn_right   sensor: F1
        coord.forward2.turn_right   sensor: F2
        """
        heading_right = get_heading_after_direction(heading, RIGHT)
        possible_coords = [coord,
                           apply_shift(coord, heading, FORWARD, 1),
                           apply_shift(coord, heading, FORWARD, 2)]
        possible_frontal_readings = []
        for c in possible_coords:
            if (c, heading_right) in arena.coord_heading_reading:
                possible_frontal_readings.append(arena.coord_heading_reading[(c, heading_right)])
            else:
                possible_frontal_readings.append(None)
        # if we've never made a valid frontal reading at R0 to the right, we can't make any judgement
        if all([r is None for r in possible_frontal_readings]):
            continue
        for i, reading in enumerate(possible_frontal_readings):
            if reading is None:
                continue
            ideal_sr = arena.physical_robot.sensor_reading_ideal[i]
            reading_sr = reading[i]
            ideal_lr = arena.physical_robot.sensor_reading_ideal[5]
            reading_lr = readings[5]
            r0r_value = arena.physical_robot.get_R0R_identity(reading_sr, ideal_sr, reading_lr, ideal_lr)
            if r0r_value != U:
                arena.set_map_at(coords[1], r0r_value)

