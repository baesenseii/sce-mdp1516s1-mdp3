from simulator import *


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
    if arena.robot_state == ST_READY:
        arena.robot_state = ST_EXP1_ING

    if arena.robot_state == ST_EXP1_ING:
        if is_at_home(arena.get_robot_coord()):
            arena.algo_substate += 1
        if arena.algo_substate == 2:  # first time start, second time getting back
            arena.robot_state = ST_EXP1_DONE
            return
        # algo_exp1_ing never ends on its own.
        if not proceed_with_algo(arena, algo_exp1_ing):
            arena.robot_state = ST_EXP1_DONE

    elif arena.robot_state == ST_EXP1_DONE:
        if not proceed_with_algo(arena, algo_exp1_done):
            arena.robot_state = ST_EXP2_ING

    elif arena.robot_state == ST_EXP2_ING:
        if not proceed_with_algo(arena, algo_exp2_ing):
            arena.robot_state = ST_EXP2_DONE

    elif arena.robot_state == ST_EXP2_DONE:
        arena.robot_state = ST_HOMING

    elif arena.robot_state == ST_HOMING:
        if not proceed_with_algo(arena, algo_homing):
            arena.robot_state = ST_HOMED

    elif arena.robot_state == ST_HOMED:
        # TODO shall perform alignment first
        arena.robot_state = ST_SHORTESTING

    elif arena.robot_state == ST_SHORTESTING:
        if not proceed_with_algo(arena, algo_shortesting):
            arena.robot_state = ST_GOAL

    elif arena.robot_state == ST_GOAL:
        return


def condition_alpha(arena):
    """
    Returns if alpha conditoin is satisfied
    Alpha condition =
        R2==U OR
        R0,R1,R2 are all in [U,C]
    :param ArenaMap arena: the arena containing the robot's state
    :return:
    """
    R_values = arena.get_Rs()
    return R_values[2] == U or all([v in [U, C] for v in R_values])


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
    while True:
        arena.do_detection()
        if condition_alpha(arena):
            arena.command_movement(CM_RIGHT)
            yield
            all_clean = arena.do_detection()
            if all_clean:
                arena.update_exploration_heading()
            else:
                arena.command_movement(CM_LEFT)
                yield
        all_clean = arena.do_detection()
        if all_clean:
            arena.command_movement(CM_FORWARD)
            yield
        else:
            if U in arena.get_Rs():
                arena.command_movement(CM_RIGHT)
                yield
                arena.do_detection()
                arena.command_movement(CM_LEFT)
                yield
            arena.command_movement(CM_LEFT)
            yield
            all_clean = arena.do_detection()
            if all_clean:
                arena.update_exploration_heading()
            else:
                arena.command_movement(CM_LEFT)
                yield
                arena.update_exploration_heading()


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

    arena.robot_state = ST_EXP2_ING


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
    assert w_robot is not None, "w_robot is none!"
    w_robot.distance = 0
    w_robot.heading = heading
    w_robot.first_direction = FORWARD
    while len(todo) > 0:
        min_w = min(todo, key=lambda w: w.distance)
        print min_w.coord, min_w.distance, min_w.heading
        todo.remove(min_w)
        done.append(min_w)
        dijkstra_update_cost(min_w, FORWARD, w_grid)
        dijkstra_update_cost(min_w, BACKWARD, w_grid)
        dijkstra_update_cost(min_w, LEFT, w_grid)
        dijkstra_update_cost(min_w, RIGHT, w_grid)


def fdivide(a, b):
    return float(a) / (b+0.00000001)


def compute_reward_per_time(a1, a2, t1, t2):
    return fdivide(a1 + a2, t1 + t2)


def get_a2_from(coord, heading, a2_store, arena):
    key = (coord, heading)
    if key in a2_store:
        return a2_store[key]
    ahead_coords = [get_alpha_coord(coord, heading, a) for a in [F0, F1, F2]]
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
    :return: nothing. w's state is mutated
    """
    directions = [FORWARD, BACKWARD, LEFT, RIGHT]
    # coord = w.coord
    # print "Looking for w: ", coord
    for d in directions:
        t2 = TURNING_COST[d]
        new_heading = get_heading_after_direction(w.heading, d)
        a2, t2_inc = get_a2_from(w.coord, new_heading, a2_store, arena)
        t2 += t2_inc
        w.a2_list[d] = a2
        w.t2_list[d] = t2
        # print "d, a2, t2=", d, a2, t2
    max_reward = 0
    for d in directions:
        reward_per_time = compute_reward_per_time(w.a1, w.a2_list[d], w.distance, w.t2_list[d])
        if reward_per_time > max_reward:
            w.second_direction = d
            w.second_heading = get_heading_after_direction(w.heading, d)
            w.a2 = w.a2_list[d]
            w.t2 = w.t2_list[d]


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
        if arena.get_map_u_count() == 0:
            raise StopIteration()
        # 1
        w_grid, w_list = arena.get_w_grid_and_list()
        # yield  # debug yield
        coord = arena.get_robot_coord()
        heading = arena.get_robot_heading()
        dijkstra(coord, heading, w_grid, w_list)
        # yield  # debug yield
        # 2 compute a1 for all W
        # 3 compute 4 a2,t2 for all W
        print "w_list: ", len(w_list), w_list
        a2_store = {}
        for w in w_list:
            w_look_4_directions(arena, w, a2_store)
        arena.update_w_number_grid(w_grid)
        # yield  # debug yield
        # 4, compute reward_per_time for each w
        w_rpt_list = [compute_reward_per_time(w.a1, w.a2, w.distance, w.t2) for w in w_list]
        argmax = w_rpt_list.index(max(w_rpt_list))
        w_max = w_list[argmax]
        print "w_max:", w_max.coord
        if max(w_rpt_list) in [0, 0.0]:
            print "Exploration cannot proceed."
            raise StopIteration()
        # yield  # debug yield
        # 5 walk to w_max
        trace = w_max.get_trace()
        print trace
        for w_togo in trace[1:]:
            direction = w_togo.first_direction
            print direction
            if direction != FORWARD:
                arena.command_movement(DIRECTION_TO_COMMAND[direction])
                yield
            arena.command_movement(CM_FORWARD)
            yield
        # 6 walk second path
        t2_already = 0
        if w_max.second_direction != FORWARD:
            arena.command_movement(DIRECTION_TO_COMMAND[w_max.second_direction])
            t2_already += TURNING_COST[w_max.second_direction]
            yield
        all_clean = arena.do_detection()
        while t2_already < w_max.t2:
            if not all_clean:
                break
            arena.command_movement(CM_FORWARD)
            all_clean = arena.do_detection()
            t2_already += 1
            yield
        pass


def algo_homing(arena):
    """
    reaching into home position
    1. compute shortest path everywhere
    2. go home
    :param ArenaMap arena: and here's another comment
    """
    # 1 compute shortest path everywhere
    w_grid, w_list = arena.get_w_grid_and_list()
    coord = arena.get_robot_coord()
    heading = arena.get_robot_heading()
    dijkstra(coord, heading, w_grid, w_list)
    arena.update_w_number_grid(w_grid)

    # 2 go home
    w_home = w_grid[2][2]
    trace = w_home.get_trace()
    for w_togo in trace[1:]:
        direction = w_togo.first_direction
        if direction != FORWARD:
            arena.command_movement(DIRECTION_TO_COMMAND[direction])
            yield
        arena.command_movement(CM_FORWARD)
        yield
    raise StopIteration()


def algo_shortesting(arena):
    """
    reaching into goal position
    1. compute shortest path everywhere
    2. go goal
    :param ArenaMap arena: and here's another comment
    """
    # 1 compute shortest path everywhere
    w_grid, w_list = arena.get_w_grid_and_list()
    coord = arena.get_robot_coord()
    heading = arena.get_robot_heading()
    dijkstra(coord, heading, w_grid, w_list)
    arena.update_w_number_grid(w_grid)

    # 2 go home
    w_goal = w_grid[14][19]
    trace = w_goal.get_trace()
    for w_togo in trace[1:]:
        direction = w_togo.first_direction
        if direction != FORWARD:
            arena.command_movement(DIRECTION_TO_COMMAND[direction])
            yield
        arena.command_movement(CM_FORWARD)
        yield
    raise StopIteration()
