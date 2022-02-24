from typing import List, Set, Tuple


class Problem:
    """
    +------------------------------------------------------------+
    |                 N Robots on an m x n Grid                  |
    +------------+-----------------------------------------------+
    | Substate j | Position of Robot j: ( xj, yj )               |
    +------------+-----------------------------------------------+
    | State N    | [ Substate 1, Substate  2, ..., Substate N ]  |
    +------------+-----------------------------------------------+
    | Actions ji | { Substate jf1, Substate jf2, ... }           |
    +------------+-----------------------------------------------+
    | Successor  | { State Nf1, State Nf2, ... }                 |
    +------------+-----------------------------------------------+
    | Conditions | (1) Robots move by one grid unit if           |
    |            |     vertically or horizontally and by two     |
    |            |     grid units if diagonally:                 |
    |            |     (a) xi - 1 <= xf <= xi + 1                |
    |            |     (b) yi - 1 <= yf <= yi + 1                |
    |            | (2) Robots cannot exit the grid:              |
    |            |     (a) 1 <= xj <= m                          |
    |            |     (b) 1 <= yj <= n                          |
    |            | (3) Robots cannot collide:                    |
    |            |     (a) See function "collision"              |
    +------------+-----------------------------------------------+
    """

    Substate = Tuple[int, int]
    State = List[Substate]
    Radius = float

    def __init__(self, init: State, goal: State, r: Radius, m: int, n: int):
        self.r = r
        self.m = m
        self.n = n
        self.init = init
        self.goal = goal
        self.robots = len(init)

    @staticmethod
    def collision(self, rj: Radius, sub_ji: Substate, sub_jf: Substate,
                  rk: Radius, sub_ki: Substate, sub_kf: Substate) -> bool:
        """
        Calculates the time at which a collision occurs, if a collision occurs.
        You can imagine the velocity calculations for vji, vjy, vkx and vky to
        be divided by 1 second. Therefore, if a collision occurs, it must occur
        within 1 second. This is an arbitrary time given to the robots to move
        from the initial to final substates, even though it is instantaneous.
        """
        xji, xjf = sub_ji[0], sub_jf[0]
        yji, yjf = sub_ji[1], sub_jf[1]
        xki, xkf = sub_ki[0], sub_kf[0]
        yki, ykf = sub_ki[1], sub_kf[1]
        vjx, vjy = xjf - xji, yjf - yji
        vkx, vky = xkf - xki, ykf - yki
        a = (vjx - vkx)**2 + (vjy - vky)**2
        b = 2*(vjx - vkx) * (xji - xki) + 2*(vjy - vky)*(yji - yki)
        c = xji**2 + xki**2 + yji**2 + yki**2 - 2*xji*xki - 2*yji*yki - (rj + rk)**2
        root = b**2 - 4*a*c
        if a == 0 or root < 0:
            return False
        t1 = (-b + root**0.5) / (2*a)
        t2 = (-b - root**0.5) / (2*a)
        if 0 <= t1 <= 1 or 0 <= t2 <= 1:
            return True
        return False

    def successor(self, state: State) -> Set[State]:
        """Generate set of reachable states from current state."""
        acts = []  # Substates reachable to each robot j, indexed by robot.
        # Generating actions for each robot j.
        for sub in state:
            acts_j = set()
            # (1) Robots move by one grid unit
            for act in [0, 1, -1]:
                xi, yi = sub[0], sub[1]
                xf, yf = sub[0] + act, sub[1] + act
                xd, yd = sub[0] + act, sub[1] - act
                # (2) Robots cannot move outside grid
                if 1 <= xf <= self.m:
                    # Horizontal movement
                    acts_j.add((xf, yi))
                if 1 <= yf <= self.n:
                    # Vertical movement
                    acts_j.add((xi, yf))
                if (1 <= xf <= self.m and
                        1 <= yf <= self.n):
                    # Upper left and lower right diagonal movement
                    acts_j.add((xf, yf))
                if (1 <= xd <= self.m and
                        1 <= yd <= self.n):
                    # Upper right and lower left diagonal movement
                    acts_j.add((xd, yd))
            acts.append(acts_j)

        succ = []
        ini_state = state  # name change

        def recurse_combine(acts, N=0, new_state=[]):
            """
            Generates N nested for loops to combine all possible actions.
            """
            if N == self.robots:
                # (3) Robots cannot collide
                for j in range(N):
                    sub_ji = ini_state[j]
                    sub_jf = new_state[j]
                    for sub_ki, sub_kf in zip(ini_state[j + 1:],
                                              new_state[j + 1:]):
                        if self.collision(self.r, sub_ji, sub_jf,
                                          self.r, sub_ki, sub_kf):
                            return  # Break all loops
                return succ.append(tuple(new_state))
            for sub_jf in acts[N]:
                if sub_jf not in new_state:
                    recurse_combine((acts, N + 1, new_state + [sub_jf])

        recurse_combine(acts)
        return set(succ)

    def goal_test(self, state):
        """True if the state is a goal."""
        return state == self.goal
