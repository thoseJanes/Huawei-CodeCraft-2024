class PIBT:
    def __init__(self, grid:np.ndarray, seed: int = 0):

        self.grid = grid
        self.robot_N = 0

        # distance table
        self.robot_dist_tables = []
        self.robot_have_goal = []

        # cache
        self.NIL = -1#self.robot_N  #代理数
        self.NIL_COORD: tuple = self.grid.shape  #地图点数
        self.occupied_now = np.full(grid.shape, self.NIL, dtype=np.int8)#占据位置。self.NIL
        self.occupied_nxt = np.full(grid.shape, self.NIL, dtype=np.int8)

        # used for tie-breaking
        self.rng = np.random.default_rng(seed)

    def robotFuncPIBT(self, Q_from: list, Q_to: Config, i: int) -> bool:
        # true -> valid, false -> invalid
        # if is_stop[i]:
        #     Q_to[i] = Q_from[i]
        #     self.occupied_nxt[Q_from[i]] = i
        #     return False
        # get candidate next vertices
        C = [Q_from[i]] + findAdjacentRoad(Q_from[i], self.grid)
        self.rng.shuffle(C)  # tie-breaking, randomize
        C = sorted(C, key=self.robot_dist_tables[i].get)
        # vertex assignment
        for v in C:
            # avoid vertex collision
            if self.occupied_nxt[v] != self.NIL:
                continue

            j = self.occupied_now[v]
            # avoid edge collision
            if j != self.NIL and Q_to[j] == Q_from[i] and \
                (not isMainRoad(Q_to[j], self.grid)) and (not isMainRoad(Q_from[i], self.grid)):
                continue
            # reserve next location
            Q_to[i] = v
            if not isMainRoad(v, self.grid):#如果不是主干道则设置为占据
                self.occupied_nxt[v] = i
            # priority inheritance (j != i due to the second condition)
            if (
                j != self.NIL
                and (Q_to[j] == self.NIL_COORD)
                and (not self.robotFuncPIBT(Q_from, Q_to, j))
            ):
                continue

            return True

        # failed to secure node
        Q_to[i] = Q_from[i]
        if not isMainRoad(Q_to[i], self.grid):#如果不是主干道则设置为占据
            self.occupied_nxt[Q_from[i]] = i
        return False

    def robotStep(self, Q_from: Config, priorities: list) -> Config:#[float]) -> Config:
        # setup
        N = len(Q_from)
        Q_to: Config = []
        for i, v in enumerate(Q_from):
            Q_to.append(self.NIL_COORD)
            if not isMainRoad(v, self.grid):
                self.occupied_now[v] = i

        # perform PIBT
        A = sorted(list(range(N)), key=lambda i: priorities[i], reverse=True)#按距离排序
        for i in A:
            if Q_to[i] == self.NIL_COORD:
                self.robotFuncPIBT(Q_from, Q_to, i)

        # cleanup
        for q_from, q_to in zip(Q_from, Q_to):
            self.occupied_now[q_from] = self.NIL
            self.occupied_nxt[q_to] = self.NIL

        return Q_to
    
    def runRobotStep(self, starts, is_work):# -> Configs:#问题在于，不仅要到达目标，还要停留一刻。
        #全部机器人都没有work也会走路？？？
        # define priorities
        no_way = []
        priorities: List[float] = []
        if not any(is_work):
            return starts, no_way
        for i in range(self.robot_N):
            if is_work[i] and self.robot_have_goal[i]:
                self.robot_dist_tables[i].setWork(True)
                dist = self.robot_dist_tables[i].get(starts[i])
                if dist < UP_LIMIT:
                    priorities.append(dist / UP_LIMIT + i*0.05)#获取距离，计算优先级。注意该策略：距离长的优先级高。也可以设置距离短的优先级高。
                else:
                    priorities.append(0+i*0.01)
                    self.robot_dist_tables[i].setWork(False)
                    no_way.append(i)
            else:
                priorities.append(0+i*0.1)#优先级最低
                self.robot_dist_tables[i].setWork(False)
        #logging.error(priorities)
        #writeFile('priorities:'+str(priorities))
        #主循环，生成机器人下一步的行动
        #configs = [self.starts]#机器人当前的位置
        Q = self.robotStep(starts, priorities)#Q为机器人下一步该走的位置

        # #更新优先级？不需要更新，实时计算即可。除非机器人处于非工作状态。
        # for i in range(self.robot_N):
        #     if Q[i] != self.goals[i]:#如果某个机器人没有达到目标点
        #         priorities[i] += 1
        #     else:
        #         priorities[i] -= np.floor(priorities[i])#如果某个机器人达到了目标点，将其priorities设置为下限。注意priorities此时有记录机器人位置的作用
        return Q, no_way#机器人下一步的位置, 以及永远无法达到目标点的机器人

    def setRobotDistTable(self, i, dist_table:np.ndarray, ready, work, goal = None, max_step = 5000, \
        Q = None):
        if i+1 > self.robot_N:
            for _ in range(i+1-self.robot_N):
                self.robot_dist_tables.append(DistTable(self.grid))
                self.robot_have_goal.append(False)
            self.robot_N = i+1
        present_dist_table = self.robot_dist_tables[i]
        present_dist_table.setTable(dist_table)
        present_dist_table.setReady(ready)
        present_dist_table.setWork(work)
        if goal:
            present_dist_table.setGoal(goal)
            self.robot_have_goal[i] = True
        if Q:
            present_dist_table.Q = Q
        present_dist_table.setMaxStep(max_step)
