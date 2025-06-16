import numpy as np
from typing import List, Tuple
import logging


Config = List[Tuple]
Configs = List[List[Tuple]]
#初始化常量字典

#C为被货物填充的港口
'''
‘.’ ： 空地
‘>’ ： 陆地主干道
‘*’ ： 海洋
‘~’ ： 海洋主航道
‘#’ ： 障碍
‘R’ ： 机器人购买地块，同时该地块也是主干道
‘S’ ： 船舶购买地块，同时该地块也是主航道
‘B’ ： 泊位,也可视为海陆立体交通地块，同时为主干道和主航道
‘K’ ： 靠泊区
‘C’ ： 海陆立体交通地块
‘c’ ： 海陆立体交通地块，同时为主干道和主航道
‘T’ ： 交货点
'''

if True:#字典
    enum = -1
    def enum_p():
        global enum
        enum+=1
        return enum
    enum = -1
    ROBOT_STATE_TO_NUM = {
        'free':enum_p(),#注意把free设为0
        'get':enum_p(),
        'pull':enum_p(),
        'stop':enum_p()
    }
    BOAT_ORDER_TO_NUM = {
        'free':enum_p(),#注意把free设为0
        'ship':enum_p(),
        'rot':enum_p(),
        'berth':enum_p(),
        'dept':enum_p(),
        'stop':enum_p()
    }
    enum = -1
    CH_TO_NUM = {
        '*':enum_p(), \
        'T':enum_p(), 'K':enum_p(), 'S':enum_p(), '~':enum_p(), \

        'C':enum_p(), 'c':enum_p(), 'B':enum_p(),\
        
        'R':enum_p(), '>':enum_p(), \
        '.':enum_p(), \
        
        '#':enum_p(),\
    }
    MOVE_TO_DIR = {(1,0):3,(-1,0):2,(0,-1):1,(0,1):0}
    DIR_TO_MOVE = {3:(1,0),2:(-1,0),1:(0,-1),0:(0,1)}

if True:#判断函数
    def isRoad(pos:tuple, grid_map):#, CH_TO_NUM:dict):
        if CH_TO_NUM['.'] >= grid_map[pos] >= CH_TO_NUM['C']:
            return True
        return False
    def isMainRoad(pos:tuple, grid_map):
        if CH_TO_NUM['c'] <= grid_map[pos] <= CH_TO_NUM['>']:
            return True
        return False

    def isWay(pos:tuple, grid_map):
        if CH_TO_NUM['B'] >= grid_map[pos]:
            return True
        return False
    def isMainWay(pos:tuple, grid_map):
        if CH_TO_NUM['*'] < grid_map[pos] < CH_TO_NUM['R'] and\
            grid_map[pos] != CH_TO_NUM['C']:
            return True
        return False

    def isRoadWay(pos:tuple, grid_map):
        if CH_TO_NUM['C'] <= grid_map[pos] <= CH_TO_NUM['B']:
            return True
        return False

    def isRoadWayButNotBerth(pos:tuple, grid_map):
        if CH_TO_NUM['C'] <= grid_map[pos] <= CH_TO_NUM['c']:
            return True
        return False

    def isBerth(pos:tuple, grid_map:np.ndarray):#, CH_TO_NUM:dict):
        if CH_TO_NUM['B'] == grid_map[pos]:
            return True
        return False
    def isGood(pos:tuple, goods_dict:dict):#注意是未被拿起的货物
        # global CH_TO_NUM,grid_map
        # if CH_TO_NUM['G'] == grid_map[pos]:
        #     return True
        # return False
        if goods_dict.get(pos, 0):
            return True
        return False

    def isKArea(pos:tuple, grid_map:np.ndarray):
        if CH_TO_NUM['K'] == grid_map[pos]:
            return True
        return False
    def isTransportPoint(pos:tuple, grid_map:np.ndarray):
        if CH_TO_NUM['T'] == grid_map[pos]:
            return True
        return False

#CH_TO_NUM['A'] = CH_TO_NUM['.']
UP_LIMIT = 8000#注意为了判断船，需要小于2^15/4？

class DoubleDistTable:#海陆双用table
    def __init__(self, grid:np.ndarray, table:np.ndarray = None, max_step:int = 5000, \
        road_way_list:list = None, road_way_table:np.ndarray = None, road_way_offset:list = None):
        self.max_step = max_step
        self.grid = grid
        self.work = True
        self.ready = False
        #self.berth_table = berth_table
        self.CH_TO_NUM = CH_TO_NUM
        self.batch_num = 6
        self.goal = None
        
        self.road_way_list = road_way_list
        self.road_way_table = road_way_table
        self.road_way_offset = road_way_offset
        if type(table) == np.ndarray:
            self.table = table
        else:
            self.table=np.full(self.grid.shape, UP_LIMIT, dtype=np.int16)

    '''    def get(self, target: tuple, regardless_work: bool = False, road = True) -> int:#原get函数
        # check valid input
        if not isRoad(target, self.grid):
            return UP_LIMIT

        if not regardless_work:
            if not self.work:
                return 0
            if not self.ready:
                if self.have_goal:
                    return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])
                return 0 
        
        if not self.have_goal:
            #dist_table没有设置目标点！！
            return UP_LIMIT

        # distance has been known
        if self.table[target] < UP_LIMIT:
            return self.table[target]

        # BFS with lazy evaluation
        while len(self.Q) > 0:
            self.Q.sort(key=lambda u: self.calKey(u, target))
            if len(self.Q)>self.batch_num:
                u_list = [self.Q.pop(0) for _ in range(self.batch_num)]
            else:
                u_list = [self.Q.pop(0) for _ in range(len(self.Q))]
            #u = self.Q.pop(0)
            for u in u_list:
                d = int(self.table[u])
                for v in findNeighbors(u, self.grid):
                    if d + 1 < self.table[v]:#如果v处的值比u处值+1还大。可以用于多目标（目标区域）情况。
                        self.table[v] = d + 1
                        self.Q.append(v)
                if u == target:
                    return d
                if d > self.max_step:
                    return UP_LIMIT
        return UP_LIMIT
'''
    def get(self, target: tuple, regardless_work: bool = False, road:bool = True):#海陆两用get函数
        '''
        如果没有ready则会返回估计值。
        '''
        # check valid input
        terrain_judge = isRoad(target, self.grid) if road else isWay(target, self.grid)
        if not terrain_judge:
            return UP_LIMIT

        if not regardless_work:
            if not self.work:
                return 0
            if not self.ready:
                return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])
        
        # distance has been known
        value_judge = self.getValue(target, road)
        if value_judge < UP_LIMIT:
            return value_judge

        # BFS with lazy evaluation
        temp_Q = self.Q_road if road else self.Q_way
        while len(temp_Q) > 0:
            temp_Q.sort(key=lambda u: self.calKey(u, target))
            if len(temp_Q)>self.batch_num:
                u_list = [temp_Q.pop(0) for _ in range(self.batch_num)]
            else:
                u_list = [temp_Q.pop(0) for _ in range(len(temp_Q))]
            # u = temp_Q.pop(0)
            #if len(temp_Q) > 20:
            for u in u_list:
                d = self.searchPoint(u, temp_Q, road)
                
                if u == target:
                    return d
                if d > self.max_step:
                    return UP_LIMIT
        return UP_LIMIT
    def preGet(self, target: tuple = None, road:bool = True) -> bool:#注意海搜索需要额外的矩阵
        # check valid input
        if target and (self.table[target] < UP_LIMIT):
            return True

        # BFS with lazy evaluation
        temp_Q = self.Q_road if road else self.Q_way
        if len(temp_Q) > 0:
            if target:
                temp_Q.sort(key=lambda u: self.calKey(u, target))
            if len(temp_Q)>self.batch_num:
                u_list = [temp_Q.pop(0) for _ in range(self.batch_num)]
            else:
                u_list = [temp_Q.pop(0) for _ in range(len(temp_Q))]
            # u = temp_Q.pop(0)
            #if len(temp_Q) > 20:
            for u in u_list:
                d = self.searchPoint(u, temp_Q, road)

                if u == target:
                    return True
                if d > self.max_step:
                    return True
        else:
            return True
        
        return False
    def searchPoint(self, u:tuple, temp_Q:list, road:bool = True):
        if road:
            d = int(self.table[u])
            for v in findAdjacentRoad(u, self.grid):
                if d + 1 < self.table[v]:#如果v处的值比u处值+1还大。可以用于多目标（目标区域）情况。
                    self.table[v] = d + 1
                    temp_Q.append(v)
        else:
            if isRoadWay(u, self.grid):
                road_way_id = self.road_way_table[u]
                road_way_offset = self.road_way_offset[road_way_id]
                relative_pos = (u[0]-road_way_offset[0], u[1]-road_way_offset[1])
                road_way_local_table = self.road_way_list[road_way_id]
                d = road_way_local_table[relative_pos]
            else:
                d = int(self.table[u])
            for v in findAdjacentWay(u, self.grid):
                length = 1
                if isMainWay(v, self.grid):#如果目标点是主航道，则运行距离增量为2
                    length = 2
                if isRoadWay(v, self.grid):#通过转移矩阵计算
                    road_way_id = self.road_way_table[v]
                    road_way_offset = self.road_way_offset[road_way_id]
                    relative_pos = (v[0]-road_way_offset[0], v[1]-road_way_offset[1])
                    road_way_local_table = self.road_way_list[road_way_id]
                    if d + length < road_way_local_table[relative_pos]:
                        road_way_local_table[relative_pos] = d + length
                        temp_Q.append(v)
                else:
                    if d + length < self.table[v]:#如果v处的值比u处值+1还大。可以用于多目标（目标区域）情况。
                        self.table[v] = d + length
                        temp_Q.append(v)
        return d

    def roadValue(self, target:tuple, guess = False):
        if isRoad(target, self.grid) and self.table[target] < UP_LIMIT:
            return self.table[target]
        if guess:
            return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])
        return UP_LIMIT
    def wayValue(self, target:tuple, guess = False):#注意海路的估计受到主航路的影响，是会很不准确的。
        if isRoadWay(target, self.grid):
            road_way_id = self.road_way_table[target]
            road_way_offset = self.road_way_offset[road_way_id]
            relative_pos = (target[0]-road_way_offset[0], target[1]-road_way_offset[1])
            road_way_local_table = self.road_way_list[road_way_id]
            if road_way_local_table[relative_pos] < UP_LIMIT:
                return road_way_local_table[relative_pos]
            if guess:
                return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])
            return UP_LIMIT
        if isWay(target, self.grid) and self.table[target] < UP_LIMIT:
            return self.table[target]
        return UP_LIMIT
    def getValue(self, target: tuple, road = True, guess = False):
        if road:
            if isRoad(target, self.grid) and self.table[target] < UP_LIMIT:
                return self.table[target]
            if guess:
                return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])
            return UP_LIMIT
        if isRoadWay(target, self.grid):
            road_way_id = self.road_way_table[target]
            road_way_offset = self.road_way_offset[road_way_id]
            relative_pos = (target[0]-road_way_offset[0], target[1]-road_way_offset[1])
            road_way_local_table = self.road_way_list[road_way_id]
            if road_way_local_table[relative_pos] < UP_LIMIT:
                return road_way_local_table[relative_pos]
            if guess:
                return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])
            return UP_LIMIT
        if isWay(target, self.grid) and self.table[target] < UP_LIMIT:
            return self.table[target]
        if guess:
            return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])
        return UP_LIMIT

    def calKey(self, inp: tuple, target: tuple):
        return abs(inp[0]-target[0])+abs(inp[1]-target[1])
    def setGoal(self, goal):
        if type(goal) == tuple:
            self.goal = goal
            self.table[goal] = 0
            self.Q_road = [goal]
            self.Q_way = [goal]
        else:
            self.goal = goal[0]
            for item in goal:
                self.table[item] = 0
            self.Q_road = goal.copy()
            self.Q_way = goal.copy()
    def setTable(self, table:np.ndarray, road_way_list:list = None, road_way_table:np.ndarray = None, road_way_offset:list = None):
        self.table = table
        self.road_way_list = road_way_list
        self.road_way_offset = road_way_offset
        self.road_way_table = road_way_table
    def setWork(self, b:bool):
        self.work = b
    def setReady(self, b:bool):
        self.ready = b
    def setMaxStep(self, max_step:int):
        self.max_step = max_step

class MultiDistTable:#标签表，用于判断域所属泊口
    def __init__(self, grid:np.ndarray, dist_table:np.ndarray = None, tag_table:np.ndarray = None, batch_num = 5, max_step = 5000, \
        road_way_tag_table_list:list = None, road_way_tag_list:list = None, road_way_table:np.ndarray = None, road_way_offset:list = None):
        self.grid = grid
        self.dist_table = dist_table if dist_table else np.full(self.grid.shape, UP_LIMIT, dtype=np.int16)
        self.tag_table = tag_table if tag_table else np.full(self.grid.shape, -1, dtype=np.int8)
        
        self.goal = []
        self.Q = []
        self.Q_way = []
        self.tag_num = 0
        self.tag_road_range = []#记录标签的地面广度
        
        self.max_step = max_step
        self.batch_num = batch_num
        
        self.road_way_tag_table_list = road_way_tag_table_list
        self.road_way_tag_list = road_way_tag_list
        self.road_way_table = road_way_table
        self.road_way_offset = road_way_offset
    def setWayInfo(self, road_way_tag_table_list:list = None, road_way_tag_list:list = None, road_way_table:np.ndarray = None, road_way_offset:list = None):
        self.road_way_tag_table_list = road_way_tag_table_list
        self.road_way_tag_list = road_way_tag_list
        self.road_way_table = road_way_table
        self.road_way_offset = road_way_offset
    
    def getTag(self, pos: tuple, road=True):#可以海陆获取
        if road:
            tag = self.tag_table[pos]
        else:
            if isRoadWay(pos, self.grid) and not isBerth(pos, self.grid):
                road_way_id = self.road_way_table[pos]
                road_way_offset = self.road_way_offset[road_way_id]
                relative_pos = (pos[0]-road_way_offset[0], pos[1]-road_way_offset[1])
                road_way_local_table = self.road_way_tag_list[road_way_id]
                tag = road_way_local_table[relative_pos]
            else:
                tag = self.tag_table[pos]
        if tag<0:
            return -1
        return tag
    def getDist(self, pos, guess=False):#只能搜陆路
        dist = self.dist_table[pos]
        if dist<UP_LIMIT:
            return dist
        if guess:
            tag = self.getTag(pos)
            if tag>=0:
                pos_from = self.goal[tag][0]
                return abs(pos_from[0] - pos[0]) + abs(pos_from[1] - pos[1])
            
            for tag in range(self.tag_num):
                dist_s = []
                pos_from = self.goal[tag][0]
                dist_s.append(abs(pos_from[0] - pos[0]) + abs(pos_from[1] - pos[1]))
                guess_pos = min(dist_s)
                return guess_pos
        return UP_LIMIT
    def getDistAndTag(self, pos, guess=False):#只能搜陆路
        dist = self.dist_table[pos]
        tag = self.getTag(pos)
        if dist<UP_LIMIT:#如果没有错误，会自动满足tag>=0
            return dist, tag
        if guess:
            if tag>=0:
                pos_from = self.goal[tag][0]
                return abs(pos_from[0] - pos[0]) + abs(pos_from[1] - pos[1]), tag
            
            dist_s = []
            for tag in range(self.tag_num):
                pos_from = self.goal[tag][0]
                dist_s.append(abs(pos_from[0] - pos[0]) + abs(pos_from[1] - pos[1]))
            guess_tag = np.argmin(dist_s)
            guess_pos = dist_s[guess_tag]
            return guess_pos, guess_tag
        return UP_LIMIT, -1

    def addTag(self, pos, tag):
        self.tag_table[pos] = tag
        self.dist_table[pos] = 0
        while len(self.Q)<tag+1:
            self.Q.append([])
            self.Q_way.append([])
            self.goal.append([])
            self.tag_road_range.append(0)
            self.tag_num += 1
        self.Q[tag].append(pos)
        self.Q_way[tag].append(pos)
        self.goal[tag].append(pos)
    def growTag(self, tag, road=True):#海陆两用扩展标签
        tag_Q = self.Q[tag] if road else self.Q_way[tag]
        if len(tag_Q) > 0:
            u_list = [tag_Q.pop(0) for _ in (range(len(tag_Q)) if len(tag_Q)<self.batch_num else range(self.batch_num))]
            for u in u_list:
                d = self.searchPoint(tag, u, tag_Q, road)
                if d > self.max_step:
                    return True
        else:
            return True
    def searchPoint(self, tag:int, u:tuple, temp_Q:list, road:bool = True):#海陆两用点搜索
        if road:
            d = int(self.dist_table[u])
            for v in findAdjacentRoad(u, self.grid):
                if d + 1 < self.dist_table[v]:#如果v处的值比u处值+1还大。可以用于多目标（目标区域）情况。
                    self.dist_table[v] = d + 1
                    self.tag_table[v] = tag
                    temp_Q.append(v)
                    self.tag_road_range[tag] += 1
        else:
            if isRoadWay(u, self.grid) and not isBerth(u, self.grid):
                road_way_id = self.road_way_table[u]
                road_way_offset = self.road_way_offset[road_way_id]
                relative_pos = (u[0]-road_way_offset[0], u[1]-road_way_offset[1])
                road_way_local_table = self.road_way_tag_table_list[road_way_id]
                d = road_way_local_table[relative_pos]
            else:
                d = int(self.dist_table[u])
            
            for v in findAdjacentWay(u, self.grid):
                length = 1
                if isMainWay(v, self.grid):#如果目标点是主航道，则运行距离增量为2
                    length = 2
                if isRoadWay(v, self.grid) and not isBerth(v, self.grid):#通过转移矩阵计算
                    road_way_id = self.road_way_table[v]
                    road_way_offset = self.road_way_offset[road_way_id]
                    relative_pos = (v[0]-road_way_offset[0], v[1]-road_way_offset[1])
                    road_way_local_table = self.road_way_tag_table_list[road_way_id]
                    road_way_local_tag = self.road_way_tag_list[road_way_id]
                    if d + length < road_way_local_table[relative_pos]:
                        road_way_local_table[relative_pos] = d + length
                        road_way_local_tag[relative_pos] = tag
                        temp_Q.append(v)
                else:
                    if d + length < self.dist_table[v]:#如果v处的值比u处值+1还大。可以用于多目标（目标区域）情况。
                        self.dist_table[v] = d + length
                        self.tag_table[v] = tag
                        temp_Q.append(v)
        return d

    def searchDistInTag(self, tag: int, target: tuple):#只能搜陆路
        if not isRoad(target, self.grid):
            return UP_LIMIT
        
        if self.table[target] < UP_LIMIT:
            return self.table[target]
        
        tag_Q = self.Q[tag]
        while len(tag_Q) > 0:
            tag_Q.sort(key=lambda u: self.calKey(u, target))
            u_list = [tag_Q.pop(0) for _ in (range(len(tag_Q)) if len(tag_Q)<self.batch_num else range(self.batch_num))]

            for u in u_list:
                d = int(self.dist_table[u])
                for v in findAdjacentRoad(u, self.grid):
                    if d + 1 < self.dist_table[v]:#如果v处的值比u处值+1还大。可以用于多目标（目标区域）情况。
                        self.dist_table[v] = d + 1
                        self.tag_table[v] = tag
                        tag_Q.append(v)
                if u == target:
                    return d
                if d > self.max_step:
                    return UP_LIMIT
        return UP_LIMIT
    def get(self, target: tuple) -> int:
        if not isRoad(target, self.grid):
            return UP_LIMIT
        return self.getDist(target, True)

    def calKey(self, inp: tuple, target: tuple):
        return abs(inp[0]-target[0])+abs(inp[1]-target[1])

    def createPath(self, pos_to: tuple):
        '''
        输入：i，j为坐标，从该点开始向下找局部最小点
        输出：path，path_pos两个列表，顺序为从局部最小点至i,j点的前一点。
        注意使用path时应该直接从末尾取值。
        '''
        for pos in findAdjacentRoad(pos_to, self.grid):
            if self.dist_table[pos] < self.dist_table[pos_to]:
                path_pos = self.createPath(pos)
                path_pos.append(pos)
                return path_pos
        return []

class OffsetTabel:
    def __init__(self, table:np.ndarray, offset:tuple, goal:tuple, grid_map:np.ndarray):
        self.table = table
        self.grid = grid_map
        self.offset = offset
        relative_goal = (goal[0] - offset[0], goal[1] - offset[1])
        self.Q = []
        self.limit_x = (offset[0], offset[0]+self.table.shape[0])
        self.limit_y = (offset[1], offset[1]+self.table.shape[1])
        for i in range(4):
            self.table[relative_goal[0], relative_goal[1], i] = 0
            self.Q.append((relative_goal[0], relative_goal[1], i))
    def get(self, target:tuple, dir:int):
        relative = (target[0] - self.offset[0], target[1] - self.offset[1])\
        #如果表内无法索引，则输出最大值。
        if relative[0]+1>self.table.shape[0] or relative[0]<0 or relative[1]+1>self.table.shape[1] or relative[1]<0:
            return UP_LIMIT
        value_judge = self.table[relative[0], relative[1], dir]
        if value_judge<UP_LIMIT:
            return value_judge

        while len(self.Q):
            u = self.Q.pop(0)
            d = self.searchPoint(u, self.Q)
            if u == value_judge:
                return d
        return UP_LIMIT
    def preGet(self):
        if len(self.Q):
            u = self.Q.pop(0)
            self.searchPoint(u, self.Q)
    def searchPoint(self, u:tuple, temp_Q:list):
        d = int(self.table[u])
        for v in findAdjacentCoreWay(u, self.limit_x, self.limit_y, self.grid, self.offset):
            if d + 1 < self.table[v]:
                    self.table[v] = d + 1
                    temp_Q.append(v)
        return d

class DistTable:#单用table，海路或陆路
    def __init__(self, grid:np.ndarray, road:bool=True, table:np.ndarray = None, batch_num = 5, max_step:int = 5000, \
        road_way_list:list = None, road_way_table:np.ndarray = None, road_way_offset:list = None):
        self.max_step = max_step
        self.grid = grid
        self.work = False
        self.ready = False
        self.CH_TO_NUM = CH_TO_NUM
        self.batch_num = batch_num
        self.goal = None
        self.road = road
        self.append_table = False
        self.to_tranp_point = False
        self.offset_table = None
        
        self.road_way_list = road_way_list
        self.road_way_table = road_way_table
        self.road_way_offset = road_way_offset

        self.table = table

    def terrainJudge(self, target):
        if self.road:
            return isRoad(target, self.grid)
        else:
            return isWay(target, self.grid)
    #这里的road只是为了兼容的占位变量。
    def get(self, target: tuple, regardless_work: bool = False, road: bool = True, dir:int=None) -> int:#原get函数
        # check valid input
        if not self.terrainJudge(target):
            logging.error('terrainJudge'+str(self.terrainJudge(target)))
            return UP_LIMIT

        if not regardless_work:
            if not self.work:
                return 0
            if not self.ready:
                return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])

        # if self.to_tranp_point and self.offset_table:
        #     core_pos = getCorePos(target, dir)
        #     value_judge = self.offset_table.get(core_pos, dir)
        #     return value_judge
        # distance has been known
        value_judge = self.getValue(target)
        if value_judge < UP_LIMIT:
            return value_judge

        # BFS with lazy evaluation
        while len(self.Q) > 0:
            self.Q.sort(key=lambda u: self.calKey(u, target))
            if len(self.Q)>self.batch_num:
                u_list = [self.Q.pop(0) for _ in range(self.batch_num)]
            else:
                u_list = [self.Q.pop(0) for _ in range(len(self.Q))]
            #u = self.Q.pop(0)
            for u in u_list:
                d = self.searchPoint(u, self.Q)
                if u == target:
                    return d
                if d > self.max_step:
                    return UP_LIMIT
        return UP_LIMIT

    def preGet(self, target: tuple = None) -> bool:
        # check valid input
        if target and (self.table[target] < UP_LIMIT):
            return True

        # BFS with lazy evaluation
        if len(self.Q) > 0:
            if target:
                self.Q.sort(key=lambda u: self.calKey(u, target))
            if len(self.Q)>self.batch_num:
                u_list = [self.Q.pop(0) for _ in range(self.batch_num)]
            else:
                u_list = [self.Q.pop(0) for _ in range(len(self.Q))]
            # u = self.Q.pop(0)
            #if len(self.Q) > 20:
            for u in u_list:
                d = self.searchPoint(u, self.Q)

                if u == target:
                    return True
                if d > self.max_step:
                    return True
        else:
            return True
        
        return False
    def searchPoint(self, u:tuple, temp_Q:list):
        d = int(self.table[u])
        if self.append_table:
            if isRoadWay(u, self.grid) and not isBerth(u, self.grid):
                road_way_id = self.road_way_table[u]
                road_way_offset = self.road_way_offset[road_way_id]
                relative_pos = (u[0]-road_way_offset[0], u[1]-road_way_offset[1])
                road_way_local_table = self.road_way_tag_table_list[road_way_id]
                d = road_way_local_table[relative_pos]
            else:
                d = int(self.dist_table[u])
            
            for v in findAdjacentWay(u, self.grid):
                length = 1
                if isMainWay(v, self.grid):#如果目标点是主航道，则运行距离增量为2
                    length = 2
                if isRoadWay(v, self.grid) and not isBerth(v, self.grid):#通过转移矩阵计算
                    road_way_id = self.road_way_table[v]
                    road_way_offset = self.road_way_offset[road_way_id]
                    relative_pos = (v[0]-road_way_offset[0], v[1]-road_way_offset[1])
                    road_way_local_table = self.road_way_tag_table_list[road_way_id]
                    road_way_local_tag = self.road_way_tag_list[road_way_id]
                    if d + length < road_way_local_table[relative_pos]:
                        road_way_local_table[relative_pos] = d + length
                        road_way_local_tag[relative_pos] = tag
                        temp_Q.append(v)
                else:
                    if d + length < self.dist_table[v]:#如果v处的值比u处值+1还大。可以用于多目标（目标区域）情况。
                        self.dist_table[v] = d + length
                        self.tag_table[v] = tag
                        temp_Q.append(v)
        elif self.road:
            for v in findAdjacentRoad(u, self.grid):
                if d + 1 < self.table[v]:#如果v处的值比u处值+1还大。可以用于多目标（目标区域）情况。
                    self.table[v] = d + 1
                    temp_Q.append(v)
            return d
        else:
            for v in findAdjacentWay(u, self.grid):
                if isMainWay(u, self.grid):
                    length = 2
                else:
                    length = 1
                if d + length < self.table[v]:#如果v处的值比u处值+1还大。可以用于多目标（目标区域）情况。
                    self.table[v] = d + length
                    temp_Q.append(v)
            return d
    
    def getValue(self, target: tuple, guess = False):
        if self.terrainJudge(target) and self.table[target] < UP_LIMIT:
            return self.table[target]

        if not self.append_table:
            if self.terrainJudge(target) and self.table[target] < UP_LIMIT:
                return self.table[target]
            if guess:
                return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])
            return UP_LIMIT
        if isRoadWay(target, self.grid):
            road_way_id = self.road_way_table[target]
            road_way_offset = self.road_way_offset[road_way_id]
            relative_pos = (target[0]-road_way_offset[0], target[1]-road_way_offset[1])
            road_way_local_table = self.road_way_list[road_way_id]
            if road_way_local_table[relative_pos] < UP_LIMIT:
                return road_way_local_table[relative_pos]
            if guess:
                return abs(self.goal[0]-target[0])+abs(self.goal[1]-target[1])
            return UP_LIMIT
        if isWay(target, self.grid) and self.table[target] < UP_LIMIT:
            return self.table[target]
        return UP_LIMIT

    def calKey(self, inp: tuple, target: tuple):
        return abs(inp[0]-target[0])+abs(inp[1]-target[1])
    
    def setGoal(self, goal, Q=None):
        if type(goal) == tuple:
            self.goal = goal
            self.table[goal] = 0
            if Q:
                self.Q = Q
            else:
                self.Q = [goal]
        else:
            self.goal = goal[0]
            for item in goal:
                self.table[item] = 0
            if Q:
                self.Q = Q
            else:
                self.Q = goal.copy()
    def setTable(self, table:np.ndarray):
        self.table = table
    def setAppendTable(self, table:np.ndarray, road_way_list:list = None, road_way_table:np.ndarray = None, road_way_offset:list = None):
        self.table = table
        self.road_way_list = road_way_list
        self.road_way_offset = road_way_offset
        self.road_way_table = road_way_table
    def setOffsetTable(self, offset_table:OffsetTabel):
        self.offset_table = offset_table

    def setWork(self, b:bool):
        self.work = b
    def setReady(self, b:bool):
        self.ready = b
    def setMaxStep(self, max_step:int):
        self.max_step = max_step

class PIBT:
    def __init__(self, grid:np.ndarray, seed: int = 0):

        self.grid = grid
        self.robot_N = 0
        self.boat_N = 0

        # distance table
        self.robot_dist_tables:List[DistTable] = []
        self.boat_dist_tables:List[DistTable] = []

        # cache
        self.NIL = -1#self.robot_N  #代理数
        self.NIL_COORD: tuple = self.grid.shape  #地图点数
        self.occupied_now = np.full(grid.shape, self.NIL, dtype=np.int8)#占据位置。self.NIL
        self.occupied_nxt = np.full(grid.shape, self.NIL, dtype=np.int8)
        self.occupied_aft = np.full(grid.shape, self.NIL, dtype=np.int8)

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
            if j != self.NIL and Q_to[j] == Q_from[i] and (not isMainRoad(Q_to[j], self.grid)):
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
    def boatFuncPIBT(self, Q_from: list, attitudes:list, restore:list, Q_to: Config, i: int, priorities: list) -> bool:
        #对于同一艘船，找四个点里距离最大的那个点，并且会有一步的遗留效应，也就是说只要进入了，那么就会把效应遗留一步。这是由
        #具体的特性决定的。但是这样的误差大概是能够忍受的。同时应当注意恢复期的船只会阻挡未处于恢复期的船只？因此预测两步也是很有效的。
        if restore[i] == -1:#如果机器人处于恢复状态且不确定恢复所需时间
            C_2 = [(Q_from[i], Q_from[i], attitudes[i], attitudes[i])]
        else:
            attitudes_1 = [attitudes[i]]
            C = [Q_from[i]]
            if restore[i] == 0:#如果未处在恢复状态
                Q_from_1, temp_attitudes_1 = findAdjacentWayByAttitude(Q_from[i], attitudes[i], self.grid)#找到可行的行动域
                attitudes_1.extend(temp_attitudes_1)
                C.extend(Q_from_1)
            
            assert len(C) == len(attitudes_1)
            
            C_2 = []
            for from_pos, from_attitude in zip(C, attitudes_1):
                C_2.append([from_pos, from_pos, from_attitude, from_attitude])#加入原位
                if restore[i] == 0 and self.entersMainWay(from_pos):#如果开始未处于恢复状态，但是第一步船进入了主航道，则第二步只能待在原地
                    continue
                Q_to_1, temp_attitudes_2 = findAdjacentWayByAttitude(from_pos, from_attitude, self.grid)#找到第二步可行的行动域
                for to_pos, to_attitude in zip(Q_to_1, temp_attitudes_2):
                    C_2.append([from_pos, to_pos, from_attitude, to_attitude])#存储两步内容#未完成！！考虑船处于恢复状态时的情况！！
        
            self.rng.shuffle(C_2)  # tie-breaking, randomize
            #排列方式，以四个点的距离之和为代价。是否应该以方块中最大的那个为准？或者还是以核心点为准？毕竟已经预测两步了？
            C_2 = sorted(C_2, key=lambda u: self.calKeys(u, i))#计算代价,仅以第二个未来步计算（受六个点中最大的那个限制吗？是否应该选择两步内的adjacent判断？
            # vertex assignment
        for v in C_2:#从排列的行动中逐次找
            v_1 = v[0]#第一个未来步(左上角)
            v_2 = v[1]#第二个未来步（左上角）
            att_1 = v[2]#第一个未来姿态
            att_2 = v[3]#第二个未来姿态
            # avoid vertex collision
            if detectBoatCollision(v_1, att_1, self.occupied_nxt) or detectBoatCollision(v_2, att_2, self.occupied_aft):#如果下一步或者再下一步被占据了，就继续。
                continue
            #找到要警告的那些点
            agent_list = []
            temp_pos_1, temp_pos_2 = getNewVolume(v_1, att_1)
            agent_list.append(self.occupied_now[temp_pos_1])#占据当前点的船j。
            agent_list.append(self.occupied_now[temp_pos_2])#占据当前点的船k。
            temp_pos_1, temp_pos_2 = getNewVolume(v_2, att_2)#只要是可能运动到temp_pos_1和temp_pos_2的，都得受牵连！！！
            agent_list.append(self.occupied_nxt[temp_pos_1])#占据下一点的船p。
            agent_list.append(self.occupied_nxt[temp_pos_2])#占据下一点的船q。会导致反向推演！！！
            
            agent_list.append(self.occupied_now[temp_pos_1])#
            agent_list.append(self.occupied_now[temp_pos_2])#
            agent_list = list(set(agent_list))
            
            # plan_list = []
            # for agent in agent_list:
            #     if agent != self.NIL and (agent not in plan_list):
            #         plan_list.append(agent)
            if len(agent_list)>=2:
                agent_list.sort(key=lambda agent: priorities[agent], reverse=True)
            #这一步意在使j让出位置。但是不仅是这一步，下一步也可能需要某条船让出位置？先这样试试吧。
            # avoid edge collision
            # if j != self.NIL and Q_to[j] == Q_from[i]:#船的情况下会有边冲突吗？貌似不会有。但如果要预测两步的边冲突，则
            #     continue
            # reserve next location
            Q_to[i] = v_1#设置下一步位置
            
            setOccupiedByAttitude(i, v_1, att_1, self.occupied_nxt, self.grid)#设置第一步后的占据位置
            setOccupiedByAttitude(i, v_2, att_2, self.occupied_aft, self.grid)#设置第二步后的占据位置
            # priority inheritance (j != i due to the second condition)
            flag = False
            occupied_point_list = []
            for agent in agent_list:
                if (
                    agent != self.NIL
                    and (Q_to[agent] == self.NIL_COORD)#如果v处当前有船j，且j还未规划，则规划j，如果j规划失败，则失败。
                ):#只要有一个规划不成功，就擦除，然后走人。但是只要有一个规划成功了就会造成Occupied矩阵不可逆的影响？！！！！！
                    #记录当前临时规划成功的船占据的位置
                    judgement = self.boatFuncPIBT(Q_from, attitudes, restore, Q_to, agent, priorities)
                    if judgement:
                        occupied_point_list.append(judgement)
                        continue
                    #应当同时搜索规划成功者相应的点并进行擦除？或者返回成功占据的位置？
                    #清除被占据的位置!
                    setOccupiedByAttitude(self.NIL, v_1, att_1, self.occupied_nxt, self.grid)#清除第一步后的占据位置
                    setOccupiedByAttitude(self.NIL, v_2, att_2, self.occupied_aft, self.grid)#清除第二步后的占据位置
                    #清除被已规划成功的船占据的位置————
                    for two_step_status in occupied_point_list:
                        setOccupiedByAttitude(self.NIL, two_step_status[0][0], two_step_status[0][1], self.occupied_nxt, self.grid)
                        setOccupiedByAttitude(self.NIL, two_step_status[1][0], two_step_status[1][1], self.occupied_aft, self.grid)
                    flag = True
                    break
            if flag:
                continue
            return [(v_1, att_1), (v_2, att_2)]

        # failed to secure node
        Q_to[i] = self.NIL_COORD
        # setOccupiedByAttitude(i, Q_from[i], attitudes[i], self.occupied_nxt, self.grid)#设置第一步后的占据位置
        # setOccupiedByAttitude(i, Q_from[i], attitudes[i], self.occupied_aft, self.grid)#设置第二步后的占据位置
        return False#[(Q_from[i], attitudes[i]), (Q_from[i], attitudes[i])]
    def calKeys(self, C_2, i):#方块计算
        dist_table:DistTable = self.boat_dist_tables[i]
        if dist_table.to_tranp_point:
            return dist_table.offset_table.get(getCorePos(C_2[0], C_2[2]), C_2[2])
        x = C_2[1][0]
        y = C_2[1][1]
        return min(dist_table.get((x, y), road=False)+dist_table.get((x+1, y), road=False)+dist_table.get((x, y+1), road=False)+dist_table.get((x+1, y+1), road=False), UP_LIMIT)
        
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
    def boatStep(self, Q_from: Config, attitudes:list, restore:list, priorities: list) -> Config:#[float]) -> Config:
        # setup
        N = len(Q_from)
        Q_to: Config = []
        logging.error('set_occupied')
        for i, v in enumerate(Q_from):
            Q_to.append(self.NIL_COORD)
            setOccupiedByAttitude(i, v, attitudes[i], self.occupied_now, self.grid)
            
        # perform PIBT
        logging.error('pibt')
        A = sorted(list(range(N)), key=lambda i: priorities[i], reverse=True)#按距离排序
        logging.error(A)
        for i in A:
            if Q_to[i] == self.NIL_COORD:
                self.boatFuncPIBT(Q_from, attitudes, restore, Q_to, i, priorities)
                
        logging.error('erase')
        # cleanup 待改进擦除方式！按行动的最大范围擦除，三格之内？也就是7*7的范围擦除？需要判断左上和右下点有没有超过范围
        self.eraseOccupiedTable(Q_from, self.occupied_now, self.occupied_nxt, self.occupied_aft)

        return Q_to

    def runRobotStep(self, starts, is_work):# -> Configs:#问题在于，不仅要到达目标，还要停留一刻。
        #全部机器人都没有work也会走路？？？
        # define priorities
        ag_num = len(starts)
        if not ag_num:
            return [], []
        
        if ag_num > self.robot_N:
            for _ in range(ag_num-self.robot_N):
                self.robot_dist_tables.append(DistTable(self.grid))
            self.robot_N = ag_num
        
        priorities: List[float] = []
        if not any(is_work):
            return starts
        for i in range(self.robot_N):
            if is_work[i]:
                self.robot_dist_tables[i].setWork(True)
                dist = self.robot_dist_tables[i].get(starts[i])
                priorities.append(dist / UP_LIMIT + i*0.05)#获取距离，计算优先级。注意该策略：距离长的优先级高。也可以设置距离短的优先级高。
            else:
                priorities.append(0+i*0.01)#优先级最低
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
        return Q#机器人下一步的位置, 以及永远无法达到目标点的机器人
    def runBoatStep(self, starts, attitudes, restore, is_work):# -> Configs:#问题在于，不仅要到达目标，还要停留一刻。
        ag_num = len(starts)
        if not ag_num:
            return []
        
        if ag_num > self.boat_N:
            for _ in range(ag_num-self.boat_N):
                self.boat_dist_tables.append(DistTable(self.grid, road=False))
            self.boat_N = ag_num
        #全部机器人都没有work也会走路？？？
        # define priorities
        priorities: List[float] = []
        for i in range(self.boat_N):
            if is_work[i]:
                self.boat_dist_tables[i].setWork(True)
                dist = self.boat_dist_tables[i].get(starts[i], road = False)
                priorities.append(dist / UP_LIMIT + i*0.05)#获取距离，计算优先级。注意该策略：距离长的优先级高。也可以设置距离短的优先级高。
            else:
                priorities.append(0+i*0.01)#优先级最低
                self.boat_dist_tables[i].setWork(False)
        #logging.error(priorities)
        #writeFile('priorities:'+str(priorities))
        #主循环，生成机器人下一步的行动
        #configs = [self.starts]#机器人当前的位置
        logging.error('boat_step')
        Q = self.boatStep(starts, attitudes, restore, priorities)#Q为机器人下一步该走的位置

        return Q

    def setRobotDistTable(self, i:int, dist_table:np.ndarray, ready:bool, work, goal = None, max_step = 5000, \
        Q = None):
        if i+1 > self.robot_N:
            for _ in range(i+1-self.robot_N):
                self.robot_dist_tables.append(DistTable(self.grid))
            self.robot_N = i+1
        present_dist_table = self.robot_dist_tables[i]
        present_dist_table.setTable(dist_table)
        present_dist_table.setReady(ready)
        present_dist_table.setWork(work)
        present_dist_table.setGoal(goal, Q)
        present_dist_table.setMaxStep(max_step)
    def setBoatDistTable(self, i:int, p_table:DoubleDistTable, ready, work, max_step = 5000):
        '''
        注意运输点设置append为False
        '''
        if i+1 > self.boat_N:
            for _ in range(i+1-self.boat_N):
                self.boat_dist_tables.append(DistTable(self.grid, road=False))
            self.boat_N = i+1
        present_dist_table = self.boat_dist_tables[i]
        present_dist_table.setAppendTable(p_table.table, p_table.road_way_list, p_table.road_way_table, p_table.road_way_offset)
        present_dist_table.setReady(ready)
        present_dist_table.setWork(work)
        present_dist_table.setMaxStep(max_step)
        if type(p_table) == DoubleDistTable:
            present_dist_table.setGoal(p_table.goal, p_table.Q_way)
            present_dist_table.append_table = False
        if type(p_table) == DistTable:
            present_dist_table.setGoal(p_table.goal, p_table.Q)
            present_dist_table.setOffsetTable(p_table.offset_table)
            present_dist_table.append_table = True
            

    def eraseOccupiedTable(self, Q_from: list, table1, table2, table3):
        for pos in Q_from:
            left_up_x, right_down_x = pos[0]-3, pos[0]+3
            left_up_y, right_down_y = pos[1]-3, pos[1]+3
            
            if left_up_x <0:
                left_up_x = 0
            if right_down_x >= self.grid.shape[0]:
                right_down_x = self.grid.shape[0]
            if left_up_y <0:
                left_up_y = 0
            if right_down_y >= self.grid.shape[1]:
                right_down_y = self.grid.shape[1]
            

            table1[left_up_x:(right_down_x+1), left_up_y:(right_down_y+1)] = self.NIL
            table2[left_up_x:(right_down_x+1), left_up_y:(right_down_y+1)] = self.NIL
            table3[left_up_x:(right_down_x+1), left_up_y:(right_down_y+1)] = self.NIL

    def entersMainWay(self, u):#方块判别
        x = u[0]
        y = u[1]
        if isMainWay(u, self.grid) or isMainWay((x+1, y), self.grid) or \
            isMainWay((x, y+1), self.grid) or isMainWay((x+1, y+1), self.grid):
            return True
        return False

#船的碰撞table操作函数
def detectBoatCollision(new_pos, new_attitude, table):#注意，如果是core_pos，则为核心点，如果为pos，则为方块左上点
    for pos in getVolume(new_pos, new_attitude):
        if table[pos] >= 0:
            return True
    return False

def setOccupiedByAttitude(boat_id:int, pos:tuple, attitude:int, table:np.ndarray, grid_map:np.ndarray):#2*3
    '''
    会把船未在主航道部分填入table中
    '''
    volume = []
    if attitude == 0:
        for i in range(pos[0], pos[0]+2):
            for j in range(pos[1]-1,pos[1]+2):
                if not isMainWay((i,j), grid_map):
                    table[i,j] = boat_id
                    volume.append((i,j))
    elif attitude == 1:
        for i in range(pos[0], pos[0]+2):
            for j in range(pos[1],pos[1]+3):
                if not isMainWay((i,j), grid_map):
                    table[i,j] = boat_id
                    volume.append((i,j))
    elif attitude == 2:
        for i in range(pos[0], pos[0]+3):
            for j in range(pos[1],pos[1]+2):
                if not isMainWay((i,j), grid_map):
                    table[i,j] = boat_id
                    volume.append((i,j))
    elif attitude == 3:
        for i in range(pos[0]-1, pos[0]+2):
            for j in range(pos[1],pos[1]+2):
                if not isMainWay((i,j), grid_map):
                    table[i,j] = boat_id
                    volume.append((i,j))
    return volume

#船的体积操作函数
def getVolume(pos:tuple, attitude:int):
    '''
    获取船舶的全部位置
    '''
    volume = [(pos[0], pos[1]), (pos[0]+1, pos[1]), (pos[0], pos[1]+1), (pos[0]+1, pos[1]+1)]
    if attitude == 1:
        volume.extend([(pos[0], pos[1]+2), (pos[0]+1, pos[1]+2)])
        return volume
    elif attitude == 0:
        volume.extend([(pos[0], pos[1]-1), (pos[0]+1, pos[1]-1)])
        return volume
    elif attitude == 2:
        volume.extend([(pos[0]+2, pos[1]), (pos[0]+2, pos[1]+1)])
        return volume
    elif attitude == 3:
        volume.extend([(pos[0]-1, pos[1]), (pos[0]-1, pos[1]+1)])
        return volume

def getBackVolume(pos:tuple, attitude:int):
    '''
    获取船舶除了前面的方块之外占据的位置，
    也即船舶的尾部位置
    '''
    if attitude == 0:
        return [(pos[0], pos[1]+2), (pos[0]+1, pos[1]+2)]
    elif attitude == 1:
        return [(pos[0], pos[1]-1), (pos[0]+1, pos[1]-1)]
    elif attitude == 2:
        return [(pos[0]+2, pos[1]), (pos[0]+2, pos[1]+1)]
    elif attitude == 3:
        return [(pos[0]-1, pos[1]), (pos[0]-1, pos[1]+1)]

def getNewVolume(new_pos:tuple, new_attitude:int):
    '''
    获取船舶新的状态下其新占据的位置
    '''
    if new_attitude == 0:
        return [(new_pos[0], new_pos[1]+1), (new_pos[0]+1, new_pos[1]+1)]
    elif new_attitude == 1:
        return [(new_pos[0], new_pos[1]), (new_pos[0]+1, new_pos[1])]
    elif new_attitude == 2:
        return [(new_pos[0], new_pos[1]), (new_pos[0], new_pos[1]+1)]
    elif new_attitude == 3:
        return [(new_pos[0]+1, new_pos[1]), (new_pos[0]+1, new_pos[1]+1)]

def judgeNewVolumePos(pos:tuple, attitude:int, go_attitude:int, grid_map:np.ndarray):
    '''
    判断船下一步是否与障碍物或边界相撞，如果相撞返回False，未相撞返回True
    '''
    if attitude != 0 and go_attitude == 1:#如果船向着1方向走且船姿态不为0
        if pos[1]-1>=0 and isWay((pos[0], pos[1]-1), grid_map) and isWay((pos[0]+1, pos[1]-1), grid_map):
            return True
        return False
    elif attitude != 1 and go_attitude == 0:
        if pos[1]+2<grid_map.shape[1] and isWay((pos[0], pos[1]+2), grid_map) and isWay((pos[0]+1, pos[1]+2), grid_map):
            return True
        return False
    elif attitude != 2 and go_attitude == 3:
        if pos[0]+2<grid_map.shape[0] and  isWay((pos[0]+2, pos[1]), grid_map) and isWay((pos[0]+2, pos[1]+1), grid_map):
            return True
        return False
    elif attitude != 3 and go_attitude == 2:
        if pos[0]-1>=0 and isWay((pos[0]-1, pos[1]), grid_map) and isWay((pos[0]-1, pos[1]+1), grid_map):
            return True
        return False
    return False

def getBoxLeftUpPos(core_pos:tuple, attitude:int) -> tuple:
    if attitude == 0:
        return (core_pos[0], core_pos[1]+1)
    elif attitude == 1:
        return (core_pos[0]-1, core_pos[1]-2)
    elif attitude == 2:
        return (core_pos[0]-2, core_pos[1])
    elif attitude == 3:
        return (core_pos[0]+1, core_pos[1]-1)

def getCorePos(box_left_up_pos:tuple, attitude:int) -> tuple:
    if attitude == 0:
        return (box_left_up_pos[0], box_left_up_pos[1]-1)
    elif attitude == 1:
        return (box_left_up_pos[0]+1, box_left_up_pos[1]+2)
    elif attitude == 2:
        return (box_left_up_pos[0]+2, box_left_up_pos[1])
    elif attitude == 3:
        return (box_left_up_pos[0]-1, box_left_up_pos[1]+1)
    
def rotateAnticlockwise(attitude):
    '''
    返回：方向tuple，new_attitude
    '''
    if attitude == 0:
        return (-1, 0), 2
    if attitude == 1:
        return (1, 0), 3
    if attitude == 2:
        return (0, -1), 1
    if attitude == 3:
        return (0, 1), 0

def rotateClockwise(attitude):
    '''
    返回：方向tuple，new_attitude
    '''
    if attitude == 0:
        return (1, 0), 3
    if attitude == 1:
        return (-1, 0), 2
    if attitude == 2:
        return (0, 1), 0
    if attitude == 3:
        return (0, -1), 1

#寻找相邻块函数
def findAdjacentRoad(pos:tuple, grid_map:np.ndarray):
    x = pos[0]
    y = pos[1]
    out = []
    if y+1<grid_map.shape[1] and isRoad((x,y+1), grid_map):
        out.append((x,y+1))
    if y-1>=0 and isRoad((x,y-1), grid_map):
        out.append((x,y-1))
    if x-1>=0 and isRoad((x-1,y), grid_map):
        out.append((x-1,y))
    if x+1<grid_map.shape[0] and isRoad((x+1,y), grid_map):
        out.append((x+1,y))
    return out

def findAdjacentWay(pos:tuple, grid_map:np.ndarray):
    x = pos[0]
    y = pos[1]
    out = []
    if y+1<grid_map.shape[1] and isWay((x,y+1), grid_map):
        out.append((x,y+1))
    if y-1>=0 and isWay((x,y-1), grid_map):
        out.append((x,y-1))
    if x-1>=0 and isWay((x-1,y), grid_map):
        out.append((x-1,y))
    if x+1<grid_map.shape[0] and isWay((x+1,y), grid_map):
        out.append((x+1,y))
    return out

def findAdjacentCoreWay(axis:tuple, limit_x:tuple, limit_y:tuple, grid_map:np.ndarray, offset:tuple=None):
    '''
    limit左闭右开
    '''
    if offset:
        x = axis[0]+offset[0]
        y = axis[1]+offset[1]
    else:
        x = axis[0]
        y = axis[1]
    z = axis[2]
    move = DIR_TO_MOVE[z]
    out = []
    new_x = x-move[0]
    new_y = y-move[1]
    if limit_x[0]<=new_x<limit_x[1] and limit_y[0]<=new_y<limit_y[1] and\
        isWay((new_x,new_y), grid_map):
        if offset:
            new_x = new_x - offset[0]
            new_y = new_y - offset[1]
        out.append((new_x,new_y,z))

    move_2, new_z = rotateAnticlockwise(z)
    new_x = x-move[0]*2
    new_y = y-move[1]*2
    if limit_x[0]<=new_x<limit_x[1] and limit_y[0]<=new_y<limit_y[1] and\
        isWay((new_x,new_y), grid_map):
        if offset:
            new_x = new_x - offset[0]
            new_y = new_y - offset[1]
        out.append((new_x,new_y,new_z))
        
    move_2, new_z = rotateClockwise(z)
    new_x = x-move[0]+move_2[0]
    new_y = y-move[1]+move_2[1]
    if limit_x[0]<=new_x<limit_x[1] and limit_y[0]<=new_y<limit_y[1] and\
        isWay((new_x,new_y), grid_map):
        if offset:
            new_x = new_x - offset[0]
            new_y = new_y - offset[1]
        out.append((new_x,new_y,new_z))
        
    return out

def findAdjacentWayByAttitude(pos:tuple, attitude:int, grid_map:np.ndarray):#获船舶的行动算子.注意越界的判断会有所不同。
    x = pos[0]
    y = pos[1]
    out_attitudes = []
    out_pos = []
    for go_attitude in range(4):
        if judgeNewVolumePos(pos, attitude, go_attitude, grid_map):
            pos_offset = DIR_TO_MOVE[go_attitude]
            out_attitudes.append(go_attitude)
            out_pos.append((x+pos_offset[0], y+pos_offset[1]))
    # if y+1<grid_map.shape[1] and isWay((x,y+1), grid_map) and attitude != 1:
    #     out.append((x,y+1))
    # if y-1>=0 and isWay((x,y-1), grid_map) and attitude != 0:
    #     out.append((x,y-1))
    # if x-1>=0 and isWay((x-1,y), grid_map) and attitude != 3:
    #     out.append((x-1,y))
    # if x+1<grid_map.shape[0] and isWay((x+1,y), grid_map) and attitude != 2:
    #     out.append((x+1,y))
    return out_pos, out_attitudes

def findAdjacentRoadWay(pos:tuple, grid_map:np.ndarray):
    x = pos[0]
    y = pos[1]
    out = []
    if y+1<grid_map.shape[1] and isRoadWayButNotBerth((x,y+1), grid_map):
        out.append((x,y+1))
    if y-1>=0 and isRoadWayButNotBerth((x,y-1), grid_map):
        out.append((x,y-1))
    if x-1>=0 and isRoadWayButNotBerth((x-1,y), grid_map):
        out.append((x-1,y))
    if x+1<grid_map.shape[0] and isRoadWayButNotBerth((x+1,y), grid_map):
        out.append((x+1,y))
    return out

def findAdjacentBerth(pos:tuple, grid_map:np.ndarray):
    x = pos[0]
    y = pos[1]
    out = []
    if y+1<grid_map.shape[1] and isBerth((x,y+1), grid_map):
        out.append((x,y+1))
    if y-1>=0 and isBerth((x,y-1), grid_map):
        out.append((x,y-1))
    if x-1>=0 and isBerth((x-1,y), grid_map):
        out.append((x-1,y))
    if x+1<grid_map.shape[0] and isBerth((x+1,y), grid_map):
        out.append((x+1,y))
    return out


