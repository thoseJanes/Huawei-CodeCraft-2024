import numpy as np
import time
from typing import Dict, Tuple
from object import *
from pibt import *
import sys

#未完成：
#机器人工位策略（流入价值多的工位是好工位，可以看看近500帧的流入价值）
#机器人及船舶购买策略（机器人近500帧最低获取速率、船舶上一趟最小f_val值、当前泊口平均最大堆积货物数）
#船舶改进策略：估计泊口预计到达系数（譬如以机器人所在tag）、价值激励（每2000激励回航，每8000激励回航）
finally_buy_frame = 13000
robot_limit = 15
boat_limit = 2
robot_boat = 5
robot_price, boat_price, up_price = 2000, 8000, 30000
#import logging
test_mode = True
log_text_judge = True
log_text = []
max_display_lines = 500
purchase_interval = 20#购买行为间隔帧
go_zhen = 14998
collect_zhen = 12000
berth_dist_factor = 500
max_stop_time = 3#当停留多少步时给其分派别的工作
berth_good_factor = 1.2#对有船或船将要来泊口的货物价值计算进行一定增幅。
head_goods = 7#用于控制每次船舶取前几个货物留在每层.

good_stack_frame_range = (300, 400)#一次性压入机器人的货物长度。没有必要，直接从机器人货物集中取。

map_type = 0#0表明为平均图。
MAP_TYPE_DICT = {'Average map':0}
def getBackFactor(goods_value, frame):#用于调控船的回航时间
    if goods_value>=2000:
        if frame<300:
            return 0.7
        return 0.9
    if goods_value>=8000:
        if frame<300:
            return 0.5
        return 0.7
    return 1
def getBerthGoodFactor(boat_num):#用于调控有船泊口的机器人偏好
    if boat_num:
        return berth_good_factor
    return 1
#初始化常量和对象
if True:
    N = 200
    money = 25000
    frame_id = 0
    
    #初始化泊位
    #berths_pos = [() for _ in range(berth_num)]
    berth_num = 0#通过初始化时获取的数据确定泊口数量
    berths_list:List[Berth] = []
    #berths_dict:Dict[Tuple, Berth] = {}#从坐标到船舶

    #初始化机器人,由于数量不确定，完全使用列表
    robot_num = 0#机器人数量是购买得来的
    robots_list:List[Robot] = []
    robots_pos = []
    robots_goods = []
    robots_work = []
    
    #初始化货物
    goods_dict:Dict[Tuple, Good] = {}
    goods_berth_dist = []
    valid_time = 1000
    
    goods_unsc = []#原始未筛选的货物
    goods_pos = []#用于容纳未确定所属泊口的货物
    goods_left = []#用于容纳最终多余的货物
    #初始化船
    boat_capacity = 0#船容量
    boat_num = 0#船是购买得来的，一开始不确定数量
    boats_pos = []
    boats_dir = []
    boats_work = []
    boats_restore = []
    boats_list:List[Boat] = []
#初始化相关矩阵
if True:
    grid_map = np.zeros((N,N), np.int8)
    berths_dist_tables:List[DoubleDistTable] = []
    transp_dist_tables:List[DistTable] = []#用于记录运输点和其它位置的距离
    multi_tag_table = MultiDistTable(grid_map)#用于记录地块所属的泊口。是否需要multiTable？用于判断货物所处区块。
    #robot_berth_table = np.zeros((robot_num, berth_num), bool)#机器人和berth的数量都不能确定，因此也无法确定robot_berth_table

    #注意该矩阵只是用于索引标号的矩阵，如果用于判断地形只能识别包括berth在内的table。因为berth查询也放在该矩阵内。
    #该矩阵的使用需要严加勘察，一般情况下需要与grid_map结合使用。
    road_way_table = np.full((N,N), -1, np.int8)#用于记录区块标号，以及berth查询，只需一个
    road_way_offset = []#用于记录区块位置，只需一个
    road_way_lists = []#用于记录区块的海域距离，需要多个（与泊口图数相等的数量，一般是10个）
    
    road_way_tag_table_list = []#用于记录区块的海域距离，需要一个。
    road_way_tag_list = []#用于记录区块的海域tag，需要一个。

    #记录特殊位置
    robot_purchase_points:List[PurchasePoint] = []#用于记录机器人购买点的位置
    boat_purchase_points:List[PurchasePoint] = []#用于记录船购买点的位置
    delivery_point = []#用于记录运输点的位置
#初始化地图处理函数
if True:
    def process_map(grid, berth_num):
        road_way_id = berth_num
        for i in range(N):
            for j in range(N):
                pos = (i,j)
                if grid[i][j] == CH_TO_NUM['R']:
                    robot_purchase_points.append(PurchasePoint(pos, berth_num))
                elif grid[i][j] == CH_TO_NUM['S']:
                    boat_purchase_points.append(PurchasePoint(pos, berth_num))
                elif grid[i][j] == CH_TO_NUM['T']:
                    delivery_point.append(pos)
                    transp_dist_table = DistTable(grid_map, False, np.full((N,N), UP_LIMIT, dtype=np.int16))
                    transp_dist_table.setGoal((i, j))
                    left_up_x, right_down_x = max(i-3, 0), min(i+3,N-1)
                    left_up_y, right_down_y = max(j-3, 0), min(j+3,N-1)
                    offset_table = OffsetTabel(np.full((right_down_x-left_up_x+1, right_down_y-left_up_y+1, 4), UP_LIMIT, np.int8), (left_up_x, left_up_y), (i, j), grid_map)
                    transp_dist_table.setOffsetTable(offset_table)
                    transp_dist_tables.append(transp_dist_table)#增加表
                    
                elif isRoadWayButNotBerth(pos, grid) and road_way_table[pos]<0:#找到了一个新的区块
                    extendRoadWayArea(pos, road_way_id)
                    road_way_id += 1
    def extendRoadWayArea(pos, id):#扩展c的区域，记录数据并设置road_way_list
        #规划区域
        left_up_x, right_down_x = pos[0], pos[0]
        left_up_y, right_down_y = pos[1], pos[1]
        road_way_table[pos] = id
        temp_Q = [pos]
        while len(temp_Q) != 0:#找到所有相同区块的点，并记录在road_way_table中
            u = temp_Q.pop(0)
            for v in findAdjacentRoadWay(u, grid_map):
                if road_way_table[v] < 0:#如果还没有记录，则进行记录
                    temp_Q.append(v)
                    road_way_table[v] = id
                    if v[0]>right_down_x:
                        right_down_x = v[0]
                    elif v[0]<left_up_x:
                        left_up_x = v[0]
                    if v[1]>right_down_y:
                        right_down_y = v[1]
                    elif v[1]<left_up_y:
                        left_up_y = v[1]
        #建立相应的位置索引和模板矩阵
        road_way_offset.append((left_up_x, left_up_y))
        for road_way_list in road_way_lists:
            road_way_list.append(np.full((right_down_x-left_up_x+1, right_down_y-left_up_y+1), UP_LIMIT, np.int16))
        road_way_tag_list.append(np.full((right_down_x-left_up_x+1, right_down_y-left_up_y+1), UP_LIMIT, np.int8))
        road_way_tag_table_list.append(np.full((right_down_x-left_up_x+1, right_down_y-left_up_y+1), UP_LIMIT, np.int16))
    def extendBerthArea(pos, berth:Berth, id):
        '''
        将除pos外的其它相连berth点放入berth.side,并在road_way_table中记录
        '''
        #规划区域
        left_up_x, right_down_x = pos[0], pos[0]
        left_up_y, right_down_y = pos[1], pos[1]
        road_way_table[pos] = id
        temp_Q = [pos]
        while len(temp_Q) != 0:#找到所有相同区块的点，并记录在road_way_table中
            u = temp_Q.pop(0)
            for v in findAdjacentBerth(u, grid_map):
                if road_way_table[v] < 0:#如果还没有记录，则进行记录
                    temp_Q.append(v)
                    road_way_table[v] = id
                    berth.side.append(v)
                    if v[0]>right_down_x:
                        right_down_x = v[0]
                    elif v[0]<left_up_x:
                        left_up_x = v[0]
                    if v[1]>right_down_y:
                        right_down_y = v[1]
                    elif v[1]<left_up_y:
                        left_up_y = v[1]
        road_way_offset.append((left_up_x, left_up_y))
        for berth_id in range(len(road_way_lists)):
            road_way_list = road_way_lists[berth_id]
            if id == berth_id:
                road_way_list.append(np.full((right_down_x-left_up_x+1, right_down_y-left_up_y+1), 0, np.int16))#起始点
            else:
                road_way_list.append(np.full((right_down_x-left_up_x+1, right_down_y-left_up_y+1), UP_LIMIT, np.int16))
        road_way_tag_list.append(0)#只用来占位。因为搜不到别的泊口范围，所以不用。
        road_way_tag_table_list.append(0)
def logText(x):
    global log_text
    if log_text_judge:
        log_text.append(str(x))
if test_mode:#判题器函数
    from threading import Timer
    import win32gui
    import logging
    import matplotlib.pyplot as plt
    import traceback
    from qtWindow import QApplication, WindowManager, TableTips

    move_collision_table_now = np.full(grid_map.shape, -1, np.int8)#碰撞检测表
    move_collision_table_next = np.full(grid_map.shape, -1, np.int8)#碰撞检测表
    robot_collision = None
    boat_collision = None
    
    global_boats_restore = []#船回复时间
    berths_dir = []#停泊时方向
    berths_boat_pos = []#停泊时位置
    global_boat_last_action = []
    
    global_boats_goods:List[List[Good]] = []
    global_berths_goods:List[List[Good]] = []#泊口货物存储
    global_robots_goods:List[Good] = []#机器人当前手上的货物
    global_goods_dict:Dict[Tuple, Good] = {}
    
    global_frame = -1
    global_money = 25000
    
    def initGlobalBerthsGoods(berth_num):
        for _ in range(berth_num):
            global_berths_goods.append([])

    #碰撞检测
    def backstepRobotCollision(robot_id, robot_pos, robot_collision, grid_map):#回溯机器人碰撞
        robot_collision[robot_id] = True
        back_robot_id = move_collision_table_next[robot_pos]
        if back_robot_id>=0 and back_robot_id!=robot_id:
            backstepRobotCollision(back_robot_id, robots_pos[back_robot_id], robot_collision, grid_map)
        if not isMainRoad(robot_pos, grid_map):
            move_collision_table_next[robot_pos] = robot_id
        logText('robot '+str(robot_id)+' collide')
    def backstepBoatCollision(boat_id, boat_pos, boat_dir, boats_new_pos, boats_new_dir, boat_collision, grid_map):#回溯船的碰撞，注意这里的位置和姿态都是旧的
        boat_collision[boat_id] = True
        
        back_boat_pos_1, back_boat_pos_2 = getBackVolume(boat_pos, boat_dir)
        back_j, back_k = move_collision_table_next[back_boat_pos_1], move_collision_table_next[back_boat_pos_2]
        #由于船的占位比较多，因此需要先清除船当前占位。
        if back_j >= 0 and back_j!=boat_id and not boat_collision[boat_id]:
            setOccupiedByAttitude(-1, boats_new_pos[back_j], boats_new_dir[back_j], move_collision_table_next, grid_map)
            backstepBoatCollision(back_j, boats_pos[back_j], boats_dir[back_j], boats_new_pos, boats_new_dir, boat_collision, grid_map)
        if  back_k != back_j and back_k != boat_id and back_k>=0 and  not boat_collision[back_k]:
            setOccupiedByAttitude(-1, boats_new_pos[back_k], boats_new_dir[back_k], move_collision_table_next, grid_map)
            backstepBoatCollision(back_k, boats_pos[back_k], boats_dir[back_k], boats_new_pos, boats_new_dir, boat_collision, grid_map)
        #设置船占位为当前占位
        setOccupiedByAttitude(boat_id, boat_pos, boat_dir, move_collision_table_next, grid_map)
        logText('boat '+str(boat_id)+' collide')
    def getRobotCollisionCondition(robots_move_pos, robot_collision):#检测机器人是否碰撞
        for robot_id in range(robot_num):#检测机器人冲突
            if robot_collision[robot_id]:
                continue
            move_pos = robots_move_pos[robot_id]
            if robots_pos[robot_id] == move_pos:#如果机器人没有移动
                move_collision_table_next[robots_pos[robot_id]] = robot_id
                continue
            
            other_robot_id = move_collision_table_now[move_pos]#先检测边冲突，不然可能导致检测点冲突时无限回溯
            if other_robot_id >= 0 and other_robot_id!=robot_id and robots_move_pos[other_robot_id] == robots_pos[robot_id] and \
                (not isMainRoad(move_pos, grid_map)) and (not isMainRoad(robots_pos[robot_id], grid_map)):#存在边冲突
                logText('robot '+str(robot_id)+' collide with robot '+str(other_robot_id))
                robot_collision[robot_id] = True
                robot_collision[other_robot_id] = True
                if not isMainRoad(move_pos, grid_map):
                    move_collision_table_next[move_pos] = other_robot_id
                if not isMainRoad(robots_pos[robot_id], grid_map):
                    move_collision_table_next[robots_pos[robot_id]] = robot_id
                continue
            
            other_robot_id = move_collision_table_next[move_pos]
            if (other_robot_id >= 0 and other_robot_id!=robot_id) or (not isRoad(move_pos, grid_map)):#检测点冲突和地形冲突
                backstepRobotCollision(robot_id, robots_pos[robot_id], robot_collision, grid_map)#回溯检测点冲突
                if (other_robot_id >= 0 and other_robot_id!=robot_id) and robots_pos[other_robot_id] != move_pos:
                    move_collision_table_next[move_pos] = -1
                    backstepRobotCollision(other_robot_id, robots_pos[other_robot_id], robot_collision, grid_map)#回溯检测点冲突
                continue
            
            #如果没有冲突
            if not isMainRoad(move_pos, grid_map):
                move_collision_table_next[move_pos] = robot_id
        return robot_collision
    def getBoatCollisionCondition(boats_new_pos, boats_new_dir, boat_collision):#检测船是否碰撞.注意复杂性：由于船有体积，因此要先清除其上一步占位再进行回溯。
        for boat_id in range(boat_num):#检测船冲突
            if boat_collision[boat_id]:
                continue
            old_pos, new_pos = boats_pos[boat_id], boats_new_pos[boat_id]
            old_dir, new_dir= boats_dir[boat_id], boats_new_dir[boat_id]
            if old_pos == new_pos:#如果船舶没有移动
                volume = setOccupiedByAttitude(boat_id, old_pos, old_dir, move_collision_table_next, grid_map)#占位
                continue

            temp_pos_1, temp_pos_2 = getNewVolume(new_pos, new_dir)
            boat_j, boat_k = move_collision_table_next[temp_pos_1], move_collision_table_next[temp_pos_2]
            # print('temp_pos_1:{},temp_pos_2{}'.format(temp_pos_1, temp_pos_2))
            # print(boat_j, boat_k)
            print(judgeNewVolumePos(old_pos, old_dir, new_dir, grid_map))

            if (boat_j >= 0 and boat_j!=boat_id) or (boat_k>=0 and boat_k!=boat_id) or (not judgeNewVolumePos(old_pos, old_dir, new_dir, grid_map)):#检测点冲突和地形冲突
                #当前还未占位，所以不用清除#setOccupiedByAttitude(-1, boats_pos[boat_id], boats_dir[boat_id, move_collision_table_next, grid_map)#清除其占位
                backstepBoatCollision(boat_id, boats_pos[boat_id], boats_dir[boat_id], boats_new_pos, boats_new_dir, boat_collision, grid_map)#回溯检测点冲突
                if (boat_j >= 0 and boat_j!=boat_id) and boats_pos[boat_j] != boats_new_pos[boat_j] and not boat_collision[boat_j]:
                    setOccupiedByAttitude(-1, boats_new_pos[boat_j], boats_new_dir[boat_j], move_collision_table_next, grid_map)#清除其新位占位
                    backstepBoatCollision(boat_j, boats_pos[boat_j], boats_dir[boat_j], boats_new_pos, boats_new_dir, boat_collision, grid_map)#回溯检测点冲突
                if boat_j != boat_k and (boat_k>=0 and boat_k!=boat_id) and boats_pos[boat_k] != boats_new_pos[boat_k] and not boat_collision[boat_k]:
                    setOccupiedByAttitude(-1, boats_new_pos[boat_k], boats_new_dir[boat_k], move_collision_table_next, grid_map)#清除其新位占位
                    backstepBoatCollision(boat_k, boats_pos[boat_k], boats_dir[boat_k], boats_new_pos, boats_new_dir, boat_collision, grid_map)#回溯检测点冲突
                continue

            #如果没有冲突
            volume = setOccupiedByAttitude(boat_id, new_pos, new_dir, move_collision_table_next, grid_map)#占位
            # print('volume:{}'.format(volume))
        return boat_collision
    #输入生成
    def getRobotInput(orders) -> list:#通过指令计算出机器人的输入
        global robot_num, robot_collision
        robot_input_list = []
        robots_orders_list = [[] for _ in range(robot_num)]
        robots_move_pos = [robots_pos[i] for i in range(robot_num)]
        robots_moved = [False for _ in range(robot_num)]
        robot_collision = [False for _ in range(robot_num)]
        #分配指令（已经处理了指令的前期简单条件，譬如购买金钱是否足够）
        for order in orders:#分配order到列表，但是注意这里没有进行指令的冲突检测。
            order: str
            order_split = order.split()
            if order_split[0] not in ['move', 'get', 'pull', 'lbot']:
                continue
            robot_id = int(order_split[1])
            if 'move' == order_split[0]:
                if robots_moved[robot_id]:
                    logging.error('move twice!!!')
                    continue
                robot_move = DIR_TO_MOVE[int(order_split[2])]
                robots_move_pos[robot_id] = (robot_move[0]+robots_pos[robot_id][0], robot_move[1]+robots_pos[robot_id][1])
                robots_moved[robot_id] = True
            if 'get' == order_split[0]:
                robots_orders_list[robot_id].append(order)
            if 'pull' == order_split[0]:
                robots_orders_list[robot_id].append(order)
            if 'lbot' == order_split[0]:
                global global_money
                if global_money>=2000:
                    robots_orders_list.append([order])
                continue
        #碰撞检测
        for robot_id in range(robot_num):
            if not isMainRoad(robots_pos[robot_id], grid_map):#如果不是主干道，则占据。
                move_collision_table_now[robots_pos[robot_id]] = robot_id
        robot_collision = getRobotCollisionCondition(robots_move_pos, robot_collision)
        #执行指令
        for robot_id, robot_order in enumerate(robots_orders_list):
            assert len(robot_order) <= 1
            if robot_id+1 > robot_num:#新加的机器人
                assert len(robot_order) == 1
                order = robot_order[0]
                order_split = order.split()
                assert order_split[0] == 'lbot'
                robot_input_list.append((robot_id, 0, int(order_split[1]), int(order_split[2])))
                global_robots_goods.append(None)
                global_money -= 2000
                continue
            #如果发生碰撞
            if robot_collision[robot_id]:
                robot_input_list.append((robot_id, robots_goods[robot_id], robots_pos[robot_id][0], robots_pos[robot_id][1]))
                continue
            #如果未发生碰撞
            if len(robot_order) == 1:
                order = robot_order[0]
                order_split = order.split()
                if order_split[0] == 'get' and isGood(robots_move_pos[robot_id], global_goods_dict):###注意判断货物使用的字典！！
                    #将货物从世界放置到机器人
                    global_robots_goods[robot_id] = global_goods_dict[robots_move_pos[robot_id]]
                    #添加输入
                    robot_input_list.append((robot_id, True, robots_move_pos[robot_id][0], robots_move_pos[robot_id][1]))
                elif order_split[0] == 'pull' and robots_goods[robot_id] and isBerth(robots_move_pos[robot_id], grid_map):
                    #将货物从机器人放置到泊口
                    global_berths_goods[road_way_table[robots_move_pos[robot_id]]].append(global_robots_goods[robot_id])
                    global_robots_goods[robot_id] = None
                    #添加输入
                    robot_input_list.append((robot_id, False, robots_move_pos[robot_id][0], robots_move_pos[robot_id][1]))
                else:
                    logging.error('unvalid order!!!')
                    robot_input_list.append((robot_id, robots_goods[robot_id], robots_move_pos[robot_id][0], robots_move_pos[robot_id][1]))
                continue
            assert len(robot_order) == 0
            robot_input_list.append((robot_id, robots_goods[robot_id], robots_move_pos[robot_id][0], robots_move_pos[robot_id][1]))
        assert len(robot_input_list) == len(robots_orders_list)
        for robot_pos, robot_move_pos in zip(robots_pos, robots_move_pos):#清除碰撞表
            move_collision_table_now[robot_pos] = -1
            move_collision_table_now[robot_move_pos] = -1
            move_collision_table_next[robot_pos] = -1
            move_collision_table_next[robot_move_pos] = -1

        return robot_input_list
    def getBoatInput(orders) -> list:
        global boat_num, boat_collision
        boat_input_list = []
        boat_orders_list = [[] for _ in range(boat_num)]
        new_poses = [boat_pos for boat_pos in boats_pos]
        new_dirs = boats_dir.copy()
        ordered = [False for _ in range(boat_num)]
        boat_collision = [False for _ in range(boat_num)]
        #分配指令（已经处理了指令的前期简单条件，譬如购买金钱是否足够）
        for order in orders:#分配order到列表
            order: str
            order_split = order.split()
            if order_split[0] not in ['rot', 'ship', 'dept', 'berth', 'lboat']:
                continue
            
            if 'lboat' == order_split[0]:
                global global_money
                if global_money>=8000:
                    boat_orders_list.append([order])
                continue
            # print(boats_list)
            # print(order_split)
            boat_id = int(order_split[1])
            # print(boat_id)
            boat_attitude = boats_dir[boat_id]
            if global_boats_restore[boat_id] > 0:#如果船处于恢复状态，则忽略命令。
                logging.error('order when restores!!!')
                continue
            if boats_list[boat_id].status == 2 and order_split[0] != 'dept':#如果船在取货状态
                logging.error('boat move when berth!!!')
                continue
            if 'rot' == order_split[0]:
                if ordered[boat_id]:
                    logging.error('ordered twice!!!')
                    continue
                if int(order_split[2]) == 0:
                    boat_move, new_dirs[boat_id] = rotateClockwise(boat_attitude)
                elif int(order_split[2]) == 1:
                    boat_move, new_dirs[boat_id] = rotateAnticlockwise(boat_attitude)
                
                new_poses[boat_id] = (boat_move[0]+boats_pos[boat_id][0], boat_move[1]+boats_pos[boat_id][1])
                ordered[boat_id] = True
            if 'ship' == order_split[0]:
                if ordered[boat_id]:
                    logging.error('ordered twice!!!')
                    continue
                boat_move, new_dirs[boat_id] = DIR_TO_MOVE[boat_attitude], boat_attitude
                
                new_poses[boat_id] = (boat_move[0]+boats_pos[boat_id][0], boat_move[1]+boats_pos[boat_id][1])
                ordered[boat_id] = True
            if 'dept' == order_split[0] and boats_list[boat_id].status == 2:
                if ordered[boat_id]:
                    logging.error('ordered twice!!!')
                    continue
                boat_orders_list[boat_id].append(order)
                ordered[boat_id] = True
            if 'berth' == order_split[0]:
                if ordered[boat_id]:
                    logging.error('ordered twice!!!')
                    continue
                boat_orders_list[boat_id].append(order)
                ordered[boat_id] = True
        #碰撞检测
        for boat_id in range(boat_num):#占据当前位置
            setOccupiedByAttitude(boat_id, boats_pos[boat_id], boats_dir[boat_id], move_collision_table_now, grid_map)
        boat_collision = getBoatCollisionCondition(new_poses, new_dirs, boat_collision)
        #处理指令
        fail_purchase = 0
        for boat_id, boat_order in enumerate(boat_orders_list):
            assert len(boat_order) <= 1
            #lboat新加的船，需要检测碰撞！
            if boat_id+1 > boat_num:
                print(boat_order)
                assert len(boat_order) == 1
                order = boat_order[0]
                order_split = order.split()
                assert order_split[0] == 'lboat'
                core_pos_x, core_pos_y = int(order_split[1]), int(order_split[2])
                pos_x, pos_y = getBoxLeftUpPos((core_pos_x, core_pos_y), 0)

                if (core_pos_x, core_pos_y) not in [purchase_point.pos for purchase_point in boat_purchase_points]:
                    logText('purchase boat '+str((pos_x, pos_y))+' fail terrin')
                    fail_purchase += 1
                    continue
                
                temp_collision = [move_collision_table_next[temp_pos] for temp_pos in getVolume((pos_x, pos_y), 0)]#与其它船是否有碰撞
                if max(temp_collision) >= 0:
                    logText('purchase boat '+str((pos_x, pos_y))+' fail collision')
                    fail_purchase += 1
                    continue
                
                setOccupiedByAttitude(boat_id-fail_purchase, (pos_x, pos_y), 0, move_collision_table_next, grid_map)
                global_boats_goods.append([])
                boat_input_list.append((boat_id-fail_purchase, 0, core_pos_x, core_pos_y, 0, 0))
                global_boats_restore.append(0)
                global_boat_last_action.append('')
                global_money -= 8000
                continue
            
            boat = boats_list[boat_id]
            
            if global_boats_restore[boat_id]>0:#如果船处于恢复状态
                global_boats_restore[boat_id] -= 1
                core_x, core_y = getCorePos(boats_pos[boat_id], boats_dir[boat_id])
                if global_boats_restore[boat_id] > 0:#如果船仍然处于恢复状态
                    boat_input_list.append((boat_id, boat.goods_num, core_x, core_y, boats_dir[boat_id], 1))
                    continue
                if isBerth((core_x, core_y), grid_map) and global_boat_last_action[boat_id] == 'berth':#如果船的核心点在泊口，则判定船应该是处于运货状态
                    boat_input_list.append((boat_id, boat.goods_num, core_x, core_y, boats_dir[boat_id], 2))
                    continue
                boat_input_list.append((boat_id, boat.goods_num, core_x, core_y, boats_dir[boat_id], 0))
                continue
            
            if boat_collision[boat_id]:#如果发生碰撞
                x, y = getCorePos(boats_pos[boat_id], boats_dir[boat_id])
                boat_input_list.append((boat_id, boat.goods_num, x, y, boats_dir[boat_id], 0))
                continue
            
            if len(boat_order) == 1:#如果未发生碰撞, 执行指令
                order = boat_order[0]
                order_split = order.split()
                x, y = getCorePos(boats_pos[boat_id], boats_dir[boat_id])
                logText('tag')
                if order_split[0] == 'berth' and isKArea(boat.core_pos, grid_map):
                    tag = multi_tag_table.getTag(boat.core_pos, False)
                    if tag<0:
                        logText('泊口，你最近的k兄弟都不认得了？？')
                    berth_x, berth_y = berths_boat_pos[tag]
                    boat_input_list.append((boat_id, boat.goods_num, berth_x, berth_y, berths_dir[tag], 1))#进入恢复状态?还是恢复，但是进入运货状态？
                    global_boats_restore[boat_id] += 2*abs(x-berth_x)+2*abs(y-berth_y)
                    global_boat_last_action[boat_id] = 'berth'
                    continue
                elif order_split[0] == 'dept' and boat.status == 2:#处于取货状态且
                    dept_x, dept_y = searchDeptPos((x, y))
                    boat_input_list.append((boat_id, boat.goods_num, dept_x, dept_y, boats_dir[boat_id], 1))#进入恢复状态?
                    global_boats_restore[boat_id] += 2*abs(x-dept_x)+2*abs(y-dept_y)
                    global_boat_last_action[boat_id] = 'dept'
                    continue
                logging.error('error condition?????????')
            else:#可能在移动，也可能没在移动。但肯定没有发生碰撞
                assert len(boat_order)==0
                if new_poses[boat_id] == boats_pos[boat_id]:#未移动
                    new_num = boat.goods_num
                    if boat.status == 2:
                        new_num += loadGoods(boat_id)
                    x, y = getCorePos(boats_pos[boat_id], boats_dir[boat_id])
                    boat_input_list.append((boat_id, new_num, x, y, boats_dir[boat_id], boat.status))
                else:#发生移动
                    new_x, new_y = getCorePos(new_poses[boat_id], new_dirs[boat_id])
                    entered_main_way = [isMainWay(pos, grid_map) for pos in getVolume(new_poses[boat_id], new_dirs[boat_id])]
                    # print('entered_main:{}'.format(entered_main_way))
                    # print([grid_map[pos] for pos in getVolume(new_poses[boat_id], new_dirs[boat_id])])
                    # print(getVolume(new_poses[boat_id], new_dirs[boat_id]))
                    restore = 0
                    good_num = boat.goods_num
                    if max(entered_main_way):
                        restore = 1
                        logText('boat '+str(boat_id)+' restore')
                        global_boats_restore[boat_id] += 1
                    if isTransportPoint((new_x, new_y), grid_map):
                        settleBoatGoods(boat_id)
                        good_num = 0
                    boat_input_list.append((boat_id, good_num, new_x, new_y, new_dirs[boat_id], restore))
                continue
        
        for boat_pos, boat_dir in zip(boats_pos, boats_dir):#清除碰撞表
            setOccupiedByAttitude(-1, boat_pos, boat_dir, move_collision_table_now, grid_map)
        for boat_pos, boat_dir in zip(new_poses, new_dirs):
            setOccupiedByAttitude(-1, boat_pos, boat_dir, move_collision_table_next, grid_map)
        for boat_pos in [purchase_point.pos for purchase_point in boat_purchase_points]:#新加的船也得清除
            setOccupiedByAttitude(-1, boat_pos, 0, move_collision_table_next, grid_map)
        move_collision_table_next[:] = -1
            
            # print(e)
            # logText('boat_order_fail')
            # print(traceback.print_exc()) #打印异常到屏幕
            # logText('robot_order_fail:' + str(traceback.print_exc()))
            

        return boat_input_list
    def generateGoods():
        while True:
            x = np.random.randint(0, N)
            y = np.random.randint(0, N)
            if goods_dict.get((x, y), 0):
                continue
            if not isRoad((x,y), grid_map):
                continue
            
            value = np.random.randint(0, 1000)
            return x, y, value
    def getChangeGoods(frame):
        out_goods = []
        new_goods_num = np.random.randint(-3, 3)#每帧生成0-2个货物。range(0)不会迭代。
        for _ in range(max(new_goods_num, 0)):
            x, y, val = generateGoods()
            global_goods_dict[(x,y)] = Good(val, frame)
            out_goods.append([x,y,val])
        out_goods.extend(changeOldGoods(frame))
        return out_goods
    def changeOldGoods(frame):
        out_goods = []
        pop_goods = []
        for good_pos, good in global_goods_dict.items():
            if frame - good.time >= 1000:
                out_goods.append([good_pos[0], good_pos[1], 0])
                pop_goods.append(good_pos)
            elif good in global_robots_goods:
                out_goods.append([good_pos[0], good_pos[1], 0])
                pop_goods.append(good_pos)
        for good_pos in pop_goods:
            k = global_goods_dict.pop(good_pos, 0)
            if not k:
                sys.exit()
        return out_goods
    
    def loadGoods(boat_id):#先执行船的指令，后泊位装载货物
        boat = boats_list[boat_id]
        core_pos = boats_list[boat_id].core_pos
        berth_id = road_way_table[core_pos]
        berth = berths_list[berth_id]
        num = 0
        if len(global_berths_goods[berth_id]):
            num += min(len(global_berths_goods[berth_id]), berth.loading_speed, boat_capacity-boat.goods_num)
            for i in range(num):
                global_boats_goods[boat_id].append(global_berths_goods[berth_id].pop(0))
        return num
    def settleBoatGoods(boat_id):#结算船的货物
        global global_money, global_boats_goods
        boat_good_list = global_boats_goods[boat_id]
        if len(boat_good_list):
            for good in boat_good_list:
                good:Good
                global_money += good.value
        global_boats_goods[boat_id] = []

    def searchDeptPos(core_pos):
        temp_Q = [core_pos]
        u_list = []
        while len(temp_Q):
            u_list.append(temp_Q.pop(0))
            for v in findAdjacentWay(u_list[-1], grid_map):
                if isMainWay(v, grid_map) and (not isBerth(v, grid_map)):
                    return v
                if v not in u_list:
                    temp_Q.append(v)
if test_mode:#命令窗口函数
    def orderWindowProcess(orders, order_times, just_go):
        if order_times <= 1:#执行回数
            just_go = False
            order_window.activateWindow()
            order_window.clearTextDisplay(order_window.timesText)#清除循环次数
            out_info = input()
            while out_info != '\n' and out_info != '':#当有指令要执行时（有变量要查看时）
                if 'just_go' in out_info:
                    just_go = True
                    break
                out = None
                try:
                    out = eval(out_info)
                except Exception as ex:
                    print(ex)
                    print('['+ str(out_info) +']'+' unable excute')
                order_window.setDisplayExcute('\n['+ str(out_info) +']:\n' + str(out))
                order_window.activateWindow()
                out_info = input()
            
            if not just_go:
                orders, order_times = order_window.getOrder()
            else:
                _, order_times = order_window.getOrder()
        else:
            order_times -= 1
            if not just_go:#如果不按设计的控制算法给出指令
                orders, _ = order_window.getOrder()#使指令为窗口指令，但是一直运行
        return orders, order_times, just_go


    def textDisplay(frame, money):
        temp_boats_dir = [DIR_TO_MOVE[boat_dir] for boat_dir in boats_dir]
        text_all = ''
        text_all += 'frame:{}  money:{}\n'\
            .format(frame, money)
        text_all += 'robot_purchase_points:{}\nboat_purchase_points:{}\ndelivery_point:{}\n\n'\
            .format([point.pos for point in robot_purchase_points], \
                [point.pos for point in boat_purchase_points], delivery_point)
        text_all += 'boats_pos:{}\nboats_dir:{}\nrobots_pos:{}\nrobots_goods:{}\nrobot_des:{}\n\n'\
            .format(boats_pos, boats_dir, robots_pos, robots_goods, [robot.des for robot in robots_list])
        text_all += 'boats_num{}\nboats_value{}\nboat_des:{}\nberth_pos:{}\nberth_goods:{}\nberth_value:{}\nberth_attached:{}\n\n'\
            .format([boat.goods_num for boat in boats_list], [boat.goods_value for boat in boats_list], \
                [boat.des for boat in boats_list], \
                [berth.side[0] for berth in berths_list],[berth.present_goods_num for berth in berths_list], \
                    [sum(berth.goods_value) for berth in berths_list], [berth.attached_robots for berth in berths_list])
        text_all += 'berth_pre{}\nberth_ids{}\n\n'\
            .format([berth.boat_stop for berth in berths_list],[berth.boat_stop_ids for berth in berths_list])
        text_all += 'log:{}\n\n'.format(log_text)
        order_window.setDisplayText(text_all)
    def clearLogText():
        for _ in range(len(log_text)):
            log_text.pop(0)
if test_mode:#绘图写文件函数
    def initImage():
        plt.ion()
        # plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)
        # plt.margins(0, 0)
        size = 0.2
        global figure, ax
        figure, ax = plt.subplots(figsize=(3,3),dpi = 310)
        plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)
        #plt.figure()
        #figure.size
        #去除边框
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)
        
        ax.set_xlim(-1,N+1)
        ax.set_ylim(-1,N+1)
        
        ax.set_xticks([])
        ax.set_yticks([])
        
        x_zhangai,y_zhangai = np.where(grid_map==CH_TO_NUM['#'])
        ax.scatter(x_zhangai,y_zhangai,marker='s',c='grey', s=size)
        x_berth, y_berth = np.where(grid_map==CH_TO_NUM['B'])
        ax.scatter(x_berth, y_berth, marker='s',c=(0.7,0.5,0.9), s=size)
        x_berth_k, y_berth_k = np.where(grid_map==CH_TO_NUM['K'])
        ax.scatter(x_berth_k, y_berth_k, marker='s',c='lightgray', s=size)
        x_water, y_water = np.where(grid_map==CH_TO_NUM['*'])
        ax.scatter(x_water, y_water, marker='s',c='lightgreen', s=size)
        x_mainWater, y_mainWater = np.where(grid_map==CH_TO_NUM['~'])
        ax.scatter(x_mainWater, y_mainWater, marker='s',c='aquamarine', s=size)
        x_transport_point, y_transport_point = np.where(grid_map==CH_TO_NUM['T'])
        ax.scatter(x_transport_point, y_transport_point, marker='s',c='darkblue', s=size)
        x_c, y_c = np.where(grid_map==CH_TO_NUM['c'])
        ax.scatter(x_c, y_c, marker='s',c='green', s=size)
        x_C, y_C = np.where(grid_map==CH_TO_NUM['C'])
        ax.scatter(x_C, y_C, marker='s',c='darkgreen', s=size)
        x_S, y_S = np.where(grid_map==CH_TO_NUM['S'])
        ax.scatter(x_S, y_S, marker='s',c='powderblue', s=size)
        x_R, y_R = np.where(grid_map==CH_TO_NUM['R'])
        ax.scatter(x_R, y_R, marker='s',c='powderblue', s=size)
        # marginal = ax.scatter([], [], linewidths=0.3)
        # sc = ax.scatter([], [])

        # #paths_plt = ax.scatter([], [], c='green', linewidths=0.03)
        global robots_plt, boats_plt, goods_plt
        robots_plt = ax.scatter([], [], marker='x',  c='black', s=size)
        boats_plt = ax.scatter([], [], marker='*', c='blue', s=size)
        goods_plt = ax.scatter([], [],marker='P', c='red', s=size)

        plt.pause(0.01)
        plt.show() 
        return figure, ax
        # figure.canvas.draw()
        # figure.canvas.flush_events() 
        # plt.figure(dpi=300,figsize=(24,8))
    def drawImage(robots_pos_plt, boats_pos_plt, boats_attitude_plt, goods_pos_plt):#注意这里用的位置都是方块左上角位置。
        #plot goods
        if len(goods_pos_plt):

            goods_pos_array = np.array(list(goods_dict.keys()))
            x_data = goods_pos_array[:, 0]
            y_data = goods_pos_array[:, 1]
            goods_plt.set_offsets(np.c_[x_data, y_data])
        #plot robot
        color = np.ones((len(global_robots_goods), 3))*0.1
        color[[j!=None for j in global_robots_goods], 1] = 1
        color[np.where(np.array(robot_collision)==True), 0] = 1
        robots_plt.set_facecolors(color)
        if len(robots_pos_plt):
            robots_pos_array = np.array(robots_pos_plt)
            x_data = robots_pos_array[:, 0]
            y_data = robots_pos_array[:, 1]
            robots_plt.set_offsets(np.c_[x_data, y_data])
        #plot boat
        boats_have_goods = np.array([boat.goods_num>0 for boat in boats_list])
        color = np.ones((6*len(boats_have_goods), 3))*0.1
        color_0 = np.ones((len(boats_have_goods), 3))*0.1
        color_0[np.where(boats_have_goods==True), 1] = 1
        color_0[np.where(np.array(global_boats_restore)>0), 2] = 1
        color_0[np.where(np.array(boat_collision)==True), 0] = 1
        color_0[np.where(np.array(boat_collision)==True), 1] = 0.1
        for i in range(len(color_0)):
            for j in range(6):
                color[i*6+j, :] = color_0[i, :]
        boats_plt.set_facecolors(color)
        if len(boats_pos_plt):
            x_data = np.zeros(6*len(boats_pos_plt))
            y_data = np.zeros(6*len(boats_pos_plt))
            i = 0
            for boat_pos, boat_attitude in zip(boats_pos_plt, boats_attitude_plt):
                for temp_pos in getVolume(boat_pos, boat_attitude):
                    x_data[i] = temp_pos[0]
                    y_data[i] = temp_pos[1]
                    i+=1
            boats_plt.set_offsets(np.c_[x_data, y_data])
        
        plt.pause(0.01)
        plt.show()
    def writeFile(info, file_name:str = 'info.txt'):
        with open(file_name, 'a') as file:
            if type(info) == np.ndarray:
                np.savetxt(file, info, fmt='%.3e')
                return
            file.write(info)
            return
if test_mode:#接收输入函数
    def testInit(start_time, init_tag_time, init_all_time):
        global boat_capacity, berth_num
        with open('./maps/map1.txt', 'r') as map:
            for i in range(N):
                line = map.readline().strip()
                #ch.append([c for c in line.split(sep=" ")])·
                grid_map[i,:] = [CH_TO_NUM[c] for c in line[0:N]]
            line = map.readline()
            berth_num = int(line)
            initGlobalBerthsGoods(berth_num)
            
            for i in range(berth_num):
                road_way_lists.append([])

            for i in range(berth_num):
                id = i
                line = map.readline().strip().split(' ')
                x, y, dir, loading_speed = int(line[0]), int(line[1]), int(line[2]), int(line[3])
                pos = (x,y)
                
                berths_dir.append(dir)#仅供判题器使用
                berths_boat_pos.append(pos)#仅供判题器使用
                
                berth = Berth(id, pos, loading_speed)#注意初始化的时候会将pos放入berth.side
                extendBerthArea(pos, berth, id)#将除pos外的其它相连berth点放入berth.side,并在road_way_table中记录
                berths_list.append(berth)

                berths_dist_tables.append(DoubleDistTable(grid_map, np.full((N,N), UP_LIMIT, dtype=np.int16) , \
                    road_way_list = road_way_lists[id], road_way_table = road_way_table, road_way_offset = road_way_offset))#使其适用于海路
                berths_dist_tables[i].setGoal(berth.side)
                for temp_pos in berth.side:
                    multi_tag_table.addTag(temp_pos, id)
        process_map(grid_map, berth_num)#process_map应当可以在之前也可以在之后，放在之后是为了方便获取泊口数量，以设置road_way_list的数量
        multi_tag_table.setWayInfo(road_way_tag_table_list, road_way_tag_list, road_way_table, road_way_offset)

        while (time.perf_counter() - start_time) < init_tag_time:
            for temp_tag in range(berth_num):#搜寻multiTagTable
                multi_tag_table.growTag(temp_tag)#搜寻地面
                multi_tag_table.growTag(temp_tag, False)#搜寻海域
                if (time.perf_counter() - start_time) > init_tag_time:
                    break
        while (time.perf_counter() - start_time) < init_all_time:
            for berth_dist_table in berths_dist_tables:
                berth_dist_table.preGet()#搜寻地面
                berth_dist_table.preGet(road=False)#搜寻海域
                if (time.perf_counter() - start_time) > init_all_time:
                    break
            for item in transp_dist_tables:
                item.preGet()
                item.offset_table.preGet()
            if (time.perf_counter() - start_time) > init_all_time:
                break
        boat_capacity = 50
        #okk = input()
        print("OK")
        
        #sys.stdout.flush()
    def testInput(orders:list = []):
        #处理
        global global_frame
        global_frame += 1
        robot_input_list = getRobotInput(orders)
        boat_input_list = getBoatInput(orders)
        print(boat_input_list)
        change_goods_list = getChangeGoods(global_frame)
        
        #接收
        goods_num = len(change_goods_list)
        for i in range(goods_num):
            x, y, val = change_goods_list[i]
            x, y, val = int(x), int(y), int(val)
            if val==0:#无法判断是被机器人拿走了还是超时消失了？
                if goods_dict.get((x,y), 0) and (valid_time-(global_frame-goods_dict.get((x,y)).time))<=0:
                    goods_dict.pop((x, y), 0)#如果未从字典中拿出，则拿出
                continue
            goods_unsc.append((x,y))
            goods_dict[(x,y)] = Good(val, frame_id)
        
        global robot_num
        robot_num = len(robot_input_list)
        for i in range(robot_num):
            if i+1 > len(robots_list):
                addRobot(i)
            id, robots_goods[i], x, y = robot_input_list[i]
            assert id == robots_list[i].id
            robots_pos[i] = (x, y)
        
        global boat_num
        boat_num = len(boat_input_list)
        for i in range(boat_num):
            if i+1 > len(boats_list):
                addBoat(i, global_frame)
            boat = boats_list[i]
            id, boat.goods_num, x, y, boats_dir[i], boat.status = boat_input_list[i]
            boat.goods_num = int(boat.goods_num)
            boats_dir[i] = int(boats_dir[i])
            boat.status = int(boat.status)
            assert id == boat.id
            boat.core_pos = (x, y)
            boats_pos[i] = getBoxLeftUpPos((x, y), boats_dir[i])

#增加agent需要预先做的工作boats_list
def addRobot(i):
    robots_list.append(Robot(i))
    robots_goods.append(False)
    robots_work.append(False)
    robots_pos.append(())    
def addBoat(i, frame_id):
    boats_list.append(Boat(i, 0, send_time=frame_id))
    boats_pos.append(())
    boats_dir.append(0)
    boats_restore.append(0)
    boats_work.append(False)

#接收判题器输出
def Init_p(start_time):
    global boat_capacity, berth_num
    for i in range(0, N):
        line = input()
        #ch.append([c for c in line.split(sep=" ")])·
        grid_map[i,:] = [CH_TO_NUM[c] for c in line]
    berth_num = int(input())

    for i in range(berth_num):
        goods_distributed.append([])
        road_way_lists.append([])

    for i in range(berth_num):
        id, x, y, loading_speed = map(int, input().split())
        logging.error(str(id)+' '+str(x)+' '+str(y)+' '+str(loading_speed))
        pos = (x,y)
        berth = Berth(id, pos, loading_speed)#注意初始化的时候会将pos放入berth.side
        extendBerthArea(pos, berth, id)#将除pos外的其它相连berth点放入berth.side,并在road_way_table中记录
        berths_list.append(berth)

        berths_dist_tables.append(DoubleDistTable(grid_map, np.full((N,N), UP_LIMIT, dtype=np.int16) , \
            road_way_list = road_way_lists[id], road_way_table = road_way_table, road_way_offset = road_way_offset))#使其适用于海路
        berths_dist_tables[i].setGoal(berth.side)
        for temp_pos in berth.side:
            multi_tag_table.addTag(temp_pos, id)
    process_map(grid_map, berth_num)#process_map应当可以在之前也可以在之后，放在之后是为了方便获取泊口数量，以设置road_way_list的数量

    while (time.perf_counter() - start_time) < 4.4:
        for berth_dist_table in berths_dist_tables:
            berth_dist_table.preGet()#搜寻地面
            berth_dist_table.preGet(road=False)#搜寻海域
        if (time.perf_counter() - start_time) > 4.4:
            break
        for temp_tag in range(berth_num):#搜寻multiTagTable
            multi_tag_table.growTag(temp_tag)

    boat_capacity = int(input())
    logging.error(str(boat_capacity))
    okk = input()

    print("OK")
    sys.stdout.flush()
def Init(start_time):
    global boat_capacity, berth_num
    for i in range(0, N):
        line = input()
        #ch.append([c for c in line.split(sep=" ")])·
        grid_map[i,:] = [CH_TO_NUM[c] for c in line]
    berth_num = int(input())
    for i in range(berth_num):
        road_way_lists.append([])

    for i in range(berth_num):
        id, x, y, loading_speed = map(int, input().split())
        pos = (x,y)
        
        #berths_dir.append(dir)#仅供判题器使用
        #berths_boat_pos.append(pos)#仅供判题器使用
        
        berth = Berth(id, pos, loading_speed)#注意初始化的时候会将pos放入berth.side
        extendBerthArea(pos, berth, id)#将除pos外的其它相连berth点放入berth.side,并在road_way_table中记录
        berths_list.append(berth)

        berths_dist_tables.append(DoubleDistTable(grid_map, np.full((N,N), UP_LIMIT, dtype=np.int16) , \
            road_way_list = road_way_lists[id], road_way_table = road_way_table, road_way_offset = road_way_offset))#使其适用于海路
        berths_dist_tables[i].setGoal(berth.side)
        for temp_pos in berth.side:
            multi_tag_table.addTag(temp_pos, id)
    process_map(grid_map, berth_num)#process_map应当可以在之前也可以在之后，放在之后是为了方便获取泊口数量，以设置road_way_list的数量
    multi_tag_table.setWayInfo(road_way_tag_table_list, road_way_tag_list, road_way_table, road_way_offset)

    limit_time = 4.4
    while (time.perf_counter() - start_time) < limit_time:
        for temp_tag in range(berth_num):#搜寻multiTagTable
            multi_tag_table.growTag(temp_tag)#搜寻地面
            multi_tag_table.growTag(temp_tag, False)#搜寻海域
            if (time.perf_counter() - start_time) > limit_time:
                break
        for berth_dist_table in berths_dist_tables:
            berth_dist_table.preGet()#搜寻地面
            berth_dist_table.preGet(road=False)#搜寻海域
            if (time.perf_counter() - start_time) > limit_time:
                break
        for item in transp_dist_tables:
            item.preGet()
            item.offset_table.preGet()
        if (time.perf_counter() - start_time) > limit_time:
            break
    boat_capacity = int(input())
    okk = input()

    print("OK")
    sys.stdout.flush()
    
    #sys.stdout.flush()

def Input_p():
    goods_num = int(input())
    for i in range(goods_num):
        x, y, val = map(int, input().split())
        goods_pos.append((x,y))
        goods_dict[(x,y)] = Good(val, frame_id)
    global robot_num
    robot_num = int(input())
    for i in range(robot_num):
        id, robots_goods[i], x, y = map(int, input().split())
        assert id == robots_list[i].id
        robots_pos[i] = (x, y)
    global boat_num
    boat_num = int(input())
    for i in range(boat_num):
        boat = boats_list[i]
        id, boat.goods_num, x, y, boats_dir[i], boat.status = map(int, input().split())
        assert id == boat.id == i
        boats_pos[i] = (x, y)
    okk = input()
def Input(frame):
    #接收
    goods_num = int(input())
    for i in range(goods_num):
        x, y, val = map(int, input().split())
        if val==0:#无法判断是被机器人拿走了还是超时消失了？
            if goods_dict.get((x,y), 0) and (valid_time-(frame-goods_dict.get((x,y)).time))<=0:
                goods_dict.pop((x, y), 0)#如果未从字典中拿出，则拿出
            continue
        goods_unsc.append((x,y))
        goods_dict[(x,y)] = Good(val, frame_id)
    
    global robot_num
    robot_num = int(input())
    for i in range(robot_num):
        if i+1 > len(robots_list):
            addRobot(i)
        id, robots_goods[i], x, y = map(int, input().split())
        assert id == robots_list[i].id
        robots_pos[i] = (x, y)
    
    global boat_num
    boat_num = int(input())
    for i in range(boat_num):
        if i+1 > len(boats_list):
            addBoat(i, frame)
        boat = boats_list[i]
        id, boat.goods_num, x, y, boats_dir[i], boat.status = map(int, input().split())
        boat.goods_num = int(boat.goods_num)
        boats_dir[i] = int(boats_dir[i])
        boat.status = int(boat.status)
        assert id == boat.id
        boat.core_pos = (x, y)
        boats_pos[i] = getBoxLeftUpPos((x, y), boats_dir[i])
    okk = input()
#货物处理函数#根据机器人需求构建相应的货物集#包分配机器人
def distributGoods():#分派货物到泊口，只有需要给泊口机器人分派时，或者不存在机器人未分配泊口，才调用此函数
    to_be_popped = []
    for temp_id, pos in enumerate(goods_pos):
        tag = goods_dict[pos].berth_id
        if len(berths_list[tag].attached_robots)==0:#如果泊口无系连机器人
            continue
        else:
            to_be_popped.append(temp_id)
            berths_list[tag].assignGood(pos, 0)#将货物分配到泊口集中的第一级,但是这样会不好处理消失货物？
    if len(to_be_popped):
        to_be_popped.reverse()
        for temp_id in to_be_popped:
            goods_pos.pop(temp_id)

def assignGoodsForAttached(frame, berth_id:int):#给所有有系连泊口的机器人分配货物。铁饭碗，包分配
    #分配货物
    berth = berths_list[berth_id]
    attached_num = len(berth.attached_robots)
    
    while attached_num+1>=len(berth.goods):#berth.goods[-1]即为最终垃圾集
        berth.goods.append([])
    while attached_num+1>len(berth.goods):#如果有机器人解绑定该泊口，留下一些货物，则将这些货物放入开集。
        left_goods_list = berth.goods.pop(-2)#找到倒数第二个货集
        if len(left_goods_list):
            goods_pos.extend(left_goods_list)
    if not attached_num:
        return
    for level, robot_id in enumerate(berth.attached_robots):
        
        goods_list = berth.goods[level]
        rubbish_list = berth.goods[level+1]
        
        robot = robots_list[robot_id]        
        #如果当前机器人还有货物，那么应该先把除了第一个货物之外的货物出栈，放进相应的货物集中，然后将第一个货物的完成时间作为起始使用时间？
        gross_pull_time = frame if not robots_work[robot_id] else robot.guess_pull_time
        #构建货物集,难点：如何分配多个泊口的货物集货物？
        goods_list, rubbish_list, time_use = setGoods(frame, goods_list, rubbish_list, gross_pull_time, offset=20, preserve_num=head_goods)#会直接改变泊口内的goods_list
    for _ in rubbish_list:#将最末尾的垃圾货物装进其它货物集
        goods_left.append(rubbish_list.pop())
def updateGoods():#清除过期货物。注意过期货物在input中判别并清除goods_dict，但是在该函数中清除位置
    to_be_popped = []
    for i, good_pos in enumerate(goods_pos):
        good = goods_dict.get(good_pos, 0)
        if not good:
            to_be_popped.append(i)
    to_be_popped.sort(reverse=True)
    for i in to_be_popped:
        goods_pos.pop(i)
    
    to_be_popped = []
    for i, good_pos in enumerate(goods_left):
        good = goods_dict.get(good_pos, 0)
        if not good:
            to_be_popped.append(i)
    to_be_popped.sort(reverse=True)
    for i in to_be_popped:
        goods_left.pop(i)
    #将货物分配到连通货物集中。
    to_be_popped = []
    for temp_id, pos in enumerate(goods_unsc):
        tag = multi_tag_table.getTag(pos)
        if tag<0:
            continue
        else:
            good = goods_dict[pos]
            good.berth_id = tag
            good.berth_dist = multi_tag_table.getDist(pos)
            to_be_popped.append(temp_id)
            goods_pos.append(pos)
            berths_list[tag].areaGoodArise(good.time, good.value)
    to_be_popped.reverse()
    for temp_id in to_be_popped:
        goods_unsc.pop(temp_id)
#货物排序函数
def setGoods(frame, goods_list, rubbish_list, start_time:int = 0, offset:int = 10, preserve_num:int = 10):#这种排序应该在机器人取货时进行！泊口只需要按价值排序就行了,会把单个机器人无法获取的货物都吐出来。
    '''
    按货物的价值和消失时间进行排序，offset为预计阻塞损耗时间。
    '''
    zhen = frame + start_time + offset
    #print('setGood')
    goods_list_num = len(goods_list)
    if not goods_list_num:#如果泊口境内没有货物
        return [], rubbish_list, []
    if goods_list_num == 1:
        good = goods_dict[goods_list[0]]
        if valid_time-zhen+good.time > 0:#如果
            return goods_list, rubbish_list, [good.berth_dist]
        if valid_time-frame+good.time > 0:
            rubbish_list.append(goods_list.pop(0))
            return goods_list, rubbish_list, []
        goods_list.pop(0)
        return goods_list, rubbish_list, []
    
    goods_list.sort(key=sortGoodsKey)
    if goods_list_num>preserve_num:
        for _ in range(goods_list_num - preserve_num):
            rubbish_list.append(goods_list.pop())
    #按代价从小到大（即按价值从大到小）排序
    #初始化参量
    time_waste = [0 for _ in goods_list]
    time_use = []
    time_appear = []
    
    pop_list = []
    for good_id, good_pos in enumerate(goods_list):
        good = goods_dict.get(good_pos, 0)
        if not good:
            pop_list.append(good_id)
            continue
        time_use.append(good.berth_dist)
        time_appear.append(good.time)
    pop_list.sort(reverse=True)
    for i in pop_list:
        goods_list.pop(i)
    #循环货物排序，无法获取者放入开集
    goods_list_num = len(goods_list)
    i = 0
    while i < goods_list_num-1:#计算货物最优获取顺序
        #print(i)
        time_left_after = valid_time - zhen + time_appear[i+1]
        time_waste_after = time_left_after-time_use[i+1]-sum(time_use[:i+1])*2
        
        time_left_ahead = valid_time - zhen + time_appear[i]
        time_waste_ahead = time_left_ahead-time_use[i]-sum(time_use[:i])*2
        if valid_time - frame + time_appear[i] <= 0:#如果货物已经不可能获取，则不装进垃圾桶，直接删除货物
            goods_list.pop(i)
            time_use.pop(i)
            time_waste.pop(i)
            goods_list_num -= 1
            continue
        if valid_time - frame + time_appear[i+1] <= 0:#如果货物已经不可能获取，则不装进垃圾桶，直接删除货物
            goods_list.pop(i+1)
            time_use.pop(i+1)
            time_waste.pop(i+1)
            goods_list_num -= 1
            continue
        if time_waste_ahead<0:#如果前一货物无法获得，则直接pop前一货物
            rubbish_list.append(goods_list.pop(i))
            time_use.pop(i)
            time_waste.pop(i)
            goods_list_num -= 1
            continue
        if time_waste_ahead>0 and time_waste_after>0:#如果两货物都能获得，则尝试循环向前交换，查看交换后最小剩余时间。
            for j in range(i):#前移至剩余时间最大
                index = i-j-1#当j=0时，index=i-1
                time_waste_after = time_left_after-time_use[index+1]-sum(time_use[:index+1])*2
                
                time_left_ahead = valid_time - zhen + time_appear[index]
                time_waste_ahead = time_left_ahead-time_use[index]-sum(time_use[:index])*2
                
                new_time_waste_after = time_waste_ahead - 2*time_use[index+1]
                new_time_waste_ahead = time_waste_after + 2*time_use[index]
                if new_time_waste_after > time_waste_after:
                    replaceGoods(index, index+1, goods_list, time_waste, time_use, time_appear, new_time_waste_ahead, new_time_waste_after)
                else:
                    break
        if time_waste_ahead>0 and time_waste_after<0:#由于已按优先级排序，因此只需要比较时间消耗。如果能挽救
            for j in range(i):#前移至可挽救
                index = i-j-1
                #由于good_after只可能为1个，所以不更新good_after和time_left_after
                time_waste_after = time_left_after-time_use[index+1]-sum(time_use[:index+1])*2
                
                time_left_ahead = valid_time - zhen + time_appear[index]
                time_waste_ahead = time_left_ahead-time_use[index]-sum(time_use[:index])*2
                
                new_time_waste_after = time_waste_ahead - 2*time_use[index+1]
                new_time_waste_ahead = time_waste_after + 2*time_use[index]
                if new_time_waste_after > 0:#如果前一个货物仍能够获取，则继续
                    if new_time_waste_ahead > 0:
                        if new_time_waste_after > time_waste_after:#如果两个货物都能够获取，并且新的after大，那么进行交换
                            replaceGoods(index, index+1, goods_list, time_waste, time_use, time_appear, new_time_waste_ahead, new_time_waste_after)
                            #交换，下一轮将比较good_after和再上一个good_ahead
                        else:
                            break
                    if new_time_waste_ahead < 0:#如果移动一次仍然无法挽救，那么就继续移动
                        replaceGoods(index, index+1, goods_list, time_waste, time_use, time_appear, new_time_waste_ahead, new_time_waste_after)
                if new_time_waste_after <= 0:#如果导致前一个货物无法获取，则停止移动。
                    if time_waste_ahead <= 0:
                        #如果此时货物仍然无法获取，则将其扔掉(或者放在最后？)，然后将后面的货物时间都恢复？(waste_time+, )
                        rubbish_list.append(goods_list.pop(index+1))
                        for temp_index in range(index+2, i+1):
                            time_waste[temp_index] += 2*time_use[index+1]#后者时间都+
                        time_use.pop(index+1)
                        time_waste.pop(index+1)
                        goods_list_num -= 1
                        break
                    #如果此时货物能够获取，则直接退出循环。挽救货物成功
                    break
        i += 1
    return (goods_list, rubbish_list, time_use)
def sortGoodsKey(good_pos):#计算货物价值还是代价？
    good = goods_dict[good_pos]
    return good.berth_dist/good.value#这里计算的是代价
def replaceGoods(i, j, goods_list, time_waste, time_use, time_appear, new_time_waste_ahead, new_time_waste_after):
    goods_list[i], goods_list[j] = goods_list[j], goods_list[i]
    
    time_use[j], time_use[i] = time_use[i], time_use[j]
    time_appear[j], time_appear[i] = time_appear[i], time_appear[j]

    time_waste[i], time_waste[j] = new_time_waste_ahead, new_time_waste_after

def updateRobot(pibt:PIBT, frame:int):
    #print('update_robot')
    for i in range(robot_num):
        temp_pos = robots_pos[i]
        robot = robots_list[i]
        robot_des = robot.des
        if robot.last_state == ROBOT_STATE_TO_NUM['get'] and robots_goods[i]:
            robot.good = goods_dict.pop(temp_pos, 0)
            if not robot.good:
                sys.exit()
            robot.last_state = ROBOT_STATE_TO_NUM['free']
            robots_work[i] = False
            robot.des = None
            if log_text:
                robot.log_text += 'get '
            continue
        if robot.last_state == ROBOT_STATE_TO_NUM['pull'] and robots_goods[i] == False:
            berth = berths_list[road_way_table[temp_pos]]
            temp_pos = robots_pos[i]
            #grid_map[temp_pos] = CH_TO_NUM['C']
            berth.takeInGood(frame, robot.good.value)
            logText('take In')
            #print('take in')
            robot.good = None
            robots_work[i] = False
            robot.des = None
            setRobotBerths(i, road_way_table[temp_pos], frame)#尝试设置机器人从属泊口
            robot.last_state = ROBOT_STATE_TO_NUM['free']
            #设置泊位相关信息
            continue
        if robots_work[i] and (type(temp_pos)==tuple and robot_des==temp_pos) or \
            (type(robot_des)==list and temp_pos in robot_des):
            #已经到达目的地,但是没有pull或者get，可能是货物超时消失了
            #如果货物超时消失，可以检测当前位置有没有记录货物
            goods_dict.pop(temp_pos, 0)
            robots_work[i] = False
            robot.last_state = ROBOT_STATE_TO_NUM['free']
            robot.des = None
            continue
def updateRobotGet(pibt:PIBT, robot:Robot, frame:int):#指派固定泊口机器人行动
    #print('update_get_robot')
    #logText('update_get_robot')
    berth_id = robot.berths_attached
    assert berth_id >= 0
    goods_list = berths_list[berth_id].goods[robot.level]
    while len(goods_list):
        good_pos = goods_list.pop(0)
        good = goods_dict[good_pos]
        time_left = valid_time - frame + good.time - 5
        if time_left<=0:
            goods_dict.pop(good_pos, 0)
            continue
        robot.planGood(good_pos, good.value, good.berth_dist*2)
        robots_work[robot.id] = True
        pibt.setRobotDistTable(robot.id, np.full(grid_map.shape, UP_LIMIT, dtype=np.int16), False, True, good_pos, time_left - 5)
        break
def updateRobotPull(pibt:PIBT, robot:Robot, frame:int):#指派固定泊口机器人行动
    if not robot.good:#机器人身上有货物
        robots_goods[robot.id] = False
        if test_mode:
            logText('wrong ')
            return
    berth_id = robot.good.berth_id
    assert berth_id >= 0
    robots_work[robot.id] = True
    robot.des = berths_list[berth_id].side
    pibt.setRobotDistTable(robot.id, berths_dist_tables[berth_id].table, False, True, berths_dist_tables[berth_id].goal, Q=berths_dist_tables[berth_id].Q_road)
#机器人和船舶指派命令，讨饭机器人
def assignRobotWork(pibt:PIBT, grid_map:np.ndarray, dist_factor:tuple, zhen, start_time):#dist_factor为距离转换因子
    #如果所有机器人都在工作。注意工作判定。当给机器人发送get或者pull指令并执行后，即机器人未工作。
    #分为4种情况：在找货物的路上（work,!good）；取完货物后(!work,good)；放置完货物后(!work,!good)；未工作且未在泊口情况(!work,!good)
    #其中，在找货物的路上、未工作且未在泊口情况、放置完货物后，皆需要矩阵进行判断。
    #先对没有目标的机器人进行判断，再对有目标的机器人进行判断。
    if all(robots_work):#所有机器人都在工作，不指派。
        return
    #如果存在机器人未工作，则规划其路径。
    to_be_control_get = np.zeros(robot_num, bool)
    to_be_control_pull = np.zeros(robot_num, bool)
    for temp_index, is_work in enumerate(robots_work):#找到无工作机器人，并放入to_be_control_pull和to_be_control_get
        if is_work:#如果正在工作，则不指派
            continue
        
        robot = robots_list[temp_index]
        berth_id = robot.berths_attached
        if berth_id >= 0 and len(berths_list[berth_id].goods[robot.level]):
            if robots_goods[temp_index]:
                updateRobotPull(pibt, robot, zhen)
                continue
            else:
                updateRobotGet(pibt, robot, zhen)
                continue
        
        if not connectBerthTest(robots_pos[temp_index]):
            continue
        logText('d1 ')
        #——————取完货物后
        
            
        #完成：删除good， 给robot路径， 给机器人判断是否前往港口的方法。
        
        if robots_goods[temp_index] and not is_work: #and robots_stop[temp_index]:#如何判断机器人是否需要前往港口？
            if robot.good:#机器人身上有货物
                to_be_control_pull[temp_index] = True
            else:
                robots_goods[temp_index] = False
                if test_mode:
                    logText('wrong ')
        else:
            to_be_control_get[temp_index] = True
            logText('control ')
            
    #先分配垃圾，垃圾分配不了再分配新货物。由于只有2、3个机器人，所以直接循环分配。但是注意，所有机器人一开始都要按此种方式搜索
    goods_leave_judge = assignRobotGet(pibt, to_be_control_get, goods_left, dist_factor, zhen, start_time)
    deleteAssignedGoods(goods_leave_judge, goods_left)
    goods_leave_judge = assignRobotGet(pibt, to_be_control_get, goods_pos, dist_factor, zhen, start_time)
    deleteAssignedGoods(goods_leave_judge, goods_pos)
    #——————在找货物的路上，暂时不考虑。注意恢复时要重新放入goods_berth_dist
    #注意上一步删除了被指派的货物，因此这里也要相应重新生成货物矩阵，或者屏蔽原来的货物矩阵
    if any(to_be_control_pull):#——————判断放货的机器人（注意先后顺序，要先判断在起始点的，因为它们离得近，而且放货需要修改矩阵goods_berth_dist_array
        robot_ids = np.where(to_be_control_pull)[0]
        for temp_index,robot_id in enumerate(robot_ids):
            robot = robots_list[robot_id]
            berth_id = robot.good.berth_id

            robot.des = berths_list[berth_id].side
            pibt.setRobotDistTable(robot_id, berths_dist_tables[berth_id].table, False, True,berths_dist_tables[berth_id].goal, Q=berths_dist_tables[berth_id].Q_road)
            robots_work[robot_id] = True
            #robots_stop[temp_index] = False
def assignRobotGet(pibt:PIBT, to_be_control_get, goods_pos_list, dist_factor, zhen, start_time):
    #构建代价矩阵——————判断在起始点的机器人
    good_num = len(goods_pos_list)
    goods_leave_judge = np.ones(good_num, bool)
    if any(to_be_control_get) and good_num:
        goods_pos_array = np.array(goods_pos_list, np.uint8)
        goods_value_array = np.ndarray(good_num, np.int16)
        goods_time_leave_array = np.ndarray(good_num, np.int16)
        goods_berth_dist_array = np.ndarray(good_num, np.int16)
        goods_berth_id_array = np.ndarray(good_num, np.int16)
        for good_i, good_pos in enumerate(goods_pos_list):
            good = goods_dict[good_pos]
            #————————————————————————————参数调整:有船泊口增益————————————————————————————————————————————————————————————————————————————————
            goods_value_array[good_i] = good.value * getBerthGoodFactor(len(berths_list[good.berth_id].boat_stop))
            goods_time_leave_array[good_i] = valid_time - zhen + good.time
            goods_berth_dist_array[good_i] = good.berth_dist
            goods_berth_id_array[good_i] = good.berth_id
        #收集货物和机器人信息
        robot_ids = np.where(to_be_control_get)[0]
        
        f_value = np.full((len(robot_ids), good_num), UP_LIMIT, np.float16)
        d_value = np.full((len(robot_ids), good_num), UP_LIMIT, np.float16)
        for temp_index,robot_id in enumerate(robot_ids):
            robot_pos = robots_pos[robot_id]
            robot = robots_list[robot_id]
            
            if isBerth(robot_pos, grid_map):
                robot_good_dist_array = berths_dist_tables[road_way_table[robot_pos]].table[goods_pos_array[:,0], goods_pos_array[:,1]]
                d_value[temp_index, :] = goods_berth_dist_array + robot_good_dist_array#真实距离，涉及时间判别，最好不要变动
                #代价函数，可以修改。
                f_value[temp_index, :] = 10*d_value[temp_index, :]/goods_value_array*(30-goods_time_leave_array/100)/30
            else:
                g_dist = (abs(robot_pos[0]-goods_pos_array[:,0]) + abs(robot_pos[1]-goods_pos_array[:,1]))
                d_value[temp_index, :] = g_dist*dist_factor[0] + goods_berth_dist_array#估计距离转精确距离因子
                f_value[temp_index, :] = 10*d_value[temp_index, :]/goods_value_array*(30-goods_time_leave_array/100)/30#注意，使用100乘会发生溢出，要么用浮点数。

        f_value[goods_time_leave_array - d_value<=0] = UP_LIMIT
        goods_leave_judge[goods_time_leave_array<=0] = False

        for temp_index in range(len(robot_ids)):
            temp_index2 = np.argmin(f_value)
            tp_i = temp_index2//good_num#机器人号
            tp_j = temp_index2%good_num#货物号
            if f_value[tp_i, tp_j] >= UP_LIMIT-1:#已经没有可分配的任务。
                break

            good_pos = goods_pos_list[tp_j]
            good = goods_dict.get(good_pos, 0)
            #泊口记录货物
            berths_list[good.berth_id].preserveForGood(zhen + d_value[tp_i, tp_j], good.value)
            
            #分配任务:
            robot_id = robot_ids[tp_i]
            robot = robots_list[robot_id]
            robot.planGood(good_pos, good.value, d_value[tp_i, tp_j])
            robots_work[robot_id] = True
            
            if log_text:
                logText('robot'+str(robot_id)+' work')
            pibt.setRobotDistTable(robot_id, np.full(grid_map.shape, UP_LIMIT, dtype=np.int16), False, True, good_pos, valid_time - zhen + good.time - 10)
            
            to_be_control_get[robot_id] = False
            goods_leave_judge[tp_j] = False
            if temp_index +1 == good_num:
                break
            if time.perf_counter()-start_time>0.012:
                return goods_leave_judge
            #清空该机器人与对应货物
            f_value[tp_i, :] = UP_LIMIT
            f_value[:, tp_j] = UP_LIMIT
    return goods_leave_judge
def deleteAssignedGoods(goods_leave_judge:np.ndarray, goods_pos_list):
    #如果有时间就删除已指派的货物，如果没有时间会在clearGoods中删除
    if any(goods_leave_judge==False):
        for temp_index in np.flip(np.where(goods_leave_judge==False)[0]):
            goods_pos_list.pop(temp_index)
def orderRobots(robots_present_pos, robots_next_pos):
    out_order = []
    num = len(robots_pos)
    for i in range(num):
        robot = robots_list[i]
        present_pos = robots_present_pos[i]
        next_pos = robots_next_pos[i]
        if robots_pos[i] != present_pos:#判断机器人位置是否与之前位置相符
            next_pos = present_pos#注意，这里需要改进！！
            present_pos = robots_pos[i]
            if log_text:
                robots_list[i].log_text += 'notMatch '
            
        #停滞超时则寻找别的货物
        if present_pos == next_pos:# and not robots_stop[i]:
            robot.stop_time += 1
            if robot.stop_time >= max_stop_time and robots_work[i] and not robots_goods[i]:
                good_pos = robot.des
                # if not goods_dict.get(good_pos, 0):#如果货物集中找不到机器人的目标货物位置，则可能是消失了。
                #     continue
                goods_pos.append(good_pos)
                robots_work[i] = False
                robot.stop_time = 0
            continue
        elif robot.stop_time:
            robot.stop_time = 0
        #if robots_work[i]:
        robot_step = (next_pos[0] - present_pos[0],\
                    next_pos[1] - present_pos[1])
        robot_order = MOVE_TO_DIR[robot_step]
        out_order.append("move {} {}".format(i, robot_order))
            # if log_text:
            #     robots_list[i].log_text += 'moveWrong'
        robot_des = robots_list[i].des
        if robots_work[i] and (type(robot_des)==tuple and robot_des==next_pos) or \
        (type(robot_des)==list and next_pos in robot_des):#已经到达目的地
            if (robots_goods[i] == False and isGood(next_pos, goods_dict)):
                out_order.append("get {}".format(i))
                robots_list[i].last_state = ROBOT_STATE_TO_NUM['get']
            elif (robots_goods[i] and isBerth(next_pos, grid_map)):#当前位置有设施...
                if isBerth(next_pos, grid_map):
                    out_order.append("pull {}".format(i))
                    robots_list[i].last_state = ROBOT_STATE_TO_NUM['pull']
            else:#机器人到达目标点但是目标无设施
                robots_work[i] = False
                robots_list[i].last_state = ROBOT_STATE_TO_NUM['free']
                robots_list[i].des = None
                if log_text:
                    robots_list[i].log_text += 'leaveTo '
    return out_order
            #如果达到了目的地，但是目的地为已有物品的berth，则通过assignWork处理
            #如果达到了目的地，但是目的地啥也没有，则通过assignWork处理
#分派机器人泊口
def getLeftRobotNum():
    left_robot_num = 0
    for robot in robots_list:
        if robot.berths_attached < 0:
            left_robot_num += 1
    return left_robot_num
def setRobotBerths(robot_id:int, berth_id:int, frame:int):#每当有机器人送完货时调用，判断是否应当把机器人attach到一个泊口
    #print('set berth')
    robot = robots_list[robot_id]
    if robot.berths_attached >= 0 :
        return

    #查看捡垃圾的机器人有多少个
    left_robot_num = getLeftRobotNum()
    if left_robot_num//berth_num == 0:#没有整数
        #余数捡垃圾。
        pass
        # devide_num = calLCM(leave_robot_num, berth_num)#计算最小公倍数,按最小公倍数来分配。
        # devide_num = devide_num//leave_robot_num#每个机器人分配多少个泊口
        # berth_id = multi_tag_table.getTag(robots_pos)#分配泊口，找到一个泊口和另一个与其最近的泊口？应当从table中获取。如果获取不到就只分配一个泊口？？？
    else:#可以除尽
        robot = robots_list[robot_id]
        berths_flow = [berth.calNetFlowInArea(robots_list, frame) for berth in berths_list]

        if berth_id == np.max(berths_flow):#如果机器人位置是最大净流入的泊口，
            robot.berths_attached = berth_id#将机器人分派到的泊口
            temp_list = berths_list[berth_id].attached_robots
            robot.level = len(temp_list)
            temp_list.append(robot_id)
            #如果是其它位置，不分配，让它捡垃圾。直到流入机器人数最少的泊口。每当机器人放下货物时调用此函数。

    #     dist_list = [berths_dist_tables[berth_id].getValue(berth.side[0]) for berth in berths_list]
    #     dist_list[berth_id] = UP_LIMIT
    #     min_dist_list = np.argmin(dist_list)
    # for robot in robots_list:
    #     if not len(robot.berths_attached):
            
    #         if min(berths_attached_robots) == 0:
                
    #         else:
    #             if map_type == MAP_TYPE_DICT['Average map']:#如果是平均图，则平均分派。按机器人数量来。
    #                 leave_robot_num = robot_num % berth_num#取余
    #                 if leave_robot_num != 0:
    #                     devide_num = calLCM(leave_robot_num, berth_num)#计算最小公倍数,按最小公倍数来分配。
    #                     devide_num = devide_num//leave_robot_num#
    #                     berth_id = multi_tag_table.getTag(robots_pos)#分配泊口，找到一个泊口和另一个与其最近的泊口？应当从table中获取。如果获取不到就只分配一个泊口？？？
                        
    #                     dist_list = [berths_dist_tables[berth_id].getValue(berth.side[0]) for berth in berths_list]
    #                     dist_list[berth_id] = UP_LIMIT
    #                     min_dist_list = np.argmin(dist_list)
                        
    #                 robot.berths_attached.append(np.argmin(berths_attached_robots))
                #指标？
                #泊口范围、泊口剩余工作量、泊口剩余价值、泊口消失货物数（1000帧后）、
                #先按泊口剩余价值来？否，如果是平均图，应当平均分派，如果非平均图，则有所偏倚。
def disConnectRobotBerths():#每回合选取某几个机器人解除连接？
    pass
def calLCM(m, n):#计算最小公倍数
    c = m*n
    if c == 0:
        return 0
    while n!=0:
        rem = m%n
        m, n = n, rem
    return int(c/m)

def updateBerth(zhen):
    for berth_id, berth in enumerate(berths_list):
        dist_list = [berths_dist_tables[berth_id].getValue(pos) for pos in delivery_point]
        berth.trans_dist = min(dist_list)
        berth.trans_id = dist_list.index(berth.trans_dist)
def connectBerthTest(pos, robot = True):
    for i in range(berth_num):#如果berths_dist_tables还没有搜索到当前机器人位置，则直接放弃指派
        if berths_dist_tables[i].getValue(pos, robot) < UP_LIMIT:
            return True
    return False

def updateBoat(pibt:PIBT, frame):
    #处理restore
    for boat_id, boat in enumerate(boats_list):
        #找到船离运送点最小距离
        dist_list = [transp_dist_table.getValue(boat.core_pos, guess=True) for transp_dist_table in transp_dist_tables]
        boat.trans_dist = min(dist_list)
        boat.trans_id = dist_list.index(boat.trans_dist)
        
        #装载货物
        #print(boat.goods_num , boat.last_num)
        in_num = boat.goods_num - boat.last_num
        if in_num != 0:
            berth = berths_list[multi_tag_table.getTag(boat.core_pos, road=False)]
            boat.goods_value += berth.takeOutGoods(in_num)
        boat.last_num = boat.goods_num
        if boat.status == 2:
            boats_restore[boat_id] = -1
            continue
        #状态判断
        if boat.status == 1:
            if boats_restore[boat_id] == -1 or boat.last_order == BOAT_ORDER_TO_NUM['berth'] or boat.last_order == BOAT_ORDER_TO_NUM['dept']:
                if boat.last_order == BOAT_ORDER_TO_NUM['berth'] and boats_restore[boat_id] != -1:
                    boat.last_order = BOAT_ORDER_TO_NUM['free']
                    boats_restore[boat_id] = -1
                    berths_list[road_way_table[boat.core_pos]].boatIn()
                    boats_work[boat.id] = False
                    # boat.last_order = BOAT_ORDER_TO_NUM['free']
                if boat.last_order == BOAT_ORDER_TO_NUM['dept']:#出航后，其boats_restore因为在泊口时为-1，所以必为-1
                    if boat.des not in delivery_point:#如果船舶未改变目标为运输点
                        boat.des = None
                    boat.last_order = BOAT_ORDER_TO_NUM['free']
                    boats_restore[boat_id] = -1
                    berths_list[multi_tag_table.getTag(boat.core_pos, road=False)].boatLeave(boat_id)
                    # boat.last_order = BOAT_ORDER_TO_NUM['free']
            else:
                boats_restore[boat_id] = 1
            continue
        boats_restore[boat_id] = 0
        #两种情况：虚拟点送货，或是到达靠泊区且目标点为泊口
        if boat.des:
            if isTransportPoint(boat.des, grid_map):
                if isTransportPoint(boat.core_pos, grid_map) and (boat.goods_num == 0):#成功送货
                    boat.des = None
                    boat.send_time = frame
                    boats_work[boat.id] = False
                    boat.goods_value = 0
                    pibt.boat_dist_tables[boat_id].to_tranp_point = False
                boat_pos = boats_pos[boat_id]
                if isTransportPoint(boat_pos, grid_map) or isTransportPoint((boat_pos[0]+1, boat_pos[1]), grid_map) or\
                    isTransportPoint((boat_pos[0]+1, boat_pos[1]+1), grid_map) or isTransportPoint((boat_pos[0], boat_pos[1]+1), grid_map):
                    pibt.boat_dist_tables[boat_id].to_tranp_point = True
            #如果船舶在其目标泊口的靠泊区内，则直接令船停止？需要判断其靠泊区内有无其它船舶
            tag = multi_tag_table.getTag(boat.core_pos, road=False)
            if isKArea(boat.core_pos, grid_map) and tag == road_way_table[boat.des]:
                berth = berths_list[tag]
                if not berth.have_boat:
                    boats_restore[boat_id] = 1#注意，这里有所不同！！船舶下一刻就会消失（去泊口）可能应该设置别的状态。
def assignBoatWork(pibt:PIBT, frame_id:int, start_time):#必须实时进行（可能每过10帧进行一次？）
    boat_ids = []
    for boat in boats_list:
        if boat.status == 0:# and not boats_work[boat.id]:#只规划正常行驶状态的船舶
            boat_ids.append(boat.id)
        if boat.status == 2:
            if boat_capacity - boat.goods_num <= 0:#如果船容量为0，则规划
                boat_ids.append(boat.id)
            berth_id = multi_tag_table.getTag(boat.core_pos)
            if berths_list[berth_id].present_goods_num == 0:#如果当前船所在泊口没有货物了，则规划
                boat_ids.append(boat.id)
    
    temp_boat_num = len(boat_ids)
    action_num = berth_num + 2
    if not temp_boat_num:
        return
    
    boats_passed_time = np.ndarray((temp_boat_num, 1), np.int16)
    boats_begin_time = np.ndarray((temp_boat_num, action_num), np.int16)
    boats_true_begin = np.ndarray((temp_boat_num, action_num), np.int16)
    
    boats_goods_value = np.ndarray((temp_boat_num, 1), np.int32)
    boats_left_capacity = np.ndarray((temp_boat_num, 1), np.int16)
    
    berths_goods_value = np.ndarray((temp_boat_num, action_num), np.int32)
    berths_trans_dist = np.ndarray((temp_boat_num, action_num), np.int16)
    berths_goods_get_time = np.ndarray((temp_boat_num, action_num), np.int16)
    for tp_id, boat_id in enumerate(boat_ids):
        boat = boats_list[boat_id]
        left_capacity = int(boat_capacity - boat.goods_num)
        boats_passed_time[tp_id] = frame_id - boat.send_time
        boats_left_capacity[tp_id] = left_capacity
        boats_goods_value[tp_id] = boat.goods_value
        for berth in berths_list:
            berth_id = berth.id
            berths_trans_dist[tp_id, berth_id] = berth.trans_dist
            boats_begin_time[tp_id, berth_id] = berths_dist_tables[berth_id].getValue(boat.core_pos, False) + boats_passed_time[tp_id]
            boats_true_begin[tp_id, berth_id], begin_num = berth.testBoatTime(boat_id, boats_begin_time[tp_id, berth_id], frame_id)
            berths_goods_value[tp_id, berth_id], berths_goods_get_time[tp_id, berth_id] = berth.calSimpleGetRate(left_capacity, begin_num)
        #归航行动
        action_id = berth_num
        trans_dist_factor = getBackFactor(boat.goods_value, frame_id)
        berths_goods_get_time[tp_id, action_id] = 0
        berths_goods_value[tp_id, action_id] = 0
        boats_true_begin[tp_id, action_id] = boats_passed_time[tp_id]*trans_dist_factor
        berths_trans_dist[tp_id, action_id] = boat.trans_dist*trans_dist_factor
        if boat.goods_value == 0:
            boats_begin_time[tp_id, action_id] = UP_LIMIT
        if left_capacity == 0:
            berths_trans_dist[tp_id, action_id] = 0
        #停泊行动
        boat = boats_list[boat_id]
        action_id = berth_num + 1
        if boat.status == 2:#如果船当前在泊口, 且泊口货已尽？那等待者也没必要等待了？所以可以默认没有等待者？
            berth_id = road_way_table[boat.core_pos]
            berth = berths_list[berth_id]
            action_id = berth_num + 1
            get_time_per, value_per =  berth.calVacancyRate(frame_id)
            berths_goods_get_time[tp_id, action_id] = get_time_per*boats_left_capacity[tp_id]
            berths_goods_value[tp_id, action_id] = value_per*boats_left_capacity[tp_id]#可以忽略具体取货时间
            boats_begin_time[tp_id, action_id] = boats_passed_time[tp_id]
            boats_true_begin[tp_id, action_id] = boats_passed_time[tp_id]
            berths_trans_dist[tp_id, action_id] = boat.trans_dist
        else:
            berths_goods_get_time[tp_id, action_id] = 0
            berths_goods_value[tp_id, action_id] = 0
            boats_true_begin[tp_id, action_id] = boats_passed_time[tp_id]
            boats_begin_time[tp_id, action_id] = UP_LIMIT
            berths_trans_dist[tp_id, action_id] = boat.trans_dist
            
    f_value = (boats_true_begin + berths_goods_get_time + berths_trans_dist+10)/(boats_goods_value + berths_goods_value+1)#不可匈牙利,需要计算价值增值量？首先判断是否需要回航？
    f_value[boats_begin_time>=UP_LIMIT] = UP_LIMIT
    logText(f_value)
    

    for boat_id in boat_ids:#有可能串货，但是可以忍受。
        temp_index2 = np.argmin(f_value)
        tp_i = temp_index2//action_num#船舶号
        tp_j = temp_index2%action_num#泊口号
        if f_value[tp_i, tp_j] >= UP_LIMIT-1:#已经没有可分配的任务。
            break
        
        boat_id = boat_ids[tp_i]
        boat = boats_list[boat_id]
        if tp_j == berth_num:#回泊命令
            logText('back boat '+str(boat_id))
            f_value[tp_i, :] = UP_LIMIT
            boat.des = delivery_point[boat.trans_id]
            pibt.setBoatDistTable(boat_id, transp_dist_tables[boat.trans_id], False, True)
            if boat.preserve >= 0:#如果船舶改变目标。
                temp_berth = berths_list[boat.preserve]
                boat.preserve = -1
                logText('cancel')
                if not temp_berth.cancelForBoat(boat_id):#取消之前的预定。
                    logText('error_cancel')
            continue
        if tp_j == berth_num + 1:#在当前泊口等待命令,即不命令船舶
            logText('boat wait'+str(boat_id))
            f_value[tp_i, :] = UP_LIMIT
            continue
        
        berth = berths_list[tp_j]
        if boat.preserve >= 0:#如果船舶改变目标。
            if boat.preserve == tp_j:#分配的目标为其原目标点
                f_value[tp_i, :] = UP_LIMIT#设置该船舶分配完毕
                continue
            temp_berth = berths_list[boat.preserve]
            boat.preserve = -1
            logText('cancel')
            if not temp_berth.cancelForBoat(boat_id):#取消之前的预定。
                logText('error_cancel')
        berth.preserveForBoat(boat_id ,boats_true_begin[tp_i, tp_j], \
                berths_goods_get_time[tp_i, tp_j], boats_left_capacity[tp_i, 0], frame_id)
        boat.preserve = tp_j
        boat.des = berth.side[0]
        if log_text:
            logText('boat'+str(boat_id)+' work berth '+str(berth.id))
        #dist_table.preGet(robots_pos[robot_id], 300)
        pibt.setBoatDistTable(boat_id, berths_dist_tables[tp_j], False, True)
        #boats_work[boat_id] = True
        if time.perf_counter()-start_time>0.012:
            return
        
        #更新f_value
        f_value[tp_i, :] = UP_LIMIT
        for tp_id, left_capacity in enumerate(boats_left_capacity):
            if f_value[tp_id, tp_j]>=UP_LIMIT:#如果该处无法获取，则直接跳过
                continue
            
            boats_true_begin[tp_id, tp_j], begin_num = berth.testBoatTime(tp_id, boats_begin_time[tp_id, tp_j], frame_id)#找到起始时间
            berths_goods_value[tp_id, tp_j], berths_goods_get_time[tp_id, tp_j] = berth.calSimpleGetRate(left_capacity, begin_num)
            f_value[tp_id, tp_j] = (boats_true_begin[tp_id, tp_j] + berths_goods_get_time[tp_id, tp_j] + berths_trans_dist[tp_id, tp_j])\
                /(boats_goods_value[tp_id] + berths_goods_value[tp_id, tp_j])#更新f_value
def orderBoats(boats_present_pos, boats_next_pos, frame_id):
    out_orders = []
    berth_put = [False for _ in range(berth_num)]
    for boat_id, boat in enumerate(boats_list):
        if boat.status == 1:#船舶若处于恢复状态，则跳过不命令。
            continue
        
        #是否应该进泊口的判断
        if boat.des:
            tag = road_way_table[boat.des]
            if isKArea(boat.core_pos, grid_map) and multi_tag_table.getTag(boat.core_pos, road=False) == tag:
                if (not berths_list[tag].have_boat) and (not berth_put[tag]):
                    out_orders.append('berth {}'.format(boat.id))
                    boat.last_order = BOAT_ORDER_TO_NUM['berth']
                    berth_put[tag] = True
                else:
                    pass#不命令，让船指派时船自己决定是否换路。
                continue
        if boat.status == 2:
            berth_id = multi_tag_table.getTag(boat.core_pos)
            if boat.des != berths_list[berth_id].side[0]:#如果船的目标不为当前泊口id,则出航。
                out_orders.append('dept {}'.format(boat.id))
                boat.last_order = BOAT_ORDER_TO_NUM['dept']
                
            # berth_id = multi_tag_table.getTag(boat.core_pos)
            # if berths_list[berth_id].present_goods_num == 0:
            #     out_orders.append('dept {}'.format(boat.id))
            #     boat.last_order = BOAT_ORDER_TO_NUM['dept']

        present_pos, next_pos = boats_present_pos[boat_id], boats_next_pos[boat_id]
        if present_pos==next_pos:
            continue
        move_pos = (next_pos[0] - present_pos[0], next_pos[1] - present_pos[1])
        dir = boats_dir[boat_id]
        if move_pos == DIR_TO_MOVE[dir]:
            out_orders.append('ship {}'.format(boat_id))
        if move_pos == rotateClockwise(dir)[0]:
            out_orders.append('rot {} {}'.format(boat_id, 0))
        if move_pos == rotateAnticlockwise(dir)[0]:
            out_orders.append('rot {} {}'.format(boat_id, 1))
    return out_orders

def assignBoatsWork(pibt:PIBT, boat_ids:list, frame_id:int, start_time):#必须实时进行（可能每过10帧进行一次？）
    temp_boat_num = len(boat_ids)
    action_num = berth_num + 2
    if not temp_boat_num:
        return
    
    boats_passed_time = np.ndarray((temp_boat_num, 1), np.int16)
    boats_begin_time = np.ndarray((temp_boat_num, action_num), np.int16)
    boats_true_begin = np.ndarray((temp_boat_num, action_num), np.int16)
    
    boats_goods_value = np.ndarray((temp_boat_num, 1), np.int32)
    boats_left_capacity = np.ndarray((temp_boat_num, 1), np.int16)
    
    berths_goods_value = np.ndarray((temp_boat_num, action_num), np.int32)
    berths_trans_dist = np.ndarray((temp_boat_num, action_num), np.int16)
    berths_goods_get_time = np.ndarray((temp_boat_num, action_num), np.int16)
    for tp_id, boat_id in enumerate(boat_ids):
        boat = boats_list[boat_id]
        left_capacity = int(boat_capacity - boat.goods_num)
        boats_passed_time[tp_id] = frame_id - boat.send_time
        boats_left_capacity[tp_id] = left_capacity
        boats_goods_value[tp_id] = boat.goods_value
        for berth in berths_list:
            berth_id = berth.id
            berths_trans_dist[tp_id, berth_id] = berth.trans_dist
            boats_begin_time[tp_id, berth_id] = berths_dist_tables[berth_id].getValue(boat.core_pos, False) + boats_passed_time[tp_id]
            boats_true_begin[tp_id, berth_id], begin_num = berth.testBoatTime(boat_id, boats_begin_time[tp_id, berth_id], frame_id)
            berths_goods_value[tp_id, berth_id], berths_goods_get_time[tp_id, berth_id] = berth.calSimpleGetRate(left_capacity, begin_num)
        #归航行动
        action_id = berth_num
        trans_dist_factor = getBackFactor(boat.goods_value, frame_id)
        berths_goods_get_time[tp_id, action_id] = 0
        berths_goods_value[tp_id, action_id] = 0
        boats_true_begin[tp_id, action_id] = boats_passed_time[tp_id]*trans_dist_factor
        berths_trans_dist[tp_id, action_id] = boat.trans_dist*trans_dist_factor
        if boat.good_value == 0:
            boats_begin_time[tp_id, action_id] = UP_LIMIT
        if left_capacity == 0:
            berths_trans_dist[tp_id, action_id] = 0
        #停泊行动
        boat = boats_list[boat_id]
        action_id = berth_num + 1
        if boat.status == 2:#如果船当前在泊口, 且泊口货已尽？那等待者也没必要等待了？所以可以默认没有等待者？
            berth_id = road_way_table[boat.core_pos]
            berth = berths_list[berth_id]
            action_id = berth_num + 1
            get_time_per, value_per =  berth.calVacancyRate(frame_id)
            berths_goods_get_time[tp_id, action_id] = get_time_per*boats_left_capacity[tp_id]
            berths_goods_value[tp_id, action_id] = value_per*boats_left_capacity[tp_id]#可以忽略具体取货时间
            boats_begin_time[tp_id, action_id] = boats_passed_time[tp_id]
            boats_true_begin[tp_id, action_id] = boats_passed_time[tp_id]
            berths_trans_dist[tp_id, action_id] = boat.trans_dist
        else:
            berths_goods_get_time[tp_id, action_id] = 0
            berths_goods_value[tp_id, action_id] = 0
            boats_true_begin[tp_id, action_id] = boats_passed_time[tp_id]
            boats_begin_time[tp_id, action_id] = UP_LIMIT
            berths_trans_dist[tp_id, action_id] = boat.trans_dist
            
    f_value = (boats_true_begin + berths_goods_get_time + berths_trans_dist+10)/(boats_goods_value + berths_goods_value+1)#不可匈牙利,需要计算价值增值量？首先判断是否需要回航？
    f_value[boats_begin_time>=UP_LIMIT] = UP_LIMIT
    logText(f_value)

    for boat_id in boat_ids:#有可能串货，但是可以忍受。
        temp_index2 = np.argmin(f_value)
        tp_i = temp_index2//action_num#船舶号
        tp_j = temp_index2%action_num#泊口号
        if f_value[tp_i, tp_j] >= UP_LIMIT-1:#已经没有可分配的任务。
            break
        
        boat_id = boat_ids[tp_i]
        boat = boats_list[boat_id]
        if tp_j == berth_num:#回泊命令
            logText('back boat '+str(boat_id))
            f_value[tp_i, :] = UP_LIMIT
            boat.des = delivery_point[boat.trans_id]
            pibt.setBoatDistTable(boat_id, transp_dist_tables[boat.trans_id], False, True)
            if boat.preserve >= 0:#如果船舶改变目标。
                temp_berth = berths_list[boat.preserve]
                boat.preserve = -1
                logText('cancel')
                if not temp_berth.cancelForBoat(boat_id):#取消之前的预定。
                    logText('error_cancel')
            continue
        if tp_j == berth_num + 1:#在当前泊口等待命令,即不命令船舶
            logText('boat wait'+str(boat_id))
            f_value[tp_i, :] = UP_LIMIT
            continue
        
        berth = berths_list[tp_j]
        if boat.preserve >= 0:#如果船舶改变目标。
            if boat.preserve == tp_j:#分配的目标为其原目标点
                f_value[tp_i, :] = UP_LIMIT#设置该船舶分配完毕
                continue
            temp_berth = berths_list[boat.preserve]
            boat.preserve = -1
            logText('cancel')
            if not temp_berth.cancelForBoat(boat_id):#取消之前的预定。
                logText('error_cancel')
        berth.preserveForBoat(boat_id ,boats_true_begin[tp_i, tp_j], \
                berths_goods_get_time[tp_i, tp_j], boats_left_capacity[tp_i, 0], frame_id)
        boat.preserve = tp_j
        boat.des = berth.side[0]
        if log_text:
            logText('boat'+str(boat_id)+' work berth '+str(berth.id))
        #dist_table.preGet(robots_pos[robot_id], 300)
        pibt.setBoatDistTable(boat_id, berths_dist_tables[tp_j], False, True)
        #boats_work[boat_id] = True
        if time.perf_counter()-start_time>0.012:
            return
        
        #更新f_value
        f_value[tp_i, :] = UP_LIMIT
        for tp_id, left_capacity in enumerate(boats_left_capacity):
            if f_value[tp_id, tp_j]>=UP_LIMIT:#如果该处无法获取，则直接跳过
                continue
            
            boats_true_begin[tp_id, tp_j], begin_num = berth.testBoatTime(tp_id, boats_begin_time[tp_id, tp_j], frame_id)#找到起始时间
            berths_goods_value[tp_id, tp_j], berths_goods_get_time[tp_id, tp_j] = berth.calSimpleGetRate(left_capacity, begin_num)
            f_value[tp_id, tp_j] = (boats_true_begin[tp_id, tp_j] + berths_goods_get_time[tp_id, tp_j] + berths_trans_dist[tp_id, tp_j])\
                /(boats_goods_value[tp_id] + berths_goods_value[tp_id, tp_j])#更新f_value
def getBoatsIds():#计算量富裕情况下，对于所有未在恢复状态船规划
    boat_ids = []
    for boat in boats_list:
        if boat.status == 0:# and not boats_work[boat.id]:#只规划正常行驶状态的船舶
            boat_ids.append(boat.id)
        if boat.status == 2:
            if boat_capacity - boat.goods_num <= 0:#如果船容量为0，则规划
                boat_ids.append(boat.id)
            berth_id = multi_tag_table.getTag(boat.core_pos)
            if berths_list[berth_id].present_goods_num == 0:#如果当前船所在泊口没有货物了，则规划
                boat_ids.append(boat.id)
    return boat_ids
def getFewBoatsIds():
    boat_ids = []
    for boat in boats_list:
        if boat.status == 0 and not boat.des:#如果船舶没有目标
            boat_ids.append(boat.id)
        if boat.status == 2:
            if boat_capacity - boat.goods_num <= 0:#如果船容量为0，则规划
                boat_ids.append(boat.id)
            berth_id = multi_tag_table.getTag(boat.core_pos)
            if berths_list[berth_id].present_goods_num == 0:#如果当前船所在泊口没有货物了，则规划
                boat_ids.append(boat.id)
    return boat_ids


def detectMapType():#用于检测地图类型。譬如：泊口陆路平均，但是船舶距离不平均的图（堆货时优先堆船舶离运输点近者，但是取货速度又可能受影响，需要综合指标）
    pass

#分派任务、发布指令函数
def setFree(no_way:list):
    for i in no_way:
        robots_work[i] = False
        robots_list[i].des = None
        robots_list[i].log_text += 'setFree '

#购买机器人函数
def updatePurchasePoint():#仅更新tag#注意当前未考虑海域
    for  purchase_point in enumerate(robot_purchase_points):
        purchase_pos = purchase_point.pos
        '''if not purchase_point.berths_dist_ready:#更新与泊口距离
            purchase_berths_dist = purchase_point.berths_dist
            for berth_id, berth_table in enumerate(berths_dist_tables):
                purchase_berths_dist[berth_id] = berth_table.getValue(purchase_pos)
            if np.max(purchase_berths_dist)<UP_LIMIT:
                purchase_point.berths_dist_ready = True'''
        
        if not purchase_point.tag_ready:#更新所属泊口范围
            tag = multi_tag_table.getTag(purchase_pos)
            if tag >= 0:
                purchase_point.tag = tag
                purchase_point.tag_ready = True
                
    for purchase_point in enumerate(boat_purchase_points):
        purchase_pos = purchase_point.pos
        '''if not purchase_point.berths_dist_ready:#更新与泊口距离
            purchase_berths_dist = purchase_point.berths_dist
            for berth_id, berth_table in enumerate(berths_dist_tables):
                purchase_berths_dist[berth_id] = berth_table.getValue(purchase_pos, road=False)
            if np.max(purchase_berths_dist)<UP_LIMIT:
                purchase_point.berths_dist_ready = True'''
        
        # if not purchase_point.tag_ready:#更新所属泊口范围
        #     tag = multi_tag_table.getTag(purchase_pos)
        #     if tag >= 0:
        #         purchase_point.tag = tag
        #         purchase_point.tag_ready = True
def purchaseRobots(num, pos:tuple=None, berth_id:int=-1, tag:int=-1, purchase_point_id:int=-1) -> list:
    '''
    way1:一个与pos最近的连通区域
    way2:在距离泊口id最近区域内买num个机器人
    way3:在任意与tag相同区域内买num个机器人
    way4:在第id个购买点处
    如果按多种方式，则顺序：pos->berth_id->tag->purchase_point_id
    输出->List[str]
    '''
    out_orders = []
    
    if pos:#考虑：连通的，最近的。连通通过对两方面的距离查询来保证，最近通过估计距离计算来保证
        guess_dist = np.full(len(robot_purchase_points), UP_LIMIT)
        for point_id, purchase_point in enumerate(robot_purchase_points):
            purchase_pos = purchase_point.pos
            
            connect_flag = False
            for berth_table in berths_dist_tables:
                if berth_table.getValue(pos) < UP_LIMIT and berth_table.getValue(purchase_pos) < UP_LIMIT:#证明pos和purchase_point连通性
                    connect_flag = True
                    break

            if connect_flag:
                guess_dist[point_id] = abs(purchase_pos[0]-pos[0]) + abs(purchase_pos[1]-pos[1])#记录估计距离
                
        purchase_point_id = np.argmin(guess_dist)
        if guess_dist[purchase_point_id] < UP_LIMIT:
            purchase_pos = robot_purchase_points[purchase_point_id].pos
            for _ in range(num):
                out_orders.append('lbot {} {}'.format(purchase_pos[0], purchase_pos[1]))
            return out_orders

    if berth_id>=0:
        purchase_pos = None
        berth_dist = UP_LIMIT
        berth_table = berths_dist_tables[berth_id]
        for point_id, purchase_point in enumerate(robot_purchase_points):
            temp_purchase_pos = purchase_point.pos
            temp_berth_dist = berth_table.getValue(temp_purchase_pos)
            if temp_berth_dist < berth_dist:
                berth_dist = temp_berth_dist
                purchase_pos = temp_purchase_pos
        if berth_dist < UP_LIMIT:
            for _ in range(num):
                out_orders.append('lbot {} {}'.format(purchase_pos[0], purchase_pos[1]))
            return out_orders
        
    if tag>=0:#通过tag来获取
        for point_id, purchase_point in enumerate(robot_purchase_points):
            if purchase_point.tag == tag:
                purchase_pos = purchase_point.pos
                for _ in range(num):
                    out_orders.append('lbot {} {}'.format(purchase_pos[0], purchase_pos[1]))
                return out_orders

    if purchase_point_id>=0:#通过id来获取
        purchase_pos = robot_purchase_points[purchase_point_id].pos
        for _ in range(num):
            out_orders.append('lbot {} {}'.format(purchase_pos[0], purchase_pos[1]))
        return out_orders
            
    return None#找不到满足要求的购买点
def purchaseBoats(pos:tuple=None, berth_id:int=-1, purchase_point_id:int=-1) -> str:#注意需要先让multiTable能够搜索海域才能使用tag搜索
    '''
    #根据判题器测试，可以在不同购买点买两艘船，但是不能在同一购买点购买两艘船
    '''
    out_orders = []
    
    if pos:#考虑：连通的，最近的。连通通过对两方面的距离查询来保证，最近通过估计距离计算来保证
        guess_dist = np.full(len(boat_purchase_points), UP_LIMIT)
        for point_id, purchase_point in enumerate(boat_purchase_points):
            purchase_pos = purchase_point.pos
            
            connect_flag = False
            for berth_table in berths_dist_tables:
                if berth_table.getValue(pos, road=False) < UP_LIMIT and berth_table.getValue(purchase_pos, road=False) < UP_LIMIT:#证明pos和purchase_point连通性
                    connect_flag = True
                    break

            if connect_flag:
                guess_dist[point_id] = abs(purchase_pos[0]-pos[0]) + abs(purchase_pos[1]-pos[1])#记录估计距离
                
        purchase_point_id = np.argmin(guess_dist)
        if guess_dist[purchase_point_id] < UP_LIMIT:
            purchase_pos = boat_purchase_points[purchase_point_id].pos
            out_orders.append('lboat {} {}'.format(purchase_pos[0], purchase_pos[1]))
            return out_orders
        
    if berth_id>=0:
        purchase_pos = None
        berth_dist = UP_LIMIT
        berth_table = berths_dist_tables[berth_id]
        for point_id, purchase_point in enumerate(boat_purchase_points):
            temp_purchase_pos = purchase_point.pos
            temp_berth_dist = berth_table.getValue(temp_purchase_pos)
            if temp_berth_dist < berth_dist:
                berth_dist = temp_berth_dist
                purchase_pos = temp_purchase_pos
        if berth_dist < UP_LIMIT:
            out_orders.append('lboat {} {}'.format(purchase_pos[0], purchase_pos[1]))
            return out_orders

    if purchase_point_id>=0:#通过id来获取
        purchase_pos = boat_purchase_points[purchase_point_id].pos
        out_orders.append('lboat {} {}'.format(purchase_pos[0], purchase_pos[1]))
        return out_orders
            
    return None#找不到满足要求的购买点
    pass
def purchaseControl():
    
    pass
def initPurchaseRobotsBoats():#初始情况下购买机器人和船. 8个机器人，1艘船
    out_orders = []
    num = 8
    while num > 0:
        for i in range(len(robot_purchase_points)):
            if num > 0:
                append_orders = purchaseRobots(1, purchase_point_id=i)
                if append_orders:
                    out_orders.extend(append_orders)
                num -= 1
    
    berth_id = np.argmax(multi_tag_table.tag_road_range)
    out_orders.extend(purchaseBoats(berth_id=berth_id, purchase_point_id=0))
    #out_orders.extend(purchaseBoats(berth_id=berth_id, purchase_point_id=1))
    return out_orders

def purchaseRobotsBoats(frame):
    pass

def restTimeKill(pibt, start_time):#剩余时间利用
    while time.perf_counter() - start_time < 0.014:#剩余时间利用，搜寻机器人货物table
            flag = True
            for item in range(robot_num):
                table = pibt.robot_dist_tables[item]
                #print(table.work)
                if table.work:
                    if not table.ready:
                        flag = False
                        if table.preGet(robots_pos[item]):
                            table.setReady(True)
            if flag:
                break
    while time.perf_counter() - start_time < 0.014:#搜寻泊口table
        for item in berths_dist_tables:
            item.preGet()
            item.preGet(road=False)#搜寻海域
            if (time.perf_counter() - start_time) > 0.014:
                break
        for item in transp_dist_tables:#搜索泊口表
            item.preGet()
        if (time.perf_counter() - start_time) > 0.014:
            break
        for temp_tag in range(berth_num):#搜寻multiTagTable
            multi_tag_table.growTag(temp_tag)
            multi_tag_table.growTag(temp_tag, False)
            if (time.perf_counter() - start_time) > 0.014:
                break
def purchaseRobotsBoats(frame:int, money:int):#b_cycle为船运行一趟所需时间
    out_orders = []
    if money<2000 or frame>=finally_buy_frame:
        return []
    if robot_num<robot_limit and robot_num//boat_num<=4:
        while robot_num//boat_num<=4:
            if money>=robot_price:
                out_orders.extend(purchaseRobots(1, purchase_point_id=np.random.randint(0, len(robot_purchase_points))))
                money-=robot_price
            else:
                #flow.bot_agree = True
                return out_orders
    if boat_num<boat_limit:
        if money>=boat_price:
            out_orders.extend(purchaseBoats(purchase_point_id=np.random.randint(0, len(boat_purchase_points))))
            money-=boat_price
        else:
            #flow.boat_agree = True
            return out_orders
    return out_orders

if __name__ == "__main__":
    dist_factor = (2,3)
    if test_mode:
        handle = win32gui.GetWindowText(win32gui.GetForegroundWindow())#获取vscode句柄
        app = QApplication(sys.argv)
    start_time = time.perf_counter()
    pibt = PIBT(grid_map)
    #Init(start_time)
    if test_mode:#初始化
        testInit(start_time, 1.5,2.5)
        figure, ax = initImage()
        #初始化用于显示信息的界面、执行的次数参数、是按信息界面输入的指令执行还是按程序得到的结果执行
        order_window, order_times, just_go = TableTips(handle), 1, False 
    else:
        Init(start_time)
    start_time = time.perf_counter()
    orders, robots_next_pos, boats_next_pos = [], [], []#命令集
    orders.extend(initPurchaseRobotsBoats())#购买初始机器人和船
    for zhen in range(1, 15001):
        if test_mode:#接收数据
            testInput(orders)
            frame_id, money = global_frame, global_money
        else:
            frame_id, money = map(int, input().split(" "))
            Input(frame_id)
        start_time = time.perf_counter()#开始计时
        orders = []#清空指令集
        if zhen == 1:
            orders.extend(initPurchaseRobotsBoats())
        updateGoods()
        distributGoods()#分派货物到泊口集中
        assignGoodsForAttached(frame_id, frame_id%berth_num)
        updateBerth(frame_id)
        #机器人规划
        end_time = time.perf_counter()
        updateRobot(pibt, frame_id)
        assignRobotWork(pibt, grid_map, dist_factor, frame_id, start_time)
        robots_next_pos = pibt.runRobotStep(robots_pos ,robots_work)
        orders.extend(orderRobots(robots_pos, robots_next_pos))
        #print(time.perf_counter() - end_time)
        #船规划
        end_time = time.perf_counter()
        updateBoat(pibt, frame_id)
        assignBoatWork(pibt, frame_id, start_time)
        boats_next_pos = pibt.runBoatStep(boats_pos, boats_dir, boats_restore, [boat.des for boat in boats_list])
        orders.extend(orderBoats(boats_pos, boats_next_pos, frame_id))
        #print(time.perf_counter() - end_time)
        
        # if len(boats_list)<3:
        #     orders.extend(purchaseBoats(purchase_point_id=0))
        #     orders.extend(purchaseBoats(purchase_point_id=1))#购买初始机器人和船
        
        '''        # updateBoats()
        # updateGoods()
        # updateRobot()
        # goods_left = assignWork2(pibt, grid_map, dist_factor, id, start_time)
        # deleteAssignedGoods(goods_left)
        # robots_present_pos = robots_pos.copy() if zhen==1 else robots_next_pos
        # robots_next_pos, no_way = pibt.runStep(robots_pos ,robots_work)
        # setFree(no_way)

        # orderRobots(id, robots_present_pos, robots_next_pos)
        # if id < 14900:
        #     orderShip(id)
        # while time.perf_counter() - start_time < 0.014:#剩余时间利用，搜寻机器人货物table
        #     flag = True
        #     for item in range(robot_num):
        #         table = pibt.dist_tables[item]
        #         #print(table.work)
        #         if not table.ready:
        #             flag = False
        #             if table.preGet(robots_pos[item]):
        #                 table.setReady(True)
        #     if flag:
        #         break'''
        restTimeKill(pibt, start_time)#剩余时间利用
        if frame_id%purchase_interval == 3:
            orders.extend(purchaseRobotsBoats(frame_id, money))
        if test_mode:
            logText(orders)
            textDisplay(frame_id, money)#打印在DisplayText的信息
            drawImage(robots_pos, boats_pos, boats_dir.copy(), goods_pos)#绘制代理和货物
            
            orders, order_times, just_go = orderWindowProcess(orders, order_times, just_go)#命令窗口处理
            clearLogText()
        for order in orders:
            print(order)
            sys.stdout.flush()
        print("OK")
        sys.stdout.flush()
    if test_mode:
        app.exec()
    #app.exec()