import numpy as np
from pibt import UP_LIMIT

class Robot:
    def __init__(self, id):
        self.id = id
        self.des = None
        # self.status = status#碰撞状态
        self.last_state = 0#上一刻的行动。0为无行动，1为
        self.last_berth = -1#机器人上一刻所在的港口位置。注意两个机器人在同一个港口则应该相互排斥？
        
        self.good = None#只有当机器人取到货时才会有
        #self.next_good = None
        self.stop_time = 0#用于判断机器人停止在当前位置多久了，是否应该为机器人分配新任务
        
        self.guess_pull_time = 0
        self.berths_attached = -1#机器人接取货物的泊口(或机器人放置货物的泊口，机器人只能在这些地点放置货物)
        self.level = 0
        self.full = False#机器人有没有装满固定帧的货物(300帧-400帧)
        
        self.log_text = ''
        
        self.trans_value = []
        self.trans_time = []
    def calFlow(self):
        return sum(self.trans_value)/sum(self.trans_time)
    def planGood(self, good_pos, good_value, get_time):#记录最近的20个货物计算流。
        self.trans_value.append(good_value)
        self.trans_time.append(get_time)
        if len(self.trans_value)>20:
            self.trans_time.pop(0)
            self.trans_value.pop(0)
        self.des = good_pos
class Berth:
    def __init__(self, id, berthing_pos, loading_speed=0):
        self.id = id
        #self.transport_time = transport_time
        self.loading_speed = loading_speed#加载速度
        self.side = [berthing_pos]#停泊位置
        self.trans_dist = UP_LIMIT#与运输点的距离
        self.trans_id = 0#运输点id
        
        self.guess_arrive_time = []#预计到达时间，当已经到达后删除（或设置为最大时间帧）。
        self.guess_arrive_value = []
        self.attached_robots = []#固定在此处取货的机器人
        self.goods = [[]]#记录泊口tag范围的货物,但是分级记录。垃圾会流到self.goods[-1]
        self.update_goods = False#更新货物标志？
        
        self.boat_stop = []
        self.boat_stop_ids = []
        self.boat_pre_num = []
        self.present_goods_num:int = 0
        self.have_boat = False
        #存储流入泊口范围价值
        self.goods_frame = []
        self.goods_value = []
        self.goods_frame = []
        #用于计算泊口的空闲率（运输效率？）
        self.all_goods_frame = []
        self.all_goods_val = []
        self.boat_capacity = -1

    def preserveForBoat(self, boat_id:int, begin:int, length:int, num:int, frame_id:int):#按顺序排入区间
        begin += frame_id
        end = begin + length + frame_id
        insert_index = 0
        for i in self.boat_stop:
            if i[1]-1 <= begin:
                insert_index += 1
            else:
                break
        self.boat_stop.insert(insert_index, (begin, end))
        self.boat_stop_ids.insert(insert_index, boat_id)
        self.boat_pre_num.insert(insert_index, num)
    def cancelForBoat(self, boat_id:int):#取消预定
        if boat_id in self.boat_stop_ids:
            index = self.boat_stop_ids.index(boat_id)
        else:
            index = -1
        if index<0:
            return False
        
        self.boat_stop.pop(index)
        self.boat_stop_ids.pop(index)
        self.boat_pre_num.pop(index)
        return True
    '''# def testBoatTime(self, begin, length, frame_id):#计算安排时间和当前已有安排的冲突
    #     begin += frame_id
    #     end = begin + length + frame_id
    #     insert_index_b = 0
    #     insert_index_e = 0
    #     for i in self.boat_stop:
    #         if i[1] < begin:
    #             insert_index_b += 1
    #         if i[0] < end:
    #             insert_index_e += 1
    #     if insert_index_b != insert_index_e or self.boat_stop[insert_index_b][0]>begin:#如果区间发生冲突
    #         return self.boat_stop[insert_index_b][1]#安排在冲突区间后方,返回开始时间。开始时间需要减去帧时间。'''
    def testBoatTime(self, i, begin:int, frame_id:int):#计算安排时间和当前已有安排的冲突，向后排算法
        if begin>=UP_LIMIT:
            # if type(self.present_goods_num) != int:
            #     print('pren', self.present_goods_num)
            return (begin, self.present_goods_num+1)
        
        if i in self.boat_stop_ids:
            index = self.boat_stop_ids.index(i)
        else:
            index = -1
        if index>=0:#如果已有安排，应当测试前面的家伙有没有跑？
            begin_num = sum(self.boat_pre_num[:index])+1
            begin_time = self.boat_stop[index][0]
            #print('pn', begin_num)
            # if type(begin_num) != int:
            #     print(self.boat_pre_num)
            #     print(index)
            #     print(self.boat_pre_num[:index])
            #     print('begi', begin_num)
            return (begin_time - frame_id, begin_num)
        
        begin += frame_id
        insert_index = 0
        for i in self.boat_stop:
            if i[1] < begin:
                insert_index += 1
            else:
                begin = i[1]
        #print('pn', sum(self.boat_pre_num[0:0]))
        # if type(sum(self.boat_pre_num[insert_index:])) != int:
        #     print('be--', sum(self.boat_pre_num[insert_index:]))
        return (begin - frame_id, sum(self.boat_pre_num[insert_index:])+1)#返回开始取货时间、取货量。

    def assignGood(self, pos, level):#放入泊口区域内货物
        self.goods[level].append(pos)
    
    def areaGoodArise(self, frame, value):
        self.goods_value.append(value)
        self.goods_frame.append(frame)
        if len(self.goods_value)>30:
            self.goods_value.pop(0)
            self.goods_frame.pop(0)
    def calFlowInArea(self, frame):#计算价值流，价值/时间
        return sum(self.goods_value)/(frame-self.goods_frame[0])
    def calNetFlowInArea(self, robot_list:list, frame:int):#计算价值流，价值/时间
        flow_in_berth = 0
        for robot_id in self.attached_robots:
            flow_in_berth += robot_list[robot_id].calFlow()
        if len(self.goods_frame):
            return sum(self.goods_value)/(frame-self.goods_frame[0])-flow_in_berth
        return sum(self.goods_value)/frame-flow_in_berth
    def calVacancyRate(self, frame):#如果无法计算，默认5000，输出货物量+价值。价值如何估计？用最近的30个货物来计算空置率？
        num = len(self.all_goods_frame)
        if not num:
            return 1000, 0
        while len(self.all_goods_frame)>30:
            self.all_goods_frame.pop(0)
            self.all_goods_val.pop(0)
        num = len(self.all_goods_frame)
        return (frame - self.all_goods_frame[0])/num, sum(self.all_goods_val)/num
        
    def calGetRate(self, frame, travel_time, good_n_limit:int = 20, append=False):#增加了空置率的估计进货。
        self.guess_arrive_time.sort()
        guess_goods_num = len(self.guess_arrive_time)
        
        good_n = self.present_goods_num
        end_time = travel_time+frame
        able_n = 0
        temp_n = 0
        good_append_n = 0

        if guess_goods_num>0 and good_n<good_n_limit:
            for i in range(0, guess_goods_num):
                if end_time < self.guess_arrive_time[i]:
                    while end_time < self.guess_arrive_time[i] and good_n - able_n >= 0:
                        end_time += self.loading_speed
                        able_n += 1
                    end_time = self.guess_arrive_time[i]
                able_n -= 1
                temp_n += 1
                if good_n + temp_n>=good_n_limit:
                    break
            if append and end_time > self.guess_arrive_time[-1]:
                good_append_n += (end_time - self.guess_arrive_time[-1])/self.calVacancyRate(frame)

        if good_n > able_n:
            end_time += (good_append_n + good_n - able_n) * self.loading_speed
        return end_time, good_n+temp_n+good_append_n
    '''def calSimpleGetRate(self, i, max_num:int, begin_num=0, offset=5):#货物价值，时间
        index = self.boat_stop_ids.index(i)
        if index>=0:
            begin_num = sum(self.boat_pre_num[:index])
        else:
            begin_num = #预定区间的数量？
        if begin_num > self.present_goods_num:
            return 1, 0
        if max_num < self.present_goods_num - begin_num:
            return sum(self.goods_value[begin_num:(max_num+begin_num)]), max_num//self.loading_speed+offset
        else:
            return sum(self.goods_value[begin_num:]), (self.present_goods_num-begin_num)//self.loading_speed+offset'''
    def calSimpleGetRate(self, max_num:int, begin_num:int=0, offset:int=5):#货物价值，时间
        guess_goods_value = 1
        # if type(begin_num) != int:
        #     #print(begin_num, max_num)
        if begin_num > self.present_goods_num:
            return 0, 0
        if max_num < self.present_goods_num - begin_num:
            return sum(self.goods_value[begin_num:(max_num+begin_num)])+guess_goods_value, max_num//self.loading_speed+offset
        else:
            return sum(self.goods_value[begin_num:])+guess_goods_value, (self.present_goods_num-begin_num)//self.loading_speed+offset
        
    def takeInGood(self, frame, value):#存储进泊口的货物
        self.present_goods_num += 1
        self.goods_value.append(value)
        self.all_goods_frame.append(frame)
        self.all_goods_val.append(value)
        self.guess_arrive_time.sort()
        if len(self.guess_arrive_time):
            self.guess_arrive_time.pop(0)
    def takeOutGoods(self, num):#从泊口取出货物放到船舶上  对于存在船的港口进行每帧判断？
        self.present_goods_num -= num
        value = 0
        for _ in range(num):
            value += self.goods_value.pop(0)
        return value
    def boatLeave(self, boat_id):
        if self.have_boat:
            self.have_boat = False
            self.cancelForBoat(boat_id)
    def boatIn(self):#如何设置等待机制？如果船到了位置，但发现泊口内已有船，那么它会等待一段时间？或者前往其它泊口？
        if not self.have_boat:
            self.have_boat = True
        
    def updateGoods(self, boat_capacity):#对于存在船的港口进行每帧判断？
        self.boat_capacity = boat_capacity
        if self.boat_capacity and self.present_goods_num:
            num = min(self.present_goods_num, self.boat_capacity, self.loading_speed)
            self.present_goods_num -= num
            self.boat_capacity -= num
            return num
                
    def preserveForGood(self, frame, value):
        self.guess_arrive_time.append(frame)
        self.guess_arrive_value.append(value)
class Boat:
    def __init__(self, id:int, dir:int=-1, status:int=0, send_time:int = 0):
        self.id = id
        self.core_pos = None
        self.dir = dir
        self.last_order = 0
        
        self.des = None#坐标表示point
        
        self.goods_num = 0
        self.last_num = 0
        self.goods_value = 0
        self.trans_dist = UP_LIMIT
        self.trans_id = 0
        
        self.send_time = send_time
        
        self.status = status
        
        self.preserve = -1 
'''
正常行驶状态（状态 0）
恢复状态（状态 1）
装载状态（状态 2）
'''

class Good:
    def __init__(self, value, time, berth_dist = 0, berth_id = -1):
        self.value = value
        self.time = time
        self.berth_dist = berth_dist
        self.berth_id = berth_id

class PurchasePoint:
    def __init__(self, pos, berth_num):
        self.pos = pos
        self.tag = -1
        #self.berths_dist = np.full(berth_num, UP_LIMIT, np.int16)
        
        self.tag_ready = False
        #self.berths_dist_ready = False
    def setTag(self, tag):
        self.tag = tag
    def getBerthsDist(self):
        return self.berths_dist
