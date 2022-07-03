from controller import Robot
from controller import Camera
from controller import CameraRecognitionObject
from controller import Motor
from controller import GPS
import math
import time


def ensureDirection(m,gps,leftSpeed,rightSpeed):
    # 获得头部传感器坐标
    g1_x,g1_y,g1_z = gps[0].getValues()
    # 获得尾部传感器坐标
    g2_x,g2_y,g2_z = gps[1].getValues()
    
    # 计算向量g1g2
    g_x = g2_x - g1_x
    g_z = g2_z - g1_z
    
    # 计算方向向量模长
    g1g2 = math.sqrt(g_x**2 + g_z**2)
        
    # 计算向量
    mg1_x = g1_x - m[0]
    mg1_z = g1_z - m[2]
    
    # 模长
    mg1 = math.sqrt(mg1_x**2 + mg1_z**2)
    
    # 计算向量夹角
    cosA = abs((g_x*mg1_x + g_z * mg1_z) / ((g1g2 * mg1)))
    cosB = (g_x*mg1_x + g_z * mg1_z) / ((g1g2 * mg1))
    
    # 确定旋转方向
    if mg1_x * g_z - mg1_z * g_x > 0:
        turn_left = 1
    else:
        turn_left = 0
    
    #转动方向，修正速度
    if turn_left and (cosA < 0.99 and cosA > -0.99):
        leftSpeed = -2.0
        rightSpeed = 2.0
    elif not turn_left and (cosA < 0.99 and cosA > -0.99):
        leftSpeed = 2.0
        rightSpeed = -2.0
    elif cosA >= 0.99 and cosB >= 0.99:
         leftSpeed = 2.0
         rightSpeed = 2.0
    elif cosA >= 0.99 and cosB <= -0.99:
        leftSpeed = -2.0
        rightSpeed = 2.0
    return leftSpeed,rightSpeed
    
    

def main():
    # 宏定义更新延迟时间(ms)
    TIME_STEP = 64
    # 创建机器人对象
    robot = Robot()
    
    # 建立距离传感器联系
    ds = []
    dsNames = ['ds_right', 'ds_left']
    for i in range(2):
        # 获取设备并加入列表
        ds.append(robot.getDevice(dsNames[i]))
        # 使能函数，传感器使用前必须启用
        ds[i].enable(TIME_STEP)
    
    #建立GPS联系
    gps = []
    gpsNames = ['gps1','gps2']
    for i in range(2):
        gps.append(robot.getDevice(gpsNames[i]))
        gps[i].enable(TIME_STEP)
    
    # 设置相机
    my_camera = Camera("my_camera")
    my_camera.enable(TIME_STEP)
    my_camera.recognitionEnable(TIME_STEP)
    
    wheels = []
    wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
    for i in range(4):
        wheels.append(robot.getDevice(wheelsNames[i]))
        # 设置电机运行模式为速度控制模式
        wheels[i].setPosition(float('inf'))
        # 设置初始速度
        wheels[i].setVelocity(0.0)
    
    
    # 目标点坐标
    # aim = [1.2,0.025,0.9]
    # aim = [0.95,0.025,0.21]
   
    # aim = [0.72,0.02,0.27]
    aim = [0.62,0.03,-1.28]
    tstart  = time.time()
    avoidObstacleCounter = 0
    while robot.step(TIME_STEP) != -1:
    
        #获取机器人头部GPS位置
        pos=gps[0].getValues()
        # print(pos)
        
        #获取摄像头中可识别物体的个数
        numberOfObjects = my_camera.getRecognitionNumberOfObjects()
        #把物体object返回到my_object
        OBJECTS = my_camera.getRecognitionObjects()
        #如果图像中有可识别物体
        aim_id = ""
        aim_model = ""
        if (my_camera.hasRecognition):
            my_object = my_camera.getRecognitionObjects()#把物体object返回到my_object
            for i in range (numberOfObjects):
                # print(my_object[i].get_id())
                # print(my_object[i].get_model())
                # print(my_object[i].get_position())
                aim_id = my_object[i].get_id()
                aim_model = my_object[i].get_model()
            
        # 计算到目标点的直线距离
        aimdistance = math.sqrt((aim[0]-pos[0])**2 + (aim[2]-pos[2])**2)
        
        # print("aimdistance",aimdistance)
        
        leftSpeed = 3.0
        rightSpeed = 3.0
        
        # 确定方向
        leftSpeed,rightSpeed =  ensureDirection(aim,gps,leftSpeed,rightSpeed)
        
        if aimdistance <= 0.15 :
            leftSpeed = 0.0
            rightSpeed = 0.0
        elif avoidObstacleCounter > 0:
            avoidObstacleCounter -= 1
            leftSpeed = -3.0
            rightSpeed = -2.0
            # print("avoidObstacleCounter",avoidObstacleCounter)
        else:
            for i in range(2):
                if ds[i].getValue() < 600.0:
                    avoidObstacleCounter = 25
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)
        if aimdistance <= 0.15:
            print("reach target point")
            print("当前位置：",pos)
            print("目标点:",aim)
            print("aimdistance",aimdistance)
            print("目标点id:",aim_id)
            print("目标点model:",aim_model)
            break
    tend = time.time()
    print("time:{0:.3f}s".format(tend - tstart))

if __name__ == '__main__':
    main()
    



        