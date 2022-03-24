# 宁波工程学院 New Legends 嵌入式代码


Developing

双板麦轮步兵

[toc]





## 读前须知和规范

​	！！！**注意，代码编写者有义务按以下规范编写README.md以及整体代码，方便队伍代码管理和其他人阅读代码，以下的各部分文档均为步兵基础文档，其他兵种负责人根据自身兵种要求，自行修改**！！！

​	此代码是在组委会开源的C板示例代码的基础上，集合众多大学开源后搭建的，相关资料位于doc文件夹内。

**开发人员说明**：本例程为舵轮步兵的代码，是从初始的云台框架和地盘框架拉取，并

​					有义务维护该兵种的README.md文件，按以下说明维护。

每一次的commit应该在summary（总结）内添加修改的地方或者实现的功能，方便代码的维护。

***未征得分支所有者同意，不允许对该分支进行写入操作**！*

**文件目录及说明**：文件夹的删改，需要在文件夹的表格内进行修改；根据application内任务的实际调用情况，对application表格进行删改，保证一致性。doc内必须要存有对应兵种硬件连接图（统一使用diagrams.net在线编辑器编写）,官方C型嵌入式开发手册，其他文档或图片自行添加。

**场地人员说明**：需要提供必要的硬件连接说明和注意事项（位于装配阶段），代码适配及校准说明（位于调试阶段），可优化的性能及实现目标（优化阶段）。

**操作手说明**：首先是必要的遥控器控制和键盘控制逻辑，操作手UI介绍和布局（自定义UI图片）。

**功能实现说明**：在功能实现说明内，需要按结构（建议划分为云台，底盘，发射）注明该兵种需要实现的功能，标注出已实现和未实现。



## 文件目录结构及说明

| 文件夹              | 来源            | 内容                                      |
| ------------------- | --------------- | ----------------------------------------- |
| git                 | git             | 存储git相关的配置文件以及每次修改的内容   |
| application         | 开发者          | 应用层，包含freertos需要执行的任务        |
| bsp                 | 开发者          | 提供对底层硬件功能的封装以及驱动实现      |
| components          | 开发者          | 包含需要使用的算法，协议                  |
| doc                 | 开发者          | 文档                                      |
| Drivers             | CubeMX          | CMSIS相关库、STM32 HAL                    |
| Inc                 | CubeMX          | 配置完成后，自动生成的头文件              |
| MDK-ARM             | CubeMX          | Keil uversion 项目相关文件                |
| Middlewares         | 开发者 / CubeMX | 中间件                                    |
| Src                 | CubeMX          | 配置完成后，自动生成的源文件              |
| .gitignore          | git             | 使用git更新代码时，无视部分文件格式的修改 |
| j_scope_test.jscope | j-socpe         | 一种可视化调试软件                        |
| README.md           | 开发者          | 阅读代码前必读文档                        |
| standard_robot.ioc  | CubeMX          | 对外设和硬件进行配置的图形化软件          |



| application内任务说明 | 内容                                                         |
| --------------------- | ------------------------------------------------------------ |
| calibrate_task        | 校准功能:提供云台校准，陀螺仪零漂校准，底盘重设 ID 的功能    |
| chassis_task          | 底盘控制功能：完成底盘的麦轮运动控制，底盘功率控制           |
| detect_task           | 离线判断功能：根据数据反馈的时间戳来判断设备是否离线         |
| gimbal_task           | 云台控制功能：完成云台的角度控制                             |
| ins_task              | 姿态解算功能：完成陀螺仪加速度计的角度融合，解算欧拉角       |
| led_trigger_task      | LED 的 RGB 切换：使用三色 LED 完成 RGB 显示，呼吸灯效，判断是否死机 |
| oled_task             | OLED 显示功能：将设备错误信息显示出来，方便使用者定位问题    |
| referee_control       | 裁判信息控制：裁判系统与机器人数据进行的交互，对机器人性能进行限制 |
| referee_usart_task    | 裁判系统数据解析：使用单字节解析裁判系统数据                 |
| remote_control        | 遥控器数据解析：使用串口空闲中断函数，解析接收机发送的数据   |
| servo_task            | 舵机任务功能：控制空闲PWM输出口，控制舵机                    |
| shoot_task            | 射击任务功能：完成对摩擦轮电机和拨弹电机的控制               |
| software_reset_task   | 软件复位功能：当机器人出现意外情况时，可以手动重启机器人     |
| super_cap_task        | 超级电容任务：读取超级电容信息，和超电管理以及的单片机进行通信 |
| ui_task               | 操作手UI画面：用于人机交互和战场信息处理                     |
| vision                | 视觉信息通信：和上位机进行通信，传输视觉数据和指令           |
| voltage_task          | 电源采样:采样电源电压，并估计当前电池电量，作为简单电量判断  |







## 场地人员说明:

如果不加特殊说明,默认的坐标系为,人站在车后面,正前方为Y正方向,正右方为X轴正方向,正上方为Z轴正方向.符合右手坐标系。

绕X轴旋转为pitch角度，绕Y轴转旋转为roll角度，绕Z轴旋转为yaw角度，逆时针为正方向。

### 硬件连接说明：

从左向右数:
裁判串口线: C板UART1  G TX RX
裁判学生串口接口 左侧串口 RX TX G
视觉串口线: C板UART2  RX TXV G V

CAN1: L H
CAN2: V G H L

底盘电机：can2  ID 为1 2 3 4 右前，左前，左后，右后
///////
2     1
3     4   
///////
云台电机：can1 yaw 5 pitch 6

摩擦轮电机：can1 left 1 right 2

拨盘电机：can1 3

弹仓电机: can1 4
限位舵机: 左边数第3个PWM  

超级电容 0x211

板间通信ID 301 -304


射弹 触发条件为  BUTTEN_TRIG_PIN 为低电平 对应C板最左侧的PWM口
        

关于电机正反装
#define YAW_TURN    0
#define PITCH_TURN  0

这两个宏定义与电机正反装相关,当yaw轴电机转子与云台相对运动时,YAW_TURN为0,否则为1;pitch轴电机转子与云台相对运动时,PITCH_TURN为0,否则为1,


### 校准操作说明：

开启校准模式：          左摇杆右下，右摇杆左下  且左右按键拨至下档

陀螺仪校准           		左摇杆左下，右摇杆右下  且左右按键拨至下档

云台校准              		 左摇杆左上，右摇杆右上  且左右按键拨至下档

底盘校准             		 左摇杆右上，右摇杆左上   且左右按键拨至下档

### 代码适配说明:

以下以步兵的适配为例: 调试前应将底盘悬空, 云台调试和陀螺仪调试过程应将云台CAN输出为0
1.完成硬件的连接
2.底盘校准,调整底盘电机ID
3.陀螺仪校准,根据C板固定方式,读取INS_angle内的三轴角度,通过调整C板固定方式或者
修改代码内的INS_YAW_ADDRESS_OFFSET等宏定义以及云台绝对坐标更新函数,调整适配的欧拉角
4.云台调试,根据电机正反装调节宏定义,YAW_TURN,PITCH_TURN;根据反馈的电机编码值,调整YAW_OFFSET, PITCH_OFFSET为云台归中编码值,再依此调节限幅,
5.调试完以上一个







## 操作手说明：

### 遥控器控制:

左侧按键：上 打开和关闭摩擦轮
         中 无状态
         下 从中拨到下,快速拨回中为单发
            从中拨到下,停留为连发


右侧按键：上为云台跟随底盘

​		  中为底盘跟随云台

​		  下为刹车，云台静止

拨杆：
左侧拨杆控制底盘前后左右，右侧拨杆控制云台pitch和yaw，在底盘跟随底盘模式下也控制底盘旋转。

### 键鼠控制：将遥控器左右按键拨至中间 !!! 如果车辆失控,将遥控器右按键拨至下
注意!!! 同时按下ctrl z x 一秒,单片机将重启  !!!

若无为特殊说明,均为单击操作

底盘：
    W A S D 前后左右平移
    F 小陀螺 
    C 扭腰
    SHIFT 长按 超级电容加速
    X 45度角对敌

云台:
    鼠标左右移动 yaw轴左右运动
    鼠标前后移动 pitch轴上下运动
    Q 左90度转头 E 右 90度转头 V 180度转头

射击: 
    G 打开和关闭摩擦轮 以及激光   
    R 双击 打开和关闭弹仓 !注意,弹仓打开时,云台无法运动
    鼠标左键 单击单发 长按连发 
    鼠标右键 打开和关闭自瞄


### 操作手UI界面:

底盘模式: 
L45 左45度角对敌
R45 右45度角对敌
top 小陀螺状态
swing 扭腰状态

T 为打开 F 为关闭

云台模式:
normal 默认底盘跟随云台
auto    自瞄模式
hit_buff 打符

弹仓:
magazine T or F

摩擦轮:
fric T or F

## 功能:


### 已完成:
1.基础底盘运动,云台运动,发射机构

2.视觉数据收发

### 未完成:

1.UI已写完,但是需要调试

2.OLED屏幕写完,但是需要调试

3.裁判功率及热量控制

4.超电控制

5.云台PID还需调整

6.遥控器掉线保护









//附加功能

1.PID后添加卡尔曼
2.反陀螺
3.设备离线监测
4.flash读取，自动校准
5.陀螺回正
7.PID运算频率，调整角度环-角速度后的系数 调整1000
8.机器人间通信





