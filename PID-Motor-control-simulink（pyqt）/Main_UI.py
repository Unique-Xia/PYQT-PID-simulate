from PySide2.QtWidgets import QApplication
from PyQt5.QtCore import QTimer,QDateTime
from PyQt5 import uic
import math
import pyqtgraph as pg
import sys
'''
在类中，定义函数可以在调用的下方
'''
class Main_UI:
    def __init__(self):                                     # 构造函数,组件初始化,仅执行一次
        self.system_init()

        self.P = 1                                          # P=1.00：纯比例时等幅振荡
        self.I = 0.4                                        # PID 参数在此设置
        self.D = -0.5


#======================================================================================================================#
# =====================================================================================================================#

    '''系统初始化'''
    def system_init(self):
        self.ui_init()          # UI 组件初始化
        self.pid_data_init()    # PID相关变量初始化
        self.motor_model_init() # 电机相关变量初始化
        self.time_init()        # 时间组件初始化
        self.plotwindow_init()  # 绘图窗口初始化

    '''绘图窗口初始化'''
    def plotwindow_init(self):
        self.ui.plot_speed.setTitle("电机速度控制跟踪曲线",color='008080',size='12pt')
        self.ui.plot_speed.setLabel("left", "速度")
        self.ui.plot_speed.setLabel("bottom", "时间")                     # 设置上下左右的label
        self.ui.plot_speed.setYRange(min=-10,max=50)                        # 设置Y轴 刻度 范围
        self.ui.plot_speed.showGrid(x=True, y=True)                        # 显示表格线
        self.ui.plot_speed.setBackground('w')                               # 背景色改为白色
        self.commonx = 0                                                    # 初始化 X 轴坐标从0开始
        self.sety = 0                                                       # 初始化参考值为0，代表电机停转
        self.runy = 0                                                       # 初始化实际电机转速为0，保证安全
        self.commonxlist = []                                               # x轴的值
        self.setylist = []                                                  # y轴的值      设置的数据
        self.runylist = []                                                  # 实际运行的数据
        self.senddatalist = []                                              # 待发送的数据列表
        self.set_curve = self.ui.plot_speed.getPlotItem().plot(pen=pg.mkPen('l', width=5))  # 设定运行的曲线
        self.run_curve = self.ui.plot_speed.getPlotItem().plot(pen=pg.mkPen('r', width=1))  # 实际运行的曲线

    '''PID相关变量初始化'''
    def pid_data_init(self):
        self.Kp = 0.0           #比例项结果
        self.Ki = 0.0           #积分项结果
        self.Kd = 0.0           #微分项结果
        self.output = 0.0       #控制器输出
        self.integral = 0.0     #积分
        self.last_error = 0.0   #上次误差
        self.error = 0.0        #当前误差

    '''电机相关变量初始化'''
    def motor_model_init(self):
        self.motoeSetSpeed = 0          # 电机设定速度
        self.motoeFactSpeed = 0         # 电机实际输出速度
        self.t = 0.01                   # 控制周期0.01s
        self.Tm = 0.5                   # 电机时间常数
    '''时间组件初始化'''
    def time_init(self):
        self.Timer = QTimer()  # 创建时钟对象QTimer
        self.Timer.start(5)  # 每5ms产生一次timeout信号
        self.Timer.timeout.connect(self.updateTime)  # 连接updateTime函数槽，更新时间
        self.Timer.timeout.connect(self.plot)  # 连接绘图函数
    '''UI组件初始化'''
    def ui_init(self):
        self.ui = uic.loadUi('main_window.ui')  # 导入自定义控件（基于pyqt5）
        self.ui.pushbtn_speedset.clicked.connect(self.motor_speed_set)  # 设置电机目标速度按钮
        self.ui.pushbtn_PID_set.clicked.connect(self.pid_set)  # 设置电机目标速度按钮

    '''更新系统时间'''
    def updateTime(self):
        self.ui.label_time.setText(QDateTime.currentDateTime().toString('yyyy-MM-dd hh:mm:ss dddd'))

    '''设置PID参数'''
    def pid_set(self):
        self.P = float(self.ui.line_edit_Pp.text())  # 抓取 P 文本框内容
        self.I = float(self.ui.line_edit_Pi.text())  # 抓取 I 文本框内容
        self.D = float(self.ui.line_edit_Pd.text())  # 抓取 D 文本框内容

    '''设置电机目标速度'''
    def motor_speed_set(self):
        self.motoeSetSpeed = float(self.ui.lineEdit_speedset.text())
    '''串口数据绘图（5ms定时器调用）'''
    def plot(self):
        self.motoeFactSpeed = self.motor_model()        # 得到最终的电机速度输出值
        self.commonx += 1                                                   # X 轴坐标更新
        self.runy = self.motoeFactSpeed
        self.sety = self.motoeSetSpeed
        self.commonxlist.append(self.commonx)
        self.setylist.append(self.sety)
        self.runylist.append(self.runy)
        self.set_curve.setData(self.commonxlist,self.setylist)               # 绘制设定数值的曲线
        self.run_curve.setData(self.commonxlist, self.runylist)              # 绘制实际运行的曲线

    '''PID 控制器'''
    def pid_control(self):
        self.last_error = self.error                                # 迭代
        self.error = self.motoeSetSpeed - self.motoeFactSpeed       # 比例项
        self.integral += self.error                                 # 积分
        self.Kp = self.P * self.error
        self.Ki = self.I * self.integral
        self.Kd = self.D * (self.error - self.last_error)          # 微分
        self.output = self.Kp + self.Ki + self.Kd                  # 输出
        return self.output

    '''电机的数学模型'''
    def motor_model(self):
        self.t = self.t + 0.01
        speed = self.pid_control() * (1 - math.exp(-(self.t/self.Tm)))  # 仿真步长0.01s
        return speed

'''====================================================================================================='''
'''====================================================================================================='''
'''====================================================================================================='''
'''
功能  ：主线程循环，关闭窗体结束
输入  ：无
输出  ：无
'''
if __name__ == '__main__':
        app = QApplication(sys.argv)    # 创建QApplication指针
        mainwin = Main_UI()             # 创建UI对象
        mainwin.ui.show()               # 显示对象的UI界面
        sys.exit(app.exec_())           # 主循环

'''====================================================================================================='''
'''====================================================================================================='''
'''====================================================================================================='''