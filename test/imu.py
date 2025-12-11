import serial
from datetime import datetime


class IMUHandler:
    def __init__(self, ser,imu_publisher,com_port):
        self.ser = ser
        self.buf_length = 11
        self.RxBuff = [0] * self.buf_length
        self.ACCData = [0.0] * 8
        self.GYROData = [0.0] * 8
        self.AngleData = [0.0] * 8
        self.FrameState = 0
        self.CheckSum = 0
        self.start = 0
        self.data_length = 0
        self.acc = [0.0] * 3
        self.gyro = [0.0] * 3
        self.Angle = [0.0] * 3
        self.imu_publisher=imu_publisher
        self.com_port=com_port

    def GetDataDeal(self,list_buf):
        if list_buf[self.buf_length - 1] != self.CheckSum:
            return

        if list_buf[1] == 0x51:
            for i in range(6):
                self.ACCData[i] = list_buf[2 + i]
            self.acc = self.get_acc(self.ACCData)

        elif list_buf[1] == 0x52:
            for i in range(6):
                self.GYROData[i] = list_buf[2 + i]
            self.gyro = self.get_gyro(self.GYROData)

        elif list_buf[1] == 0x53:
            for i in range(6):
                self.AngleData[i] = list_buf[2 + i]
            self.angle = self.get_angle(self.AngleData)
            print(self.angle,datetime.now().strftime('%H:%M:%S.%f'))


    def DueData(self,inputdata):
        if inputdata == 0x55 and self.start == 0:
            self.start = 1
            self.data_length = 11
            self.CheckSum = 0
            for i in range(11):
                self.RxBuff[i] = 0

        if self.start == 1:
            self.CheckSum += inputdata
            self.RxBuff[self.buf_length - self.data_length] = inputdata
            self.data_length = self.data_length - 1 
            if self.data_length == 0: 
                self.CheckSum = (self.CheckSum - inputdata) & 0xff
                self.start = 0
                self.GetDataDeal(self.RxBuff) 

    def get_acc(self,datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc
        return acc_x, acc_y, acc_z

    def get_gyro(self,datahex):#鏃犺
        wxl = datahex[0]
        wxh = datahex[1]
        wyl = datahex[2]
        wyh = datahex[3]
        wzl = datahex[4]
        wzh = datahex[5]
        k_gyro = 2000.0
        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        return gyro_x, gyro_y, gyro_z

    def get_angle(self,datahex):#鏃犺
        rxl = datahex[0]
        rxh = datahex[1]
        ryl = datahex[2]
        ryh = datahex[3]
        rzl = datahex[4]
        rzh = datahex[5]
        k_angle = 180.0
        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        return angle_x, angle_y, angle_z
    def read_data(self):
        while True:
            RXdata = self.ser.read(1)
            if RXdata:
                RXdata = int(RXdata.hex(), 16)
                self.DueData(RXdata)


if __name__=="__main__":
    ser=serial.Serial(baudrate=921600,port="/dev/ttyUSB7")
    IMU=IMUHandler(ser=ser,imu_publisher=None,com_port="ttyUSB7")
    IMU.read_data()