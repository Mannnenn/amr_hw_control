#!/usr/bin/python3
# -*- coding: utf-8 -*-

#
# ●対象機種:
#       BLV-R
#
# ●ドライバ設定:
#       パラメータ名称: [1軸目, 2軸目]
#       通信ID: [1, 2]
#       Baudrate: [19200, 19200]
#
# ●launchファイル設定:
#       com:="/dev/ttyUSB0" topicID:=1 baudrate:=19200 updateRate:=1000 firstGen:="1," globalID:="10" axisNum:="2"
#
# ●処理内容:
#       Writeにより連続運転(速度制御)でモーターを運転させる。
#       Readにより一定周期でモーターの検出速度を取得し、表示させる。

import rospy
import time
import datetime
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state
from std_msgs.msg import Float32MultiArray

# グローバル変数
gState_driver = 0   # 0:通信可能 1:通信中
gState_mes = 0      # 0:メッセージなし 1:メッセージ到達 2:メッセージエラー
gState_error = 0    # 0:エラーなし 1:無応答 2:例外応答
pub = None
msg = om_query()
gMotor_pos = 0
gMotor_speed = 0
gDoesWorkTimer = False

right_com = 0
left_com = 0

right_res = 0
left_res = 0

res_pub = None

# ドライバ状態のコールバック関数


def stateCallback(res):
    global gState_driver
    global gState_mes
    global gState_error
    gState_driver = res.state_driver
    gState_mes = res.state_mes
    gState_error = res.state_error

# レスポンスのコールバック関数


def resCallback(res):
    global gMotor_pos
    global gMotor_speed
    # 例外応答のとき
    if gState_error == 2:
        print("Exception")
        return
    # ID Shareモードのとき
    if isIdShare(res) and (res.func_code == 0x03):
        # roslaunchで設定した軸数情報の取得
        axis_num = int(rospy.get_param("/om_modbusRTU_1/axis_num"))
        data_num = int(msg.read_num)    # データ数
        global right_res
        global left_res
        right_res = res.data[0] / (286 / 2) * (2 * 3.14)
        left_res = res.data[1] / (286 / 2) * (2 * 3.14)

        for axis in range(axis_num):
            print('{0}: {1}[rad/s], {2}[rad/s]'.format(datetime.datetime.now(),
                  res.data[0], res.data[1]))  # [0]:1軸目の検出速度、[1]:2軸目の検出速度

# パラメータサーバとresponseのslave_idから、現在ID Shareモードか調べる


def motorCallback(msg):
    global right_com
    global left_com
    right_com = msg.data[0]
    left_com = msg.data[1]


def isIdShare(res):
    global_id = rospy.get_param("/om_modbusRTU_1/global_id")
    return int(global_id) == res.slave_id


def wait(t):
    global gState_driver
    time.sleep(t)
    while (gState_driver == 1):
        pass

# 一定周期で実行する処理


def timeProcess(event):
    global gDoesWorkTimer
    if gDoesWorkTimer:
        msg.slave_id = 32       # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 0       # 0:Read
        msg.read_addr = 0x000E  # 読み出すアドレスの起点
        msg.read_num = 2        # 各軸1個ずつ
        pub.publish(msg)        # 配信する


def main():
    global gState_mes
    global gState_error
    global pub
    global gDoesWorkTimer

    global right_com
    global left_com

    global right_res
    global left_res

    global res_pub

    MESSAGE_ERROR = 2
    EXCEPTION_RESPONSE = 2

    rospy.init_node("blv_idshare_sample1_1", anonymous=True)    # 上位ノード作成
    # masterにメッセージを渡すpublisher作成
    pub = rospy.Publisher("om_query1", om_query, queue_size=10)
    # 通信状態に関するメッセージを受け取るsubscriber作成
    rospy.Subscriber("om_state1", om_state, stateCallback)
    # ドライバのレスポンスに関するメッセージを受け取るsubscriber作成
    rospy.Subscriber("om_response1", om_response, resCallback)
    rospy.Subscriber("motor_command", Float32MultiArray, motorCallback)

    res_pub = rospy.Publisher(
        "motor_response", Float32MultiArray, queue_size=10)

    # ID Shareモードで通信するため、global_id=32に設定
    rospy.set_param("/om_modbusRTU_1/global_id", "32")

    wait(0.5)

    # 一定周期でtimeProcessを実行する
    gDoesWorkTimer = True
    rospy.Timer(rospy.Duration(0.5), timeProcess)

    # ID Shareモードで各モーターを運転する
    # 5Hzで運転指令を送信する
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        print("Move at left_com: %f, right_com: %f" %
              (left_com, right_com))
        msg.slave_id = 32           # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 1           # 0:read 1:write 2:read/write
        msg.write_addr = 0x0000     # 書き込むアドレスの起点
        msg.write_num = 12           # 全軸合わせたデータ項目数を代入する
        # 1軸目のデータ
        msg.data[0] = 16        # DDO運転方式 16:連続運転(速度制御)
        msg.data[1] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[2] = round(1000 * left_com * 60 / (3.14 * 2 * 1.3)
                            )     # DDO運転速度(初期単位：r/min)
        msg.data[3] = 50000      # DDO加速レート(初期単位：ms)
        msg.data[4] = 50000      # DDO減速レート(初期単位：ms)
        msg.data[5] = 1         # DDO運転トリガ設定
        # 2軸目のデータ
        msg.data[6] = 16        # DDO運転方式 16:連続運転(速度制御)
        msg.data[7] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        # DDO運転速度(初期単位：r/min)
        msg.data[8] = round(1000 * right_com * 60 / (3.14 * 2 * 1.3))
        msg.data[9] = 50000      # DDO加速レート(初期単位：ms)
        msg.data[10] = 50000     # DDO減速レート(初期単位：ms)
        msg.data[11] = 1        # DDO運転トリガ設定
        # 配信
        gDoesWorkTimer = False
        wait(0.03)
        pub.publish(msg)
        wait(0.03)
        gDoesWorkTimer = True

        response_msg = Float32MultiArray()
        response_msg.data = [right_res, left_res]
        res_pub.publish(response_msg)

        rate.sleep()

    print("END")
    rospy.spin()


if __name__ == '__main__':
    main()
