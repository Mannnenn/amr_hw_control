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
from sensor_msgs.msg import Joy

# グローバル変数
gState_driver = 0   # 0:通信可能 1:通信中
gState_mes = 0      # 0:メッセージなし 1:メッセージ到達 2:メッセージエラー
gState_error = 0    # 0:エラーなし 1:無応答 2:例外応答
pub = None
msg = om_query()
gMotor_pos = 0
gMotor_speed = 0


# ドライバ状態のコールバック関数


def stateCallback(res):
    global gState_driver
    global gState_mes
    global gState_error
    gState_driver = res.state_driver
    gState_mes = res.state_mes
    gState_error = res.state_error


# パラメータサーバとresponseのslave_idから、現在ID Shareモードか調べる
def isIdShare(res):
    global_id = rospy.get_param("/om_modbusRTU_1/global_id")
    return int(global_id) == res.slave_id

# t[s]待機する


def wait(t):
    global gState_driver
    time.sleep(t)
    while (gState_driver == 1):
        pass


def main():

    wait(1)  # Wait for connection

    global gState_mes
    global gState_error
    global pub

    MESSAGE_ERROR = 2
    EXCEPTION_RESPONSE = 2

    rospy.init_node("blv_switch_son", anonymous=True)    # 上位ノード作成
    # masterにメッセージを渡すpublisher作成
    pub = rospy.Publisher("om_query1", om_query, queue_size=1)
    # 通信状態に関するメッセージを受け取るsubscriber作成
    rospy.Subscriber("om_state1", om_state, stateCallback)

    wait(1)

    s_on = rospy.get_param("s_on")

    # 運転指令(S-ONをSwitchする)
    msg.slave_id = 1      # スレーブID
    msg.func_code = 1     # ファンクションコード: 0:Read 1:Write 2:Read/Write
    msg.write_addr = 124  # アドレス指定： ドライバ入力指令
    msg.write_num = 1     # 書き込みデータ数: 1
    msg.data[0] = s_on       # S-ONを立ち上げる
    pub.publish(msg)      # 配信
    wait(0.5)

    msg.slave_id = 2      # スレーブID
    msg.func_code = 1     # ファンクションコード: 0:Read 1:Write 2:Read/Write
    msg.write_addr = 124  # アドレス指定： ドライバ入力指令
    msg.write_num = 1     # 書き込みデータ数: 1
    msg.data[0] = s_on       # S-ONを立ち上げる
    pub.publish(msg)      # 配信
    wait(0.5)
    print("Set S-ON", s_on)

    print("END")


if __name__ == '__main__':
    main()
