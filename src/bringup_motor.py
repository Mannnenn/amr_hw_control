#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import time
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state

# グローバル変数
gState_driver = 0
gMotor_spd = 0

# レスポンス購読


def resCallback(res):
    global gMotor_spd
    if (res.slave_id == 1):
        gMotor_spd = res.data[0]

# ステータス購読


def stateCallback(res):
    global gState_driver
    gState_driver = res.state_driver

# 通信が終わるまで待機


def wait():
    global gState_driver
    time.sleep(0.01)
    while (gState_driver == 1):
        pass


def main():
    rospy.init_node("sample1", anonymous=True)
    pub = rospy.Publisher("om_query0", om_query, queue_size=1)
    rospy.Subscriber("om_response0", om_response, resCallback)
    rospy.Subscriber("om_state0", om_state, stateCallback)
    msg = om_query()
    time.sleep(1)

    # 運転指令(S-ONをONする)
    msg.slave_id = 1        # スレーブID
    msg.func_code = 1       # ファンクションコード: 0:Read 1:Write 2:Read/Write
    msg.write_addr = 124    # アドレス指定： ドライバ入力指令
    msg.write_num = 1       # 書き込みデータ数: 1
    msg.data[0] = 1         # S-ONを立ち上げる
    pub.publish(msg)        # 配信
    wait()

    time.sleep(1)           # 5秒待機

    # 回転開始指令(ダイレクトドライブモード)
    msg.slave_id = 1
    msg.func_code = 1
    msg.write_addr = 90
    msg.write_num = 7
    msg.data[0] = 48        # DDO運転方式 16:連続運転(速度制御)
    msg.data[1] = 0         # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
    msg.data[2] = 4000000      # DDO運転速度(初期単位：r/min)
    msg.data[3] = 1000000      # DDO加速レート(初期単位：ms)
    msg.data[4] = 1000000      # DDO減速レート(初期単位：ms)
    msg.data[5] = 1000      # トルク制限値 (100.0%)
    msg.data[6] = 1         # DDO運転トリガ設定
    pub.publish(msg)        # 配信
    wait()

    time.sleep(1)          # 10秒待機


if __name__ == '__main__':
    main()
