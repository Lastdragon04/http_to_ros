import can
import time
from datetime import datetime


bus=can.interface.Bus(interface='socketcan', channel="can0", bitrate=1000000)

def send_can_message(data,can_id):
    """向 STM32 发送 CAN 消息"""
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        print(f"已发送消息到 {bus.channel_info}: ID={hex(msg.arbitration_id)}, 数据={msg.data.hex()} {datetime.now().strftime('%H:%M:%S.%f')}")
    except can.CanError:
        print("消息发送失败")

send_can_message([1,1,1,1,1,1,1,1],40)
send_can_message([1,1,1,1,1,1,1,1],40)
send_can_message([1,1,1,1,1,1,1,1],40)
# start_time=time.time()
# count=0
# while True:
#     send_can_message([80,3,1,1,1,1,1,1],81)
#     count+=1
#     if time.time()-start_time>1:
#         break
# print(count)


