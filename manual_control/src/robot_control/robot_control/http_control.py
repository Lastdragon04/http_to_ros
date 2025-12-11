import rclpy
from rclpy.node import Node
from fastapi import FastAPI,Request, Depends, HTTPException,Form,File, UploadFile,Query
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from typing import List,Optional
from pydantic import BaseModel
import uvicorn
import asyncio
from websockets import serve
import threading
from .db.crud import InvertedIndexSearcher as IIS
from collections import defaultdict
from bodyctrl_msgs.msg import SetMotorPosition,CmdSetMotorPosition
import json
import time
import fastapi_cdn_host
from datetime import datetime
from .bll import map_part



class FastAPINode(Node):
    def __init__(self):
        super().__init__('fastapi_node')

        self.declare_parameter('dist_path', "/home/zck/workspace/http_to_ros/dist")
        self.dist_path = self.get_parameter('dist_path').get_parameter_value().string_value

        self.declare_parameter('db_path', "/home/zck/workspace/http_to_ros/Memories/robot_control_v2.db")
        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value

        self.IIS=IIS(self.db_path)
        self.IIS.init_position()
        self.head_pub = self.create_publisher(CmdSetMotorPosition, '/head/cmd_pos', 10)
        self.arm_pub = self.create_publisher(CmdSetMotorPosition, '/arm/cmd_pos', 10)
        self.waist_pub = self.create_publisher(CmdSetMotorPosition, '/waist/cmd_pos', 10)
        self.leg_pub = self.create_publisher(CmdSetMotorPosition, '/leg/cmd_pos', 10)
        self.pub={"head":self.head_pub,"arm":self.arm_pub ,"waist":self.waist_pub,"leg":self.leg_pub}
        self.parts_name={"arm":"手臂","leg":"腿","waist":"腰","head":"头"} 
        self.declare_parameter('server_port', 3754)
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.circulate=False
        self.app = FastAPI()

        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],  # 允许所有来源，可以根据需要调整
            allow_credentials=True,
            allow_methods=["*"],  # 允许所有方法，可以根据需要调整
            allow_headers=["*"],  # 允许所有头，可以根据需要调整
        )
        fastapi_cdn_host.patch_docs(self.app)
        self.setup_routes(self.dist_path)
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.start()

    def _publish(self, msg,part):
        """发布消息到手臂控制器"""
        try:
            self.pub[part].publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"发布失败: {e}")
            return False

    def _create_msg(self,command,part='arm'):
        """创建关节控制消息，严格匹配SetMotorPosition类型要求"""
        msg = CmdSetMotorPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = part
        msg.cmds=[]
        for data in command: 
            # 构建电机控制指令（所有字段均为正确类型）
            motor_cmd = SetMotorPosition()
            motor_cmd.name = data.name_index                  # 关节ID（整数，符合name字段要求）
            motor_cmd.pos = float(data.value)                 # 位置（float类型）
            motor_cmd.spd = data.speed                       # 速度（float类型）
            motor_cmd.cur = 5.0                        # 电流（float类型，已修复）
            msg.cmds.append(motor_cmd)
        return msg

    def make_action_simple(self,cmd):
        for part_name,data in cmd.items():
                message=self._create_msg(data,part=part_name)
                self._publish(message,part_name)

    def get_db(self)->IIS:
        return IIS(self.db_path)

    def setup_routes(self, dist_path):
        # @self.app.post("/speak")
        # async def speak(text:str=Form(...)):
        #     try:
        #         self.ser_speaker.send_data(data=text)
        #     except Exception as e:
        #         self.get_logger().info("语音合成模块线松了")
        #     return "success"

        # 创建一个路由来接收上传的文件

        class CommandModel(BaseModel):
            value: float
            speed: float
            part: str
            name_index: int

        class GroupRunModel(BaseModel):
            group_id:int
            gap_time: List[float]
            cycle:bool

        class ActionRunModel(BaseModel):
            part:str
            cmd:List[CommandModel]

        class ActionChangeModel(BaseModel):
            action_name:str
            group_id:int
            seq:int
            command:List[CommandModel]

        @self.app.get("/")
        async def index():
            with open(dist_path + "/index.html", "r") as file:
                html_content = file.read()
            return HTMLResponse(content=html_content)

        @self.app.get("/control/get_all")
        async def get_all_control(robot_type:int=Query(...),db:IIS=Depends(self.get_db)):
            print(robot_type,type(robot_type))
            try:
                if robot_type == 2:
                    robot_type="天轶2.0Pro"
                if robot_type == 1:
                    robot_type="天工2.0Plus"
                part=db.get_allmodules(robot_type)
                motors=db.get_all_motor_config(robot_type)
                part_names=map_part(self.parts_name,part)
                return {"topic":part,"motors":motors,"part_names":part_names}
            except Exception as e:
                print(e)
                return {"topic":"","motors":"","part_names":""}

        @self.app.post("/control/run")
        def control_run(cmd:List[CommandModel],db:IIS=Depends(self.get_db)):
            result = defaultdict(list)
            for item in cmd:
                result[item.part].append(item)
            print(result)
            self.make_action_simple(result)
            return {"message":"执行完毕"}

        @self.app.post("/action_group/add")
        def action_group_add(name:str=Form(...),action_callback:str=Form(None),description:str=Form(...),db:IIS=Depends(self.get_db)):
            db.insert_action_group(name,action_callback,description)
            return {"message":"success"}
        
        @self.app.get("/action_group/get_all")
        def get_all_action_group(db:IIS=Depends(self.get_db)):
            groups=db.query_all_action_group()
            return {"groups":groups}
        
        @self.app.delete("/action_group/delete")
        def delete_action_groups(group_id:int=Form(...),db:IIS=Depends(self.get_db)):
            try:
                db.delete_action_group(group_id)
                return {"message":"success"}
            except Exception:
                return {"message":"failed"}

        @self.app.get("/action/get_all")
        def get_all_action(group_id:int=Query(...),db:IIS=Depends(self.get_db)):
            action=db.query_action(group_id)
            group=db.query_action_group_by_id(group_id)
            return {"action":action,"group":group}
        
        #此处有问题
        @self.app.post("/action/add")
        def action_add(action:ActionChangeModel,db:IIS=Depends(self.get_db)):
            if db.check_action_seq_repeat(action.group_id,action.seq):
                return {"message":f"排序为{str(action.seq)}的动作已经存在，请勿重复添加"}
            command = defaultdict(list)
            for item in action.command:
                command[item.part].append(dict(item))
            db.insert_action(action.action_name,action.group_id,action.seq,json.dumps(command))
            return {"message":"success"}

        @self.app.get("/action/get")
        def get_action(action_id:int=Query(...),db:IIS=Depends(self.get_db)):
            action=db.get_action_modify_bar(action_id,self.parts_name)
            return {"action":action}

        @self.app.post("/action/run")
        def action_run(data:List[ActionRunModel],db:IIS=Depends(self.get_db)):
            print(data)
            return {"message":"success"}

        @self.app.delete("/action/delete")
        def action_seq_delete(action_id:int=Form(...),db:IIS=Depends(self.get_db)):
            db.delete_action_seq_change(action_id)
            return {"message":"success"}

        @self.app.post("/action_group/run")
        def action_group_run(form:GroupRunModel,db:IIS=Depends(self.get_db)):
            self.get_logger().info(f"接到请求 | {datetime.now().strftime('%H:%M:%S.%f')}")
            action_group=db.query_cmd_by_group_id(form.group_id)
            self.circulate=form.cycle
            commands=[]
            while True:
                for data in action_group:
                    for part,value in data.items():
                        msg=self._create_msg(value,part)
                        self._publish(msg,part)
                if self.circulate==False:
                    break
            return {"message":"执行完毕"}

        self.app.mount("/static", StaticFiles(directory=dist_path), name="static")
        templates = Jinja2Templates(directory=dist_path+"/templates")


    def run_server(self):
        uvicorn.run(self.app, host="0.0.0.0", port=self.server_port)


    # async def imu_echo(self,ws):
    #     while True:
    #         joint=self.joint_angle.copy()
    #         joint=json.dumps(joint)
    #         await ws.send(joint)

    # async def imu_angle_websocket(self):
    #     self.get_logger().info("IMU_Anlge_websocket已开启")
    #     async with serve(lambda ws: self.imu_echo(ws,), "0.0.0.0", 1100):
    #         await asyncio.Future()  # run forever

    # def imu_angle_ws(self):
    #     asyncio.run(self.imu_angle_websocket())

def main(args=None):
    rclpy.init(args=args)
    fastapi_node = FastAPINode()
    rclpy.spin(fastapi_node)
    fastapi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()