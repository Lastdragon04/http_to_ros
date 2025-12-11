import sqlite3
import time
import re
import json



def map_can_to_body(mapping,original_list):
    for can_single in original_list:
        if can_single not in ["can0","can1","can2","can3","can4"]:
            return "shit"
    # 使用列表推导式进行映射转换
    return [mapping[item] for item in original_list]

class Command:
    def __init__(self, name_index=None, value=None, speed=None, part=None):
        self.name_index = name_index if name_index is not None else 0
        self.value = value if value is not None else 0.0
        self.speed = speed if speed is not None else 0.0
        self.part = part if part is not None else ""

class ControlBar(Command):
    def __init__(self):
        super().__init__()
        self.location=""
        self.joint_name=""
        self.default=0
        self.max=0
        self.min=0
        self.motor_id=0


class InvertedIndexSearcher:
    def __init__(self, db_path):
        self.db_path=db_path
    
    def _execute_query(self, query, params=None):
        with sqlite3.connect(self.db_path) as con:
            cursor = con.cursor()
            try:
                cursor.execute("PRAGMA foreign_keys=ON;")
                if params:
                    cursor.execute(query, params)
                else:
                    cursor.execute(query)
                results = cursor.fetchall()
                return results
            except sqlite3.Error as e:
                raise Exception(e)
    
    def change(self,query,params=None):
        with sqlite3.connect(self.db_path) as con:
            cursor = con.cursor()
            try:
                cursor.execute("PRAGMA foreign_keys=ON;")
                if params:
                    cursor.execute(query, params)
                else:
                    cursor.execute(query)
                results = con.commit()
                return results
            except sqlite3.Error as e:
                raise Exception(e)

    def init_position(self):
        sql="UPDATE motor_config set current_position = 0.0"
        self.change(sql)

    def update_position(self,names,positions):
        db=sqlite3.connect(self.db_path)
        cursor=db.cursor()
        start_time=time.time()
        sql=""
        for i in range(len(positions)):
            sql="UPDATE motor_config set current_position = '%s' where name_index='%s';"%(positions[i],names[i])
            cursor.execute(sql)
        db.commit()
        cursor.close()
        db.close()
        print(time.time()-start_time)

    def get_allmodules(self,robot_type):
        sql="SELECT DISTINCT topic FROM motor_config WHERE robot_type=? ORDER BY topic"
        temp=self._execute_query(sql,(robot_type,))
        result=[]
        if temp:
            for i in temp:
                result.append(i[0])
        return result

    def get_all_motor_config(self,robot_type="天轶2.0Pro"):
        sql="SELECT * FROM motor_config where robot_type='%s' ORDER BY topic"%(robot_type,)
        return self._execute_query(sql)
    
    def insert_action_group(self,*params):
        sql="""
            INSERT INTO action_groups (name,callback,description) VALUES (?, ?,?);
            """
        result = self.change(sql,params)

    def query_all_action_group(self):
        sql = """
            SELECT * FROM action_groups
        """
        result = self._execute_query(sql)
        return result

    def query_action(self,group_id):
        sql="""SELECT * FROM action WHERE group_id=? ORDER BY seq;"""
        return self._execute_query(sql,(group_id,))

    def query_action_group_by_id(self,group_id):
        sql="""SELECT * FROM action_groups where id=?"""
        result= self._execute_query(sql,(group_id,))
        if result:
            return result[0]
        else:
            return None

    def check_action_seq_repeat(self,*params):
        sql="select * from action where group_id=? and seq=?"
        result = self._execute_query(sql,params)
        return result

    def insert_action(self,*params):
        print(params)
        sql="""
            INSERT INTO action (name, group_id, seq, command) VALUES ( ?, ?, ?, ?);
            """
        result = self._execute_query(sql,params)

    def delete_action_seq_change(self,action_id):
        sql0="""SELECT group_id, seq FROM action WHERE id = ?"""
        temp=self._execute_query(sql0,(action_id,))
        group_id,seq=temp[0]
        sql1="DELETE FROM ACTION WHERE ID=?"
        self.change(sql1,(action_id,))
        if temp:
            sql2="""SELECT group_id, seq FROM action WHERE group_id = ? and seq = ?"""
            result=self._execute_query(sql2,(group_id,seq))
            if not result:
                sql2="UPDATE ACTION SET seq=seq-1 WHERE GROUP_ID=? AND seq>?"
                self.change(sql2,(group_id,seq))

    def delete_action_group(self,group_id):
        sql="DELETE FROM action_groups WHERE ID=?"
        self.change(sql,(group_id,))

    def get_shit_by_name_index(self,name_index):
        sql="""SELECT id,name,can_name,min,max FROM motor_config where name_index=?"""
        result=self._execute_query(sql,(name_index,))
        if result:
            return result[0]
        else:
            return None,None,None,None,None

    def get_action_modify_bar(self,action_id,mapping):
        sql="""SELECT * FROM action WHERE ID=?"""
        result=self._execute_query(sql,(action_id,))
        if result:
            result=list(result[0])
            string1=re.sub("\'",'\"',result[4])
            string2=re.sub("None",'null',string1)
            # 将字符串转换为Python列表
            temp = json.loads(string2)
            control_bars=[]
            for i in range(len(temp["position"])):
                control_bar=ControlBar()
                motor_id,joint_name,can_name,min_position,max_position=self.get_shit_by_name_index(temp["name_index"][i])
                control_bar.motor_id=motor_id
                control_bar.joint_name=joint_name
                control_bar.location=map_can_to_body(mapping=mapping,original_list=[can_name])[0]
                
                control_bar.name_index=temp["name_index"][i]
                control_bar.position=temp["position"][i]
                control_bar.speed=temp["speed"][i]
                control_bar.efforts=temp["efforts"][i]
                
                control_bar.default=0
                control_bar.min=min_position
                control_bar.max=max_position
                control_bars.append(control_bar)
            result[4]=control_bars
        return result

    def update_action(self,action_id,name,command):
        sql="""UPDATE action
                set name=?,
                    command=?
               WHERE id = ?
            """
        self.change(sql,(name,command,action_id))

    def parse_command_data(self,temp):
        result = []
        for data in temp:
            command_dict = json.loads(data[0].replace("'", '"').replace("None", 'null'))    
            # 直接构建结果
            for part, commands in command_dict.items():
                print(commands)
                result.append({
                    part: [Command(**cmd) for cmd in commands]
                })
        return result

    def query_cmd_by_group_id(self,group_id):
        db=sqlite3.connect(self.db_path)
        cursor=db.cursor()
        sql="""SELECT action.command from action_groups,action where action_groups.id=action.group_id and action_groups.id='%s'"""%(group_id,)
        cursor.execute(sql)
        temp=cursor.fetchall()
        return self.parse_command_data(temp)

    def update_seq(self,action_id,way,group_id):
        """UP:ASC DOWN:DESC"""
        db=sqlite3.connect(self.db_path)
        cursor=db.cursor()
        try:
            print(action_id,way)
            sql_temp1="""SELECT seq,id
                    FROM ACTION
                    WHERE seq > (SELECT seq FROM ACTION WHERE id = '%s') and group_id='%s'
                    ORDER BY seq ASC
                    LIMIT 1;    
                """%(action_id,group_id)
            sql_temp2="""SELECT seq,id
                    FROM ACTION
                    WHERE seq < (SELECT seq FROM ACTION WHERE id = '%s') and group_id='%s'
                    ORDER BY seq DESC
                    LIMIT 1;    
                """%(action_id,group_id)
            sql=sql_temp1 if way=="down" else sql_temp2
            cursor.execute(sql)
            result=cursor.fetchall()
            if not result:
                return
            next_seq=result[0][0]
            next_id=result[0][1]
            sql2="""
                SELECT seq FROM ACTION where id='%s'
                """%(action_id,)
            cursor.execute(sql2)
            now_seq=cursor.fetchone()
            if not now_seq:
                return
            now_seq=now_seq[0]

            sql3="""UPDATE ACTION
                    SET seq = '%s'
                    WHERE id = '%s';"""%(next_seq,action_id)
            cursor.execute(sql3)
            sql4="""UPDATE ACTION
                    SET seq = '%s'
                    WHERE id = '%s';"""%(now_seq,next_id)
            cursor.execute(sql4)
            db.commit()
        finally:
            cursor.close()
            db.close()


if __name__=="__main__":
    searcher = InvertedIndexSearcher('/home/zck/workspace/website_v2/Memories/robot_control_v2.db')
    start_time=time.time()
    a=searcher.query_cmd_by_group_id(5)
    for i in a:
        for key,value in i.items():
            for j in value:
                print(j.name_index)