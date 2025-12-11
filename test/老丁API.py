import sqlite3
import yaml

def load_motor_config(file_path):
    """解析电机配置文件并返回结果"""
    print("读取中...", end='', flush=True)
    
    try:
        with open(file_path, 'r') as file:
            config_data = yaml.safe_load(file)
        
        # 提取配置数据
        params = config_data['can_bus_node']['ros__parameters']
        motor_names = params['motor_names']
        motors_config = params['motors']
        
        # 按CAN总线分组
        can_groups = {}
        for name, config in motors_config.items():
            can_name = config['can_name']
            if can_name not in can_groups:
                can_groups[can_name] = []
            can_groups[can_name].append({
                'name': name,
                'id': config['can_rx_id'],
                'offset': config['offset'],
                'min': config['min'],
                'max': config['max']
            })
        
        # 返回结果
        result = {
            'total_motors': len(motor_names),
            'motor_names': motor_names,
            'motors_config': motors_config,
            'can_groups': can_groups
        }
        
        print("\r配置文件解析完成！")
        return result
        
    except Exception as e:
        print(f"\r错误: {e}")
        return None


# 原始数据
motor_data = load_motor_config("/home/ygsj/workspace/website_v2/Memories/motors_config.yaml")

# 连接数据库（若不存在将自动创建）
conn = sqlite3.connect('/home/ygsj/workspace/website_v2/Memories/robot_control_v2.db')
cursor = conn.cursor()

# 准备插入数据：将字典转换为元组列表
insert_data = []
for name, config in motor_data["motors_config"].items():
    # 格式: (name, name_index, can_name, can_rx_id, offset, max, min)
    # 注：name_index 无数据，设为 None (插入时为 NULL)
    row = (
        None,                   # 电机名称
        name,                   # name_index (设为NULL)
        config['can_name'],     # CAN总线名称
        config['can_rx_id'],    # CAN接收ID
        None,
        config['offset'],       # 偏移值
        config['max'],          # 最大值
        config['min']           # 最小值
    )
    insert_data.append(row)

# 执行批量插入
cursor.executemany('''
    INSERT INTO motor_config (
        name, name_index, can_name, can_rx_id,current_position, offset, max, min
    ) VALUES (?, ?, ?, ?, ?, ?,?, ?)
''', insert_data)

# 提交更改并关闭连接
conn.commit()
conn.close()