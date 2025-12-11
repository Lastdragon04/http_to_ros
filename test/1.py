import yaml

def parse_motors_config(file_path):
    """解析电机配置文件并返回结果"""
    print(f"读取配置文件: {file_path}")
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

# 使用示例
if __name__ == "__main__":
    config = parse_motors_config("/home/ygsj/workspace/website/Memories/motors_config.yaml")
    print(config["motor_names"])
    print(config["motors_config"])