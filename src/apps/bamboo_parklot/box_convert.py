import os
import glob

def convert_ortho_to_grid(input_folder):
    """
    遍历输入文件夹中的所有子文件夹，读取_.ortho文件并生成_.grid文件
    
    Args:
        input_folder: 包含子文件夹的主文件夹路径
    """
    # 获取所有子文件夹
    subfolders = [f.path for f in os.scandir(input_folder) if f.is_dir()]
    
    for subfolder in subfolders:
        # 查找子文件夹中的_.ortho文件
        ortho_file = os.path.join(subfolder, "_.ortho")
        
        if os.path.exists(ortho_file):
            # 读取_.ortho文件内容
            ortho_data = {}
            with open(ortho_file, 'r') as f:
                for line in f:
                    key, value = line.strip().split(': ')
                    ortho_data[key] = value
            
            # 创建_.grid文件内容
            grid_content = f"""grid_x {{
  origin: {ortho_data.get('origin_x', '0')}
  res: {ortho_data.get('res_x', '0')}
  count: {ortho_data.get('width', '0')}
}}
grid_y {{
  origin: {ortho_data.get('origin_y', '0')}
  res: {ortho_data.get('res_y', '0')}
  count: {ortho_data.get('height', '0')}
}}
"""
            
            # 写入_.grid文件
            grid_file = os.path.join(subfolder, "_.grid")
            with open(grid_file, 'w') as f:
                f.write(grid_content)
            
            os.remove(ortho_file)
            
            print(f"已处理: {subfolder}")
        else:
            print(f"未找到_.ortho文件: {subfolder}")

if __name__ == "__main__":
    # 使用示例
    import sys
    
    if len(sys.argv) > 1:
        input_folder = sys.argv[1]
    else:
        input_folder = input("请输入要处理的文件夹路径: ")
    
    convert_ortho_to_grid(input_folder)
    print("处理完成!")