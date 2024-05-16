# collada_hook.py
 
from PyInstaller.utils.hooks import collect_data_files
 
def hook(hook_api):
    # 返回空的数据文件列表，表示不包含任何 collada 库的资源文件
    return []
