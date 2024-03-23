import os
import time
import zipfile


def files2zip(zip_file_name: str, file_list: list):
    with zipfile.ZipFile(zip_file_name, mode='w', compression=zipfile.ZIP_DEFLATED) as zf:
        for fn in file_list:
            parent_path, name = os.path.split(fn)
            zf.write(fn, arcname=name)


file_list=[file for file in os.listdir("./") if file.endswith(".cpp") or file.endswith(".hpp") or file.endswith(".h") or file=="CMakeLists.txt"]
# file_list=[file for file in os.listdir("./") if file.endswith(".cpp") or file.endswith(".hpp") or file.endswith(".h")]

output_dir="./output"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

files2zip(zip_file_name=os.path.join(output_dir,time.asctime().replace(' ','_').replace(':','-')+'.zip'),file_list=file_list)
