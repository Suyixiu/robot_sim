import os
from os import path
import shutil

# dir_now = path.dirname(__file__)
dir_now = "./"
print(dir_now)

for dirpath, dirnames, filenames in os.walk(dir_now):
    for filename in filenames:
        if ".png" in filename:
            try:
                os.remove(dirpath + '/' + filename)
            except BaseException:
                pass
    continue