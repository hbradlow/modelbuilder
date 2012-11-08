from asyncproc import Process
import json
import os

data = json.load(open("db.json"))

for datum in data[0::3]:
    base_dir = datum["base_dir"]
    os.chdir(base_dir)
    print base_dir
    proc = Process(["meshlab","output.ply"])
