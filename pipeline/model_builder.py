from combine_clouds import ReconstructionPipeline
from transform_processor import TransformProcessor
import json
import sys
import sh
import os.path
import uuid
import time

class ModelBuilder:
    def __init__(self,database="db.json"):
        self.database_filename = database
        f = open(database)
        self.database = json.load(f)
        f.close()
        self.data = {}

        self.find_output_dir()

    def find_output_dir(self):
        self.output_dir = "/media/New Volume/hbradlow/PCD_FILES/MODEL_BUILDER/" + str(uuid.uuid4())
        self.tmp_dir = "tmp_combine_clouds"
        self.data["base_dir"] = self.output_dir

    def save_to_database(self):
        #some meta data for the object
        print "Name of object >>> ",
        name = sys.stdin.readline().strip()
        self.data["name"] = name
        self.data["time"] = time.time()

        sh.mkdir(self.output_dir)

        sh.cp(["-r",self.tmp_dir,os.path.join(self.output_dir,"pcd_files")]) #copy tmp files to database location
        for f_name in self.pipeline.files():
            sh.cp([f_name,self.output_dir])

        self.database.append(self.data)
        f = open(self.database_filename,"w")
        json.dump(self.database,f)
        f.close()

    def process(self,debug=True):
        import time
        start = time.time()

        self.pipeline = ReconstructionPipeline(skip=30,debug=True,base_dir=self.tmp_dir)
        self.pipeline.collect_data()
        self.pipeline.calculate_transforms()
        self.pipeline.process_transforms(show=False)
        self.pipeline.concatenate_clouds()
        self.pipeline.reconstruct()
        self.pipeline.generate_mesh_pcd()
        self.pipeline.generate_convex_hull()

        end = time.time()
        print "Took",str(round(end-start,2)),"seconds to comlete"

        self.pipeline.show()

if __name__ == "__main__":
    mb = ModelBuilder()

    mb.process()

    print "Save the model to the database? (does it look good)"
    print ">>> ([y]/n) ",
    output = sys.stdin.readline().strip()
    print output
    if output == "n":
        print "Not saving model"
    else:
        print "Saving to database..."
        mb.save_to_database()
