def pcd_to_pts(input,output):
    """
        Converts a pcd file to a pts file.
    """
    line = input.readline()
    while line:
        try:
            numbers = [float(a) for a in line.split(" ")[0:6]]
            output.write(" ".join([str(a) for a in numbers]) + "\n")
        except:
            pass
        line = input.readline()
    output.close()

def ply_to_pcd(input,output):
    pass
if __name__ == "__main__":
    pcd_to_pts(open("output.pcd"),open("test.pts","w"))
