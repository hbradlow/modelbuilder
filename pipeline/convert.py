
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
    from pyply import read_file,Point
    points = read_file(input)
    output.write("VERSION 0.7\n")
    output.write("FIELDS x y z\n")
    output.write("SIZE 4 4 4\n")
    output.write("TYPE F F F\n")
    output.write("COUNT 1 1 1\n")
    output.write("WIDTH " + str(len(points)) + "\n")
    output.write("HEIGHT 1\n")
    output.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    output.write("POINTS " + str(len(points)) + "\n")
    output.write("DATA ascii\n")
    for p in points:
        output.write(str(p.x) + " " + str(p.y) + " " + str(p.z) + "\n")
    output.close()

if __name__ == "__main__":
    pcd_to_pts(open("output.pcd"),open("test.pts","w"))
