class Point:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
    def __repr__(self):
        return "Point: (" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ")"

def convert_to_ascii(f):
    """
    """
    import uuid
    import os
    ascii_name = str(uuid.uuid4())
    os.system("./ply2ascii <" + f.name + ">" + ascii_name)
    return ascii_name

def read_file(f):
    """
        Reads a ply file and outputs a list of points.
    """
    import regex
    f.readline()

    binary = True if "binary" in f.readline() else False
    if binary:
        import os

        name = convert_to_ascii(f)
        ascii_f = open(name)
        points = read_file(ascii_f)
        ascii_f.close()
        os.system("rm " + name)
        return points
    else:
        points = []
        line = f.readline()
        while line:
            number = r"[\d\w-.]+"
            match = regex.match(r"^\s*(" + number + r")\s+(" + number + r")\s+(" + number + r")\s*$",line)
            try:
                points.append(Point(float(match.group(1)),float(match.group(2)),float(match.group(3))))
            except ValueError:
                pass
            except AttributeError:
                pass
            line = f.readline()
        return points

if __name__=="__main__":
    import sys
    print "Filename >>>",
    fname = sys.stdin.readline().strip()
    with open(fname) as f:
        read_file(f)[0:500]
