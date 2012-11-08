from chull import Vector,Hull
from pyply import read_file, Point

with open("output.ply") as f:
    points = read_file(f)
    points = map(lambda p: Vector(float(p.x),float(p.y),float(p.z)),points[0:2000])
    h = Hull(points)
    h.Print()
