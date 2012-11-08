import numpy as np
import scipy
import math
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

class Arrow3D(FancyArrowPatch):
    """
        An arrow object for a matplotlib 3d plot.
        
        Code from http://stackoverflow.com/questions/11140163/python-matplotlib-plotting-a-3d-cube-a-sphere-and-a-vector
    """
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)
    def __repr__(self):
        return self._verts3d

class Ellipse:
    def __init__(self,origin=[0,0,0],radius=(0,0),angle=0):
        self.origin = origin
        self.radius = radius
        self.angle = angle
        self.axis = [0,0,1]
    def __repr__(self):
        return "center: " + str(self.origin) + ", radius: " + str(self.radius)

class Point:
    def __init__(self,x=0,y=0,z=0,is_valid=True):
        self.x = x
        self.y = y
        self.z = z
        self.is_valid = is_valid
    def set(self,l):
        self.x = l[0]
        self.y = l[1]
        self.z = l[2]
    def list(self):
        return [self.x,self.y,self.z]

class CircleFit:
    def __init__(self,points=[]):
        self.circle = Circle()
        self.points = []
        for p in points:
            self.points.append(Point(p[0],p[1],p[2]))
        
        self.plane_to_xy_transform = None
        self.flatten_transform = None

    def process(self):
        self.calculate_plane_to_xy_transform()
        self.transform_data(self.plane_to_xy_transform)
        self.calculate_flatten_transform()
        self.transform_data(self.flatten_transform)

        self.show()

        self.calculate_best_fit_ellipse()
        """

        self.transform_data(self.flatten_transform,inverse=True)
        self.transform_data(self.plane_to_yz_transform,inverse=True)
        """

    def transform_data(self,t,inverse=False):
        def transform(t,v):
            return np.dot(t,np.array(v)).tolist()
        if inverse:
            t = np.linalg.inv(np.array(t))
        else:
            t = np.array(t)
        for (index, point) in enumerate(self.points):
            self.points[index].set(transform(t,point.list()+[1]))

        self.circle.origin = transform(t,self.circle.origin + [1])[0:3]
        self.circle.axis = transform(t,self.circle.axis + [0])[0:3]
        self.normal = transform(t,self.normal + [0])[0:3]

    def best_fit_plane(self):
        """
            Find the plane that best fits the set of translations
        """
        def zeros(i):
            return [0 for a in range(i)]
        A = np.array([zeros(3) for j in range(3)])
        b = np.array(zeros(3))
        for point in self.points:
            A = np.add(np.array([   [point.x*point.x, point.x*point.y,  point.x],
                        [point.x*point.y, point.y*point.y,  point.y],
                        [point.x,          point.y,           1]]),A)
            b = np.add(np.array([point.x*point.z,point.y*point.z,point.z]),b)
        x = np.linalg.solve(A,b)
        return x

    def calculate_plane_to_xy_transform(self):
        """
            Calculate the transform to rotate the plane of the circle into the yz plane.
        """
        def chunks(l, n):
            """ Yield successive n-sized chunks from l.
            """
            for i in xrange(0, len(l), n):
                yield l[i:i+n]

        x = self.best_fit_plane()
        normal = [x[0],x[1],-1]
        self.normal = normal

        from cgkit.cgtypes import quat, mat3, slerp
        axis = np.cross(np.array(normal),np.array([0,0,1]))
        angle = math.acos(np.dot(np.array(normal),np.array([0,0,1]))/np.linalg.norm(normal))
        q = quat()
        q = q.fromAngleAxis(angle,axis.tolist())
        transform = [i for i in chunks(q.toMat4().toList(rowmajor=True),4)]
        self.plane_to_xy_transform = transform

        return transform

    def calculate_flatten_transform(self):
        """
            Calculate the transform to move all the translation points into the yz plane. Basically just remove the x values.
        """
        def ave(l):
            return reduce(lambda x,y: x+y,l)/len(l)
        a = ave([point.z for point in self.points if point.is_valid])
        transform = [   [1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,-a],
                        [0,0,0,1]]
        self.flatten_transform = transform
        return transform

    def calculate_best_fit_ellipse(self):
        """
            http://math.stackexchange.com/questions/214661/circle-least-squares-fit
        """
        A = []
        b = []
        def f(b,*args):
            det = b[1]**2 - 4*b[0]*b[2]
            if det > -.1:
                return 999999
            total = 0
            for point in self.points:
                total += np.dot(np.array([point.x**2,point.x*point.y,point.y**2,point.x,point.y,1]),np.array(b))**2
            return total
        x = scipy.optimize.fmin(f,(1,1,1,1,1,1))

        self.circle = Circle([0] + x[0].tolist()[0:2],x[0].tolist()[2])
        self.circle.radius = math.sqrt(self.circle.radius + self.circle.origin[1]**2 + self.circle.origin[2]**2)
        self.circle.axis = [1,0,0]

    def show(self):
        from matplotlib.patches import FancyArrowPatch
        from mpl_toolkits.mplot3d import proj3d
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        import mpl_toolkits.mplot3d.art3d as art3d
        import matplotlib.patches
        fig = plt.figure("Circle Fit")
        ax = fig.add_subplot(111,projection="3d",aspect=1)
        x = [];y = [];z = []
        for point in self.points:
            x.append(point.x)
            y.append(point.y)
            z.append(point.z)
        ax.scatter(x,y,z,color="r",s=200)
        ax.auto_scale_xyz([-.5, .5], [-.5, .5], [-0, 1])

        circle_axis = Arrow3D((0,self.normal[0]),(0,self.normal[1]),(0,self.normal[2]),mutation_scale=20,lw=3,arrowstyle="-|>", color="g")
        ax.add_artist(circle_axis)

        plt.show()
