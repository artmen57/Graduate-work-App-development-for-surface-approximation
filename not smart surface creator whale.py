from math import atan2, pi
from itertools import groupby
from scipy.spatial import ConvexHull
from geomdl import BSpline, utilities, exchange, multi
from geomdl.visualization import VisVTK as vis
import numpy as np 
#dorsal1.pts - concave   dorsal1.pts - coonvex
f1=["dorsal1.pts","dorsal2.pts","dorsal3.pts","dorsal4.pts","dorsal5.pts","dorsal6.pts","dorsal7.pts","dorsal8.pts","dorsal9.pts","dorsal10.pts"]
#f1=["dorsal1.pts","dorsal2.pts"]

f_root="examples/"

X=[];Y=[];Z=[]
highest_pair = [];lowest_pair = []
highest_z = float("-inf");lowest_z = float("inf")
f3=[]; vals=[]; tables=[];obj_mult=[]

def plotter(point_cloud):
    X=[];Y=[];Z=[]
    import plotly.express as px
    for p in point_cloud:
        X.append(p[0])
        Y.append(p[1])
        Z.append(p[2])
    fig = px.scatter_3d(x=X, y=Y, z=Z)
    fig.update_traces(marker_size = 4)
    fig.show(autosize=False,width=800, height=400,poi=2)
def surface_gen(val2,new_list1):
    surf = BSpline.Surface()
    u=len(new_list1)
    v=len(new_list1[-1])
    surf.degree_u = 1
    surf.degree_v = 2
    surf.set_ctrlpts(val2,u,v)
    surf.knotvector_u = utilities.generate_knot_vector(surf.degree_u, surf.ctrlpts_size_u)
    surf.knotvector_v = utilities.generate_knot_vector(surf.degree_v, surf.ctrlpts_size_v)
    surf.delta = 0.025
    surf.evaluate()
    return surf
def surf_approx():
    pass
def renderer(surf):
    vis_comp = vis.VisSurface()
    surf.vis = vis_comp
    surf.render(cpcolor="red")
    print("hi")
def is_surface_convex(point_cloud):
    # Convert the point cloud to a NumPy array
    points = np.array(point_cloud)

    # Calculate the centroid of the point cloud
    centroid = np.mean(points, axis=0)

    # Calculate the normal vectors of each triangle formed by the points
    triangles = points[:-2]
    vectors = np.cross(triangles - centroid, points[1:-1] - centroid)

    # Check if all the normal vectors have the same orientation
    is_convex = np.all(np.dot(vectors, vectors[0]) >= 0)


    """     import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    # Plot defining corner points
    ax.plot(point_cloud.T[0], point_cloud.T[1], point_cloud.T[2], "ko")
    # 12 = 2 * 6 faces are the simplices (2 simplices per square face)
    for s in hull.simplices:
        s = np.append(s, s[0])  # Here we cycle back to the first coordinate
        ax.plot(point_cloud[s, 0], point_cloud[s, 1], point_cloud[s, 2], "r-")
    # Make axis label
    for i in ["x", "y", "z"]:
        eval("ax.set_{:s}label('{:s}')".format(i, i))
    ax.set_box_aspect([1,1,1])
    plt.show() """


    return is_convex
def reader (f:str):
    val7=[];val8=[]
    for xyz in open(f,"r"):
        try:
            val=[float(coord) for coord in xyz.split()]
            val7.append(val)
            if val!=[] and val!=[[type(str)]]:
                val8.append(val)
        except(ValueError):
            continue
    new_list_ = [list(l) for i, l in groupby(val7, bool) if i]
    ready_list=val8

    if f=="dorsal1.pts" or f=="dorsal2.pts" or f=="dorsal7.pts"or f=="dorsal9.pts":
        new_list_[len(new_list_)//2:],new_list_[:len(new_list_)//2] = new_list_[:len(new_list_)//2],new_list_[len(new_list_)//2:]
        ready_list=[elem for row in new_list_ for elem in row]
    elif f=="dorsal3.pts" or f=="dorsal4.pts" or f=="dorsal5.pts" or f=="dorsal11.pts" or f=="dorsal12.pts":
        new_list4=new_list_[len(new_list_)//2:]
        new_list4[0], new_list4[1] = new_list4[1], new_list4[0]
        nl444=[i[::-1] for i in new_list4[::-1]]
        nl444[0], nl444[1] = nl444[1], nl444[0]
        new_list44= np.concatenate([new_list_[0:len(new_list_)//2], nl444], axis=1)
        new_list_=new_list44.tolist() 
        ready_list=[elem for row in new_list_ for elem in row]
    return ready_list,new_list_,
""" 
for i in f1:
    f2=f_root+i
    f3.append(f2) """
for j in f1:
    print(j)
    arr,tabl=reader(j)
    vals.append(arr)
    tables.append(tabl)
for arr_ready,tabl_ready in zip(vals,tables):
    obj=surface_gen(arr_ready,tabl_ready)
    obj_mult.append(obj)
    #renderer(obj)
sum = multi.SurfaceContainer(obj_mult)
sum.sample_size = 50


sum.vis = vis.VisSurface(ctrlpts=True, legend=False, figure_size=[1280, 720])


sum.render(cpcolor="gray")
exchange.export_obj(sum,"layers.obj")


