from math import atan2, pi
from itertools import groupby
from scipy.spatial import ConvexHull
from geomdl import BSpline, utilities, exchange, multi
from geomdl.visualization import VisVTK as vis
import numpy as np 
#dorsal1.pts - concave   dorsal1.pts - coonvex
#f1=["dorsal1.pts","dorsal2.pts","dorsal3.pts","dorsal4.pts","dorsal5.pts","dorsal6.pts","dorsal7.pts","dorsal8.pts","dorsal9.pts","dorsal10.pts"]
f1=["dorsal1.pts","dorsal2.pts"]

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
    #make 3d list for two_dimentional option
    #exchange.import_txt(new_list, two_dimensional=True)
    surf.set_ctrlpts(val2,u,v)
    surf.knotvector_u = utilities.generate_knot_vector(surf.degree_u, surf.ctrlpts_size_u)
    surf.knotvector_v = utilities.generate_knot_vector(surf.degree_v, surf.ctrlpts_size_v)
    surf.delta = 0.025
    #surf.remove_knot(0.2)
    #operations.split_surface_u(surf, 0.2)
    # Evaluate surface
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
    renderer(obj)
sum = multi.SurfaceContainer(obj_mult)
sum.sample_size = 50

# Visualization configuration
sum.vis = vis.VisSurface(ctrlpts=True, legend=False, figure_size=[1280, 720])
#sum.vis = VisVTK.VisSurface(ctrlpts=True, legend=False, figure_size=[940, 940])

sum.render(cpcolor="gray")
exchange.export_obj(sum,"layers.obj")


#exchange.export_obj(obj,"examples/3d_model of "+f1+".obj")

""" print("highest pair ",highest_pair)
print("lowest pair ",lowest_pair)
print("highest point ",above)
print("lowest point ",below)
print(len(symmetrical_points),sep="\n")
 """

# разворот 2х половин массива Alg - No.1
#new_list_[len(new_list_)//2:],new_list_[:len(new_list_)//2] = new_list_[:len(new_list_)//2],new_list_[len(new_list_)//2:]
#ready_list=[elem for row in new_list_ for elem in row]

""" # check symmetr
symmetrical_points = [];above=[];below=[]
for point in ready_list:
    x, y, z = point
    # Find symmetrical points 
    for sym_point in ready_list:
        sym_x, sym_y, sym_z = sym_point
        if sym_x!=x or sym_y!=y or sym_z!=z  :
            if sym_x == x and abs(sym_y) == abs(y) and sym_z == z: 
                symmetrical_points.append(sym_point)           
for i in range(len(symmetrical_points)):
    for j in range(i+1, len(symmetrical_points)):
        point_1 = symmetrical_points[i]
        point_2 = symmetrical_points[j]
        if point_1[0] == point_2[0] and abs(point_1[1]) == abs(point_2[1]) and point_1[2] > highest_z:
            highest_z = point_1[2]
            highest_pair=[point_1,point_2]
        elif point_1[0] == point_2[0] and abs(point_1[1]) == abs(point_2[1]) and point_1[2] < lowest_z:
            lowest_z = point_1[2]
            lowest_pair = [point_1, point_2]
sym1_x, sym1_y, sym1_z = highest_pair[0]
sym2_x, sym2_y, sym2_z = highest_pair[-1]
sym3_x, sym3_y, sym3_z = lowest_pair[0]
sym4_x, sym4_y, sym4_z = lowest_pair[-1]
for point in ready_list:
    x, y, z = point
    if (z>sym1_z and x<sym1_x and y>=sym1_y and y<=sym2_y):
        print(point," is above & between",highest_pair[0],"and",highest_pair[-1])
        above.append(point) 
    elif (z<sym3_z and x<sym3_x and y>=sym3_y and y<=sym4_y):
        print(point," is below & between",lowest_pair[0],"and",lowest_pair[-1])
        below.append(point) 
above = [list(t) for t in set(tuple(element) for element in above)]
below = [list(t) for t in set(tuple(element) for element in below)]
if len(above)>0:
    new_list_[len(new_list_)//2:],new_list_[:len(new_list_)//2] = new_list_[:len(new_list_)//2],new_list_[len(new_list_)//2:]
    ready_list=[elem for row in new_list_ for elem in row]
elif len(below)>0:
    pass
 """
#is_convex = is_surface_convex(val8)
#if is_convex:
#    print("The surface is convex.")
#else:
#    print("The surface is not convex.")
""" if len(above)>0:
    new_list_[len(new_list_)//2:],new_list_[:len(new_list_)//2] = new_list_[:len(new_list_)//2],new_list_[len(new_list_)//2:]
    ready_list=[elem for row in new_list_ for elem in row]
elif len(below)>0:
    pass
else:
    # alg - No.2
    new_list4=new_list_[len(new_list_)//2:]
    new_list4[0], new_list4[1] = new_list4[1], new_list4[0]
    nl444=[i[::-1] for i in new_list4[::-1]]
    nl444[0], nl444[1] = nl444[1], nl444[0]
    new_list44= np.concatenate([new_list_[0:len(new_list_)//2], nl444], axis=1)
    new_list_=new_list44.tolist()
    #uv-array into xyz array
    ready_list=[elem for row in new_list_ for elem in row]
 """

                # использовать в анализе 2 симметричные точки вместо одной  
                # сравнить их с точкой из общего массива 
                # повторить до нахождения одной точки без симетричной и выше/ниже никого не станет
                #VX если нет симметричной точки и точки выше - реверсируем (измени местами половины строк в таблице uv) Alg - No.1
                #VX если есть симметричная точка и нет точки выше -  alg - No.2
                #VX если нет симметричной точки и точки ниже - оставим как после чтения
                                        # ДОРОЖНАЯ КАРТА
                # !!!!!!!Если разорваны сетки слева и справа - разделить на 2 инстанции геометрических объекта для отдельного рендера(Придумай КОД!!!!!!!)
                # Итог - неверно мыслил по определения выпуклости или вогнутости поверхности
                #    придется переделывать все то, что ты додумывал и генерировал тебе ИИ
                #    найти или как - то написать алгоритм для определения выпуклости/вогнутости поверхности
                #    найти алгоритм для нахождения расстояния между ближайшими точками и 
                #       разрыву поверхностей в случае превышения расстояния между ближайшими точками
                # для неструктуриррованного облака точек придать структуру,
                # по которой можно наложитьь сетку и сортировать их для получения сетки с поверхностью
                                        # Второстепенные задачи
                # Отрисовывать графики VTK пряио на виджет в интерфейсе программы после открытия файла
                # Применять методы аппрксимации
                # # разобраться в практической разнице между всеми методами
            
                # получить исходные точки сферы для аппрксимации поверхности самой сферы
                # собрать приложение в исполняемый файл???

