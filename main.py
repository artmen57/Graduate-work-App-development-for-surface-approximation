from PyQt5 import QtCore, QtWidgets
import pyvista as pv
import pyvistaqt
from interface import Ui_MainWindow
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import numpy as np
import open3d as o3d
import vtk
import funcs as fu
from geomdl import BSpline, utilities, exchange, multi
import trimesh
vals=[];tables=[];obj_mult=[];panel_vertices = np.empty((0, 3));panel_faces = np.empty(0)
#from  geomdl.visualization import VisVTK

class VTK_Render(QtWidgets.QMainWindow):
    def __init__(self, parent = None):
        QtWidgets.QMainWindow.__init__(self, parent)

        colors = vtk.vtkNamedColors()
        colors.SetColor('ParaViewBkg', [255, 255, 255, 255])

        self.frame = QtWidgets.QFrame()
        self.vl = QtWidgets.QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        self.vl.addWidget(self.vtkWidget)

        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(colors.GetColor3d('ParaViewBkg'))
        
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        mapper = vtk.vtkPolyDataMapper()
        # Create sourceсв

        source = vtk.vtkSphereSource()
        source.SetCenter(0, 0, 0)
        source.SetRadius(5.0)
        mapper.SetInputConnection(source.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        self.ren.AddActor(actor)
        self.ren.ResetCamera()
    
        # Create an actor
        self.frame.setLayout(self.vl)
        self.setCentralWidget(self.frame)
        self.iren.Initialize()
        self.iren.Start()
        
    def closeEvent(self, QCloseEvent):
        super().closeEvent(QCloseEvent)
        self.vtkWidget.Finalize()     

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self,*args, **kwargs):
        """ Initialize Main Window \n
            & create connection signals """
        super().__init__(*args, **kwargs)

        self.ui=Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.View3D.close()
        self.vtkWidget=pyvistaqt.QtInteractor()
        self.ui.horizontalLayout.insertWidget(0,self.vtkWidget)
        self.ui.actOpen.triggered.connect(self.Open_file)
        self.ui.acSave.triggered.connect(self.Save_file)
        self.ui.actExit.triggered.connect(self.closeEvent)
        self.cam_orient_manipulator = vtk.vtkCameraOrientationWidget()
        self.cam_orient_manipulator.SetParentRenderer(self.vtkWidget.renderer)
        self.cam_orient_manipulator.On()
        timer = QtCore.QTimer(self)
        timer.setInterval(20)   # period, in milliseconds
        #timer.timeout.connect(self.ui.View3D.updateGL)
        #timer.start()
        self.show()
        
    def Open_file(self):
        facesqt=[];obj_mult=[];sum=0
        """ Func for opening point cloud   """
        files=QtWidgets.QFileDialog.getOpenFileNames(self, "Open file","")
        #print (files[0])
        last_item = files[0][-1]
        for file in files[0]:
            with open(file) as w:
                print(w.name)
                arr,tabl=fu.reader(w.name)
                vals.append(arr)
                tables.append(tabl)
                for arr_ready,tabl_ready in zip(vals,tables):
                    if file ==last_item:    
                        obj,approx=fu.surface_gen(arr_ready,tabl_ready)
                        if approx==0:
                            win.ui.method.setCurrentIndex(0)
                        if approx==1:
                            win.ui.method.setCurrentIndex(1)
                        win.ui.v_knot.setText(str(obj.knotvector_v))
                        win.ui.u_knot.setText(str(obj.knotvector_u))
                        win.ui.u_P.setValue(obj.degree_u)
                        win.ui.v_P.setValue(obj.degree_v)
                        obj_mult.append(obj)
                if file ==last_item:
                    tmesh=0;mesh=0
                    sum = multi.SurfaceContainer(obj_mult)
                    sum.sample_size = 50
                    for f in sum.faces:
                        facesqt.append(f.data)
                    #surf = draft.delaunay_2d()
                    tmesh = trimesh.Trimesh(sum.vertices, faces=facesqt, process=False)
                    mesh = pv.wrap(tmesh)
                    win.vtkWidget.add_mesh(mesh)
                    if not file[0]:
                        return 
                    return sum
    def Save_file(self):
        name = QtWidgets.QFileDialog.getSaveFileName(self, 'Save 3d model')
        win.vtkWidget.export_obj("hello.obj")
        """ file = open(name,'w')
        text = self.textEdit.toPlainText()
        file.write(text)
        file.close() """


    def closeEvent(self,QCloseEvent):
        self.vtkWidget.close()
        app.exit()

if __name__ == '__main__':
    from sys import exit,argv
    app = QtWidgets.QApplication(argv)
    win = MainWindow()
    win.show()
    win.cam_orient_manipulator.On()
    exit(app.exec_())