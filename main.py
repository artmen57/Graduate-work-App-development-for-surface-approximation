from PyQt5 import QtCore, QtWidgets
from interface import Ui_MainWindow
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import numpy as np
import open3d as o3d
import vtk
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
        cam_orient_manipulator = vtk.vtkCameraOrientationWidget()
        cam_orient_manipulator.SetParentRenderer(self.ren)
        # Enable the widget.
        cam_orient_manipulator.On
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
        #self.iren.SetRenderWindow(self.ren)

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
        self.vtkWidget=VTK_Render()
        self.ui.horizontalLayout.insertWidget(0,self.vtkWidget)
        self.ui.actOpen.triggered.connect(self.Open_file)
        self.ui.actExit.triggered.connect(self.closeEvent)
        timer = QtCore.QTimer(self)
        timer.setInterval(20)   # period, in milliseconds
        #timer.timeout.connect(self.ui.View3D.updateGL)
        #timer.start()
        self.show()
        
    def Open_file(self):
        """ Func for opening point cloud   """
        files=QtWidgets.QFileDialog.getOpenFileNames(self, "Open file","")
        print (files[0])
        for file in files[0]:
            print(file,"\n")
            #ev = o3d.visualization.ExternalVisualizer()
            object=o3d.io.read_point_cloud(file)
            print(object)
            xyz_load = np.asarray(object.points)
            print(xyz_load)
            if not file[0]:
                return 
            
    def Save_file(self):
        print("")

    def closeEvent(self,QCloseEvent):
        self.vtkWidget.close()
        app.exit()

if __name__ == '__main__':
    from sys import exit,argv
    app = QtWidgets.QApplication(argv)
    win = MainWindow()
    win.show()
    exit(app.exec_())