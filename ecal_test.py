import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtobufSubscriber
from ecal.msg.proto.core import Publisher as ProtobufPublisher
from ecal.msg.common.core import ReceiveCallbackData
import lidar_data_pb2 as lidar_pb
import time
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtWidgets, QtCore
import numpy as np

# LidarWatcher reste inchangé
class LidarWatcher:
    def __init__(self, scatter):
        self.scatter = scatter
        if not ecal_core.is_initialized():
            # Initialisation d'ECAL
            ecal_core.initialize("ecal test rcv")
        
        # Set the state for the program.
        ecal_core.process.set_state(ecal_core.process.Severity.HEALTHY, ecal_core.process.SeverityLevel.LEVEL1, "I feel good!")

        # Creating eCAL Subscribers
        self.sub = ProtobufSubscriber[lidar_pb.Lidar](lidar_pb.Lidar, "lidar_data")
        self.sub.set_receive_callback(self.data_callback)
        
        self.subcaca = ProtobufSubscriber[lidar_pb.Lidar](lidar_pb.Lidar, "caca boudin")
        self.subcaca.set_receive_callback(self.data_blabla)
        
        # Publisher to send data
        self.tt = ProtobufPublisher[lidar_pb.Lidar](lidar_pb.Lidar, "caca boudin")

    def __enter__(self):
        return self
    
    def __exit__(self, type, value, traceback):
        self.sub.remove_receive_callback()
        ecal_core.finalize()

    def data_blabla(self, pub_id : ecal_core.TopicId, data : ReceiveCallbackData[lidar_pb.Lidar]) -> None:
        datas = data.message
        print(datas.nb_pts)
        print(datas.angle_increment)
        print(datas.distances)
        print(datas.angles)
        print(datas.quality)

    def data_callback(self, pub_id : ecal_core.TopicId, data : ReceiveCallbackData[lidar_pb.Lidar]) -> None:
        nb_pts = len(data.message.distances)
        x = [data.message.distances[i]/50*np.cos(-data.message.angles[i]) for i in range(nb_pts)]
        y = [data.message.distances[i]/50*np.sin(-data.message.angles[i]) for i in range(nb_pts)]
        size = [5 for _ in range(nb_pts)]
        colors = [(255-q, q, 0) for q in data.message.quality]
        self.scatter.setData(x, y, size=size, brush=colors)
        mean_dist = sum(data.message.distances) / len(data.message.distances)
        min_angle = data.message.angles[0]
        max_angle = data.message.angles[-1]
        print(f"[{min_angle:.2f}-{max_angle:.2f}]{mean_dist: .0f} mm")

    def send_data(self):
        """Send a LIDAR data message"""
        l = lidar_pb.Lidar()
        l.nb_pts = 999
        l.angle_increment = 888
        l.angles.extend([4, 4, 4])
        l.distances.extend([8, 8, 8])
        l.quality.extend([7, 7, 7])
        self.tt.send(l)
        print("Data sent to eCAL!")

# MainWindow est maintenant le widget PyQt
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LIDAR Data Viewer")
        self.setGeometry(100, 100, 800, 600)

        # Create PlotWidget and ScatterPlotItem
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot = self.plot_widget.addPlot()

        self.plot.setAspectLocked(True)  # Circle will not be distorted
        self.plot.showGrid(x=True, y=True)
        self.plot.setRange(xRange=[-100, 100], yRange=[-100, 100])

        self.scatter = pg.ScatterPlotItem(size=5, pen=None)
        self.plot.addItem(self.scatter)

        # Layout for the PyQt window
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

        # Create LidarWatcher instance
        self.lidar_watcher = LidarWatcher(self.scatter)

    def keyPressEvent(self, event):
        """
        This method will be called when a key is pressed.
        We will check for specific keys and send data accordingly.
        """

        self.lidar_watcher.send_data()

        # For other keys, you can handle them as needed
        print(f"Key pressed: {event.text()}")

    def closeEvent(self, event):
        """Handle the close event of the window"""
        ecal_core.finalize()
        event.accept()

# Main entry point
if __name__ == '__main__':
    app = QtWidgets.QApplication([])

    # Create and show the main window
    main_window = MainWindow()
    main_window.show()

    # Start the PyQt event loop
    app.exec()
