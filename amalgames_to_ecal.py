import ecal.nanobind_core as ecal_core
from ecal.msg.proto.core import Subscriber as ProtobufSubscriber
from ecal.msg.proto.core import Publisher as ProtobufPublisher
import lidar_data_pb2 as lidar_pb
from ecal.msg.common.core import ReceiveCallbackData
import time
from GPS import Point, GPS
import numpy as np

class LidarWatcher:
    def __init__(self):
        if not ecal_core.is_initialized():
            # Initialisation d'ECAL
            ecal_core.initialize("RadarQt receiver")
        
        # Set the state for the program.
        # ecal_core.process.set_state(ecal_core.process.Severity.HEALTHY, ecal_core.process.SeverityLevel.LEVEL1, "I feel good!")

        # Creating eCAL Subscribers
        self.sub = ProtobufSubscriber[lidar_pb.Lidar](lidar_pb.Lidar, "lidar_data")
        self.sub.set_receive_callback(self.data_callback)
        
        self.pub_amal = ProtobufPublisher[lidar_pb.Amalgames](lidar_pb.Amalgames, "amalgames")
        
        self.pub_balise = ProtobufPublisher[lidar_pb.Balises](lidar_pb.Balises, "balises_near_odom")

        self.pub_position = ProtobufPublisher[lidar_pb.Position](lidar_pb.Position, "lidar_pos")

    def __enter__(self):
        return self
    
    def __exit__(self, type, value, traceback):
        self.sub.remove_receive_callback()
        ecal_core.finalize()

    def data_callback(self, pub_id : ecal_core.TopicId, data : ReceiveCallbackData[lidar_pb.Lidar]) -> None:
        print('data')
        res=0.7 #450 points pour 360 degrés
        points=[Point(angle=data.message.angles[i], distance=data.message.distances[i], qualite=data.message.quality[i]) for i in range(len(data.message.distances))]
        dico, balises=GPS(points,res)
        self.send_data_position(dico["robot"])
        if balises!=None:
            self.send_data_balises(balises)

    def send_data_amal(self, paquets):
        """Send a LIDAR data message"""
        amal = lidar_pb.Amalgames()
        for paq in paquets:
            centre = paq.centre
            amal.x.extend([centre.x])
            amal.y.extend([centre.y])
            amal.size.extend([paq.size])
        self.pub_amal.send(amal)
        # print(f"{len(paquets)} data sent to ecal!")
        
    def send_data_balises(self, balises):
        # balises : paquet tuple
        balise = lidar_pb.Balises()
        for i, bal in enumerate(balises):
            balise.index.extend([i+1])
            balise.x.extend([bal.centre.x])
            balise.y.extend([bal.centre.y])
        self.pub_balise.send(balise)

    def send_data_position(self, position):
        p = lidar_pb.Position()
        p.x, p.y, p.theta = position
        self.pub_position.send(p)

if __name__ == "__main__":
    with LidarWatcher() as lw:
        while ecal_core.ok():

            time.sleep(0.5)








