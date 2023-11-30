import copy
import io
import math
import sys
import time
import numpy as np
import open3d as o3d
from PIL import Image
from pointcloud2 import read_points

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher

# proto_messages directory
import datatypes.ros.sensor_msgs.PointCloud2_pb2 as point_cloud_pb
import datatypes.ros.sensor_msgs.CompressedImage_pb2 as img_pb
import datatypes.ros.sensor_msgs.NavSatFix_pb2 as gnss_pub

from coordinate import calc_dir, WGS84_a, WGS84_e2
from bump_detection import find_obstacles


class ProtoMsg:
    def __init__(self, msgFreq, slidingWinLen, msgType):
        self.msgCnt = 0
        self.msgFreq = 0
        self.slidingWinLen = 0
        self.msgType = ""
        self.data = []

        self.msgFreq = msgFreq
        self.slidingWinLen = int(slidingWinLen * msgFreq)
        self.msgType = msgType

    def getLatestFrame(self):
        if self.msgCnt > 0:
            return self.data[-1]
        return None

    def getFrameByIdx(self, idx):
        if 0 <= idx < self.msgCnt:
            return self.data[idx]

    def addData(self, data):
        self.data.append(data)
        if self.msgCnt >= self.slidingWinLen:
            self.data.pop(0)
            return
        self.msgCnt = self.msgCnt + 1

    def getDataCnt(self):
        return self.msgCnt

    def dataPop(self):
        if self.msgCnt > 0:
            self.data.pop(0)
            self.msgCnt = self.msgCnt - 1


def img_callback(topic_name, proto_msg, time):
    # print("Message {} from {}: {}".format(proto_msg.id
    #                                       , proto_msg.name
    #                                       , proto_msg.msg))
    # imgData.addData(proto_msg)
    a = 1


def nav_callback(topic_name, proto_msg, time):
    gnss_msg = gnss_pub.NavSatFix()
    gnss_msg.header.seq = proto_msg.header.seq
    gnss_msg.header.frame_id = proto_msg.header.frame_id
    gnss_msg.header.stamp.sec = proto_msg.header.stamp.sec
    gnss_msg.header.stamp.nsec = proto_msg.header.stamp.nsec
    gnss_msg.latitude = proto_msg.latitude
    gnss_msg.longitude = proto_msg.longitude
    gnssData.addData(gnss_msg)


def pcd_callback(topic_name, proto_msg, time):
    pc = read_points(proto_msg)

    cnt = len(pc)
    pc_data = np.zeros((cnt, 3))
    for i in range(len(pc)):
        pc_data[i][0] = pc[i][0]
        pc_data[i][1] = pc[i][1]
        pc_data[i][2] = pc[i][2]
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(pc_data)
    pcdData.addData(copy.deepcopy(point_cloud))
    # o3d.io.write_point_cloud(str(proto_msg.header.stamp.sec) + "." +
    # str(proto_msg.header.stamp.nsec) + ".pcd", point_cloud)


class BumpDetector:
    def __init__(self):
        self.roiPCD = []
        self.warnings = []
        self.category = []
        self.bumpPos = []

        self.pcdSub = ProtoSubscriber("ROSVLS128CenterCenterRoof", point_cloud_pb.PointCloud2)  # Set the Callback
        self.pcdSub.set_callback(pcd_callback)

        self.imgSub = ProtoSubscriber("ROSFrontCenterImage", img_pb.CompressedImage)
        self.imgSub.set_callback(img_callback)

        self.navSub = ProtoSubscriber("ROSGlobalPosition", gnss_pub.NavSatFix)
        self.navSub.set_callback(nav_callback)

        self.bumpPub = ProtoPublisher("BumpWarning", gnss_pub.NavSatFix)

    def clear(self):
        self.roiPCD = []
        self.warnings = []
        self.category = []
        self.bumpPos = []

    def bumpDetection(self):
        if pcdData.getLatestFrame() is None:
            return False
        self.roiPCD, self.warnings, self.category, self.bumpPos = find_obstacles(pcdData.getFrameByIdx(0))
        pcdData.dataPop()
        if len(self.warnings) > 0:
            return True
        return False

    def sendBumpWarning(self):
        for i in range(len(self.warnings)):
            pos = self.bumpPos[i]
            dis = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
            if not gnssData.getDataCnt() >= 5:
                return
            lastGnssData = gnssData.getLatestFrame()
            firstGnssData = gnssData.getFrameByIdx(0)
            enu_dir = calc_dir([[firstGnssData.latitude, firstGnssData.longitude, firstGnssData.altitude],
                                 [lastGnssData.latitude, lastGnssData.longitude, lastGnssData.altitude]])
            ang1 = math.atan2(enu_dir[0], enu_dir[1])
            ang2 = math.atan2(pos[1], -pos[0])
            ang = ang1 + ang2
            pos_enu = [enu_dir[0] + dis*math.cos(ang), enu_dir[1] + dis*math.sin(ang), enu_dir[2]]

            Rm = WGS84_a * (1 - WGS84_e2) / math.sqrt((1 - WGS84_e2 * math.sin(lastGnssData.latitude/180*math.pi)**2))**3
            h = lastGnssData.altitude + pos_enu[2]
            lat = lastGnssData.latitude/180*math.pi + pos_enu[1]/(Rm+h)
            Rn = WGS84_a / (1 - WGS84_e2 * math.sin(lat)**2)**0.5
            lng = lastGnssData.longitude + pos_enu[0]/(Rn+h)/math.cos(lat) / math.pi*180
            lat = lat * 180 / math.pi

            protobuf_message = gnss_pub.NavSatFix()
            protobuf_message.header.seq = lastGnssData.header.seq
            protobuf_message.header.frame_id = lastGnssData.header.frame_id
            protobuf_message.header.stamp.sec = lastGnssData.header.stamp.sec
            protobuf_message.header.stamp.nsec = lastGnssData.header.stamp.nsec
            protobuf_message.latitude = lat
            protobuf_message.longitude = lng
            protobuf_message.altitude = h
            protobuf_message.position_covariance_type = self.category[i]    # here temporarily use this as flag
            self.bumpPub.send(protobuf_message)
            time.sleep(0.05)
        self.clear()


def byte2image(byte_data):
    image = Image.open(io.BytesIO(byte_data))
    return image


if __name__ == "__main__":
    # initialize eCAL API. The name of our Process will be
    # "Python Protobuf Subscriber"

    # global variable
    gnssData = ProtoMsg(50, 3, "Nav")
    pcdData = ProtoMsg(10, 2, "PCD")
    imgData = ProtoMsg(30, 2, "IMG")

    ecal_core.initialize(sys.argv, "Bump Detector")

    bumpDetector = BumpDetector()
    while ecal_core.ok():
        if bumpDetector.bumpDetection():
            bumpDetector.sendBumpWarning()

        time.sleep(0.5)
    ecal_core.finalize()
