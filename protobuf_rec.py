import io
import sys
import time
import numpy as np

import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber

# Import the "hello_world_pb2.py" file that we have just generated from the
# proto_messages directory
import proto_messages.hello_world_pb2 as hello_world_pb2
import datatypes.ros.sensor_msgs.PointCloud2_pb2 as point_cloud_pb
import datatypes.ros.sensor_msgs.CompressedImage_pb2 as img_pb
import open3d as o3d
from PIL import Image
from pointcloud2 import read_points
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

pc_total = []
img_total = []


def byte2image(byte_data):
    image = Image.open(io.BytesIO(byte_data))
    return image


# Callback for receiving messages
def callback(topic_name, proto_msg, time):
    # print("Message {} from {}: {}".format(proto_msg.id
    #                                       , proto_msg.name
    #                                       , proto_msg.msg))
    # print("Width {}, Height {}".format(proto_msg.width, proto_msg.height))
    # pc = o3d.convertCloudFromRosToOpen3d(proto_msg)
    # pc_total.append(proto_msg)
    img_total.append(proto_msg)
    # print("data: {}".format(proto_msg.fields))


if __name__ == "__main__":
    # initialize eCAL API. The name of our Process will be
    # "Python Protobuf Subscriber"
    ecal_core.initialize(sys.argv, "Python Protobuf Subscriber")

    # Create a Protobuf Publisher that publishes on the topic
    # "hello_world_python_protobuf_topic". The second parameter tells eCAL which
    # datatype we are expecting to receive on that topic.
    # sub = ProtoSubscriber("hello_world_python_protobuf_topic"
    #                       , hello_world_pb2.HelloWorld)

    # point cloud data
    # sub = ProtoSubscriber("ROSVLS128CenterCenterRoof"
    #                       , point_cloud_pb.PointCloud2)    # Set the Callback

    sub = ProtoSubscriber("ROSFrontCenterImage", img_pb.CompressedImage)

    sub.set_callback(callback)

    # Just don't exit
    while ecal_core.ok():
        if len(img_total) >= 3:
            break
        time.sleep(0.5)

    img = img_total[0]
    img_conv = byte2image(img.data)
    img_conv.show()
    for j in range(len(pc_total)):
        if not j % 10 == 0:
            continue
        pc = read_points(pc_total[0])
    # o3d.visualization.draw_geometries(pc)
        x = []
        y = []
        z = []
        cnt = len(pc)
        pc_data = np.zeros((cnt, 3))
        for i in range(len(pc)):
            # x.append(pc[i][0])
            # y.append(pc[i][1])
            # z.append(pc[i][2])
            pc_data[i][0] = pc[i][0]
            pc_data[i][1] = pc[i][1]
            pc_data[i][2] = pc[i][2]

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pc_data)
        # o3d.visualization.draw_geometries([point_cloud])
        o3d.io.write_point_cloud("test" + str(j) + ".pcd", point_cloud)
    # fig = plt.figure() # 创建一个画布figure，然后在这个画布上加各种元素。
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(x,y,z) # 画出(xs1,ys1,zs1)的散点图。 # ax1.scatter3D(x,y,z, cmap='Blues')  #绘制散点图    b = a + 1

    # c: 颜色  可为单个，可为序列    # finalize eCAL API
    # depthshade: 是否为散点标记着色以呈现深度外观。对 scatter() 的每次调用都将独立执行其深度着色。
    # marker：样式
    ecal_core.finalize()
    
