import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import TOFcam611
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import re

# Port
PORT = "/dev/ttyUSB0"
# DBSCAN Parameters
EPSILON = 400
MIN_SAMPLE = 10


class TofCam611(Node):
    def __init__(self):
        super().__init__("TofCam611")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        # camera connection
        self.com = TOFcam611.SerialInterface(PORT)
        self.tofCam = TOFcam611.Camera(self.com)
        # set camera settings
        self.tofCam.powerOn()
        self.tofCam.setIntTime_us(500)  # integration time in µSeconds

    def plane_detection(self, arr):
        """
        Function to perform DBSCAN clustering on the input array.
        """
        clustering = DBSCAN(eps=EPSILON, min_samples=MIN_SAMPLE).fit(arr)
        return clustering.labels_

    def checkPlaneNum(self, label_array):
        """
        Function to check the number of planes detected.
        """
        num = []
        counter = 0
        for i in range(3):
            num.append(np.count_nonzero(label_array == i))

        for i in range(len(num)):
            if num[i] != 0:
                counter += 1

        return counter

    def checkWarning(self, counter):
        if counter > 1:
            print("Warning: Edge detected.")
            return True
        else:
            return False

    def run(self):
        # callback function
        while True:
            tof_distance = np.array(self.tofCam.getDistance())
            original_data = np.reshape(tof_distance, (8, 8))
            reshaped_data = original_data.reshape(64, 1)
            label_data = self.plane_detection(reshaped_data)
            label_data = label_data.reshape(8, 8)
            planeNum = self.checkPlaneNum(label_data)
            if self.checkWarning(planeNum):
                msg = String()
                msg.data = "Warning: Edge detected."
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
                print(original_data)
                print(label_data)


def main(args=None):
    rclpy.init(args=args)

    publisher = TofCam611()
    publisher.run()

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
