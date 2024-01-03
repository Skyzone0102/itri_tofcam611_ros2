import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from . import TOFcam611_pack
import numpy as np
from sklearn.cluster import DBSCAN

# Port
PORT = "/dev/ttyUSB0"
# DBSCAN Parameters
EPSILON = 400
MIN_SAMPLE = 28


class TofCam611(Node):
    def __init__(self):
        super().__init__("TofCam611")
        self.publisher_ = self.create_publisher(Bool, "TofCam611_Warning", 10)
        self.timer_ = self.create_timer(0.0005, self.run)
        self.msg = Bool()
        self.msg.data = False
        # camera connection
        self.com = TOFcam611_pack.SerialInterface(PORT)
        self.tofCam = TOFcam611_pack.Camera(self.com)
        # set camera settings
        self.tofCam.powerOn()
        self.tofCam.setIntTime_us(500)  # integration time in µSeconds

    def plane_detection(self, arr):
        """
        Function to perform DBSCAN clustering on the input array.
        """
        clustering = DBSCAN(eps=EPSILON, min_samples=MIN_SAMPLE).fit(arr)
        # filtering out
        for i in range(len(arr)):
            if arr[i] < 300:
                clustering.labels_[i] = -1
            if arr[i] > 160000:
                clustering.labels_[i] = -1

        return clustering.labels_

    def checkPlaneNum(self, label_array):
        """
        Function to check the number of planes detected.
        """
        num = []
        counter = 0
        for i in range(3):
            num.append(np.count_nonzero(label_array == i))

        if num[1] > 10:
            num[1] = 0
            counter = 2

        return counter

    def checkWarning(self, counter):
        if counter > 1:
            print("Warning: Edge detected.")
            return True
        else:
            return False

    def run(self):
        # callback function

        tof_distance = np.array(self.tofCam.getDistance())
        original_data = np.reshape(tof_distance, (8, 8))
        reshaped_data = original_data.reshape(64, 1)
        label_data = self.plane_detection(reshaped_data)
        label_data = label_data.reshape(8, 8)
        planeNum = self.checkPlaneNum(label_data)
        print(original_data)
        print(label_data)
        if self.checkWarning(planeNum):
            self.msg.data = True
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: "%s"' % self.msg.data)

        else:
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: "%s"' % self.msg.data)


def main(args=None):
    rclpy.init(args=args)

    publisher = TofCam611()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
