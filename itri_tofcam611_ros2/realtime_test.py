import TOFcam611_pack
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import re

# Port
PORT = "COM4"
# DBSCAN Parameters
EPSILON = 600
MIN_SAMPLE = 28


class TofCam611():
    def __init__(self):
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
        #filtering out 
        for i in range(len(arr)):
            if arr[i] > 160000:
                clustering.labels_[i] = -1
        # print(clustering.labels_)
        return clustering.labels_

    def checkPlaneNum(self, label_array):
        """
        Function to check the number of planes detected.
        """
        num = []
        counter = 0
        # print(label_array)
        for i in range(3):
            num.append(np.count_nonzero(label_array == i))

        for i in range(len(num)):
            if num[i] != 0:
                counter += 1
        # print(num)
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
            # print(original_data)
            label_data = label_data.reshape(8, 8)
            planeNum = self.checkPlaneNum(label_data)
            if self.checkWarning(planeNum):
                # msg = String()
                # msg.data = "Warning: Edge detected."
                # self.publisher_.publish(msg)
                # self.get_logger().info('Publishing: "%s"' % msg.data)
                print(original_data)
                print(label_data)
                # print("Warning: Edge detected.")


def main(args=None):

    publisher = TofCam611()
    publisher.run()


if __name__ == "__main__":
    main()
