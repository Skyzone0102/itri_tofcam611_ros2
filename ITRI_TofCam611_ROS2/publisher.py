import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import TOFcam611
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import re

# #DBSCAN
epsilon = 70
minSample = 7

class TofCam611(Node):
    def __init__(self):
        super().__init__("TofCam611")
        # self.publisher_ = self.create_publisher(String, "topic", 10)
        # # camera connection
        # self.com = TOFcam611.SerialInterface("COM4")
        # self.tofCam = TOFcam611.Camera(self.com)
        # # set camera settings
        # self.camera.powerOn()
        # self.camera.setIntTime_us(500)  # integration time in µSeconds

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = "Hello World: %d" % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1
    
    # def filterArray(self, array):
    #     #smooth by using gaussian filter
    #     sigma = sigmaG
    #     filtered_arr = gaussian_filter(array, sigma)
    #     return filtered_arr

    def planeDetection(self, arr):
        clustering = DBSCAN(eps=epsilon, min_samples=minSample).fit(arr)
        return clustering.labels_
    
    # def planeDepthDetection(self,label_array,depth_array):
    #     depth = []
    #     depth1 = [] #store the total depth and the number added
    #     depth2 = []
    #     depth3 = []
        
    #     for i in range(7):
    #         for j in range(8):
    #             if label_array[i][j] == 0:
    #                  depth1.append(depth_array[i][j])
    #             elif label_array[i][j]==1:
    #                 depth2.append(depth_array[i][j])
    #             elif label_array[i][j] == 2:
    #                 depth3.append(depth_array[i][j])
                        
    #     depth.append(depth1)
    #     depth.append(depth2)
    #     depth.append(depth3)
        
        
    #     num = []         
    #     for i in range(3):
    #         temp_depth = 0
    #         num.append(len(depth[i]))
    #         for j in range(num[i]):
    #             temp_depth +=depth[i][j]
    #         if num [i]==0:
    #             depth[i] = 0
    #         else:
    #             depth[i] = temp_depth / num[i]
        
    #     before_depth = depth.copy()
    #     depth = [x for x in depth if x != 0]
    #     #sorting
    #     depth = self.insertionSort(depth)
    #     # for i in range(len(depth)):
    #     #     if i < len(depth)-1:
    #     #         if(depth[i+1]-depth[i]) < 300:
    #     #             print("what the hell")
            

    #     #comparison remapping
    #     swap=[]
    #     for i in range(len(depth)):
    #         for j in range(len(before_depth)):
    #             if depth[i]==before_depth[j]:
    #                 swap.append(j)
    #     print(swap)
    #     # if abs(depth[0]-depth[1]) < 100:
    #     #     depth[0] = (depth[0] * num[0] + depth[1] * num[1])/(num[0]+num[1])
    #     #     depth[1] = 0
        
    #     return depth,swap            

    # def insertionSort(self,nums):
    #     for i in range(1, len(nums)) :
    #         j, fixed = i-1, nums[i]
    #         while j >=0 and nums[j] > fixed:
    #             nums[j+1] = nums[j]
    #             j -= 1
    #         nums[j+1] = fixed
    #     return nums    

    # def swapLabel(self,label_array,swap):
    #     label_filtered_array = label_array.copy()
    #     label_filtered_array = label_array.reshape(56,1)
            
    #     for i in range(len(label_filtered_array)):
    #         for j in range(len(swap)):
    #             if label_filtered_array[i] == j:
    #                 label_filtered_array[i] = swap[j]
    #                 break

    #     label_filtered_array=label_filtered_array.reshape(7,8)
    #     return label_filtered_array
    
    # def checkWarning(self,swap,check):
    #     status = False
    #     if self.counter == 0:
    #         if len(swap) > 1: # 2 plane?
    #             # print("warning")
    #             self.counter +=1
    #             check.append(True)
    
    #     elif self.counter ==1 or self.counter ==2 or self.counter ==3 :
    #         self.counter +=1
    #         if len(swap) > 1:
    #             # print("warning")
    #             check.append(True)
    #     elif self.counter ==4:
    #         if len(swap) > 1:
    #             # print("warning")
    #             check.append(True)
    #         if len(check) >=3:
    #             status = True
    #             msg = String()
    #             msg.data = 'Warning' 
    #             self.publisher_.publish(msg)
    #             self.get_logger().info('Publishing: "%s"' % msg.data)
    #         check.clear()  
    #         self.counter = 0
    #         return status
    

    def run(self):

        # callback function
        while True:
            # tof_distance = np.array(self.camera.getDistance())
            tof_distance = np.load("data_64px_1hr.npz")
            # print("TOF distance image:")
            # print(np.around(tof_distance, decimals=1))
            for k in tof_distance.files:
                key = int(re.search(r"\d+", k).group())
                reshaped_data = np.reshape(tof_distance[k], (1, 64))
                print(self.planeDetection(reshaped_data[0]))


            # # reshape into
            # tof_distance_reshape = tof_distance.reshape(64,1)
            
            # # kick out (16200) (too small)
            # tof_distance_reshape = tof_distance_reshape[tof_distance_reshape!=16200]
                
            # # DBscan
            # tof_distance_reshape = self.planeDetection(tof_distance_reshape)

            # # smooth the plane(Gaussian blur)----------------------------------------------------
            # filtered_array = self.filterArray(depth_array)
            # # labeling plane------------------------------------------------------------
            # label_array = self.planeDetection(filtered_array)

            # # remapping label and calculate the distance
            # planeDepth, swap = self.planeDepthDetection(label_array, depth_array)

            # warning = self.checkWarning(swap, check)
            # print (warning)
            # -----------------------------------------------------------
            


def main(args=None):
    rclpy.init(args=args)

    publisher = TofCam611()
    publisher.run()
    # rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()