import TOFcam611_pack
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import re

# Constants
EPSILON = 400
MIN_SAMPLE = 10


def plane_detection(arr):
    """
    Function to perform DBSCAN clustering on the input array.
    """
    clustering = DBSCAN(eps=EPSILON, min_samples=MIN_SAMPLE).fit(arr)
    return clustering.labels_


def checkPlaneNum(label_array):
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


def checkWarning(counter):
    if counter > 1:
        print("Warning: Edge detected.")
        return True
    else:
        return False


def main():
    tof_distance = np.load("data64px_tilt.npz")
    for k in tof_distance.files:
        key = int(re.search(r"\d+", k).group())
        original_data = np.reshape(tof_distance[k], (8, 8))
        reshaped_data = original_data.reshape(64, 1)
        label_data = plane_detection(reshaped_data)
        label_data = label_data.reshape(8, 8)
        planeNum = checkPlaneNum(label_data)
        if checkWarning(planeNum):
            print(key)
            print(original_data)
            print(label_data)


if __name__ == "__main__":
    main()
