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
    # filtering out
    for i in range(len(arr)):
        if arr[i] < 300:
            clustering.labels_[i] = -1
        if arr[i] > 160000:
            clustering.labels_[i] = -1
    return clustering.labels_


def checkPlaneNum(label_array):
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


def checkWarning(counter):
    if counter > 1:
        print("Warning: Edge detected.")
        return True
    else:
        return False


def main():
    tof_distance = np.array(
        [
            [7289.7, 150.7, 96.9, 208.1, 292.8, 188.4, 58.4, 7040.9],
            [7381.7, 7408.5, 160.1, 7475.8, 7416.3, 7486.2, 7448.5, 7424.7],
            [7418.7, 7461.3, 174.7, 176.2, 260.3, 24.4, 20.7, 7353.2],
            [7360.5, 121.2, 7448.1, 128.0, 206.3, 162.9, 174.0, 7392.7],
            [184.7, 377.6, 264.0, 7476.7, 7434.1, 67.1, 169.1, 7391.5],
            [38.3, 276.2, 319.1, 259.5, 16.8, 7220.2, 7344.3, 7394.6],
            [444.3, 7390.4, 128.9, 7443.4, 7491.1, 7283.9, 133.5, 7338.9],
            [149.1, 374.8, 14.9, 7358.0, 7408.8, 7202.7, 7173.9, 7213.5],
        ]
    )

    original_data = np.reshape(tof_distance, (8, 8))
    reshaped_data = original_data.reshape(64, 1)
    label_data = plane_detection(reshaped_data)
    label_data = label_data.reshape(8, 8)
    planeNum = checkPlaneNum(label_data)
    print(original_data)
    print(label_data)
    checkWarning(planeNum)
    # tof_distance = np.load("data64px_tilt.npz")

    # for k in tof_distance.files:
    #     key = int(re.search(r"\d+", k).group())
    #     original_data = np.reshape(tof_distance[k], (8, 8))
    #     reshaped_data = original_data.reshape(64, 1)
    #     label_data = plane_detection(reshaped_data)
    #     label_data = label_data.reshape(8, 8)
    #     planeNum = checkPlaneNum(label_data)
    #     if checkWarning(planeNum):
    #         print(key)
    #         print(original_data)
    #         print(label_data)


if __name__ == "__main__":
    main()
