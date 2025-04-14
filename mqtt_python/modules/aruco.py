import cv2
import os
import numpy as np

CM_PATH = "config/camera/calibration_matrix.npy"
DIST_PATH = "config/camera/distortion_coefficients.npy"

MTX = np.load(CM_PATH)
DIST = np.load(DIST_PATH)

MARKER_SIZE = 0.035

MARKER_POINTS = np.array(
    [
        [-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
        [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
        [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    ],
    dtype=np.float32,
)

OFFSET = 0.08


ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
PARAMETERS = cv2.aruco.DetectorParameters()

# Create the ArUco detector
DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, PARAMETERS)
# plt.figure(figsize=(10, 10))


def get_pose(img, save_path=None):
    corners, ids, rejected = DETECTOR.detectMarkers(
        cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    )

    poses = {}
    rvecs, tvecs = [], []

    for _id, _corners in zip(ids, corners):
        ret, rvec, tvec = cv2.solvePnP(MARKER_POINTS, _corners, MTX, DIST)
        if ret:
            poses[_id[0]] = (rvec, tvec)

        if save_path:
            img_plot = cv2.drawFrameAxes(img, MTX, DIST, rvec, tvec, MARKER_SIZE)

    if save_path:
        cv2.imwrite(save_path, img_plot)

    return poses

def drop_point(rvec, tvec):
    offset_vector = np.dot(cv2.Rodrigues(rvec)[0], np.array([0, 0, OFFSET]))
    position = tvec.flatten() + offset_vector
    
    return position
    


# if __name__ == "__main__":
#     DIR_PATH = "C:/programmering/DTU/robobot/data/aruco"
    
#     files = os.listdir(DIR_PATH)


#     images = [cv2.imread(f"{DIR_PATH}/image_{i}.jpg") for i in range(1, len(files) + 1)]
#     # images = [cv2.imread(os.path.join(DIR_PATH, f)) for f in files]
#     images = [cv2.cvtColor(img, cv2.COLOR_BGR2RGB) for img in images]


#     for i, img in enumerate(images):
#         poses = get_pose(img)
#         for _id, (rvec, tvec) in poses.items():
#             print(f"Image {i+1}, Marker {_id}:")
#             print(f"Rotation vector: {rvec}")
#             print(f"Translation vector: {tvec}")
