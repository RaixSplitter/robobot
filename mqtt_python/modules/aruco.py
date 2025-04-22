import cv2
import os
import numpy as np

CM_PATH = "config/camera/calibration_matrix.npy"
DIST_PATH = "config/camera/distortion_coefficients.npy"

MTX = np.load(CM_PATH)
DIST = np.load(DIST_PATH)

MARKER_SIZE = 0.035


OFFSET = 0.08


ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
ARUCO_MAP = {
	# Sorting Center, MARKERSIZE 10 CM, 0.1 M
	10: ("A", 0.1),
	11: ("A", 0.1),
	12: ("B", 0.1),
	13: ("B", 0.1),
	14: ("C", 0.1),
	15: ("C", 0.1),
	16: ("D", 0.1),
	17: ("D", 0.1),
	# Luggage, MARKERSIZE 4 CM, 0.04 M
	5: ("car", 0.035),  # 0.35
	20: ("LA", 0.035),
	53: ("LD", 0.035),
	# EXIT
	25: ("EXIT", 0.1),
	# EXTRA
	18: ("extra1", 0.1),
	19: ("extra2", 0.1),
}


PARAMETERS = cv2.aruco.DetectorParameters()

# Create the ArUco detector
DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, PARAMETERS)


# plt.figure(figsize=(10, 10))
def get_marker_points(marker_size) -> np.array:
	marker_points = np.array(
		[
			[-marker_size / 2, marker_size / 2, 0],
			[marker_size / 2, marker_size / 2, 0],
			[marker_size / 2, -marker_size / 2, 0],
			[-marker_size / 2, -marker_size / 2, 0],
		],
		dtype=np.float32,
	)
	
	return marker_points


def get_pose(img, save_path=None) -> dict:
	corners, ids, rejected = DETECTOR.detectMarkers(
		cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	)
	
	if corners is None or ids is None:
		if save_path:
			cv2.imwrite(save_path, img)
		return {}

	poses = {}

	for _id, _corners in zip(ids, corners):
		
		identifier, _marker_size = ARUCO_MAP.get(_id[0], (None, None))
		print(_id, identifier, _marker_size)
		# assert identifier
		# assert _marker_size
		
		_marker_points = get_marker_points(_marker_size)
		
		ret, rvec, tvec = cv2.solvePnP(_marker_points, _corners, MTX, DIST)
		if ret:
			poses[_id[0]] = (rvec, tvec, identifier)

		if save_path:
			img_plot = cv2.drawFrameAxes(img, MTX, DIST, rvec, tvec, _marker_size)

	if save_path:
		cv2.imwrite(save_path, img_plot)

	return poses


def drop_point(rvec, tvec, delivery=True, offset=OFFSET, show = False, img = None):
	if delivery:
		offset_vector = np.dot(cv2.Rodrigues(rvec)[0], np.array([0, 0, offset]))
	else:
		offset_vector = np.dot(cv2.Rodrigues(rvec)[0], np.array([offset, 0, 0]))
	position = tvec.flatten() + offset_vector
 
	if show:
		projected_point, _ = cv2.projectPoints(position.reshape(-1, 3), np.zeros((3,1)), np.zeros((3,1)), MTX, DIST)
		center = tuple(projected_point.ravel().astype(int))
		img = cv2.circle(img, center, radius=5, color=(0, 255, 0), thickness=-1)
		cv2.imwrite("TARGETVISION.jpg", img)

	return position

# if __name__ == "__main__":
#     DIR_PATH = "C:/programmering/DTU/robobot/data/aruco"

#     files = os.listdir(DIR_PATH)


#     images = [cv2.imread(f"{DIR_PATH}/image_{i}.jpg") for i in range(1, len(files) + 1)]
#     # images = [cv2.imread(os.path.join(DIR_PATH, f)) for f in files]
#     images = [cv2.cvtColor(img, cv2.COLOR_BGR2RGB) for img in images]


#     for i, img in enumerate(images):
#         poses = get_pose(img)
#         for _id, (rvec, tvec, identifier) in poses.items():
#             print(f"Image {i+1}, Marker {_id}:")
#             print(f"Rotation vector: {rvec}")
#             print(f"Translation vector: {tvec}")
