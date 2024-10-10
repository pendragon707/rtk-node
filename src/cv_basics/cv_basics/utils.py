import cv2
import numpy as np

def get_dominant_color(image, n_colors):
    pixels = np.float32(image).reshape((-1, 3)) 
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1)
    flags = cv2.KMEANS_RANDOM_CENTERS
    flags, labels, centroids = cv2.kmeans(
        pixels, n_colors, None, criteria, 10, flags)
    # palette = np.uint8(centroids)[0]
    palette = np.array(centroids)
    values, counts = np.unique(labels, return_counts=True)    

    return palette[np.argmax( counts ) ]

def homotography(x, y, m):
    xn = m[0]*x + m[3]*y + m[6]
    yn = m[1]*y + m[4]*x + m[7]
    w = m[2]*x + m[5]*y + m[8]
    xn /= w
    yn /= w
    return [xn, yn]

def get_frame(object_detected):
    width = int(object_detected.shape[0])
    height = int(object_detected.shape[1])
    m = object_detected[2:11]
    tl = homotography(0, 0, m)
    tr = homotography(width, 0, m)
    bl = homotography(0, height, m)
    br = homotography(width, height, m)
    L = [tl,tr,bl,br]
    ctr = np.array(L).reshape((-1,1,2)).astype(np.int32)
    return cv2.boundingRect(ctr)
    
def resize(im, new_shape=(224, 224)):
    """
    Resize and pad image while preserving aspect ratio.

    Parameters
    ----------
    im : np.ndarray
        Image to be resized.
    new_shape : Tuple[int]
        Size of the new image.

    Returns
    -------
    np.ndarray
        Resized image.
    """
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    # Compute padding
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))  # add border
    return im


def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

def get_focal_length(measured_distance, real_width, width_in_rf_image): 
    focal_length = (width_in_rf_image* measured_distance)/ real_width 
    return focal_length 

def distance_to_camera(knownWidth, focalLength, perWidth):	
	return (knownWidth * focalLength) / perWidth