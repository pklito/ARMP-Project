import cv2
import numpy as np
from filterpy.kalman import KalmanFilter

class BallTracker:
    def __init__(self, initial_state=[0, 0, 0, 0], process_noise_std=1.0, measurement_noise_std=2.0):
        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # State transition matrix (considering constant velocity model)
        self.kf.F = np.array([[1, 0, 1, 0],
                              [0, 1, 0, 1],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

        # Measurement function
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]])

        # Initial state covariance
        self.kf.P *= 1000.0

        # Measurement noise covariance
        self.kf.Q = np.array([[2, 0, 0, 0],
                      [0, 2, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        self.kf.R = np.array([[6, 0],
                            [0, 6]])

        # Initial state
        self.kf.x = np.array(initial_state)

    def predict(self):
        self.kf.predict()

    def update(self, x, y):
        self.kf.update([x, y])

    def get_state(self):
        return self.kf.x[:2]

# Function to detect ball (returns bounding circles)
def detect_ball(frame):
    frame = cv2.GaussianBlur(frame, (17, 17), 0)
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_limit1, upper_limit1 = (0, 154, 146), (9, 255, 255)
    lower_limit2, upper_limit2 = (176, 154, 146), (179, 255, 255)

    mask1 = cv2.inRange(hsv_image, lower_limit1, upper_limit1)
    mask2 = cv2.inRange(hsv_image, lower_limit2, upper_limit2)
    mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bounding_circles = []
    for contour in contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        bounding_circles.append((int(x), int(y), int(radius)))

    return bounding_circles

# Real-time tracking loop
# camera = CameraStreamer()
cap = cv2.VideoCapture(0)
tracker = BallTracker()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    bounding_circles = detect_ball(frame)
    if bounding_circles:
        x, y, radius = bounding_circles[0]
        tracker.update(x, y)
    else:
        # Use the last known position if no ball is detected
        tracker.update(tracker.kf.x[0], tracker.kf.x[1])

    tracker.predict()
    state = tracker.get_state()

    cv2.circle(frame, (int(state[0]), int(state[1])), radius, (0, 255, 0), 2) # predicted Center is Green
    cv2.circle(frame, (x, y), radius, (0, 0, 255), 2) # circle Center detected by the camera  is Red
    print(f'Center: ({x}, {y}). ' + f' Predicted Center: ({int(state[0]), int(state[1])}). ' + f'Difference: ({x - int(state[0]), y - int(state[1])})')
    cv2.imshow('Ball Tracking', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
