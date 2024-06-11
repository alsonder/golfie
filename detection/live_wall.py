import cv2
import numpy as np
from sklearn.cluster import DBSCAN
import time

class LiveStreamProcessor:
    def __init__(self, camera_id=2):
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise Exception("Could not open camera")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
        self.last_time = time.time()

    def process_frame(self, frame):
        current_time = time.time()
        # Process the frame as before
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([50, 50, 255])
        mask = cv2.inRange(frame, lower_red, upper_red)
        red_img = cv2.bitwise_and(frame, frame, mask=mask)
        gray = cv2.cvtColor(red_img, cv2.COLOR_BGR2GRAY)
        _, bw = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        kernel = np.ones((5, 5), np.uint8)
        bw = cv2.dilate(bw, kernel, iterations=1)
        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel)
        bw = cv2.erode(bw, kernel, iterations=1)
        bw = cv2.dilate(bw, kernel, iterations=1)
        lines = cv2.HoughLinesP(bw, 1, np.pi/180, threshold=62, minLineLength=0, maxLineGap=5)

        intersections = []
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            for i in range(len(lines)):
                for j in range(i + 1, len(lines)):
                    pt = self.line_intersection(lines[i][0], lines[j][0])
                    if pt and self.is_within_bounds(pt, lines[i][0]) and self.is_within_bounds(pt, lines[j][0]):
                        intersections.append(pt)
                        cv2.circle(frame, (int(pt[0]), int(pt[1])), 5, (255, 0, 0), -1)

        # DBSCAN clustering and centroid calculation every 5 seconds
        if current_time - self.last_time >= 5:
            self.last_time = current_time
            if intersections:
                dbscan = DBSCAN(eps=10, min_samples=2)
                points = np.array(intersections)
                clusters = dbscan.fit_predict(points)
                centroids = [points[clusters == i].mean(axis=0) for i in range(max(clusters) + 1) if i != -1]
                
                # Select the four most central centroids
                if len(centroids) > 4:
                    centroids = sorted(centroids, key=lambda x: (x[0] - frame.shape[1]/2)**2 + (x[1] - frame.shape[0]/2)**2)[:4]

                for centroid in centroids:
                    cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 10, (255, 255, 0), 2)
                    print(f"Cluster centroid at {current_time}: {centroid}")

        cv2.imshow('Live Stream', frame)

    def line_intersection(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denominator == 0:
            return None  # Lines are parallel
        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator
        return (px, py)

    def is_within_bounds(self, pt, line):
        x, y = pt
        x1, y1, x2, y2 = line
        return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            self.process_frame(frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

if __name__ == "__main__":
    processor = LiveStreamProcessor()
    processor.run()
