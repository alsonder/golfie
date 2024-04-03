import cv2
import numpy as np

def detect_size(stream, size_cm):
    # Convert size in cm to size in pixels
    # This depends on the actual camera calibration and real-world setup
    size_px = size_cm * 10  # This is just an example conversion, adjust as needed

    # Initialize the list of center coordinates
    centers = []

    while True:
        # Capture a frame from the stream
        ret, frame = stream.read()

        # Check if the frame was successfully captured
        if not ret:
            break

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect objects in the frame using a method of your choice
        # Here we use simple thresholding as an example
        _, thresholded = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # Find contours in the thresholded image
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Calculate the area of the contour
            area = cv2.contourArea(contour)

            # Check if the area of the contour is approximately equal to the desired size
            if size_px * 0.9 <= area <= size_px * 1.1:
                # Calculate the center of the contour
                M = cv2.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Add the center to the list
                centers.append((cX, cY))

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the stream
    stream.release()

    # Return the number of detected objects and their center coordinates
    return len(centers), centers