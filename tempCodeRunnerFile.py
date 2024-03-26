        if aruco_position and aruco_orientation:
            front_point = calculate_front_point(aruco_position[0][0:2].flatten(), aruco_orientation[0], 1.5)
            # Draw the front point as an orange dot
            cv2.circle(frame, front_point, 5, (0, 165, 255), -1)