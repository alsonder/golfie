
# this file is only for generating imagsdes for training the model to detect balls

import cv2
import time
import os
from gtts import gTTS
from pygame import mixer

def capture_images(interval=5, output_dir='images'):
    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Open the default camera
    cap = cv2.VideoCapture(0)

    # Check if the camera is opened correctly
    if not cap.isOpened():
        print("Cannot open camera")
        return

    # Initialize the mixer for playing audio
    mixer.init()

    try:
        count = 0
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            # If frame is read correctly, ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            # Draw countdown on the frame
            countdown = interval - 1 - (count % interval)
            cv2.putText(frame, str(countdown), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Generate voice for the countdown
            tts = gTTS(text=str(countdown), lang='en')
            tts.save("countdown.mp3")
            mixer.music.load("countdown.mp3")
            mixer.music.play()

            # Display the resulting frame
            cv2.imshow('frame', frame)

            # Save frame as image when countdown reaches 0
            if countdown == 0:
                img_name = os.path.join(output_dir, f'image_{int((count + 1) / interval)}.png')
                cv2.imwrite(img_name, frame)
                print(f'Saved image: {img_name}')

                # Print the current iteration count
                print(f'Iteration: {int((count + 1) / interval)}')

            count += 1

            # Wait for 1 second
            time.sleep(1)

            # Break the loop when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_images()