# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2

def find_marker(image):
    # Convert the image to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)
    # Find the contours in the edged image and keep the largest one
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    if len(cnts) == 0:
        return None
    c = max(cnts, key=cv2.contourArea)
    # Compute the bounding box of the paper region and return it
    return cv2.minAreaRect(c)

def distance_to_camera(knownHeight, focalLength, perHeight):
    # Compute and return the distance from the marker to the camera
    return (knownHeight * focalLength) / perHeight

def main():
    # Define the known width of the object and the focal length of the camera
    KNOWN_HEIGHT = 4.5  # Example width in inches
    focalLength = 4850  # Example focal length in pixels

    # Open the camera
    cap = cv2.VideoCapture(0)  # 0 for default camera

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        # Find the marker in the image
        marker = find_marker(frame)

        if marker is not None:
            # Compute the distance to the marker
            inches = distance_to_camera(KNOWN_HEIGHT, focalLength, marker[1][0])
            # Draw a bounding box around the image and display it
            box = cv2.boxPoints(marker)
            box = np.int0(box)
            cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
            cv2.putText(frame, "%.2fft" % (inches / 12),
                        (frame.shape[1] - 200, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        2.0, (0, 255, 0), 3)

        # Display the resulting frame
        cv2.imshow("Frame", frame)

        # Exit loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


# Mismo código que DistanceCalcGPT.py pero enfocado en altura (Height) y algunos ajustes pequeños