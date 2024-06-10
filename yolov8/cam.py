import cv2
import pandas as pd
from ultralytics import YOLO
import cvzone
import numpy as np

cap = cv2.VideoCapture(-1)  # Use default camera (0) or change to desired camera index

# Set camera properties
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

model = YOLO('yolov8n.pt') #This loads a pre-trained YOLO model from the file yolov8n.pt

#This opens a file named coco.txt in read mode. This file typically contains the list of class names for the COCO dataset.
my_file = open("coco.txt", "r") #
data = my_file.read()
class_list = data.split("\n")

count = 0

# Starts an infinite loop to continuously capture and process video frames.
while True:
    ret, frame = cap.read() 
    if not ret:# Checks if the frame was not captured successfully
        break #Breaks out of the loop if no frame was captured.

    count += 1 #Increments the frame counter by one
    if count % 3 != 0: #Checks if the current frame count is not a multiple of three.
        continue #Skips processing for this frame if the frame count is not a multiple of three. This effectively processes every third frame.

    # frame = cv2.flip(frame, -1)
    cv2.imshow("Camera", frame) # Displays the flipped frame in a window named "Camera".

    results = model.predict(frame) #Uses the YOLO model to predict objects in the current frame.
    a = results[0].boxes.data #Extracts the bounding box data from the prediction results.

    px = pd.DataFrame(a).astype("float") #Converts the bounding box data to a Pandas DataFrame and ensures all data is of type float.

    for index, row in px.iterrows():
        x1 = int(row[0]) # Extracts and converts the x-coordinate of the top-left corner of the bounding box to an integer.
        y1 = int(row[1]) # Extracts and converts the y-coordinate of the top-left corner of the bounding box to an integer.
        x2 = int(row[2]) # Extracts and converts the x-coordinate of the bottom-right corner of the bounding box to an integer
        y2 = int(row[3]) # Extracts and converts the y-coordinate of the bottom-right corner of the bounding box to an integer.
        d = int(row[5]) #Extracts and converts the class index of the detected object to an integer
        c = class_list[d] #Retrieves the class name corresponding to the class index.

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2) # Draws a red rectangle around the detected object.
        cvzone.putTextRect(frame, f'{c}', (x1, y1), 1, 1) # Puts a text label with the class name near the top-left corner of the bounding box.

    cv2.imshow("Camera", frame) # Displays the frame with the bounding boxes and labels in the "Camera" window.


    if cv2.waitKey(1) == ord('q'):
        break

cap.release() # Releases the video capture object.
cv2.destroyAllWindows()
