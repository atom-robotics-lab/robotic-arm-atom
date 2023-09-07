from ultralytics import YOLO
import cv2

model = YOLO("nag_raj_description/scripts/model.pt")
image=cv2.imread("nag_raj_description/scripts/image.png")
results = model.predict(source=image,conf=0.2) # Display preds. Accepts all YOLO predict arguments
print(results[0].boxes.xywh)  # Boxes object for bbox outputs
for i in results[0].boxes.xywh:
    cv2.circle(image,(int(i[0]),int(i[1])),5,(0,0,255),2)
cv2.imshow("Image",image)
cv2.waitKey(0)
