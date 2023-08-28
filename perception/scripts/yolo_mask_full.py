from ultralytics import YOLO
import matplotlib.path as mpl_path
import cv2
model = YOLO("/home/bhavay/catkin_ws/src/flipkartGrid/perception/scripts/ml_models/yolov8m-seg-custom.pt")
results=model.predict(source="/home/bhavay/catkin_ws/src/flipkartGrid/perception/scripts/image.png", show=True, save=False, hide_labels=False, hide_conf=False, conf=0.5, save_txt=False, save_crop=False, line_thickness= 2 )
masks=[]
bounding_boxes=[]
image=cv2.imread("/home/bhavay/catkin_ws/src/flipkartGrid/perception/scripts/image.png")
for result in results:
    for mask in result.masks:
        print(mask.xy[0])
        masks.append(mask.xy[0])
    for box in result.boxes:
        # print(box.xyxy[0])
        bounding_boxes.append(box.xyxy[0])

def transform_mask(mask,bounding_box):
    min_x = int(bounding_box[0])
    max_x = int(bounding_box[2])
    min_y = int(bounding_box[1])
    max_y = int(bounding_box[3])
            
    # print("mymask:",mask)

    polygon_path = mpl_path.Path(mask)

    points_inside = []

    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            if polygon_path.contains_point((x,y)):
                points_inside.append((x, y))

    return points_inside
        


for i in range(len(mask)):
    mask[i]=transform_mask(mask[i],bounding_boxes[i])

for i in masks:
    for x,y in i:
        cv2.circle(image,(int(x),int(y)),1,(0,0,255),2)
cv2.imshow("image",image)
cv2.waitKey(0)
