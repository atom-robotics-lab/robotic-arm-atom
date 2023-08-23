from ultralytics import YOLO
model = YOLO("/home/bhavay/catkin_ws/src/flipkartGrid/Perception/scripts/yolov8m-seg-custom.pt")
results=model.predict(source="/home/bhavay/catkin_ws/src/flipkartGrid/Perception/scripts/image.png", show=True, save=False, hide_labels=False, hide_conf=False, conf=0.5, save_txt=False, save_crop=False, line_thickness= 2 )
for result in results:
    for mask in result.masks:
        print(mask.xy[0])
    # print(result.masks.xy)