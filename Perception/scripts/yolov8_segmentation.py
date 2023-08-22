from ultralytics import YOLO
model = YOLO("/home/bhavay/catkin_ws/src/flipkartGrid/YoloModel/yolov8m-seg-custom.pt")
model.predict(source="/home/bhavay/catkin_ws/src/flipkartGrid/YoloModel/sample_png/images.png", show=True, save=True, hide_labels=False, hide_conf=False, conf=0.5, save_txt=False, save_crop=False, line_thickness= 2 )
