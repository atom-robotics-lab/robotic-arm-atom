from ultralytics import YOLO
from PIL import Image
import cv2

model = YOLO("nag_raj_description/scripts/model.pt")
# accepts all formats - image/dir/Path/URL/video/PIL/ndarray. 0 for webcam
# results = model.predict(source="0")
image=cv2.imread("nag_raj_description/scripts/image.png")
results = model.predict(source=image, show=True,save=True) # Display preds. Accepts all YOLO predict arguments
for result in results:
    print(result.boxes.xywh)  # Boxes object for bbox outputs
    # masks = result.masks  # Masks object for segmentation masks outputs
    # keypoints = result.keypoints  # Keypoints object for pose outputs
    # probs = result.probs  # Class probabilities for classification outputs

# from PIL
# im1 = Image.open("bus.jpg")
# results = model.predict(source=im1, save=True)  # save plotted images

# # from ndarray
# im2 = cv2.imread("bus.jpg")
# results = model.predict(source=im2, save=True, save_txt=True)  # save predictions as labels

# # from list of PIL/ndarray
# results = model.predict(source=[im1, im2])
