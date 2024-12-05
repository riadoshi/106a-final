import json_numpy
json_numpy.patch()

import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from typing import Any, Dict
import traceback

from PIL import ImageDraw
import cv2
import torch
import numpy as np
from transformers import DetrImageProcessor, DetrForObjectDetection

from PIL import Image

TARGET_OBJECTS = ["bottle", "cup", "banana"]  # Replace with soda cans, water bottles, etc.

class DETRModel():
  def __init__(self, threshold=0):
    self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
    self.model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50")
    self.thresh = threshold

  def detect_objects(self, image: np.ndarray, start_row:int, start_col:int):

    # convert img to rgb
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    inputs = self.processor(images=rgb_image, return_tensors="pt")


    with torch.no_grad():
        outputs = self.model(**inputs)

    logits = outputs.logits[0]  # ogits
    boxes = outputs.pred_boxes[0]  # bboxes (normalized [0, 1])
    probs = logits.softmax(-1)
    confidences, class_ids = probs[..., :-1].max(-1)  # ignore "no-object" class

    # unnormalize
    img_height, img_width = rgb_image.shape[:2]
    boxes = boxes * torch.tensor([img_width, img_height, img_width, img_height], dtype=torch.float)

    # filter for best
    id2label = self.model.config.id2label
    max_conf = 0
    best_result = {}
    for i, confidence in enumerate(confidences):
        if confidence>max_conf:
            label = id2label[class_ids[i].item()]
            if label in TARGET_OBJECTS:
                x_center, y_center, width, height = boxes[i].tolist()
                x1 = int(x_center - width / 2)
                y1 = int(y_center - height / 2)
                x2 = int(x_center + width / 2)
                y2 = int(y_center + height / 2)

                global_centroid = (start_row+x_center, start_col+y_center)
                best_result = {
                    "label": label,
                    "bounding_box": [x1, y1, x2, y2],
                    "centroid": global_centroid,
                    "cropped_centroid": (x_center, y_center),
                    "confidence": float(confidence)
                }
    if 'confidence' not in best_result:
        return "nothing found"
    elif best_result['confidence']>self.thresh:
        return best_result
    else:
        return "things found, but didn't meet threshold"

class HttpServer:
    def __init__(self):
        # load DETR model from transformers
        print("loading DETR")
        self.detr_model = DETRModel(threshold=0)

    def json_response(self, obj):
        return JSONResponse(json_numpy.dumps(obj))
    
    def run(self, port=8000, host="0.0.0.0"):
        self.app = FastAPI()
        self.app.post("/query")(self.get_centroid)
        uvicorn.run(self.app, host=host, port=port)  

    def get_centroid(self, payload: Dict):
        print("in centroid method!")
        try:
            image = np.array(payload['image'])
            start_row = int(0.55 * image.shape[0]) # 50% of height
            end_row = int(0.75*image.shape[0])

            start_col = int(0.35*image.shape[1])
            end_col = int(0.6*image.shape[1])
            rgb_image = image[start_row:end_row, start_col:end_col, :]
            rgb_image = rgb_image.astype(np.uint8)

            print(rgb_image.shape)

            # rgb_image = cv2.resize(rgb_image, (rgb_image.shape[1] // 2, rgb_image.shape[0] // 2))  # Downsample to half size

            print("Detecting object!")

            pilimg = Image.fromarray(rgb_image)
            pilimg.save('pilimg.png')

            centroid = self.detr_model.detect_objects(rgb_image, start_row, start_col)
            print(centroid['cropped_centroid'])


            draw = ImageDraw.Draw(pilimg)
            point_radius = 5
            point_color = (255, 0, 0)  # Red color

            # Draw a circle to represent the point
            x, y = centroid['cropped_centroid']
            draw.ellipse(
                [(x - point_radius, y - point_radius), (x + point_radius, y + point_radius)],
                fill=point_color,
                outline=point_color,
            )

            # Save the modified image
            output_path = "output_image_with_point.png"
            pilimg.save(output_path)
            
            return self.json_response(centroid)
        except:
            print(traceback.format_exc())
            return "error"


if __name__ == "__main__":
    server = HttpServer()
    server.run()  



# rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
