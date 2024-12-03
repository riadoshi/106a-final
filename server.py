import json_numpy
json_numpy.patch()

import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from typing import Any, Dict
import traceback

import cv2
import torch
import numpy as np
from transformers import DetrImageProcessor, DetrForObjectDetection

TARGET_OBJECTS = ["bottle", "cup", "orange"]  # Replace with soda cans, water bottles, etc.

class DETRModel():
  def __init__(self, threshold=0.5):
    self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50")
    self.model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50")
    self.thresh = threshold

  def detect_objects(self, image: np.ndarray):

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
                best_result = {
                    "label": label,
                    "bounding_box": [x1, y1, x2, y2],
                    "centroid": (x_center, y_center),
                    "confidence": float(confidence)
                }
    
    if best_result['confidence']>self.thresh:
        return best_result
    else:
        return None

class HttpServer:
    def __init__(self):
        # load DETR model from transformers
        self.detr_model = DETRModel(threshold=0.5)

    def json_response(self, obj):
        return JSONResponse(json_numpy.dumps(obj))
    
    def run(self, port=8000, host="0.0.0.0"):
        self.app = FastAPI()
        self.app.post("/query")(self.sample_actions)
        self.app.get("/info")(self.get_info)
        uvicorn.run(self.app, host=host, port=port)  

    def get_centroid(self, payload: Dict[Any, Any]):
        try:
            image = payload['image']  
            centroid = self.detr_model.detect_objects(image)
            return self.json_response(np.array(centroid)) 
        except:
            print(traceback.format_exc())
            return "error"


if __name__ == "__main__":
    server = HttpServer()
    server.run()  



