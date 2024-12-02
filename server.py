import json_numpy
json_numpy.patch()

import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from typing import Any, Dict
import traceback


import torch
import torchvision
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from transformers import DetrImageProcessor, DetrForObjectDetection
import os

class CocoDetection(torchvision.datasets.CocoDetection):
    def __init__(self, img_folder=None, processor=None, train=True):
        if img_folder:
            ann_file = os.path.join(img_folder, "train.json" if train else "val.json")
            super(CocoDetection, self).__init__('', ann_file)
        self.processor = processor

    def __getitem__(self, idx):
        # read in PIL image and target in COCO format
        img, target = super(CocoDetection, self).__getitem__(idx)

        # preprocess image and target (converting target to DETR format, resizing + normalization of both image and target)
        image_id = self.ids[idx]
        target = {'image_id': image_id, 'annotations': target}
        encoding = self.processor(images=img, annotations=target, return_tensors="pt")
        pixel_values = encoding["pixel_values"].squeeze() # remove batch dimension
        target = encoding["labels"][0] # remove batch dimension

        return pixel_values, target
  
    def process_img(self, pil_img):
        encoding = self.processor(images=pil_img, return_tensors="pt")
        pixel_values = encoding["pixel_values"].squeeze() # remove batch dimension

        return pixel_values

class DETRModel():
  def __init__(self, model_name, val_path):
    self.model = DetrForObjectDetection.from_pretrained(f"rdoshi21/{model_name}", id2label={0:"plastic cup", 1: "metal water bottle", 2: "soda can", 3: "plastic sprayer"})
    self.device = torch.device(f"cuda:1" if torch.cuda.is_available() else "cpu")
    self.processor = DetrImageProcessor.from_pretrained(f"rdoshi21/{model_name}")
    self.val_config = CocoDetection(img_folder=val_path, processor=self.processor, train=False)

    self.model.to(self.device)

  def eval(self, img, threshold=0.3):
    image = Image.fromarray(img)
    width, height = image.size

    arr = [self.val_config.process_img(image)]
    pixel_values = torch.cat([dd.unsqueeze(0).to(self.device) for dd in arr])[0]

    # batched inference
    with torch.no_grad():
    # forward pass to get class logits and bounding boxes
        output = self.model(pixel_values=pixel_values, pixel_mask=None)
        result = self.processor.post_process_object_detection(output,
                                                                target_sizes=[(height, width)],
                                                                threshold=threshold)

    # get centroids from boxes
    scores, labels, boxes = result['scores'], result['labels'], result['boxes']
    if len(scores)>0:
        best_match_idx = torch.argmax(scores).cpu().item()
        best_label, best_box = labels[best_match_idx], boxes[best_match_idx]
        (xmin, ymin, xmax, ymax) = best_box
        centroid = ((xmin.item() + xmax.item()) / 2, (ymin.item() + ymax.item()) / 2)
    else:
        centroid = ()
    
    return centroid, best_label


class HttpServer:
    def __init__(self):
        # load DETR model from huggingface
        self.detr_model = DETRModel(model_name='MODEL_NAME_HERE')

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
            centroid = self.detr_model.eval(image)
            return self.json_response(np.array(centroid)) 
        except:
            print(traceback.format_exc())
            return "error"


if __name__ == "__main__":
    server = HttpServer()
    server.run()  



