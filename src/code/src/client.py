import requests
import json_numpy
from json_numpy import loads
import urllib
import numpy as np

json_numpy.patch()

def get_centroid_and_recyclable_label(image):
    
    payload = {
        "image": image.tolist(),
    }


    print("waiting for server")
    output = loads(requests.post(
        # 'http://128.32.176.100:8000/query',
        # 'http://128.32.162.191:8000/query',
        'https://a6af-128-32-176-100.ngrok-free.app/query',
        json=payload,
    ).json())

    print(output)
    if type(output) == str:
        print(output)
        centroid, label = None, None
    else:
        centroid, label = output['centroid'], output['label']
    return centroid, label