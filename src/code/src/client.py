import requests
import json_numpy
from json_numpy import loads
import urllib
import numpy as np

json_numpy.patch()

SERVER_URL = "http://128.32.176.100:8000"

def get_centroid_and_recyclable_label(image):
    
    payload = {
        "image": image.tolist(),
    }

    print("waiting for server")
    output = loads(requests.post(
        # 'http://128.32.176.100:8000/query',
        'http://128.32.162.191:8000/query',
        json=payload,
    ).json())

    centroid, label = output['centroid'], output['label']
    return centroid, label