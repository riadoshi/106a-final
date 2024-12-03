import requests
import json_numpy
from json_numpy import loads
import urllib
import numpy as np

json_numpy.patch()

SERVER_URL = "128.32.176.100:8000"

def get_centroid_and_recyclable_label():
    payload = {
        "image": np.zeros((256, 256,3)),
    }
    output = loads(requests.post(
        urllib.parse.urljoin(SERVER_URL, "get_centroid"),
        json=payload,
    ).json())

    centroid, label = output['centroid'], output['label']
    return centroid, label