#!/usr/bin/env python3

import resource_retriever as rr
import yaml

from depthai import node

DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH = 'package://cv/models/depthai_models.yaml'
MODEL_IDS = {}


# Inputs: input, inputDepth, model
# Outputs:
#     - rgb_raw_input, rgb_raw_inputDepth
#     - {model}_input, {model}_inputDepth for each model


with open(rr.get_filename(DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH, use_protocol=False)) as f:
    models = yaml.safe_load(f)

    for model in models:
        MODEL_IDS[models[model]['id']] = model

current_model_id = None

while True:
    input = node.io['input'].get()
    depth = node.io['depth'].get()

    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_id = new_model.getRaw().data

    if current_model_id == 0:
        node.io["rgb_raw_input"].send(input)
        node.io["rgb_raw_inputDepth"].send(depth)
    elif current_model_id in MODEL_IDS:
        node.io[f"{MODEL_IDS[current_model_id]}_input"].send(input)
        node.io[f"{MODEL_IDS[current_model_id]}_inputDepth"].send(depth)
    else:
        node.warn(f"WARNING: {current_model_id} is not a valid model id!")
