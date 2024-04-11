#!/usr/bin/env python3

import resource_retriever as rr
import yaml

from depthai import node

DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH = 'package://cv/models/depthai_models.yaml'

with open(rr.get_filename(DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH, use_protocol=False)) as f:
    models = yaml.safe_load(f)

# Inputs: input, inputDepth, model
# Outputs:
    # - rgb_raw_input, rgb_raw_inputDepth
    # - {model}_input, {model}_inputDepth for each model

current_model_name = None

while True:
    input = node.io['input'].get()
    depth = node.io['depth'].get()

    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_name = new_model.getRaw().data

    if current_model_name == "rgb_raw":
        node.io["rgb_raw_input"].send(input)
        node.io["rgb_raw_inputDepth"].send(depth)
    elif current_model_name in models:
        node.io[f"{current_model_name}_input"].send(input)
        node.io[f"{current_model_name}_inputDepth"].send(depth)
    elif current_model_name:
        print(f"WARNING: {current_model_name} is not a valid model name!")
