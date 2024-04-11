#!/usr/bin/env python3

import resource_retriever as rr
import yaml

from depthai import node

DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH = 'package://cv/models/depthai_models.yaml'

with open(rr.get_filename(DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH, use_protocol=False)) as f:
    models = yaml.safe_load(f)

# Inputs:
    # - model
    # - rgb_raw_passthrough, rgb_raw_passthroughDepth
    # - {model}_passthrough, {model}_passthroughDepth, {model}_out
# Outputs: passthrough, passthroughDepth, out

current_model_name = None

while True:
    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_name = new_model.getRaw().data

    if current_model_name == "rgb_raw":
        passthrough = node.io['rgb_raw_passthrough'].get()
        depth = node.io['rgb_raw_passthroughDepth'].get()
    elif current_model_name in models:
        passthrough = node.io[f"{current_model_name}_passthrough"].get()
        depth = node.io[f"{current_model_name}_passthroughDepth"].get()

        out = node.io[f"{current_model_name}_out"].get()
        node.io["out"].send(out)
    elif current_model_name:
        print(f"WARNING: {current_model_name} is not a valid model name!")
        break

    node.io["passthrough"].send(passthrough)
    node.io["passthrough_Depth"].send(depth)
