#!/usr/bin/env python3

import resource_retriever as rr
import yaml

from depthai import node


# Inputs:
#     - model
#     - raw_passthrough, raw_passthroughDepth
#     - {model}_passthrough, {model}_passthroughDepth, {model}_out for each model
# Outputs: passthrough, passthroughDepth, out


DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH = 'package://cv/models/depthai_models.yaml'
MODEL_IDS = {}

with open(rr.get_filename(DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH, use_protocol=False)) as f:
    models = yaml.safe_load(f)

    for model in models:
        MODEL_IDS[models[model]['id']] = model

current_model_id = None

while True:
    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_id = new_model.getRaw().data

    if current_model_id == 0:
        passthrough = node.io['raw_passthrough'].get()
        depth = node.io['raw_passthroughDepth'].get()
    elif current_model_id in MODEL_IDS:
        passthrough = node.io[f"{MODEL_IDS[current_model_id]}_passthrough"].get()
        depth = node.io[f"{MODEL_IDS[current_model_id]}_passthroughDepth"].get()

        out = node.io[f"{MODEL_IDS[current_model_id]}_out"].get()
        node.io["out"].send(out)
    else:
        node.warn(f"WARNING: {current_model_id} is not a valid model id!")
        continue

    node.io["passthrough"].send(passthrough)
    node.io["passthrough_Depth"].send(depth)
