#!/usr/bin/env python3

from depthai import node


"""
Inputs:
    - model
    - raw_passthrough, raw_passthroughDepth
    - {model_id}_passthrough, {model_id}_passthroughDepth, {model_id}_out for each model
Outputs: passthrough, passthroughDepth, out
"""


current_model_id = None

while True:
    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_id = new_model.getRaw().data

    if current_model_id == 0:
        passthrough = node.io['raw_passthrough'].get()
        depth = node.io['raw_passthroughDepth'].get()
    elif current_model_id:
        passthrough = node.io[f"{current_model_id}_passthrough"].get()
        depth = node.io[f"{current_model_id}_passthroughDepth"].get()

        out = node.io[f"{current_model_id}_out"].get()
        node.io["out"].send(out)

    node.io["passthrough"].send(passthrough)
    node.io["passthrough_Depth"].send(depth)
