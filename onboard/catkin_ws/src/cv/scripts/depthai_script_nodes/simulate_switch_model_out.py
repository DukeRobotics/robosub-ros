#!/usr/bin/env python3

from depthai import node


"""
Inputs:
    - model
    - raw_passthrough
    - {model_id}_passthrough, {model_id}_out for each model
Outputs: passthrough, out
"""


current_model_id = None

while True:
    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_id = new_model.getRaw().data

    if current_model_id == 0:
        passthrough = node.io['raw_passthrough'].get()
    elif current_model_id:
        passthrough = node.io[f"{current_model_id}_passthrough"].get()

        out = node.io[f"{current_model_id}_out"].get()
        node.io["out"].send(out)

    node.io["passthrough"].send(passthrough)
