#!/usr/bin/env python3

from depthai import node


"""
Inputs: input, inputDepth, model
Outputs:
    - raw_input, raw_inputDepth
    - {model_id}_input, {model_id}_inputDepth for each model
"""


current_model_id = None

while True:
    input = node.io['input'].get()
    depth = node.io['depth'].get()

    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_id = new_model.getRaw().data

    if current_model_id == 0:
        node.io["raw_input"].send(input)
        node.io["raw_inputDepth"].send(depth)
    elif current_model_id:
        node.io[f"{current_model_id}_input"].send(input)
        node.io[f"{current_model_id}_inputDepth"].send(depth)
