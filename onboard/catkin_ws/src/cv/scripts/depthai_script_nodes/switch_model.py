#!/usr/bin/env python3


"""
Inputs:
    - model
    - input, inputDepth
    - {model_id}_passthrough, {model_id}_passthroughDepth, {model_id}_out for each model

Outputs:
    - {model_id}_input, {model_id}_inputDepth for each model
    - out, passthrough, passthroughDepth
"""


current_model_id = None

while True:

    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_id = new_model.getData()[0]

    input = node.io['input'].tryGet()
    depth = node.io['inputDepth'].tryGet()

    passthrough = None
    passthrough_depth = None

    if current_model_id != 0:
        if input:
            node.io[f"{current_model_id}_input"].send(input)

        if depth:
            node.io[f"{current_model_id}_inputDepth"].send(depth)

        passthrough = node.io[f"{current_model_id}_passthrough"].tryGet()
        passthrough_depth = node.io[f"{current_model_id}_passthroughDepth"].tryGet()

        out = node.io[f"{current_model_id}_out"].tryGet()
        if out:
            node.io["out"].send(out)

    elif current_model_id:
        passthrough = input
        passthrough_depth = depth

    if passthrough:
        node.io["passthrough"].send(passthrough)

    if passthrough_depth and queue_depth:
        node.io["passthroughDepth"].send(passthrough_depth)
