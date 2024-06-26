#!/usr/bin/env python3


from depthai import node


"""
Inputs:
    - model
    - input
    - {model_id}_passthrough, {model_id}_out for each model

Outputs:
    - {model_id}_input for each model
    - out, passthrough
"""


current_model_id = None

while True:
    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_id = new_model.getData()[0]

    input = node.io['input'].tryGet()

    passthrough = None

    if current_model_id != 0:
        if input:
            node.io[f"{current_model_id}_input"].send(input)

        passthrough = node.io[f"{current_model_id}_passthrough"].tryGet()

        out = node.io[f"{current_model_id}_out"].tryGet()
        if out:
            node.io["out"].send(out)

    elif current_model_id:
        passthrough = input

    if passthrough:
        node.io["passthrough"].send(passthrough)
