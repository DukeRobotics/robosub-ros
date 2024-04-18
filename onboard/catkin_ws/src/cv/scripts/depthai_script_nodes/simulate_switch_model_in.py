#!/usr/bin/env python3


from depthai import node


"""
Inputs: input, model
Outputs:
    - raw_input
    - {model_id}_input for each model
"""


current_model_id = None

while True:
    input = node.io['input'].get()

    new_model = node.io['model'].tryGet()

    if new_model:
        current_model_id = new_model.getData()[0]

    if current_model_id == 0:
        node.io["raw_input"].send(input)
    elif current_model_id:
        node.io[f"{current_model_id}_input"].send(input)
