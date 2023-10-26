import pandas as pd
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

# Load data from CSV
dataframe = pd.read_csv('path_to_your_file.csv')

# Convert the dataframe to numpy array for processing with TensorFlow
data = dataframe.values

# Split data into input (X) and output (y)
X = data[:, 2:]
y = data[:, :2]