import os
import pandas as pd
import numpy as np
from tensorflow import keras
from sklearn.model_selection import train_test_split
from tensorflow import keras
from math import atan2
from pprint import pprint

data = pd.read_csv('training.csv')  # Replace 'your_data.csv' with your actual data file
X = data[['h1_2', 'h1_3']].values * 1000
y = data['y'].values
# y = np.array([atan2(y[i, 1], y[i, 0]) for i in range(len(y))])

# load model
model = keras.models.load_model("tdoa_model_2.keras")
pred_xs = model.predict(X)

compare = [(np.argmax(i), j) for i,j in zip(pred_xs, y)]

accuracy = sum([1 for i in compare if i[0] == i[1]]) / len(compare)
print('accuracy:', accuracy)
