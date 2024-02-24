
import pandas as pd
import numpy as np
from tensorflow import keras
from tensorflow import keras
from pprint import pprint


data = pd.read_csv('oct_test_data.csv')  # Replace 'your_data.csv' with your actual data file
X = data[['h1_2', 'h1_3']].values * 1000
y = data['octant'].values
# y = np.array([atan2(y[i, 1], y[i, 0]) for i in range(len(y))])

# randompy permute the data
# perm = np.random.permutation(len(X))
# X = X[perm]
# y = y[perm]

# load model
model = keras.models.load_model("tdoa_model_2.keras")
pred_xs = model.predict(X)
top2_confidence = []
for x in pred_xs:
    top = np.argmax(x)
    top2 = np.argsort(x)[-2]
    pos1 = x[top]
    pos2 = x[top2]
    top2_confidence.append([(top, round(pos1, 6)), (top2, round(pos2, 6))])

true_octants = y

compare = list(zip(top2_confidence, true_octants))
pprint(compare)