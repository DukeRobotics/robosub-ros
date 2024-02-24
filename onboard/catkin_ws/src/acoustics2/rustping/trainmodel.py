import pandas as pd
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from tensorflow import keras
from tensorflow.keras import models, layers
from math import atan2

# Load and preprocess the data (as shown previously)
data = pd.read_csv('oct_train_data.csv')  # Replace 'your_data.csv' with your actual data file
X = data[['h1_2', 'h1_3']].values * 1000
x = filter(lambda x: x[0] < 0.5 and x[1] < 0.5 and x[2] < 0.5, X)
y = data['octant'].values
one_hot = np.zeros((len(y), 8))
one_hot[np.arange(len(y)), y] = 1
y = one_hot

train_data = pd.read_csv('oct_test_data.csv')
X_val = train_data[['h1_2', 'h1_3']].values * 1000
y_val = train_data['octant'].values
y_val_1hot = np.zeros((len(y_val), 8))
y_val_1hot[np.arange(len(y_val)), y_val] = 1


# Split the data into training, validation, and testing sets (as shown previously)

Xtrain, Xtest, ytrain, ytest = X, X_val, y, y_val_1hot

print(Xtrain.shape, Xtest.shape, ytrain.shape, ytest.shape)

# Define the neural network architecture
model = keras.Sequential([
    keras.layers.Dense(128, activation='relu', input_shape=(2,)),
    # keras.layers.Dropout(0.2),
    keras.layers.Dense(64, activation='relu'),
    # keras.layers.Dropout(0.2),
    keras.layers.Dense(32, activation='relu'),
    keras.layers.Dense(8, activation='softmax')  # Output layer for theta prediction
])

# Compile the model (as shown previously)
model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])

# Train the model (as shown previously)
history = model.fit(Xtrain, ytrain, epochs=1000, validation_data=(Xtest, ytest))

# Evaluate the model on the testing set (as shown previously)
test_loss = model.evaluate(Xtest, ytest)
print("Test loss:", test_loss)

# Use the trained model for angle (theta) prediction (inference)

# predict with probabilities
predicted_theta = model.predict(Xtest)

model.save("tdoa_model_2.keras")
