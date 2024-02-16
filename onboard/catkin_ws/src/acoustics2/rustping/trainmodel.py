import pandas as pd
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from tensorflow import keras
from tensorflow.keras import models, layers
from math import atan2

# Load and preprocess the data (as shown previously)
data = pd.read_csv('training.csv')  # Replace 'your_data.csv' with your actual data file
X = data[['h1_2', 'h1_3']].values * 1000
x = filter(lambda x: x[0] < 0.5 and x[1] < 0.5 and x[2] < 0.5, X)
y = data['y'].values
one_hot = np.zeros((len(y), 6))
one_hot[np.arange(len(y)), y] = 1
y = one_hot

# y = np.array([atan2(y[i, 1], y[i, 0]) for i in range(len(y))])


# Split the data into training, validation, and testing sets (as shown previously)
X_train, X_temp, y_train, y_temp = train_test_split(X, y, test_size=0.1, random_state=42)
X_val, X_test, y_val, y_test = train_test_split(X_temp, y_temp, test_size=0.5, random_state=42)

# Define the neural network architecture
model = keras.Sequential([
    keras.layers.Dense(128, activation='tanh', input_shape=(2,)),
    keras.layers.Dense(64, activation='tanh'),
    keras.layers.Dense(32, activation='sigmoid'),
    keras.layers.Dense(6, activation='softmax')  # Output layer for theta prediction
])

# Compile the model (as shown previously)
model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])

# Train the model (as shown previously)
history = model.fit(X_train, y_train, epochs=1000, validation_data=(X_val, y_val))

# Evaluate the model on the testing set (as shown previously)
test_loss = model.evaluate(X_test, y_test)
print("Test loss:", test_loss)

# Use the trained model for angle (theta) prediction (inference)
predicted_theta = model.predict(X_test)
print("Predicted theta values:")
print(np.round(predicted_theta))

model.save("tdoa_model_2.keras")
