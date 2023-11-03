import pandas as pd
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from tensorflow import keras

# Load and preprocess the data (as shown previously)
data = pd.read_csv('your_data.csv')  # Replace 'your_data.csv' with your actual data file
X = data[['Dt1', 'Dt2']].values
y = data['theta'].values

# Split the data into training, validation, and testing sets (as shown previously)
X_train, X_temp, y_train, y_temp = train_test_split(X, y, test_size=0.3, random_state=42)
X_val, X_test, y_val, y_test = train_test_split(X_temp, y_temp, test_size=0.5, random_state=42)

# Define the neural network architecture
model = keras.Sequential([
    keras.layers.Dense(16, activation='tanh', input_shape=(2,)),
    keras.layers.Dense(16, activation='relu'),
    keras.layers.Dense(1)  # Output layer for theta prediction
])

# Compile the model (as shown previously)
model.compile(optimizer='adam', loss='mean_squared_error')

# Train the model (as shown previously)
history = model.fit(X_train, y_train, epochs=50, batch_size=32, validation_data=(X_val, y_val))

# Evaluate the model on the testing set (as shown previously)
test_loss = model.evaluate(X_test, y_test)
print("Test loss:", test_loss)

# Use the trained model for angle (theta) prediction (inference)
predicted_theta = model.predict(X_test)
print("Predicted theta values:")
print(predicted_theta)
