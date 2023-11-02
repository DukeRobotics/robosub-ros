import pandas as pd
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from tensorflow import keras

# Load and preprocess the data
data = pd.read_csv('your_data.csv')  # Replace 'your_data.csv' with your actual data file
X = data['theta'].values
y = data[['Dt1', 'Dt2']].values

# Split the data into training, validation, and testing sets
X_train, X_temp, y_train, y_temp = train_test_split(X, y, test_size=0.3, random_state=42)
X_val, X_test, y_val, y_test = train_test_split(X_temp, y_temp, test_size=0.5, random_state=42)

# Define the neural network architecture
model = keras.Sequential([
    keras.layers.Dense(64, activation='relu', input_shape=(1,)),  # Input layer with 1 feature
    keras.layers.Dense(64, activation='relu'),
    keras.layers.Dense(2)  # Output layer for Dt1 and Dt2 prediction
])

# Compile the model
model.compile(optimizer='adam', loss='mean_squared_error')

# Train the model
history = model.fit(X_train, y_train, epochs=50, batch_size=32, validation_data=(X_val, y_val))

# Evaluate the model on the testing set
test_loss = model.evaluate(X_test, y_test)
print("Test loss:", test_loss)

# Use the trained model for TDOA estimation (inference)
predictions = model.predict(X_test)
print("Predicted TDOA values:")
print(predictions)

model.save('tdoa_model.h5')  # Save the model to a file named 'tdoa_model.h5'
print("Model saved as 'tdoa_model.h5'")
