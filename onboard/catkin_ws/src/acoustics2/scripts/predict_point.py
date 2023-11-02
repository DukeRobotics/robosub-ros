import numpy as np
from tensorflow import keras

# Load the trained Keras model
loaded_model = keras.models.load_model('tdoa_model.h5')  # Replace with the actual model path

# Define the input delta times (Dt1 and Dt2) that you want to test
test_Dt1 = -0.0004753
test_Dt2 = -0.0004831

# Create a NumPy array with the input delta times
input_Dt = np.array([[test_Dt1, test_Dt2]])

# Use the loaded model for angle (theta) prediction (inference)
predicted_theta = loaded_model.predict(input_Dt)

print("Predicted theta value for Dt1 =", test_Dt1, "and Dt2 =", test_Dt2, "is:", predicted_theta[0][0])
