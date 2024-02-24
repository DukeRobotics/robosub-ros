import numpy as np
import pandas as pd

data = pd.read_csv('octs2.csv')  # Replace 'your_data.csv' with your actual data file
X = data[['h1_2', 'h1_3']].values
Y = data['octant'].values

# random permutations
perm = np.random.permutation(len(X))
X = X[perm]
Y = Y[perm]

# separate into training and testing

X_train = X[:int(0.9*len(X))]
X_test = X[int(0.9*len(X)):]

y_train = Y[:int(0.9*len(Y))]
y_test = Y[int(0.9*len(Y)):]

# remove all values > 0.5 for x and y
# x2 = []
# y2 = []

# for i in range(len(X)):
#     if abs(X[i][0]) < 0.5 and abs(X[i][1]) < 0.5:
#         x2.append(X[i])
#         y2.append(Y[i])

# save the data
# train_data = pd.DataFrame(x2, columns=['h1_2', 'h1_3'])
# train_data['octant'] = y2
# train_data.to_csv('oct_train_data.csv', index=False)


# save the test data
#
test_data = pd.DataFrame(X_test, columns=['h1_2', 'h1_3'])
test_data['octant'] = y_test
test_data.to_csv('oct_test_data.csv', index=False)

# save the train data
train_data = pd.DataFrame(X_train, columns=['h1_2', 'h1_3'])
train_data['octant'] = y_train
train_data.to_csv('oct_train_data.csv', index=False)
