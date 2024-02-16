import os
positions = [
    [0.2, 1.9],
    [22.6, -2.74],
    [2.5, 0.25],
    [5.56, 2.6],
    [-2.1, -5.06]
]

place2 = open("place2.csv", "w")

for i in range(1, 6):
    # look in folder TrainData/Position1, TrainData/Position2, etc.
    # get all csv files and write to the file "filepath x y"
    x = positions[i-1][0]
    y = positions[i-1][1]
    
    for file in os.listdir("TrainData/Position" + str(i)):
        if file.endswith(".csv"):
            place2.write(os.path.join("TrainData/Position" + str(i), file) + " " + str(x) + " " + str(y) + "\n")
            
place2.close()