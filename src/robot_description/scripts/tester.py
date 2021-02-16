import numpy as np
import matplotlib.pyplot as plt

DATA_LENGTH = 1000
ALPHA_SMOOTH = 0.05
data = np.array([np.random.random()])

for i in range(DATA_LENGTH):
    data = np.append(data, data[i] + np.random.uniform(-2, 2.2))


# Exponential Filter
data_smooth = np.array([data[0]])
for i in range(DATA_LENGTH):
    data_smooth = np.append(data_smooth, ALPHA_SMOOTH*data[i+1] + (1-ALPHA_SMOOTH)*data_smooth[i])


plt.plot(data)
plt.plot(data_smooth)
plt.show()