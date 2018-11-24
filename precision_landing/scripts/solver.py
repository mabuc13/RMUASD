import numpy as np

# Randomly generate two of our model variables
a, c = np.random.rand(2)
b = 1
x = np.linspace(0, 2, 6)

y = a * x**2 + b * x + c
noise = np.random.normal(0, 0.1, y.size)
y += noise