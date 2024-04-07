#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

training_x = 4 * np.random.rand(100, 1) - 2
training_y = 4 + 2 * training_x + 5 * training_x ** 2 + np.random.randn(100, 1)

poly_features = PolynomialFeatures(degree = 2, include_bias = False)

training_x_poly = poly_features.fit_transform(training_x)

reg = LinearRegression()
reg.fit(training_x_poly, training_y)

test_x = np.linspace(-2, 2, 100).reshape(-1, 1)
test_x_poly = poly_features.transform(test_x)
test_y = reg.predict(test_x_poly)

plt.scatter(training_x, training_y)
plt.plot(test_x, test_y, color = 'r')
plt.show()