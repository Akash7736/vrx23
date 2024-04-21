#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
data = pd.read_csv('src/x_pose.csv')
data.plot(y=['eX'])
plt.show()


# def inv_glf(T):
#     scale = 1
#     if T > 0:
#         A, K, B, v, C, M = 0.01, 59.82, 5.00, 0.38, 0.56, 0.28
#     else:
#         A, K, B, v, C, M = -199.13, -0.09, 8.84, 5.34, 0.99, -0.57
#     # try:
#     return (M - (np.log(((K-A)/(T-A))**v - C))/B) * scale


# def glf(x):
#     if x > 0:
#         A, K, B, v, C, M = 0.01, 59.82, 5.00, 0.38, 0.56, 0.28
#     else:
#         A, K, B, v, C, M = -199.13, -0.09, 8.84, 5.34, 0.99, -0.57
#     T = A + (K - A) / ((C + np.exp(-B * (x - M)))**(1/v))
#     return T

# x = inv_glf(0)
# T = glf(x)
# print(x,T)
