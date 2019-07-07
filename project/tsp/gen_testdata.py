import numpy as np

N = 2
mean = np.zeros(N)
cov = np.identity(N)
size = 10
x = np.random.multivariate_normal(mean, cov, size)

print(x.shape)
np.savetxt("norm.txt", x)