from sklearn.decomposition import PCA
import numpy as np
import tkinter
import matplotlib
import matplotlib.pyplot as plt

def pca_transform(x):
	pca = PCA(n_components=2)
	pca.fit(x)
	ret = pca.transform(x)
	return ret

x = np.loadtxt("norm.txt")
ret = pca_transform(x)
print(ret.shape)
plt.scatter(ret[:, 0], ret[:,1])
plt.savefig("figure.png")