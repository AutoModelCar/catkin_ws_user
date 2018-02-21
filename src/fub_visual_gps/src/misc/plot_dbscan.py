# -*- coding: utf-8 -*-
"""
===================================
Demo of DBSCAN clustering algorithm
===================================

Finds core samples of high density and expands clusters from them.

"""

import numpy as np

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler

##############################################################################
# Generate sample data
centers = [[100, 100], [100, 200], [200, 200]]
X, labels_true = make_blobs(n_samples=750, centers=centers, cluster_std=5,
                            random_state=0)

X = np.round(X).astype(np.int)
print(X)

##############################################################################
# Compute DBSCAN
db = DBSCAN(eps=5, min_samples=10).fit(X)
core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
core_samples_mask[db.core_sample_indices_] = True
labels = db.labels_

# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

##############################################################################
# Plot result
import matplotlib.pyplot as plt
plt.hold(True)

# Black removed and is used for noise instead.
unique_labels = set(labels)
colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = (labels == k)

    xy = X[class_member_mask & core_samples_mask]
    plt.scatter(xy[:, 0], xy[:, 1], s=14, color=tuple(col))

    xy = X[class_member_mask & ~core_samples_mask]
    plt.scatter(xy[:, 0], xy[:, 1],  s=6, color=tuple(col))

plt.title('Estimated number of clusters: %d' % n_clusters_)
plt.show()
