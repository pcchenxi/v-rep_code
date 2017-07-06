
import numpy as np

a = np.ones((1,3,2), dtype=int)

b = a[np.newaxis, :]
print a
print b