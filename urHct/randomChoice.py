import numpy as np

'''
from numpy.random import choice
result = choice(list_of_candidates, number_of_items_to_pick, p=probability_distribution)
'''

np.random.seed() 
for _ in range(10):  
    b = np.random.choice(a, 2, replace=False, p=[0, 0, 0, 0, 0.05, 0.05, 0.05, 0.05, 0.8])
    print (b)

for _ in range(10): 
    b = np.random.choice(a, 1, replace=True, p=[0.0, 0.05, 0.1, 0.15, 0.4, 0.15, 0.1, 0.05, 0.0]) # P는 확율분포 입니다. 합이 1이 되도록 조절합니다.
    print (b)
