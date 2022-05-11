# def sum_pairs(lst, s):
#     cache = set()
#     for i in lst:
#         print(i)
#         if s - i in cache:
#             print([s - i, i])
#             return [s - i, i]
#         cache.add(i)
#
# sum_pairs([4, 3, 2, 3, 4], 6)
import math

import numpy as np

# mask = np.zeros((20, 20))
# heuristics = np.arange(20 * 20)
# heuristics = heuristics.reshape((20, 20))
# costs = np.arange(20 * 20)
# costs = costs.reshape((20, 20))
# mask[0][1] = 1
# mask[15][15] = 1
# mask[19][19] = 1
# mask[16][15] = 1
#
# nodes = {'mask': mask, 'heuristics': heuristics, 'costs': costs}
#
# scores = nodes['heuristics'] + nodes['costs']
# scores[16][15] = 2
# scores_filtered = scores[nodes['mask'] == 1]
# print(scores_filtered)
# print(np.argwhere(scores == scores_filtered.min()).shape)

l = [[1,2], [52, 3], [9, 3]]
l = np.array(l)
a = np.array([1,2])
b = np.array([25,3])
print((a == l).all(1).any())
print((b == l).all(1).any())