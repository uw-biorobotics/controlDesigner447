import numpy as np
#
# drange = 2.0
# nvals = 20
# val = 1.0
# llim = val / np.sqrt(drange)
# ulim = np.sqrt(drange)*val
# xs = []
# print('X vals: ',end='')
# for i in range(nvals):
#     x =(llim * (drange**(1.0/nvals))**i)
#     print(f'{x:8.3f} ',end='')
#     xs.append(x)
# print('')
# print('geometric increment (as a %) ')
# print(100*drange**(1.0/nvals)-100,'%')


print('\n                       geometric increment table (%)\n')
print(f'                                         Range')
print(f'{"       Navals":15}',end='')
rangeVals = [1.1, 1.25, 1.4, 1.6]
for drange in rangeVals:
    print(f'{drange:20.2f}',end='')
print('')
print('-'*110)
for nvals in range(2,20,2):
    print(f'{nvals:15d}',end='')
    for drange in rangeVals:
        delta = 100*drange**(1.0/nvals)-100
        print(f'{delta:20.3f}',end='')
    print('\n')

print('''
Notes:
    *  Table entries are percentage change per optimization step.
    *  if pct < 2%, search may be inefficient.
    *  improvements due to small incremental changes may not be
       meaningful if plant parameters are not accurate.

''')
