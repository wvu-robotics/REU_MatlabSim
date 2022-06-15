import ctypes
import numpy as np
import multiprocessing as mp

shared_arr = mp.Array(ctypes.c_double, 10*3*2)
arr = np.array(shared_arr)
arr.reshape([3,10,2])

print(arr)

def assignToSlice(arr):
    for i in range(len(arr)):
        arr[i] = [1,1]

# assiarre(original[1][2:6])


#add multiprocessing into test
pool = mp.Pool(3)
pool.apply_async(assignToSlice, args=(arr[1][2:6]))

print(arr)
