param v0 = VerifaiRange(0, 1)
param v1 = VerifaiRange(0, 1)
# param v2 = VerifaiRange(0, 1)
# param v3 = VerifaiRange(0, 1)
# param v4 = VerifaiRange(0, 1)
# param v5 = VerifaiRange(0, 1)
# param v6 = VerifaiRange(0, 1)
# param v7 = VerifaiRange(0, 1)
# param v8 = VerifaiRange(0, 1)
# param v9 = VerifaiRange(0, 1)
# param v10 = VerifaiRange(0, 1)
# param v11 = VerifaiRange(0, 1)
# param v12 = VerifaiRange(0, 1)


param yaw0 = VerifaiRange(0, 180)


obj1 = new Object at (globalParameters.v0, 0, 0), with shape SpheroidShape(dimensions=(0.5, 0.5, 0.5))
obj2 = new Object at (globalParameters.v1, 1, 0), with shape SpheroidShape(dimensions=(0.5, 0.5, 0.5))


# param yaw1 = VerifaiRange(0, 180)
# param yaw2 = VerifaiRange(0, 180)
# param yaw3 = VerifaiRange(0, 180)

# lst = [globalParameters.v0, globalParameters.v1, globalParameters.v2,
       # globalParameters.v3, globalParameters.v4, globalParameters.v5,
       # globalParameters.v6, globalParameters.v7, globalParameters.v8, 
       # globalParameters.v9, globalParameters.v10, globalParameters.v11, globalParameters.v12]

# # lst = [globalParameters.v0, globalParameters.v1, globalParameters.v2,
       # # globalParameters.v3, globalParameters.v4, globalParameters.v5,
       # # globalParameters.v6, globalParameters.v7, globalParameters.v8, 
       # # globalParameters.v9, globalParameters.v10, globalParameters.v11, globalParameters.v12]
# # lst_yaw = [globalParameters.yaw0, globalParameters.yaw1, globalParameters.yaw2,globalParameters.yaw3]
# for i, v in enumerate(lst):
    # # if i <= 3:
        # # obj_yaw = lst_yaw[i]
    # # else:
        # # obj_yaw = 0
    # obj_yaw = 0
    # obj = new Object at (v, i, 0), with yaw obj_yaw deg, with shape SpheroidShape(dimensions=(0.5, 0.5, 0.5))


