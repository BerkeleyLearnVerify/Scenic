param v0 = VerifaiRange(0, 1)
param v1 = VerifaiRange(0, 1)
param v2 = VerifaiRange(0, 1)
param v3 = VerifaiRange(0, 1)
param v4 = VerifaiRange(0, 1)
param v5 = VerifaiRange(0, 1)
param v6 = VerifaiRange(0, 1)
param v7 = VerifaiRange(0, 10)
# param v8 = VerifaiRange(0, 1)
# param v9 = VerifaiRange(0, 1)
# param v10 = VerifaiRange(0, 1)
# param v11 = VerifaiRange(0, 1)
# param v12 = VerifaiRange(0, 1)


# param yaw0 = VerifaiRange(0, 180)

lst = [globalParameters.v0, globalParameters.v1, globalParameters.v2,
       globalParameters.v3, globalParameters.v4, globalParameters.v5,
       globalParameters.v6, globalParameters.v7]

for i, v in enumerate(lst):
    # print(f"x value: {v}")
    obj = new Object at (v, i, 0), with shape SpheroidShape(dimensions=(0.5, 0.5, 0.5)), with color [ i * 0.1, 0, 0]

# The code above is faulty

# =========== DIVIDE ============

# The code blow works normally


# param v0 = VerifaiRange(0, 1)
# param v1 = VerifaiRange(0, 1)
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

# lst = [globalParameters.v0, globalParameters.v1, globalParameters.v2,
       # globalParameters.v3, globalParameters.v4, globalParameters.v5,
       # globalParameters.v6, globalParameters.v7, globalParameters.v8,
       # globalParameters.v9, globalParameters.v10, globalParameters.v11,
       # globalParameters.v12]

# for i, v in enumerate(lst):
    # obj = new Object at (v, i, 0), with shape SpheroidShape(dimensions=(0.5, 0.5, 0.5))
