a = Range(0, 1)
b = Uniform(-a, a)
c = Normal(b, 1)
d = TruncatedNormal(min(max(c, -5), 5), a+1, -10, 10)
e = (Orientation.fromEuler(1, 2, 3) + a).yaw

x = DiscreteRange(0, 2)
y = Discrete({x: 3, x*x: 5})

l = Uniform([a, b, c], [a, b, c, d], [a, b, c, d, e])
elem = Uniform(*l[x:])

vf = VectorField("foo", lambda pos: pos.x)

ego = new Object in RectangularRegion((d, y), elem, 10, 10),
    facing a + (e relative to vf)

vecs = [ego.position, ego.position.rotatedBy(a)]
shuf = Uniform(vecs, vecs[::-1])
param p = Uniform(*shuf).x + shuf[0].distanceTo(shuf[1])
