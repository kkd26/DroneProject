from bezier.curve import *
import sympy as sym


def check_triangle(c: Curve, point) -> bool:
    """checks if `point` is inside the triangle defining the curve c"""
    # a degree 2 bezier curve is defined by 3 points
    # by scaling the curve, we can reach any value within that triangle
    # in theory at least

    # translated from https://stackoverflow.com/a/2049593
    def sign(x1, y1, x2, y2, x3, y3):
        return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3)

    v1 = c.nodes[0][0], c.nodes[1][0]
    v2 = c.nodes[0][1], c.nodes[1][1]
    v3 = c.nodes[0][2], c.nodes[1][2]
    print(v1, v2, v3, point)

    d1 = sign(*point, *v1, *v2)
    d2 = sign(*point, *v2, *v3)
    d3 = sign(*point, *v3, *v1)

    has_neg = d1 < 0 or d2 < 0 or d3 < 0
    has_pos = d1 > 0 or d2 > 0 or d3 > 0

    return not (has_neg and has_pos)


def warp_curve(c: Curve, target, squish_factor=1.0):
    # now, i know what you're thinking
    # 1. what is this horrible pile of maths
    # 2. why are you using a symbolic solver instead of a numeric one
    #    (the answer is scipy is only 2x as fast but is also wrong and i cba to debug it rn)
    #    (and also i'm running it in numeric mode (it takes about 1s in symbolic mode but only 0.07s numerically)

    # The general idea is I have some pre-set curve c, and I want to "squish" it so it goes through some point (target)
    # figure 1: https://en.wikipedia.org/wiki/B%C3%A9zier_curve#/media/File:Quadratic_to_cubic_Bezier_curve.svg
    # this shows a reasonable way of squishing/stretching the curve
    # in our output curve, the start and end control points are the same,
    # and the middle two are determined by some scaling factor t, which controls how close they are to the tip
    # we then write B_t(s) = X
    # where B_t is the bezier curve defined by t, X is the target we want to hit, and s is some variable
    # since this is in 2d (for now) we have 2 equations (X is a vector!) and 2 unknowns
    # we write and expand the definition of a bezier curve, then numerically solve it

    # now, i know what you're thinking
    # why not solve it symbolically once and then bake that solution into the code
    # I tried that, and it just hung for about 5 minutes (then I killed it)
    # if you can figure out a cleaner way of doing this, please, be my guest

    s, t = sym.symbols('s,t')

    # xx is the x co-ord of X, and xy is the y co-ord.
    xx, xy = target
    # similarly, v0x is the x co-ord of the 1st control point
    v0x, v1x, v2x = c.nodes[0]
    v0y, v1y, v2y = c.nodes[1]

    # noinspection DuplicatedCode
    eq1 = sym.Eq(xx,
                 (1 - s) ** 3 * v0x
                 + 3 * s * (1 - s) ** 2 * (v0x + ((v1x - v0x) * t))
                 + 3 * s ** 2 * (1 - s) * (v2x + ((v1x - v2x) * t))
                 + s ** 3 * v2x
                 )
    # noinspection DuplicatedCode
    eq2 = sym.Eq(xy,
                 (1 - s) ** 3 * v0y
                 + 3 * s * (1 - s) ** 2 * (v0y + ((v1y - v0y) * t))
                 + 3 * s ** 2 * (1 - s) * (v2y + ((v1y - v2y) * t))
                 + s ** 3 * v2y
                 )

    result = sym.nsolve([eq1, eq2], [t, s], [0.5, 0.5])
    t, s = result[0], result[1]

    # maybe we only want to go halfway between the target and the original curve
    # so we apply a squish_factor
    # when t = 2/3, we have no squish
    delta = ((2/3)-t) * (1-squish_factor)
    t += delta

    # now we have the solution, we construct the output curve
    v3x, v3y = v2x, v2y
    v1x_ = v0x + ((v1x - v0x) * t)
    v1y_ = v0y + ((v1y - v0y) * t)
    v2x = v3x + ((v1x - v3x) * t)
    v2y = v3y + ((v1y - v3y) * t)
    points = [
        [v0x, v1x_, v2x, v3x],
        [v0y, v1y_, v2y, v3y]
    ]
    return Curve(points, degree=3)


def interpolate(c: Curve, t, target, squish_factor=0.8):
    # note: the 0.8 value of squish factor is pretty arbitrary

    c_spec = c.specialize(t, 1.0)
    if check_triangle(c_spec, target):
        return warp_curve(c_spec, target, squish_factor)
    else:
        # if we go ahead and just scale it, may fail to find a solution (and raise a ValueError)
        # we also may find one with a loop in
        # this is probably fine?
        return warp_curve(c_spec, target, squish_factor)
