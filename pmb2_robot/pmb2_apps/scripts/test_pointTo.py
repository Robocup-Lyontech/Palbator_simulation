#!/usr/bin/env python
import tf
import math
import numpy as np


class PointTo:

    def __init__(self):

        x_object = 2.0
        y_object = -1.0

        lg_bras = 0.608

        alpha = np.arctan(y_object/x_object) 

        alpha_deg = alpha*360/(2*math.pi)

        print("alpha_deg")
        print(alpha_deg)

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, alpha)

        x_bras = np.cos(alpha)*lg_bras
        y_bras = np.sin(alpha)*lg_bras


        print("q")
        print(q)

        print("x")
        print(x_bras)
        print("y")
        print(y_bras)



if __name__ == "__main__":

    pointing_instance = PointTo()

