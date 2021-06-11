#!/usr/bin/env python
from pmb2_grasp.graspGenerator import GraspGenerator

if __name__ == "__main__":
    graspGenerator = GraspGenerator()
    graspGenerator.generateGrasps("base_footprint", None, [0, 0, 0.1], [0.05, 90])