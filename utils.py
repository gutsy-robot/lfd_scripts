import os
import sys


"""

Utility functions for LfD project

"""


def get_trajectory(filename):
    with open(filename) as f:
        content = f.readlines()

    content = [x.strip() for x in content]
    # print(content)[0]
    return content