#!/usr/bin/python2.7

import rospy
from dataclasses import dataclasses

@dataclasses
class DataCluster():
    name: str
    number: int
    x: float
    y: float


    def __init__(self):
        self.number = 0

    # zmiana danych klastra

    # zwrocenie danych

