#!/usr/bin/env python
# coding: utf-8

from pypot.creatures import DarwinMini

darwin = DarwinMini(simulator='vrep')

w = WalkStraight(darwin, 0.5, 1.0)

w.start()
