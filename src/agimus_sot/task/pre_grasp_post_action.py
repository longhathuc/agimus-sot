# Copyright 2020 CNRS - Airbus SAS
# Author: Joseph Mirabel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core.feature_pose import FeaturePose
from . import SotTask

from .task import Task
from dynamic_graph.entity import Entity
from agimus_sot.tools import _createOpPoint, assertEntityDoesNotExist, \
    matrixHomoInverse, matrixHomoProduct, se3ToTuple

## \brief A post-action for pregrasp and preplace task.
#
# This task is activated when the gripper frame coincide with the handle frame.
# It ensures the velocity of the gripper is zero with respect to the handle.
#
class PreGraspPostAction (Task):
    name_prefix = "postaction_pregrasp"

    ## Constructor
    # \param gripper object of type OpFrame
    # \param otherGripper either None or a OpFrame representing a gripper
    def __init__ (self, gripper, otherGripper = None):
        super(PreGraspPostAction, self).__init__()
        self.gripper = gripper
        self.otherGripper = otherGripper

    # \param withDerivative not used. Kept for compatibility with other tasks.
    def makeTasks(self, sotrobot, withDerivative = False):
        if self.gripper.controllable:
            if self.otherGripper is not None and self.otherGripper.controllable:
                self._makeRelativeTask (sotrobot)
            else:
                self._makeAbsoluteTask (sotrobot, self.gripper)
        elif self.otherGripper is not None and self.otherGripper.controllable:
                self._makeAbsoluteTask (sotrobot, self.otherGripper)
        else:
            # TODO Both grippers are disabled so nothing can be done...
            # add a warning ?
            print("Both grippers are disabled so nothing can be done")

    def _createTaskAndGain (self, name):
        # Create a task
        alreadyInit = name+"_task" in Entity.entities
        self.task = SotTask (name + "_task")
        if not alreadyInit:
            self.task.add (self.feature.name)
            # We want to solve J * q_dot = 0
            self.task.controlGain.value = 0

    def _makeAbsoluteTask(self, sotrobot, gripper):
        name = self._name(gripper.name)

        alreadyInit = name+"_feature" in Entity.entities

        self.feature = FeaturePose (name + "_feature")
        if not alreadyInit:
            # Create the operational points
            _createOpPoint (sotrobot, gripper.link)
            plug(sotrobot.dynamic.signal(gripper.link), self.feature.oMjb)
            plug(sotrobot.dynamic.signal("J"+gripper.link), self.feature.jbJjb)
            self.feature.jbMfb.value = se3ToTuple(gripper.lMf)

            self.feature.jaJja.value = np.zeros((6, sotrobot.dynamic.getDimension()))

        self._createTaskAndGain(name)
        self.tasks = [ self.task, ]

    def _makeRelativeTask(self, sotrobot):
        name = self._name(self.gripper.name, "relative", self.otherGripper.name)

        alreadyInit = name+"_feature" in Entity.entities

        self.feature = FeaturePose (name + "_feature")
        if not alreadyInit:
            # Create the operational points
            _createOpPoint (sotrobot, self.gripper.link)
            _createOpPoint (sotrobot, self.otherGripper.link)

            plug(sotrobot.dynamic.signal(self.gripper.link), self.feature.oMja)
            plug(sotrobot.dynamic.signal("J"+self.gripper.link), self.feature.jaJja)
            self.feature.jaMfa.value = se3ToTuple(self.gripper.lMf)

            plug(sotrobot.dynamic.signal(self.otherGripper.link), self.feature.oMjb)
            plug(sotrobot.dynamic.signal("J"+self.otherGripper.link), self.feature.jbJjb)
            self.feature.jbMfb.value = se3ToTuple(self.otherGripper.lMf)

        self._createTaskAndGain(name)
        self.tasks = [ self.task, ]
