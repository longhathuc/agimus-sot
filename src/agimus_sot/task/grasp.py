# Copyright 2018, 2019 CNRS - Airbus SAS
# Author: Joseph Mirabel and Alexis Nicolin
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

from dynamic_graph import plug
from . import FeaturePose, GainAdaptive, SotTask
from dynamic_graph.sot.core.meta_tasks import setGain

from .task import Task
from agimus_sot.tools import se3ToTuple, _createOpPoint

## A grasp task
# It creates a grasp constraint only in the case where
# an object is grasped by two grippers.
class Grasp (Task):
    name_prefix = "grasp"

    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(Grasp, self).__init__()
        self.gripper = gripper
        self.handle = handle

        # self.relative = otherGraspOnObject is not None \
                # and otherGraspOnObject.handle.link == self.handle.link
        # TODO: We should make sure that the relative position is constant
        # but we have no way to do it (HPP does). We assume that the object is
        # not articulated.
        self.relative = (otherGraspOnObject is not None)
        if self.relative:
            self.otherGripper = otherGraspOnObject[0]
            self.otherHandle = otherGraspOnObject[1]

    ## \todo implement tracking of velocity
    # \c self.task.task.setWithDerivative(True)
    # which needs the following signals:
    # - velocity of object attached to gripper.sotjoint, into self.task.feature.dotposition
    # - velocity of object attached to otherGripper.sotjoint, into self.task.feature.dotpositionRef
    def makeTasks(self, sotrobot, withDerivative = False):
        if self.relative:
            basename = self._name(self.gripper.name,
                    self.handle.name,
                    self.otherGripper.name,
                    self.otherHandle.name)


            def set(oMj, jMf, jJj, g, h):
                # Create the operational points
                _createOpPoint (sotrobot, g.link)
                jMf.value = se3ToTuple(g.lMf * h.lMf.inverse())
                plug(sotrobot.dynamic.signal(    g.link), oMj)
                plug(sotrobot.dynamic.signal('J'+g.link), jJj)

            # Create the FeaturePose
            self.feature = FeaturePose (basename+"_feature")
            set(self.feature.oMja, self.feature.jaMfa, self.feature.jaJja,
                    self.otherGripper, self.otherHandle)
            set(self.feature.oMjb, self.feature.jbMfb, self.feature.jbJjb,
                    self.gripper, self.handle)

            # Wrap it into a Task
            self.task = SotTask (basename+"_task")
            self.gain = GainAdaptive (basename+"_gain")

            self.task.add (self.feature.name)
            plug (self.task.error, self.gain.error)
            plug (self.gain.gain , self.task.controlGain)

            setGain(self.gain,(4.9,0.9,0.01,0.9))
            if withDerivative:
                print("Grasp constraint with derivative is not implemented yet.")
            self.task.setWithDerivative (False)

            self.tasks = [ self.task ]
