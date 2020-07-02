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
from . import SotTask, GainAdaptive, FeaturePose
from dynamic_graph.sot.core.meta_tasks import setGain

from .task import Task
from agimus_sot.tools import parseHppName

class Foot (Task):
    name_prefix = "foot"

    def __init__ (self, footname, sotrobot, selec='111111', withDerivative = False):
        super(Foot, self).__init__()

        robotname, sotjoint = parseHppName (footname)
        basename = self._name(footname)

        # Create the FeaturePose
        self.feature = FeaturePose ('pose'+basename)

        dyn = sotrobot.dynamic
        if sotjoint not in dyn.signals():
            dyn.createOpPoint (sotjoint, sotjoint)
        plug(dyn.signal(    sotjoint), self.feature. oMjb)
        plug(dyn.signal('J'+sotjoint), self.feature.jbJjb)

        # Wrap it into a Task
        self.task    = SotTask ('task'+basename)
        self.gain    = GainAdaptive ('gain'+basename)

        self.task.add (self.feature.name)
        plug (self.task.error, self.gain.error)
        plug (self.gain.gain , self.task.controlGain)

        setGain(self.gain,(4.9,0.9,0.01,0.9))
        self.task.setWithDerivative (withDerivative)

        if selec!='111111':
            self.feature.selec.value = selec

        self.tasks = [ self.task, ]
        self.addHppJointTopic (footname, signalGetters=[ self._signalPositionRef ])
        if withDerivative:
            self.addHppJointTopic ("vel_" + footname, footname,
                    velocity = True,
                    signalGetters=[ self._signalVelocityRef ])

    def _signalPositionRef (self): return self.feature.faMfbDes
    def _signalVelocityRef (self): return self.feature.faNufafbDes
