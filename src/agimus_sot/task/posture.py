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
from dynamic_graph.sot.core.feature_posture import FeaturePosture
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
from . import SotTask
from dynamic_graph.sot.core.meta_tasks import setGain

from .task import Task

## Postural task
class Posture(Task):
    name_prefix=""

    def __init__ (self, name, sotrobot, withDerivative = False):
        super(Posture, self).__init__()

        n = self._name (name)
        self._task = SotTask (n + '_task')
        self._task.dyn = sotrobot.dynamic
        self._feature = FeaturePosture(n + '_feature_')

        q = list(sotrobot.dynamic.position.value)
        self._feature.state.value = q
        self._feature.posture.value = q

        robotDim = sotrobot.dynamic.getDimension()
        for i in range(6, robotDim):
            self._feature.selectDof (i, True)
        self._gain = GainAdaptive(n + "_gain")
        self._task.add(self._feature.name)

        # Connects the dynamics to the current feature of the posture task
        plug(sotrobot.dynamic.position, self._feature.state)

        self._task.setWithDerivative (withDerivative)

        # Set the gain of the posture task
        setGain(self._gain,(4.9,0.9,0.01,0.9))
        plug(self._gain.gain, self._task.controlGain)
        plug(self._task.error, self._gain.error)
        self.tasks = [ self._task ]
        self.topics = {
                    name: {
                        "type": "vector",
                        "topic": "/hpp/target/position",
                        "signalGetters": frozenset([ self._signalPositionRef ]) },
                }
        if withDerivative:
            self.topics["vel_" + name] = {
                    "type": "vector",
                    "topic": "/hpp/target/velocity",
                    "signalGetters": frozenset([ self._signalVelocityRef ])
                    }

    def _signalPositionRef (self): return self._feature.posture
    def _signalVelocityRef (self): return self._feature.postureDot

