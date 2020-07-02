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

import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core.feature_posture import FeaturePosture
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.operator import Mix_of_vector

from agimus_sot.control.gripper import AdmittanceControl, \
    PositionAndAdmittanceControl
from agimus_sot.events import logical_and_entity, norm_inferior_to, \
    norm_superior_to

from .task import Task
from .posture import Posture

## Control of the gripper motors.
#
# It can do:
# \li position control
# \li admittance control
# \li a mixture between position and admittance control (position control before the impact, then admittance control).
#     \todo At the time of writting, I (Joseph Mirabel) think the mixtured control is bugged. At least, there was a SEGV the last time
#     I tried to use it. However, the SEGV might have a different cause.
class EndEffector (Task):
    name_prefix = "ee"

    def __init__ (self, sotrobot, gripper, name_suffix):
        super(EndEffector, self).__init__()
        self.gripper = gripper
        self.jointNames = gripper.joints
        self.robot = sotrobot
        pinmodel = sotrobot.dynamic.model

        self.name = self._name(gripper.name, name_suffix)

        from . import SotTask
        self.tp = SotTask ('task' + self.name)
        self.tp.dyn = sotrobot.dynamic
        self.tp.feature = FeaturePosture ('feature_' + self.name)

        plug(sotrobot.dynamic.position, self.tp.feature.state)

        # Select the dofs
        self.tp.feature.posture.value = sotrobot.dynamic.position.value
        # Define the reference and the selected DoF
        self.jointRanks = []
        for name in self.jointNames:
            idJ = pinmodel.getJointId(name)
            assert idJ < pinmodel.njoints
            joint = pinmodel.joints[idJ]
            idx_v = joint.idx_v
            nv = joint.nv
            self.jointRanks.append( (idx_v, nv) )
        for idx_v, nv in self.jointRanks:
            for i in range(idx_v, idx_v + nv):
                self.tp.feature.selectDof (i, True)

        self.tp.add(self.tp.feature.name)
        if len(self.jointNames) > 0:
            self.tasks = [ self.tp ]

        self.thr_task_error = 0.0001

    ### \param type equals "open" or "close"
    ### \param period interval between two integration of SoT
    def makeAdmittanceControl (self, affordance, type, period,
            simulateTorqueFeedback = False,
            filterCurrents = True):
        # Make the admittance controller
        # type = "open" or "close"
        desired_torque = affordance.ref["torque"]
        estimated_theta_close = affordance.ref["angle_close"]

        wn, z, nums, denoms = affordance.getControlParameter ()

        if affordance.controlType[type] == "torque":
            self.ac = AdmittanceControl ("AC_" + self.name + "_" + type,
                    estimated_theta_close,
                    desired_torque, period,
                    nums, denoms)
        elif affordance.controlType[type] == "position_torque":
            theta_open = affordance.ref["angle_open"]
            threshold_up = tuple([ x / 10. for x in desired_torque ])
            threshold_down = tuple([ x / 100. for x in desired_torque ])
            self.ac = PositionAndAdmittanceControl ("AC_" + self.name + "_" + type,
                    theta_open, estimated_theta_close,
                    desired_torque, period,
                    threshold_up, threshold_down,
                    wn, z,
                    nums, denoms)
        else:
            raise NotImplementedError ("Control type " + type + " is not implemented for gripper.")

        if simulateTorqueFeedback:
            # Get torque from an internal simulation
            M,d,k,x0 = affordance.getSimulationParameters()
            self.ac.setupFeedbackSimulation(M,d,k,x0)
            # Must be done after setupFeedbackSimulation
            self.ac.readPositionsFromRobot(self.robot, self.jointNames)
        else:
            self.ac.readPositionsFromRobot(self.robot, self.jointNames)
            # Get torque from robot sensors (or external simulation)
            # TODO allows to switch between current and torque sensors
            self.ac.readCurrentsFromRobot(self.robot, self.jointNames,
                    (self.gripper.torque_constant,), filterCurrents)
            # self.ac.readTorquesFromRobot(self.robot, self.jointNames)

        mix_of_vector = Mix_of_vector (self.name + "_control_to_robot_control")
        mix_of_vector.default.value = tuple ([0,] * len(self.tp.feature.posture.value))
        mix_of_vector.setSignalNumber(2)
        for idx_v,nv in self.jointRanks:
            mix_of_vector.addSelec(1, idx_v, nv)

        # Plug the admittance controller to the posture task
        self.tp.controlGain.value = 1.
        plug(self.ac.outputPosition, mix_of_vector.signal("sin1"))
        plug(mix_of_vector.sout, self.tp.feature.posture)
        # TODO plug posture dot ?
        # I do not think it is necessary.
        # TODO should we send to the posture task
        # - the current robot posture
        # - the postureDot from self.ac.outputDerivative
        # This would avoid the last integration in self.ac.
        # This integration does not know the initial point so
        # there might be some drift (removed the position controller at the beginning)

        tnorm, tcomp = norm_superior_to (self.name + "_torquecmp",
                self.ac.currentTorqueIn, 0.95 * np.linalg.norm(desired_torque))
        pnorm, pcomp = norm_inferior_to (self.name + "_positioncmp",
                self.tp.error, self.thr_task_error)
        self.events = {
                "done_close": logical_and_entity (self.name + '_done_close_and', [tcomp.sout, pcomp.sout]),
                }

    def makePositionControl (self, position):
        q = list(self.tp.feature.state.value)
        # Define the reference
        ip = 0
        for iq, nv in self.jointRanks:
            q[iq:iq+nv] = position[ip:ip+nv]
            ip += nv
        assert ip == len(position)
        self.tp.feature.posture.value = q

        from agimus_sot.sot import SafeGainAdaptive
        self.gain = SafeGainAdaptive(self.name + "_gain")
        self.gain.computeParameters(4.9, .3, 0.02, 0.2)
        plug(self.gain.gain, self.tp.controlGain)
        plug(self.tp.error, self.gain.error)

        n, c = norm_inferior_to (self.name + "_positioncmp",
                self.tp.error, self.thr_task_error)
        self.events = {
                "done_close": c.sout,
                "done_open": c.sout,
                }
