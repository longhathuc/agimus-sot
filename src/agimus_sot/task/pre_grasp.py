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
from . import SotTask, FeaturePose

from agimus_sot.sot import SafeGainAdaptive
from .task import Task
from agimus_sot.tools import _createOpPoint, assertEntityDoesNotExist, \
    matrixHomoInverse, matrixHomoProduct, se3ToTuple, entityIfMatrixHomo

## \brief A pregrasp (and preplace) task.
# It creates a task to pose of the gripper with respect to the handle.
# This pose should follow the relative pose from planning.
# If the \c handle is attached to the joint tree (by another grasp) then,
# provide the \c otherGraspOnObject.
#
# There are 3 cases:
# - \c gripper enabled and \c otherGripper disabled or not provided:
#   Absolute pose of the \c gripper with respect to the \c handle.
# - \c gripper and \c otherGripper enabled:
#   Relative pose of the \c gripper with respect to the \c otherGripper.
# - \c gripper disabled and \c otherGripper enabled:
#   Absolute pose of the \c otherGripper with respect to the \c handle.
#
# \note For preplace task, the gripper is on the environment surface and
# the handle is on the object surface.
class PreGrasp (Task):
    name_prefix = "pregrasp"
    meas_suffix = "_measured"

    ## Constructor
    # \param gripper object of type OpFrame
    # \param handle object of type OpFrame
    # \param otherGraspOnObject either None or a tuple (otherGripper, otherHandle)
    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(PreGrasp, self).__init__()
        self.gripper = gripper
        self.handle = handle
        if otherGraspOnObject is not None:
            self.otherGripper = otherGraspOnObject[0]
            self.otherHandle  = otherGraspOnObject[1]
        else:
            self.otherGripper = None
            self.otherHandle  = None

    def makeTasks(self, sotrobot, withMeasurementOfObjectPos, withMeasurementOfGripperPos,
            withMeasurementOfOtherGripperPos = False, withDerivative = False):
        assert self.gripper.enabled
        assert self.otherGripper is None or self.otherGripper.enabled

        if self.gripper.controllable:
            if self.otherGripper is not None and self.otherGripper.controllable:
                self._makeRelativeTask (sotrobot,
                        withMeasurementOfObjectPos, withMeasurementOfGripperPos,
                        withMeasurementOfOtherGripperPos, withDerivative)
            else:
                self._makeAbsolute (sotrobot,
                        withMeasurementOfObjectPos, withMeasurementOfGripperPos,
                        withDerivative)
        else:
            if self.otherGripper is not None and self.otherGripper.controllable:
                if self.handle.robotName != self.otherHandle.robotName and \
                        self.gripper.robotName == self.otherHandle.robotName:
                    # locally inverse the gripper and the handle
                    print("swapping gripper {} and handle {}".format(self.gripper.fullName, self.handle.fullName))
                    self.gripper, self.handle = self.handle, self.gripper
                self._makeAbsoluteBasedOnOther (sotrobot,
                        withMeasurementOfObjectPos, withMeasurementOfGripperPos,
                        withMeasurementOfOtherGripperPos,
                        withDerivative)
            else:
                # TODO Both grippers are disabled so nothing can be done...
                # add a warning ?
                print("Both grippers are disabled so nothing can be done")

    ## Plug the position of linkName to \c outSignal.
    #  The pose of linkName must be computable by the SoT robot entity.
    def _plugRobotLink (self, sotrobot, linkName, poseSignal, Jsignal, withMeasurement):
        if withMeasurement:
            _createOpPoint (sotrobot, sotrobot.camera_frame)
            linkNameMeas = linkName + self.meas_suffix
            oMl = matrixHomoProduct(linkNameMeas + "_wrt_world",
                    sotrobot.dynamic.signal(sotrobot.camera_frame), None,
                    check=False)
            if_ = entityIfMatrixHomo (linkNameMeas + "_wrt_world_safe",
                    condition=None,
                    value_then=oMl.sout,
                    value_else=sotrobot.dynamic.signal(linkName),
                    check=False)
            self.addTfListenerTopic(linkNameMeas,
                    frame0 = sotrobot.camera_frame,
                    frame1 = linkNameMeas,
                    signalGetters = [ (oMl.sin1, if_.condition), ],
                    )
            plug(if_.out, poseSignal)
        else:
            plug(sotrobot.dynamic.signal(linkName), poseSignal)
            print("Plug robot link: no measument for " + linkName)
        if Jsignal is not None:
            plug(sotrobot.dynamic.signal("J"+linkName), Jsignal)

    ## Plug the position of linkName to \c outSignal.
    #  The pose of linkName is not computable by the SoT robot entity.
    #  \warning The topic linkName must have been created before.
    #  \todo reading the hpp joint topic sparsely will fail to provide the value
    #        at the expected time (the object pose will be asynchroneous with the
    #        rest of SoT).
    def _plugObjectLink (self, sotrobot, linkName, outSignal, withMeasurement):
        if withMeasurement:
            linkNameMeas = linkName + self.meas_suffix

            # Create default value
            _createOpPoint (sotrobot, sotrobot.camera_frame)
            oMl = matrixHomoProduct(linkNameMeas + "_wrt_world",
                    sotrobot.dynamic.signal(sotrobot.camera_frame),
                    None,
                    check=False,)
            name = linkNameMeas + "wrt_world"
            if_ = entityIfMatrixHomo (name, condition=None,
                    value_then=oMl.sout,
                    value_else=None,
                    check=False)
            self.addTfListenerTopic (linkNameMeas,
                    frame0 = sotrobot.camera_frame,
                    frame1 = linkNameMeas,
                    signalGetters = [(oMl.sin1, if_.condition),],
                    )
            self.addHppJointTopic (linkName, signalGetters = [ if_.else_, ],)
            plug(if_.out, outSignal)
        else:
            print("Plug object link: no measument for " + linkName)
            self.extendSignalGetters(linkName, outSignal)

    ## Compute desired pose between gripper and handle.
    #  It is decomposed as \f$ jgMg^-1 * oMjg^-1 * oMlh * lhMh \f$.
    #  It creates the entity faMfbDes.
    #  Topic \c handle.fullLink must exists.
    def _referenceSignal (self, name, gripper, handle):
        # oMjg^-1 -> HPP joint
        self.oMjaDes_inv = matrixHomoInverse (name + "_oMjaDes_inv")
        self.addHppJointTopic (gripper.fullLink, signalGetters = [ self.oMjaDes_inv.sin, ],)
        # Plug it to FeaturePose
        self.faMfbDes = matrixHomoProduct (name + "_faMfbDes",
            gripper.lMf.inverse(), # jgMg^-1
            self.oMjaDes_inv.sout, # oMjg^-1 -> HPP joint
            None,                  # oMlh -> HPP joint
            handle.lMf,            # lhMh
            )
        # oMlh -> HPP joint
        self.extendSignalGetters(handle.fullLink, self.faMfbDes.sin2)

    def _createTaskAndGain (self, name):
        # Create a task
        self.task = SotTask (name + "_task")
        self.task.add (self.feature.name)

        # Set the task gain
        self.gain = SafeGainAdaptive(name + "_gain")
        # See doc of SafeGainAdaptive to see how to plot the gain associated
        # to those values.
        self.gain.computeParameters(0.9,0.1,0.3,1.)
        plug(self.gain.gain, self.task.controlGain)
        plug(self.task.error, self.gain.error)

    ## \todo implement tracking of velocity
    def _makeAbsolute(self, sotrobot, withMeasurementOfObjectPos, withMeasurementOfGripperPos, withDerivative):
        name = self._name(self.gripper.name, self.handle.fullName)

        assertEntityDoesNotExist(name+"_feature")
        self.feature = FeaturePose (name + "_feature")

        # Create the operational points
        _createOpPoint (sotrobot, self.gripper.link)

        self._plugRobotLink (sotrobot, self.gripper.link,
                self.feature.oMja, self.feature.jaJja,
                withMeasurementOfGripperPos)
        self.feature.jaMfa.value = se3ToTuple(self.gripper.lMf)

        self.addHppJointTopic (self.handle.fullLink)
        self._plugObjectLink (sotrobot, self.handle.fullLink,
                self.feature.oMjb, withMeasurementOfObjectPos)
        self.feature.jbMfb.value = se3ToTuple(self.handle.lMf)
        self.feature.jbJjb.value = np.zeros((6, sotrobot.dynamic.getDimension()))

        # Compute desired pose between gripper and handle.
        # Creates the entity faMfbDes
        self._referenceSignal (name, self.gripper, self.handle)
        plug(self.faMfbDes.sout, self.feature.faMfbDes)

        # Create a task and gain
        self._createTaskAndGain (name)

        if withDerivative:
            print("Relative pose constraint with derivative is not implemented yet.")
        self.task.setWithDerivative (False)

        self.tasks = [ self.task, ]

    ## \todo implement tracking of velocity
    def _makeRelativeTask (self, sotrobot,
            withMeasurementOfObjectPos, withMeasurementOfGripperPos,
            withMeasurementOfOtherGripperPos,
            withDerivative):
        assert self.handle.robotName == self.otherHandle.robotName
        assert self.handle.link      == self.otherHandle.link
        name = self._name(self.gripper.name, self.handle.fullName,
            "relative", self.otherGripper.name, self.otherHandle.fullName)

        assertEntityDoesNotExist(name+"_feature")
        self.feature = FeaturePose (name + "_feature")

        # Create the operational points
        _createOpPoint (sotrobot, self.     gripper.link)
        _createOpPoint (sotrobot, self.otherGripper.link)

        # Joint A is the gripper link
        self._plugRobotLink (sotrobot, self.     gripper.link,
                self.feature.oMja, self.feature.jaJja,
                withMeasurementOfGripperPos)
        # Frame A is the gripper frame
        self.feature.jaJja.value = np.zeros((6, sotrobot.dynamic.getDimension()))
        self.feature.jaMfa.value = se3ToTuple(self.gripper.lMf)

        # Joint B is the other gripper link
        self._plugRobotLink (sotrobot, self.otherGripper.link,
                self.feature.oMjb, self.feature.jbJjb,
                withMeasurementOfOtherGripperPos)
        self.feature.jbJjb.value = np.zeros((6, sotrobot.dynamic.getDimension()))

        # Frame B is the handle frame
        # jbMfb = ogMh = ogMo(t) * oMh
        method = 3
        if method == 0: # Works
            # jbMfb        = ogMoh * ohMo * oMh
            self.feature.jbMfb.value = se3ToTuple (
                    self.otherGripper.lMf
                    * self.otherHandle.lMf.inverse()
                    * self.handle.lMf)
            self.addHppJointTopic (self.handle.fullLink)
        elif method == 1: # Does not work
            # Above, it is assumed that ogMoh = Id, which must be corrected.
            # Instead, we compute
            # jbMfb = ogMh = ogMo(t) * oMh
            #              = wMog(t)^-1 * wMo(t) * oMh
            # wMog^-1
            self.wMog_inv = matrixHomoInverse (name + "_wMog_inv")
            self._plugRobotLink (sotrobot, self.otherGripper.link,
                    self.wMog_inv.sin, None,
                    withMeasurementOfOtherGripperPos)
            self.jbMfb = matrixHomoProduct (name + "_jbMfb",
                self.wMog_inv.sout,      # wMog^-1 -> HPP joint
                None,                    # wMo -> HPP joint
                self.handle.lMf,         # oMh
                )
            # wMo
            self.addHppJointTopic (self.handle.fullLink)
            self._plugObjectLink (sotrobot, self.handle.fullLink,
                    self.jbMfb.sin1, withMeasurementOfObjectPos)
            # Plug it to FeaturePose
            plug(self.jbMfb.sout, self.feature.jbMfb)
        elif method == 2: # Seems to work
            # jbMfb        = ogMo * oMh
            self.jbMfb = matrixHomoProduct (name + "_jbMfb",
                None,                    # ogMo -> TF
                self.handle.lMf,         # oMh
                )
            plug(self.jbMfb.sout, self.feature.jbMfb)
            # ogMo
            self._defaultValue, signals = \
                    self.makeTfListenerDefaultValue(name+"_defaultValue",
                            self.otherGripper.lMf * self.otherHandle.lMf.inverse(),
                            outputs = self.jbMfb.sin0)
            self.addTfListenerTopic (
                    self.otherHandle.fullLink + self.meas_suffix + "_wrt_" + self.otherGripper.link + self.meas_suffix,
                    frame0 = self.otherGripper.link + self.meas_suffix,
                    frame1 = self.otherHandle.fullLink + self.meas_suffix,
                    signalGetters = [ signals, ],
                    )

            self.addHppJointTopic (self.handle.fullLink)
        elif method == 3:
            self.feature.jbMfb.value = se3ToTuple (self.otherGripper.lMf
                    * self.otherHandle.lMf.inverse() * self.handle.lMf)
            self.addHppJointTopic (self.handle.fullLink)

        # Compute desired pose between gripper and handle.
        # Creates the entity faMfbDes
        self._referenceSignal (name, self.gripper, self.handle)
        plug(self.faMfbDes.sout, self.feature.faMfbDes)

        # Create a task and gain
        self._createTaskAndGain (name)

        if withDerivative:
            print("Relative pose constraint with derivative is not implemented yet.")
        self.task.setWithDerivative (False)

        self.tasks = [ self.task ]
        # TODO Add velocity

    ## Placement case.
    ## An example:
    ## - the pair (gripper, handle) is the environment and an object,
    ## - the pair (otherGripper, otherHandle) is the robot end effector and the same object.
    ## \todo implement tracking of velocity
    def _makeAbsoluteBasedOnOther (self, sotrobot,
            withMeasurementOfObjectPos, withMeasurementOfGripperPos,
            withMeasurementOfOtherGripperPos, withDerivative):
        assert self.handle.robotName == self.otherHandle.robotName
        assert self.handle.link      == self.otherHandle.link
        name = self._name(self.gripper.fullName, self.handle.fullName,
            "based", self.otherGripper.name, self.otherHandle.fullName)

        assertEntityDoesNotExist(name+"_feature")
        self.feature = FeaturePose (name + "_feature")

        # Create the operational point
        _createOpPoint (sotrobot, self.otherGripper.link)

        # Joint A is the gripper link
        self.addHppJointTopic (self.gripper.fullLink)
        self._plugObjectLink (sotrobot, self.gripper.fullLink,
                self.feature.oMja, withMeasurementOfGripperPos)
        # Frame A is the gripper frame
        self.feature.jaMfa.value = se3ToTuple(self.gripper.lMf)
        self.feature.jaJja.value = np.zeros((6, sotrobot.dynamic.getDimension()))

        # Joint B is the other gripper link
        self._plugRobotLink (sotrobot, self.otherGripper.link,
                self.feature.oMjb, self.feature.jbJjb,
                withMeasurementOfOtherGripperPos)
        # Frame B is the handle frame
        method = 1
        if method == 0: # Does not work
            # jbMfb = ogMh = ogMw * wMo * oMh
            # wMog^-1
            self.wMog_inv = matrixHomoInverse (name + "_wMog_inv")
            self._plugRobotLink (sotrobot, self.otherGripper.link,
                    self.wMog_inv.sin, None,
                    withMeasurementOfOtherGripperPos)
            self.jbMfb = matrixHomoProduct (name + "_jbMfb",
                self.wMog_inv.sout,      # wMog^-1 -> HPP joint
                None,                    # wMo -> HPP joint
                self.handle.lMf,         # oMh
                )
            # wMo
            self.addHppJointTopic (self.handle.fullLink)
            self._plugObjectLink (self.handle.fullLink,
                    self.jbMfb.sin1, withMeasurementOfObjectPos)
            # Plug it to FeaturePose
            plug(self.jbMfb.sout, self.feature.jbMfb)
        elif method == 1:
            # jbMfb        = ogMo * oMh
            # Two options for ogMo measured:
            if withMeasurementOfOtherGripperPos:
                self.jbMfb = matrixHomoProduct (name + "_jbMfb",
                    None,                    # ogMo -> TF
                    self.handle.lMf,         # oMh
                    )
                plug(self.jbMfb.sout, self.feature.jbMfb)
                # We use TF to get the position of the otherHandle wrt to the otherGripper
                self._defaultValue, signals = \
                        self.makeTfListenerDefaultValue(name+"_defaultValue",
                                self.otherGripper.lMf * self.otherHandle.lMf.inverse(),
                                outputs = self.jbMfb.sin0)
                self.addTfListenerTopic (
                        self.otherHandle.fullLink + self.meas_suffix + "_wrt_" + self.otherGripper.link + self.meas_suffix,
                        frame0 = self.otherGripper.link + self.meas_suffix,
                        frame1 = self.otherHandle.fullLink + self.meas_suffix,
                        signalGetters = [ signals, ],
                        )
            else:
                ogMo = matrixHomoProduct(name + "_jbMfb_meas",
                        matrixHomoInverse (self.otherGripper.link + "_inv", sotrobot.dynamic.signal(self.otherGripper.link)).sout,
                        sotrobot.dynamic.signal(sotrobot.camera_frame),
                        None, # Tf
                        self.handle.lMf,
                        check=True,)
                if_ = entityIfMatrixHomo (name + "_jbMfb_cond",
                        condition=None,
                        value_then=ogMo.sout,
                        value_else=self.otherGripper.lMf * self.otherHandle.lMf.inverse() * self.handle.lMf,
                        check=True)
                plug(if_.out, self.feature.jbMfb)
                # We use TF to get the position of the otherHandle wrt to the camera
                # and then we compute
                # Who should we trust ?
                # - other grasp: i.e. wMjb = wMog = wMc * cMo (measured) * oMog, jbMfb = ogMo (constant) * oMh
                # - kinematics: i.e. wMjb = wMog(q), jbMfb = ogMw(q) * wMc(q) * cMo (measured) * oMh  (needs good localisation)
                # Below we trust kinematics
                self.addTfListenerTopic (
                        self.otherHandle.fullLink + self.meas_suffix,
                        frame0 = sotrobot.camera_frame,
                        frame1 = self.handle.fullLink + self.meas_suffix,
                        signalGetters = [ (ogMo.sin2, if_.condition), ],
                        )

            self.addHppJointTopic (self.handle.fullLink)

        # Compute desired pose between gripper and handle.
        # Creates the entity faMfbDes
        self._referenceSignal (name, self.gripper, self.handle)
        plug(self.faMfbDes.sout, self.feature.faMfbDes)

        # Create a task and gain
        self._createTaskAndGain (name)

        if withDerivative:
            print("Relative pose constraint with derivative is not implemented yet.")
        self.task.setWithDerivative (False)

        self.tasks = [ self.task ]

    def addVisualServoingTrace (self, tracer):
        from agimus_sot.tools import filename_escape
        self.addTrace(tracer)
        tracer.add (self.feature.name + ".faMfbDes", filename_escape(self.feature.name) + ".desired")
        tracer.add (self.feature.name + ".faMfb"   , filename_escape(self.feature.name) + ".actual")
