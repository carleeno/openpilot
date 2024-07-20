#!/usr/bin/env python3
from cereal import car
from panda import Panda
from openpilot.common.params import Params
from openpilot.selfdrive.car.tesla.values import CAR
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "tesla"

    # Steer blending with user torque is done virtually, and is limited to 2Nm of torque
    # before it temporarily disables OP Lat control for higher user torque. This is not
    # how openpilot typically works, hence dashcamOnly
    ret.dashcamOnly = False

    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.longitudinalActuatorDelay = 0.5 # s
    ret.radarUnavailable = True

    params = Params()
    stock_acc = params.get_bool("StockTaccEnabledToggle")
    if candidate in [CAR.TESLA_AP3_MODEL3, CAR.TESLA_AP3_MODELY]:
      flags = Panda.FLAG_TESLA_MODEL3_Y
      if not stock_acc:
        flags |= Panda.FLAG_TESLA_LONG_CONTROL
      ret.openpilotLongitudinalControl = not stock_acc
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, flags)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam, self.cp_adas)

    ret.events = self.create_common_events(ret).to_msg()

    return ret
