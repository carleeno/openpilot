#!/usr/bin/env python3
import time
from cereal import car
from panda import Panda
from openpilot.common.params import Params
from openpilot.selfdrive.car.tesla.values import CAR
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase

ButtonType = car.CarState.ButtonEvent.Type


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.last_mads_press = -1

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "tesla"

    # Steer blending with user torque is done virtually, and is limited to 2Nm of torque
    # before it temporarily disables OP Lat control for higher user torque. This is not
    # how openpilot typically works, hence dashcamOnly
    # ret.dashcamOnly = True

    ret.steerControlType = car.CarParams.SteerControlType.angle

    ret.longitudinalActuatorDelay = 0.5 # s
    ret.radarUnavailable = True

    params = Params()
    stock_acc = params.get_bool("StockLongTesla")

    if candidate in [CAR.TESLA_AP3_MODEL3, CAR.TESLA_AP3_MODELY]:
      flags = Panda.FLAG_TESLA_MODEL3_Y
      if not stock_acc:
        flags |= Panda.FLAG_TESLA_LONG_CONTROL
      ret.openpilotLongitudinalControl = not stock_acc
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, flags)]
      ret.enableBsm = True

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam, self.cp_adas)

    if self.enable_mads:
      for b in self.CS.button_events:
        if b.type == ButtonType.altButton2 and not b.pressed:
          # MADS button (right scroll wheel click) is used for voice command
          # So we use double-click to toggle MADS
          now = time.monotonic()
          if now - self.last_mads_press < 1.0:
            self.CS.madsEnabled = not self.CS.madsEnabled
            self.last_mads_press = -1
          else:
            self.last_mads_press = now
      self.CS.madsEnabled = self.get_acc_mads(ret.cruiseState.enabled, self.CS.accEnabled, self.CS.madsEnabled)
      self.CS.madsEnabled = False if self.CS.steering_override else self.CS.madsEnabled

    self.CS.accEnabled = ret.cruiseState.enabled  # ACC state is controlled by the car itself

    if not self.CP.pcmCruise or (self.CP.pcmCruise and self.CP.minEnableSpeed > 0) or not self.CP.pcmCruiseSpeed:
      if any(b.type == ButtonType.cancel for b in self.CS.button_events):
        self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
    if self.get_sp_pedal_disengage(ret):
      self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
      ret.cruiseState.enabled = ret.cruiseState.enabled if not self.enable_mads else False if self.CP.pcmCruise else self.CS.accEnabled

    ret, self.CS = self.get_sp_common_state(ret, self.CS)

    ret.buttonEvents = [
      *self.CS.button_events,
      *self.button_events.create_mads_event(self.CS.madsEnabled, self.CS.out.madsEnabled)  # MADS BUTTON
    ]

    events = self.create_common_events(ret, c, pcm_enable=False)

    events, ret = self.create_sp_events(self.CS, ret, events)

    ret.events = events.to_msg()

    return ret
