from openpilot.common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.tesla.teslacan import TeslaCAN
from openpilot.selfdrive.car.tesla.values import DBC, CarControllerParams


def torque_blended_angle(apply_angle, torsion_bar_torque):
  deadzone = CarControllerParams.TORQUE_TO_ANGLE_DEADZONE
  limit = CarControllerParams.TORQUE_TO_ANGLE_CLIP

  if abs(torsion_bar_torque) < deadzone:
    torque = 0
  else:
    torque = torsion_bar_torque - deadzone if torsion_bar_torque > 0 else torsion_bar_torque + deadzone
  return apply_angle + clip(torque, -limit, limit) * CarControllerParams.TORQUE_TO_ANGLE_MULTIPLIER

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_name)
    self.pt_packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.tesla_can = TeslaCAN(self.packer, self.pt_packer)

  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    # Temp disable steering when hands-on, and allow for user override
    lkas_enabled = CC.latActive and CS.hands_on_level < 2

    if self.frame % 2 == 0:
      if lkas_enabled:
        # Update steering angle request with user input torque
        apply_angle = torque_blended_angle(actuators.steeringAngleDeg, CS.out.steeringTorque)

        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.apply_angle_last, CS.out.vEgo, CarControllerParams)

        # To not fault the EPS
        apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - 20, CS.out.steeringAngleDeg + 20)
      else:
        apply_angle = CS.out.steeringAngleDeg

      self.apply_angle_last = apply_angle
      can_sends.append(self.tesla_can.create_steering_control(apply_angle, lkas_enabled, (self.frame // 2) % 16))

    # Longitudinal control (in sync with stock message, about 40Hz)
    if self.CP.openpilotLongitudinalControl:
      acc_state = CS.das_control["DAS_accState"]
      target_accel = actuators.accel
      target_speed = max(CS.out.vEgo + (target_accel * CarControllerParams.ACCEL_TO_SPEED_MULTIPLIER), 0)
      max_accel = 0 if target_accel < 0 else target_accel
      min_accel = 0 if target_accel > 0 else target_accel

      counter = CS.das_control["DAS_controlCounter"]
      can_sends.append(self.tesla_can.create_longitudinal_commands(acc_state, target_speed, min_accel, max_accel, counter))

    # Sent cancel request only if ACC is enabled
    if self.frame % 10 == 0 and pcm_cancel_cmd and CS.acc_enabled:
      counter = int(CS.sccm_right_stalk_counter)
      can_sends.append(self.tesla_can.right_stalk_press((counter + 1) % 16 , 1))  # half up (cancel acc)
      can_sends.append(self.tesla_can.right_stalk_press((counter + 2) % 16, 0))  # to prevent neutral gear warning

    # TODO: HUD control

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
