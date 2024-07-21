from openpilot.common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.tesla.teslacan import TeslaCAN
from openpilot.selfdrive.car.tesla.values import DBC, CarControllerParams


def torque_blended_angle(apply_angle, torsion_bar_torque):
  deadzone = CarControllerParams.TORQUE_TO_ANGLE_DEADZONE
  if abs(torsion_bar_torque) < deadzone:
    return apply_angle

  limit = CarControllerParams.TORQUE_TO_ANGLE_CLIP
  if apply_angle * torsion_bar_torque >= 0:
    # Manually steering in the same direction as OP
    strength = CarControllerParams.TORQUE_TO_ANGLE_MULTIPLIER_OUTER
  else:
    # User is opposing OP direction
    strength = CarControllerParams.TORQUE_TO_ANGLE_MULTIPLIER_INNER

  torque = torsion_bar_torque - deadzone if torsion_bar_torque > 0 else torsion_bar_torque + deadzone
  return apply_angle + clip(torque, -limit, limit) * strength

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.apply_angle_last = 0
    self.last_hands_nanos = 0
    self.packer = CANPacker(dbc_name)
    self.pt_packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.tesla_can = TeslaCAN(self.packer, self.pt_packer)
    self.first_cc_cancel_nanos = None

  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    # OEM system should automatically cancel, we'll only send cancel cmd as a backup.
    # Timer is necessary due to race condition between state and control, sending cancel
    # when not necessary results in a dashboard alert.
    if CC.cruiseControl.cancel and self.first_cc_cancel_nanos is None:
      self.first_cc_cancel_nanos = now_nanos
    if not CC.cruiseControl.cancel:
      self.first_cc_cancel_nanos = None
    pcm_cancel_cmd = CC.cruiseControl.cancel and now_nanos - self.first_cc_cancel_nanos > 1e9  # 1s

    can_sends = []

    if self.frame % 2 == 0:
      # Detect a user override of the steering wheel when...
      CS.steering_override = (CS.hands_on_level >= 3 or  # user is applying lots of force or...
        (CS.steering_override and  # already overriding and...
         abs(CS.out.steeringAngleDeg - actuators.steeringAngleDeg) > CarControllerParams.CONTINUED_OVERRIDE_ANGLE) and
         not CS.out.standstill)  # continued angular disagreement while moving.

      # If fully hands off for 1 second then reset override (in case of continued disagreement above)
      if CS.hands_on_level > 0:
        self.last_hands_nanos = now_nanos
      elif now_nanos - self.last_hands_nanos > 1e9:
        CS.steering_override = False

      # Reset override when disengaged to ensure a fresh activation always engages steering.
      if not CC.latActive:
        CS.steering_override = False

      # Temporarily disable LKAS if user is overriding or if OP lat isn't active
      lkas_enabled = CC.latActive and not CS.steering_override

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

    if self.frame % 10 == 0 and pcm_cancel_cmd:
      counter = int(CS.sccm_right_stalk_counter)
      can_sends.append(self.tesla_can.right_stalk_press((counter + 1) % 16 , 1))  # half up (cancel acc)

    # TODO: HUD control

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
