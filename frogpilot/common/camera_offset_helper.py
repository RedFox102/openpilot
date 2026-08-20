"""User-configurable camera offset, applied as a shear to the vision model's warp matrix.

Ported from sunnypilot (sunnypilot/sunnypilot#1614, MIT licensed, (c) Haibin Wen, sunnypilot,
and contributors). Simulates laterally translating the camera by `offset` meters: for a ground
point at distance d, the sampled pixel shifts by f * offset / d, which is exactly the shear
u' = u + (offset / height) * (v - cy) applied to the frame-to-model-input warp matrix.
Positive offsets shift the perceived viewpoint right, so the car tracks further left.
"""
import numpy as np


class CameraOffsetHelper:
  def __init__(self):
    self.camera_offset = 0.0
    self.actual_camera_offset = 0.0

  @staticmethod
  def apply_camera_offset(model_transform, intrinsics, height, offset_param):
    cy = intrinsics[1, 2]
    shear = np.eye(3, dtype=np.float32)
    shear[0, 1] = offset_param / height
    shear[0, 2] = -offset_param / height * cy
    return (shear @ model_transform).astype(np.float32)

  def set_offset(self, offset):
    self.camera_offset = offset

  def update(self, model_transform_main, model_transform_extra, dc, height, main_wide_camera):
    # smoothed toward the target so mid-drive adjustments ramp in over a few seconds
    # (runs at liveCalibration rate, ~4 Hz)
    self.actual_camera_offset = (0.9 * self.actual_camera_offset) + (0.1 * self.camera_offset)

    intrinsics_main = dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics
    model_transform_main = self.apply_camera_offset(model_transform_main, intrinsics_main, height, self.actual_camera_offset)
    model_transform_extra = self.apply_camera_offset(model_transform_extra, dc.ecam.intrinsics, height, self.actual_camera_offset)
    return model_transform_main, model_transform_extra
