"""Speed-binned torque learning, ported from zoompilot (Copyright (c) 2026-, Zeph Leggett, MIT).

Each speed bin runs torqued's total-least-squares fit on the quality-filtered points that fall in
its speed range and publishes its own latAccelFactor and friction; controlsd interpolates them by
speed. Bins and seeds come from opendbc's speed_dependent.toml; a car without a bin table there is
not speed-binned and keeps torqued's single global fit.

The bins ride on customReserved16 beside every liveTorqueParameters, at the same cadence and
validity. Their cache (values, seed version, bin centers and point buckets) is a file rather than a
Params key, so a prebuilt branch needs no native rebuild; it restores only under the same key as
torqued's own LiveTorqueParameters cache.
"""
import os
import threading
import numpy as np

import cereal.messaging as messaging
from cereal import car, log
from opendbc.car.lateral import get_speed_dep_config_for_car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths

SPEED_BINS_SERVICE = "customReserved16"
SPEED_BINS_CACHE_PATH = os.path.join(Paths.comma_home(), "torque_speed_bins") if PC else "/data/torque_speed_bins"
# the first bin's lower edge without the table's own min_speed, and the last bin's upper edge
DEFAULT_MIN_SPEED = 5.0
DEFAULT_MAX_SPEED = 40.0


def _torqued():
  # torqued imports this module, so its names resolve at call time
  from openpilot.selfdrive.locationd import torqued
  return torqued


def fit_torque_points(points):
  """torqued's estimate_params on one point set ([steer, 1, lateral accel] rows): the TLS slope
  (latAccelFactor), its offset and the spread friction. Raises LinAlgError as the SVD does."""
  torqued = _torqued()
  _, _, v = np.linalg.svd(points, full_matrices=False)
  slope, offset = -v.T[0:2, 2] / v.T[2, 2]
  _, spread = np.matmul(points[:, [0, 2]], torqued.slope2rot(slope)).T
  return slope, offset, np.std(spread) * torqued.FRICTION_FACTOR


def centers_to_bounds(centers, min_speed=None):
  """Bin bounds at the midpoints between consecutive centers; the outer edges take min_speed (5 m/s
  when the table sets none) and 40 m/s."""
  bounds = []
  for i, c in enumerate(centers):
    lo = (DEFAULT_MIN_SPEED if min_speed is None else min_speed) if i == 0 else (centers[i - 1] + c) / 2
    hi = DEFAULT_MAX_SPEED if i == len(centers) - 1 else (c + centers[i + 1]) / 2
    bounds.append((lo, hi))
  return bounds


def within_bounds(vals, bounds):
  """Per bin, whether its value is finite and inside its (lo, hi) clip range. The wire is Float32,
  so a filter sitting on a bound can read back a rounding step past it."""
  vals = np.asarray(vals, dtype=float)
  lo, hi = np.asarray(bounds, dtype=float).T
  tol = 1e-5 * np.maximum(np.abs(hi), 1.0)
  return np.isfinite(vals) & (vals >= lo - tol) & (vals <= hi + tol)


class SpeedBinLearner:
  """Mixed into TorqueEstimator, which must have set CP, factor_sanity, friction_sanity,
  min_bucket_points and fit_points before init_speed_bins runs."""
  speed_binned = False

  def init_speed_bins(self, cache_path=SPEED_BINS_CACHE_PATH):
    self.speed_bins_cache_path = cache_path
    cfg = get_speed_dep_config_for_car(self.CP) if self.CP.lateralTuning.which() == 'torque' else {}
    self.speed_binned = 'speed_bp' in cfg
    if not self.speed_binned:
      return

    # bumped with a seed refresh to retire every cache learned under the old seeds
    self.speed_dep_seed_version = int(cfg.get('seed_version', 0))
    self.speed_bin_centers = [float(c) for c in cfg['speed_bp']]
    self.speed_bin_bounds = centers_to_bounds(self.speed_bin_centers, cfg.get('min_speed'))
    n_bins = len(self.speed_bin_centers)

    self.speed_bin_points = [self._make_speed_bin_bucket() for _ in range(n_bins)]
    # a bin with points since its last fit: a full bin's length stops changing while its ring
    # buffers keep turning over, so the length cannot mark new data
    self._speed_bin_dirty = [True] * n_bins

    ref_lafs = [float(x) for x in cfg['laf_bp']]
    ref_frictions = [float(x) for x in cfg['friction_bp']]
    assert len(ref_lafs) == n_bins and len(ref_frictions) == n_bins, "speed_dependent.toml: bin lists differ in length"
    self.speed_bin_decays = [_torqued().MIN_FILTER_DECAY] * n_bins
    self.speed_bin_filtered = [
      {'latAccelFactor': FirstOrderFilter(ref_lafs[i], self.speed_bin_decays[i], DT_MDL),
       'frictionCoefficient': FirstOrderFilter(ref_frictions[i], self.speed_bin_decays[i], DT_MDL)}
      for i in range(n_bins)
    ]
    # the fits are clipped to +-sanity of the seed, as torqued clips its global fit
    self.speed_bin_lat_accel_factor_bounds = [((1.0 - self.factor_sanity) * f, (1.0 + self.factor_sanity) * f) for f in ref_lafs]
    self.speed_bin_friction_bounds = [((1.0 - self.friction_sanity) * f, (1.0 + self.friction_sanity) * f) for f in ref_frictions]
    self.restore_speed_bins()

  def _make_speed_bin_bucket(self):
    """One speed bin's buckets. Each bin sees a fraction of the data, so the per-bucket minimums are
    the global learner's divided by the bin count."""
    torqued = _torqued()
    scaled_min = np.maximum(np.asarray(self.min_bucket_points) // len(self.speed_bin_centers), 1)
    return torqued.TorqueBuckets(x_bounds=torqued.STEER_BUCKET_BOUNDS,
                                 min_points=scaled_min,
                                 min_points_total=int(scaled_min.sum()),
                                 points_per_bucket=torqued.POINTS_PER_BUCKET,
                                 rowsize=3)

  def add_speed_bin_point(self, steer, lateral_acc, vego):
    """Routes a quality-filtered point from handle_log to its speed bin."""
    if not self.speed_binned:
      return
    for i, (lo, hi) in enumerate(self.speed_bin_bounds):
      if lo <= vego < hi:
        self.speed_bin_points[i].add_point(steer, lateral_acc)
        self._speed_bin_dirty[i] = True
        break

  def restore_speed_bins(self, cache_ltp=None, cache_CP=None, cache_sp=None):
    """Restores the per-bin filters, decay and points. torqued's LiveTorqueParameters cache supplies
    the restore key, decay and the valid flag; the bin cache its own VERSION, seed version, bin
    centers, values and points. Both must carry this car's restore key (fingerprint, tuning type,
    offline seeds, VERSION) and the bin cache this config's seed_version and centers; the values are
    taken only when torqued's cache was written valid. Each bin is judged on its own: one whose value
    is non-finite or outside its clip range falls back to its seed at MIN decay while the rest
    restore. Reads Params and the cache file for whichever argument is None."""
    if not self.speed_binned:
      return
    try:
      if cache_ltp is None or cache_CP is None:
        from openpilot.common.params import Params
        params = Params()
        torque_cache = params.get("LiveTorqueParameters")
        params_cache = params.get("CarParamsPrevRoute")
        if not torque_cache or not params_cache:
          cloudlog.info("speed-dep: no torque cache, starting from the seeds")
          return
        with log.Event.from_bytes(torque_cache) as evt:
          cache_ltp = evt.liveTorqueParameters
        with car.CarParams.from_bytes(params_cache) as msg:
          cache_CP = msg
      torqued = _torqued()
      restore_key = torqued.TorqueEstimator.get_restore_key

      if restore_key(cache_CP, cache_ltp.version) != restore_key(self.CP, torqued.VERSION):
        cloudlog.info("speed-dep: torque cache restore key mismatch, starting from the seeds")
        return
      if cache_sp is None:
        if not os.path.isfile(self.speed_bins_cache_path):
          cloudlog.info("speed-dep: no bin cache, starting from the seeds")
          return
        with open(self.speed_bins_cache_path, 'rb') as f:
          with log.Event.from_bytes(f.read()) as evt:
            cache_sp = getattr(evt, SPEED_BINS_SERVICE)
      if restore_key(cache_CP, cache_sp.version) != restore_key(self.CP, torqued.VERSION):
        cloudlog.info("speed-dep: bin cache restore key mismatch, starting from the seeds")
        return
      if cache_sp.seedVersion != self.speed_dep_seed_version:
        cloudlog.info(f"speed-dep: seed version {cache_sp.seedVersion} -> {self.speed_dep_seed_version}, starting from the seeds")
        return
      n_bins = len(self.speed_bin_centers)
      cached_centers = list(cache_sp.speedBinCenters)
      if len(cached_centers) != n_bins or not np.allclose(cached_centers, self.speed_bin_centers, atol=0.01):
        cloudlog.info("speed-dep: bin centers changed, starting from the seeds")
        return
      cached_lafs = list(cache_sp.speedBinLatAccelFactors)
      cached_frictions = list(cache_sp.speedBinFrictions)
      if len(cached_lafs) != n_bins or len(cached_frictions) != n_bins:
        cloudlog.info("speed-dep: cache bin count mismatch, starting from the seeds")
        return
      decay = float(cache_ltp.decay)
      if not np.isfinite(decay):
        cloudlog.warning("speed-dep: cached decay non-finite, starting from the seeds")
        return
      # torqued only writes a decay inside this range; the clip guards a hand-edited cache
      decay = float(np.clip(decay, torqued.MIN_FILTER_DECAY, torqued.MAX_FILTER_DECAY))

      bin_ok = (within_bounds(cached_lafs, self.speed_bin_lat_accel_factor_bounds)
                & within_bounds(cached_frictions, self.speed_bin_friction_bounds))
      if not bin_ok.all():
        cloudlog.warning(f"speed-dep: cached bins {np.flatnonzero(~bin_ok).tolist()} non-finite or outside sanity bounds, reseeding them")
      # values only from a valid cache, as torqued does for its globals
      if cache_ltp.liveValid:
        for i in np.flatnonzero(bin_ok):
          self.speed_bin_filtered[i]['latAccelFactor'].x = cached_lafs[i]
          self.speed_bin_filtered[i]['frictionCoefficient'].x = cached_frictions[i]
      else:
        cloudlog.info("speed-dep: torque cache not valid, keeping seed values")
      cached_points = self._load_points_cache(cache_sp, n_bins)
      if cached_points is not None:
        for i in range(n_bins):
          self.speed_bin_points[i].load_points(cached_points[i])
      # one decay in the cache (torqued's), so every good bin resumes at it; a reseeded bin
      # re-learns from MIN like a fresh one
      self.speed_bin_decays = [decay if ok else torqued.MIN_FILTER_DECAY for ok in bin_ok]
      for filters, bin_decay in zip(self.speed_bin_filtered, self.speed_bin_decays, strict=True):
        filters['latAccelFactor'].update_alpha(bin_decay)
        filters['frictionCoefficient'].update_alpha(bin_decay)
      cloudlog.info("speed-dep: restored speed-bin torque params from cache")
    except Exception:
      cloudlog.exception("speed-dep: failed to restore cache")

  def _load_points_cache(self, cache_sp, n_bins):
    """The per-bin points from the bin cache, or None when they fail the bin-count or finiteness
    checks. A failure here never touches the values: the bins just start with empty buckets."""
    try:
      points = [[list(point) for point in bin_points] for bin_points in cache_sp.speedBinPoints]
      if len(points) != n_bins:
        cloudlog.info("speed-dep: points cache bin count mismatch, points not restored")
        return None
      if not all(np.all(np.isfinite(np.asarray(bin_points, dtype=float))) for bin_points in points if bin_points):
        cloudlog.warning("speed-dep: cached bin points non-finite, points not restored")
        return None
      return points
    except Exception:
      cloudlog.exception("speed-dep: failed to read points cache, points not restored")
      return None

  def _estimate_params_speed_binned(self):
    """Independent fit per bin. As torqued filters its global fit, a bin is fit and filtered only once
    it is valid, so until then it publishes its seed (or the value restored from its cache). A bin
    that goes NaN is reset, as torqued resets its global fit. Returns each bin's valid flag."""
    torqued = _torqued()
    valid_flags = []
    for i, bucket in enumerate(self.speed_bin_points):
      if not bucket.is_valid():
        valid_flags.append(False)
        continue
      if not self._speed_bin_dirty[i]:
        valid_flags.append(True)
        continue

      points = bucket.get_points(self.fit_points)
      try:
        slope, _, friction_coeff = fit_torque_points(points)  # slope = latAccelFactor
        if not np.isnan(slope) and not np.isnan(friction_coeff):
          factor_lo, factor_hi = self.speed_bin_lat_accel_factor_bounds[i]
          fric_lo, fric_hi = self.speed_bin_friction_bounds[i]
          self.speed_bin_decays[i] = min(self.speed_bin_decays[i] + DT_MDL, torqued.MAX_FILTER_DECAY)  # slow down filter over time
          self.speed_bin_filtered[i]['latAccelFactor'].update(np.clip(slope, factor_lo, factor_hi))
          self.speed_bin_filtered[i]['latAccelFactor'].update_alpha(self.speed_bin_decays[i])
          self.speed_bin_filtered[i]['frictionCoefficient'].update(np.clip(friction_coeff, fric_lo, fric_hi))
          self.speed_bin_filtered[i]['frictionCoefficient'].update_alpha(self.speed_bin_decays[i])
          self._speed_bin_dirty[i] = False
          valid_flags.append(True)
          continue
      except np.linalg.LinAlgError:
        pass

      cloudlog.warning(f"speed-dep: bin {i} produced NaN with valid data, resetting bin")
      self.speed_bin_points[i] = self._make_speed_bin_bucket()
      self.speed_bin_decays[i] = torqued.MIN_FILTER_DECAY
      self._speed_bin_dirty[i] = True
      valid_flags.append(False)
    return valid_flags

  def get_speed_bins_msg(self, valid=True, with_points=False):
    """The customReserved16 event: VERSION, the seed version, the bin centers and the per-bin
    values, plus the point buckets for the cache copy. No bins on a car that is not speed-binned."""
    msg = messaging.new_message(SPEED_BINS_SERVICE)
    msg.valid = valid
    sp = getattr(msg, SPEED_BINS_SERVICE)
    sp.version = _torqued().VERSION
    if self.speed_binned:
      valid_flags = self._estimate_params_speed_binned()
      sp.seedVersion = self.speed_dep_seed_version
      sp.speedBinCenters = self.speed_bin_centers
      sp.speedBinLatAccelFactors = [float(f['latAccelFactor'].x) for f in self.speed_bin_filtered]
      sp.speedBinFrictions = [float(f['frictionCoefficient'].x) for f in self.speed_bin_filtered]
      sp.speedBinValid = valid_flags
      if with_points:
        sp.speedBinPoints = [bucket.get_points()[:, [0, 2]].tolist() for bucket in self.speed_bin_points]
    return msg

  def write_speed_bins_cache(self, msg, blocking=False):
    """Writes the cache copy (with points) atomically, so a power cut never leaves half a file. Off
    torqued's loop like its own put_nonblocking: the points make it a few MB."""
    if not self.speed_binned:
      return
    data = msg.to_bytes()
    if blocking:
      self._write_cache_file(data)
    else:
      threading.Thread(target=self._write_cache_file, args=(data,), daemon=True).start()

  def _write_cache_file(self, data):
    tmp_path = self.speed_bins_cache_path + ".tmp"
    try:
      with open(tmp_path, 'wb') as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
      os.replace(tmp_path, self.speed_bins_cache_path)
    except OSError:
      cloudlog.exception("speed-dep: failed to write bin cache")
