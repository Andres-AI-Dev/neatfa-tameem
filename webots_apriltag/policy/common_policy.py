# common_policy.py — tiny utilities for data logging + model use
import os, csv, joblib, numpy as np

# Per-controller naming via env (defaults to 'apriltag_policy')
BASE_NAME  = os.environ.get("POLICY_NAME", "apriltag_policy")
DATA_PATH  = os.path.expanduser(f"~/{BASE_NAME}_data.csv")
MODEL_PATH = os.path.expanduser(f"~/{BASE_NAME}.joblib")

_log_file = None
_log_writer = None

def init_logger():
    """Start CSV logger once (append mode)."""
    global _log_file, _log_writer
    if _log_file is None:
        new_file = not os.path.exists(DATA_PATH)
        _log_file = open(DATA_PATH, "a", newline="")
        _log_writer = csv.writer(_log_file)
        if new_file:
            _log_writer.writerow(
                ["t","err_x","err_x_d","size_err","size_err_d","rot","fwd","left","right"]
            )

def log_row(t, err_x, err_x_d, size_err, size_err_d, rot, fwd, left, right):
    if _log_writer:
        _log_writer.writerow(
            [float(t), float(err_x), float(err_x_d), float(size_err), float(size_err_d),
             float(rot), float(fwd), float(left), float(right)]
        )
        _log_file.flush()  # ensure data hits disk during the run

def load_policy():
    """Load model if present, else None."""
    return joblib.load(MODEL_PATH) if os.path.exists(MODEL_PATH) else None

def features_from_detection(controller_self, detection):
    """
    Build the 4-D feature vector that we both log and later feed to the model:
    [err_x, err_x_d, size_err, size_err_d]
    """
    # horizontal pixel error (tag center vs image center)
    cx = controller_self.center_x
    tag_x = (controller_self.smoothed_center_x
             if getattr(controller_self, "smoothed_center_x", None) is not None
             else detection['center'][0])
    err_x = tag_x - cx
    err_x_d = err_x - getattr(controller_self, "last_rotation_error", 0)

    # tag size error vs target
    size_now = controller_self.calculate_tag_size(detection) or 0.0
    size_sm = (controller_self.smoothed_size
               if getattr(controller_self, "smoothed_size", None) is not None
               else size_now)
    size_err = size_sm - controller_self.target_tag_size
    size_err_d = size_err - getattr(controller_self, "last_distance_error", 0)

    return np.array([err_x, err_x_d, size_err, size_err_d], dtype=float)

def policy_predict_left_right(model, feat, vmax=6.28):
    """
    Model outputs desired [left, right] wheel speeds.
    Clip to robot limits (vmax). Add a small gain and close-range nudge to avoid stalling.
    Tunables via env:
      POLICY_GAIN        (default 1.3)
      POLICY_MIN_FWD     (default 0.6)
      POLICY_CLOSE_SIZE  (default 200)
    """
    left, right = model.predict([feat])[0]

    # Mild boost so speeds aren't tiny
    gain = float(os.environ.get("POLICY_GAIN", "1.3"))
    left *= gain
    right *= gain

    # If we're very close (tag looks big) and both wheels ~0, nudge forward
    size_err = float(feat[2])  # smoothed_size - target_tag_size
    if size_err > float(os.environ.get("POLICY_CLOSE_SIZE", "200")):
        if abs(left) < 0.25 and abs(right) < 0.25:
            min_fwd = float(os.environ.get("POLICY_MIN_FWD", "0.6"))
            left, right = min_fwd, min_fwd

    left  = float(np.clip(left,  -vmax, vmax))
    right = float(np.clip(right, -vmax, vmax))
    return left, right
