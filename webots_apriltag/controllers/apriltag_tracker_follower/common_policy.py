# common_policy.py  — tiny utilities for data logging + model use
import os, csv, joblib, numpy as np

DATA_PATH  = os.path.expanduser("~/apriltag_policy_data.csv")
MODEL_PATH = os.path.expanduser("~/apriltag_policy.joblib")

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

def policy_predict_left_right(model, feat):
    """
    Model outputs desired [left, right] wheel speeds.
    Clip to e-puck limits.
    """
    left, right = model.predict([feat])[0]
    left  = float(np.clip(left,  -6.28, 6.28))
    right = float(np.clip(right, -6.28, 6.28))
    return left, right
