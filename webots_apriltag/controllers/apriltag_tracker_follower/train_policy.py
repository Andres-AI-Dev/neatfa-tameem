#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
import joblib

# Paths (match common_policy.py)
DATA_PATH = os.path.expanduser("~/apriltag_policy_data.csv")
MODEL_PATH = os.path.expanduser("~/apriltag_policy.joblib")

print("📁 Loading data from:", DATA_PATH)
df = pd.read_csv(DATA_PATH)
print(f"Loaded {len(df)} samples")

# Features (inputs) and targets (outputs)
X = df[["err_x", "err_x_d", "size_err", "size_err_d"]].values
y = df[["left", "right"]].values

# Train/test split
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Define simple neural policy (small MLP)
model = MLPRegressor(
    hidden_layer_sizes=(32, 32),
    activation="tanh",
    solver="adam",
    learning_rate_init=0.001,
    max_iter=1000,
    random_state=42
)

print("🚀 Training model...")
model.fit(X_train, y_train)

# Evaluate
y_pred = model.predict(X_test)
mse = mean_squared_error(y_test, y_pred)
print(f"✅ Done. Test MSE: {mse:.6f}")

# Save model
joblib.dump(model, MODEL_PATH)
print(f"💾 Saved policy to: {MODEL_PATH}")
