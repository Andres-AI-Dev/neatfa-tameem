#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

# Define the model architecture (must match training)
class ActorCritic(nn.Module):
    def __init__(self, obs_dim=15, num_actions=15):
        super().__init__()
        self.feature_net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
        )
        self.actor = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, num_actions)
        )
        self.critic = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )
    def forward(self, obs):
        features = self.feature_net(obs)
        action_probs = F.softmax(self.actor(features), dim=-1)
        value = self.critic(features)
        return action_probs, value

def main():
    try:
        # Load model
        model = ActorCritic()
        checkpoint = torch.load('/root/ppo_argos/ppo_final.pt', map_location='cpu')
        model.load_state_dict(checkpoint['model_state_dict'])
        model.eval()

        print('✅ Model loaded successfully!')
        print(f'  Episode: {checkpoint["episode"]}')
        print(f'  Model parameters: {sum(p.numel() for p in model.parameters())}')

        # Test with random observation
        test_obs = torch.randn(1, 15)  # Random 15D observation
        with torch.no_grad():
            action_probs, value = model(test_obs)
            action = torch.argmax(action_probs)

        print(f'  Test observation shape: {test_obs.shape}')
        print(f'  Predicted action: {action.item()}')
        print(f'  Predicted value: {value.item():.4f}')
        print('  Action probabilities:')
        for i, prob in enumerate(action_probs.squeeze().numpy()):
            print(f'    Action {i:2d}: {prob:.4f}')

        # Test with multiple observations
        print('\nTesting with 5 random observations:')
        for j in range(5):
            test_obs = torch.randn(1, 15)
            with torch.no_grad():
                action_probs, value = model(test_obs)
                action = torch.argmax(action_probs)
            print(f'  Sample {j+1}: Action={action.item():2d}, Value={value.item():6.3f}')

    except FileNotFoundError:
        print('❌ Model file not found at /root/ppo_argos/ppo_final.pt')

    except Exception as e:
        print(f'❌ Error loading model: {e}')
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()