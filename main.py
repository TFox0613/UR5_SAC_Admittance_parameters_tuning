import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import argparse
import time

from model.network_SAC import *
from model.replay_buffer import *
from model.environment import *
from utils.record_func import *

def train():
    # Update the networks
    def soft_q_update(batch_size, gamma=0.99, mean_lambda=1e-3, std_lambda=1e-3, z_lambda=0.0, soft_tau=1e-2):
        state, action, reward, next_state, done = replay_buffer.sample(batch_size)

        state      = torch.FloatTensor(state).to(device)
        next_state = torch.FloatTensor(next_state).to(device)
        action     = torch.FloatTensor(action).to(device)
        reward     = torch.FloatTensor(reward).unsqueeze(1).to(device)
        done       = torch.FloatTensor(np.float32(done)).unsqueeze(1).to(device)

        expected_q_value = soft_q_net(state, action)
        expected_value   = value_net(state)
        new_action, log_prob, z, mean, log_std = policy_net.evaluate(state)


        target_value = target_value_net(next_state)
        next_q_value = reward + (1 - done) * gamma * target_value
        q_value_loss = soft_q_criterion(expected_q_value, next_q_value.detach())

        expected_new_q_value = soft_q_net(state, new_action)
        next_value = expected_new_q_value - log_prob
        value_loss = value_criterion(expected_value, next_value.detach())

        log_prob_target = expected_new_q_value - expected_value
        policy_loss = (log_prob * (log_prob - log_prob_target).detach()).mean()
        

        mean_loss = mean_lambda * mean.pow(2).mean()
        std_loss  = std_lambda  * log_std.pow(2).mean()
        z_loss    = z_lambda    * z.pow(2).sum(1).mean()

        policy_loss += mean_loss + std_loss + z_loss

        soft_q_optimizer.zero_grad()
        q_value_loss.backward()
        soft_q_optimizer.step()

        value_optimizer.zero_grad()
        value_loss.backward()
        value_optimizer.step()

        policy_optimizer.zero_grad()
        policy_loss.backward()
        policy_optimizer.step()
        
        
        for target_param, param in zip(target_value_net.parameters(), value_net.parameters()):
            target_param.data.copy_(
                target_param.data * (1.0 - soft_tau) + param.data * soft_tau
            )

        print(f"Q-Value Loss: {q_value_loss.item()} ", 
              f"Value Loss: {value_loss.item()} ", 
              f"Policy Loss: {policy_loss.item()} ")
        
        q_losses.append(q_value_loss.item())
        valus_losses.append(value_loss.item())
        policy_losses.append(policy_loss.item())

        return f"\nQ-Value Loss: {q_value_loss.item()}, Value Loss: {value_loss.item()}, Policy Loss: {policy_loss.item()} "

    use_cuda = torch.cuda.is_available()
    device   = torch.device("cuda" if use_cuda else "cpu")
    print("device : ", device)

    ##################### Define env params ##################### 
    env = Environment(6 , 3) # state_dim, action_dim
    env.save = [0]
    action_dim = env.action_dim
    state_dim  = env.state_dim

    hidden_dim = opt.hidden_dim

    value_net        = ValueNetwork(state_dim, hidden_dim).to(device)
    target_value_net = ValueNetwork(state_dim, hidden_dim).to(device)
    soft_q_net       = SoftQNetwork(state_dim, action_dim, hidden_dim).to(device)
    policy_net       = PolicyNetwork(state_dim, action_dim, hidden_dim).to(device)

    for target_param, param in zip(target_value_net.parameters(), value_net.parameters()):
        target_param.data.copy_(param.data)
        
    value_criterion  = nn.MSELoss()
    soft_q_criterion = nn.MSELoss()

    value_lr  = 3e-4
    soft_q_lr = 3e-4
    policy_lr = 3e-4

    value_optimizer  = optim.Adam(value_net.parameters(), lr=value_lr)
    soft_q_optimizer = optim.Adam(soft_q_net.parameters(), lr=soft_q_lr)
    policy_optimizer = optim.Adam(policy_net.parameters(), lr=policy_lr)

    ####################### Replay Buffer ######################
    replay_buffer_size = 1000000
    replay_buffer = ReplayBuffer(replay_buffer_size)

    max_frames    = opt.max_frames
    frame_idx     = 0
    frame_idxs    = []
    max_steps     = 500
    rewards       = []
    q_losses      = []
    valus_losses  = []
    policy_losses = []
    loss_log      = ""
    batch_size    = opt.batch_size

    start_time = time.time()

    while frame_idx < max_frames:
        print("Environment reset!!")
        state = env.reset()
        state = np.array(state)

        episode_reward = 0
        
        for step in range(max_steps):
            action = policy_net.get_action(state)
            next_state, reward, done = env.step(action)
             
            replay_buffer.push(state, action, reward, next_state, done)
            if len(replay_buffer) > batch_size:
                loss_log = soft_q_update(batch_size)
            
            state = next_state
            episode_reward += reward
            frame_idx += 1
            
            if done:
                break
            
        rewards.append(episode_reward)
        frame_idxs.append(frame_idx)

        log_data = {'frame_idx'      : frame_idx,
                    'max_frames'     : max_frames,
                    'episode_reward' : episode_reward,
                    'start_time'     : start_time, 
                    'loss_log'       : loss_log,
                    }
        record_training_log(log_data, result_path)

    
    # Save result
    plot_or_save(frame_idxs,          rewards,       "Reward", result_path)
    plot_or_save(frame_idxs,         q_losses,       "Q_loss", result_path)
    plot_or_save(frame_idxs,     valus_losses,   "Value_loss", result_path)
    plot_or_save(frame_idxs,    policy_losses,  "Policy_loss", result_path)
    plot_or_save(frame_idxs,   env.eff_errors,   "Eff_errors", result_path)
    plot_or_save(frame_idxs, env.track_errors, "Track_errors", result_path)

    torch.save(policy_net.state_dict(), weights_path + ".pth")

def PTP():
    use_cuda = torch.cuda.is_available()
    device   = torch.device("cuda" if use_cuda else "cpu")

    env = Environment(6 , 12)
    
    goal = [1.47164123, -1.0, 1.76184294, -1.1, -1.57079628, -1.8164123]
    env.set_Goal(goal)
    env.save = [1]

    action_dim = env.action_dim  # PD-like parmas (Kp, Kv)
    state_dim  = env.state_dim
    weight = opt.weights
    policy_net = PolicyNetwork(state_dim, action_dim, 1024).to(device)
    policy_net.load_state_dict(torch.load(weight))

    state = env.reset()

    state = np.array(state)
    action = policy_net.get_action(state)
    env.step(action)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='model.pth', help='model.pt path(s)')
    parser.add_argument('--batch_size', type=int, default=32, help='batch size')
    parser.add_argument('--task_name', type=str, default="Admittance_params", help='what task_name in path')
    parser.add_argument('--path_option', type=str, default="hidden_1024", help='path option')
    parser.add_argument('--max_frames', type=int, default=100, help='maximum iteration')
    parser.add_argument('--hidden_dim', type=int, default=256, help='hidden_dim of networks')
    parser.add_argument('--usefulness', type=str, default="train", help='train or PTP')
    opt = parser.parse_args()

    if opt.usefulness == "train":
        result_path, weights_path = create_path(opt.task_name, opt.path_option)
        train()
    elif opt.usefulness == "PTP":
        PTP()
    else:
        pass
    # python main.py --batch_size 16 --hidden_dim 1024 --max_frames 10000 --path_option hidden_1024
    # python main.py --usefulness PTP --weights ./weights/20231017_001715_PD_like_params_hidden_1024.pth