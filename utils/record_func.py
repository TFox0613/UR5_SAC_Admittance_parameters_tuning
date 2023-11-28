import os
import time
from datetime import datetime
import matplotlib.pyplot as plt


def create_path(task_name, option):
    current_datetime = datetime.now().strftime("%Y%m%d_%H%M%S")
    if option is not None:
        result_path  = "./data/result/"  + f"{current_datetime}_{task_name}_{option}"
        weights_path = "./weights/" + f"{current_datetime}_{task_name}_{option}"
    else:
        result_path  = "./data/result/"  + f"{current_datetime}_{task_name}"
        weights_path = "./weights/" + f"{current_datetime}_{task_name}"

    if not os.path.exists(result_path):
        os.makedirs(result_path)

    print("Result is saved to ", result_path)
    print("Weight is saved to ", weights_path)
    return result_path, weights_path

def plot_or_save(frame_idx, data, name, save_path, 
                 title_fontsize=50, label_fontsize=40, ticks_fontsize=30, labelpad=20):
    plt.figure(figsize=(20, 20))
    plt.title(name    , fontsize=title_fontsize)
    plt.plot(frame_idx[:len(data)], data)

    plt.xlabel('frame', fontsize=label_fontsize, labelpad=labelpad)
    plt.ylabel(name   , fontsize=label_fontsize, labelpad=labelpad)

    plt.xticks(fontsize=ticks_fontsize)
    plt.yticks(fontsize=ticks_fontsize)

    if save_path is not None:
        plt.savefig(save_path + "/" + name + ".jpg")
    else:
        plt.show()

def compute_remaining_time(start_time, max_frames, frame_idx):
    avg_time_taken = (time.time() - start_time) / frame_idx

    if avg_time_taken > 0:
        estimated_remaining_time = avg_time_taken * (max_frames - frame_idx)
        hours, remainder = divmod(estimated_remaining_time, 3600)
        minutes, seconds = divmod(remainder, 60)
    else:
        hours, minutes, seconds = 0, 0, 0

    return f"Iteration: {frame_idx:4}/{max_frames}, Remaining Time : {int(hours):02}:{int(minutes):02}:{int(seconds):02}, "

def record_training_log(log_data, output_path):
    frame_idx      = log_data['frame_idx']
    max_frames     = log_data['max_frames']
    start_time     = log_data['start_time']
    episode_reward = log_data['episode_reward']
    loss_log       = log_data['loss_log']

    time_log   = compute_remaining_time(start_time, max_frames, frame_idx)
    reward_log = "Reward : " + str(episode_reward)
    log = time_log + reward_log + loss_log
    print(log)
    output_file = output_path + "/log.txt"
    with open(output_file, 'a') as file:  # Open the file in append mode ('a')
        file.write(log + "\n")


if __name__ == "__main__":
    start_time = time.time()
    max_frames = 1000000
    frame_idx = 1 
    episode_reward = 20
    loss_log = f", Q-Value Loss: {0.1}, Value Loss: {0.2}, Policy Loss: {0.3} "
    result_path, weights_path = create_path("task_name", "path_option")
    while frame_idx < max_frames:
        log_data = {'frame_idx'      : frame_idx,
                    'max_frames'     : max_frames,
                    'episode_reward' : episode_reward,
                    'start_time'     : start_time,
                    'loss_log'       : loss_log
                    }
        record_training_log(log_data, result_path)
        frame_idx += 1 