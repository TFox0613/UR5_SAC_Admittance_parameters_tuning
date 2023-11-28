import argparse
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel

import vf_func
import handeye_func

import pyrealsense2 as rs
import numpy as np

class rs_para:
    fps = 30
    depth_scale = 1
    Color_cx = 316.331
    Color_cy = 243.958
    Color_fx = 380.584 
    Color_fy = 380.28
    Depth_cx = 313.656
    Depth_cy = 241.491
    Depth_fx = 383.384 
    Depth_fy = 383.384


def detect(save_img=False):

    # used to record the time when we processed last frame
    prev_frame_time = 0
    # used to record the time at which we processed current frame
    new_frame_time = 0

    source, weights, view_img, save_txt, imgsz, trace = opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size, not opt.no_trace

    # Directories
    save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Initialize
    set_logging()
    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size

    if trace:
        model = TracedModel(model, device, opt.img_size)

    if half:
        model.half()  # to FP16

    # Second-stage classifier
    classify = False
    if classify:
        modelc = load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model']).to(device).eval()

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
    old_img_w = old_img_h = imgsz
    old_img_b = 1

    config = rs.config()
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline = rs.pipeline()
    profile = pipeline.start(config)

    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create a file to save FPS
    fps_file_name = f"fps_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    fps_file_path = 'fps_record/' + fps_file_name
    fps_file = open(fps_file_path, 'w')

    while(True):
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame or not color_frame:
            continue
        
        img = np.asanyarray(color_frame.get_data())
        temp_yolo_img = img.copy()
        depth_image = np.asanyarray(depth_frame.get_data())
        
        #======================Calculating FPS======================
        # time when we finish processing for this frame
        new_frame_time = time.time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        # converting the fps into integer
        fps = int(fps)
    
        # converting the fps to string so that we can display it on frame
        # by using putText function
        fps_str = str(fps)
        # print(fps)

        # Save FPS to the file
        fps_file.write(fps_str + '\n')
        fps_file.flush()
        # ======================Calculating FPS======================

        # Letterbox
        im0 = img.copy()
        img = img[np.newaxis, :, :, :]        

        # Stack
        img = np.stack(img, 0)

        # Convert000
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                model(img, augment=opt.augment)[0]

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = model(img, augment=opt.augment)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        # print("Prediction : ", pred)
        t3 = time_synchronized()
        # Process detections
        for i, det in enumerate(pred):  # detections per image

            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

        # Print results and draw boxes for high-confidence detections
        conf_threshold = 0.8
        for *xyxy, conf, cls in reversed(det):
            if conf > conf_threshold:  # Set your desired confidence threshold here
                c = int(cls)  # integer class
                label = f'{names[c]} {conf:.2f}'
                plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=2)

                # Calculate the depth for each corner of the bounding box
                x1, y1, x2, y2 = [int(coord) for coord in xyxy]

                # Ensure the indices are within bounds
                if 0 <= y1 < depth_image.shape[0] and 0 <= x1 < depth_image.shape[1] and \
                0 <= y2 < depth_image.shape[0] and 0 <= x2 < depth_image.shape[1]:
                    # Extract depth values for the four corners
                    # top_left_depth = depth_image[y1, x1]  # Top-left corner
                    # top_right_depth = depth_image[y1, x2]  # Top-right corner
                    # bottom_left_depth = depth_image[y2, x1]  # Bottom-left corner
                    # bottom_right_depth = depth_image[y2, x2]  # Bottom-right corner
                    # center_depth = depth_image[int((y1+y2)/2), int((x1+x2)/2)]

                    # # Find the minimum depth value among the corner depths
                    # # min_depth = min(top_left_depth, top_right_depth, bottom_left_depth, bottom_right_depth, center_depth)
                    # min_depth = min_depth * 0.1 # cm
                    u = int((x1+x2)/2)
                    v = int((y1+y2)/2)
                    Z = depth_image[v, u]
                    Xc = ((u-rs_para_1.Color_cx)/rs_para_1.Color_fx) * Z * rs_para_1.depth_scale
                    Yc = ((v-rs_para_1.Color_cy)/rs_para_1.Color_fy) * Z * rs_para_1.depth_scale
                    Zc = Z * rs_para_1.depth_scale

                    cv2.circle(im0, (u, v), 2, (0, 0, 255), 2)

                    center_point = [Xc, Yc, Zc]
                    print(f"Xc : {Xc}, Yc : {Yc}, Zc : {Zc}")
                    center_point_robot = handeye_func.eye2hand_func(center_point)
                    min_depth = vf_func.min_distance_robot_hand(center_point_robot)

                    # Display the minimum depth value alongside the bounding box
                    if min_depth > 0:
                        # Display the minimum depth value alongside the bounding box
                        text = f"Min Depth: {min_depth:.2f} mm"
                        cv2.putText(im0, text, (x1, y1 + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                else:
                    print("Invalid indices for depth_image")

            # Print time (inference + NMS)
            # print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')
            # print(f'({(1E3 * ( t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')
            # Stream results
        
        cv2.namedWindow("Recognition result", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Recognition result", im0)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            print("End of the streaming ...")
            pipeline.stop()
            break
    
    # Close the FPS file when done
    fps_file.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # parser.add_argument('--weights', nargs='+', type=str, default='best.pt', help='model.pt path(s)')
    parser.add_argument('--weights', nargs='+', type=str, default='hand_detect_1016.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='inference/images', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    # Old NMS threshold
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    # adjust for NMS threshold
    # parser.add_argument('--conf-thres', type=float, default=0.9, help='object confidence threshold')
    # parser.add_argument('--iou-thres', type=float, default=0.8, help='IOU threshold for NMS')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--no-trace', action='store_true', help='don`t trace model')
    opt = parser.parse_args()
    print(opt)
    #check_requirements(exclude=('pycocotools', 'thop'))

    rs_para_1 = rs_para()

    with torch.no_grad():
        if opt.update:  # update all models (to fix SourceChangeWarning)
            for opt.weights in ['yolov7.pt']:
                detect()
                strip_optimizer(opt.weights)
        else:
            detect()
