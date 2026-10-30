import math, time, cv2, mediapipe as mp, roslibpy

# Connect to rosbridge inside the container
ros = roslibpy.Ros(host='localhost', port=9090); ros.run()
arm = roslibpy.Topic(ros, '/crane_plus_arm_controller/joint_trajectory', 'trajectory_msgs/JointTrajectory')
grip = roslibpy.Topic(ros, '/crane_plus_gripper_controller/joint_trajectory', 'trajectory_msgs/JointTrajectory')

ARM_JOINTS = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
GRIP_OPEN, GRIP_CLOSE = -0.5, 0.0   # adjust if the gripper moves the wrong way

def clamp(v, lo, hi): return max(lo, min(hi, v))

def angle(a, b, c):  # angle at b (radians)
    v1 = (a.x-b.x, a.y-b.y); v2 = (c.x-b.x, c.y-b.y)
    d = math.hypot(*v1) * math.hypot(*v2) + 1e-6
    return math.acos(clamp((v1[0]*v2[0]+v1[1]*v2[1]) / d, -1, 1))

def send(topic, names, pos):
    topic.publish(roslibpy.Message({
        'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': ''},
        'joint_names': names,
        'points': [{'positions': pos, 'time_from_start': {'sec': 0, 'nanosec': 300_000_000}}]}))

pose = mp.solutions.pose.Pose(min_detection_confidence=0.5)
hands = mp.solutions.hands.Hands(max_num_hands=2, min_detection_confidence=0.5)
draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)
smooth = [0.0, 0.0, 0.0, 0.0]; last = 0

while cap.isOpened():
    ok, frame = cap.read()
    if not ok: break
    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pr, hr = pose.process(rgb), hands.process(rgb)

    if pr.pose_landmarks:
        lm = pr.pose_landmarks.landmark
        hip, sh, el, wr = lm[24], lm[12], lm[14], lm[16]   # right side
        visible = min(sh.visibility, el.visibility, wr.visibility) > 0.7
        if visible:
            upper = math.hypot(sh.x-el.x, sh.y-el.y) + 1e-6
            lift = clamp((sh.y - el.y) / upper, -1, 1)     # elbow height: -1 down, 0 horizontal, +1 up
            reach = clamp((el.x - sh.x) / upper, -1, 1)    # elbow sideways
            fore = math.hypot(el.x-wr.x, el.y-wr.y) + 1e-6
            fore_up = clamp((el.y - wr.y) / fore, -1, 1)   # +1 forearm up, 0 horizontal, -1 down
            target = [clamp(reach * 1.0, -1.2, 1.2),
                      clamp(-(lift + 1) * 0.6, -1.2, 0.0),        # shoulder follows elbow height
                      clamp(-(1 - fore_up) * 0.75, -1.5, 0.0),    # forearm up = robot straight
                      0.0]
            smooth = [0.85*s + 0.15*t for s, t in zip(smooth, target)]
        cv2.putText(frame, 'ARM OK' if visible else 'ARM LOST', (20,80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0) if visible else (0,0,255), 2)
        cv2.putText(frame, ' '.join(f'{v:+.2f}' for v in smooth), (20,120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
        draw.draw_landmarks(frame, pr.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)

    is_open = None
    if hr.multi_hand_landmarks:
        for lms, hd in zip(hr.multi_hand_landmarks, hr.multi_handedness):
            if hd.classification[0].label != 'Right':
                continue
            h = lms.landmark
            draw.draw_landmarks(frame, lms, mp.solutions.hands.HAND_CONNECTIONS)
            size = math.hypot(h[0].x-h[9].x, h[0].y-h[9].y) + 1e-6
            spread = sum(math.hypot(h[i].x-h[0].x, h[i].y-h[0].y) for i in (8,12,16,20)) / 4 / size
            is_open = spread > 1.4
            cv2.putText(frame, 'OPEN' if is_open else 'CLOSE', (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    if time.time() - last > 0.1:   # 10 Hz
        send(arm, ARM_JOINTS, smooth)
        if is_open is not None:
            send(grip, ['crane_plus_joint_hand'], [GRIP_OPEN if is_open else GRIP_CLOSE])
        last = time.time()

    cv2.imshow('arm_mimic (q to quit)', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break
    if key == ord('h'):                     # home position
        smooth = [0.0, 0.0, 0.0, 0.0]
        send(arm, ARM_JOINTS, smooth)
cap.release(); cv2.destroyAllWindows(); ros.terminate()
