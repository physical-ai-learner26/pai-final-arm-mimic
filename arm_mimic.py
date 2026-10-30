import math, time, cv2, mediapipe as mp, roslibpy

# コンテナ内の rosbridge に接続
ros = roslibpy.Ros(host='localhost', port=9090); ros.run()
arm = roslibpy.Topic(ros, '/crane_plus_arm_controller/joint_trajectory', 'trajectory_msgs/JointTrajectory')
grip = roslibpy.Topic(ros, '/crane_plus_gripper_controller/joint_trajectory', 'trajectory_msgs/JointTrajectory')

ARM_JOINTS = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint3', 'crane_plus_joint4']
YAW_SIGN = -1   # 左右が逆なら -1 にする
PITCH_SIGN = 1   # 前後が逆なら -1 にする
GRIP_OPEN, GRIP_CLOSE = -0.5, 0.3   # グリッパーの開閉が逆なら値を調整する
SIDE = 'L'   # 'L' = 左腕, 'R' = 右腕
IDX = (24, 12, 14, 16) if SIDE == 'L' else (23, 11, 13, 15)   # 腰, 肩, 肘, 手首
HAND_LABEL = 'Left' if SIDE == 'L' else 'Right'

def clamp(v, lo, hi): return max(lo, min(hi, v))

def angle(a, b, c):  # b における角度（ラジアン）
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
BODY_CONN = [c for c in mp.solutions.pose.POSE_CONNECTIONS if c[0] < 17 and c[1] < 17]
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
        hip, sh, el, wr = (lm[i] for i in IDX)
        visible = min(sh.visibility, el.visibility, wr.visibility) > 0.7
        if visible:
            w = pr.pose_world_landmarks.landmark
            ws, ww = w[IDX[1]], w[IDX[3]]          # 肩, 手首（メートル単位）
            dx = ww.x - ws.x                       # + = 自分から見て右
            dz = ws.z - ww.z                       # + = 手首を奥へ押し出している
            dy = ws.y - ww.y                       # + = 手首が肩より上
            yaw = YAW_SIGN * math.atan2(wr.x - sh.x, 0.25)   # 画面上の手首の左右
            elev = math.atan2(dy, math.hypot(dx, dz) + 1e-6)   # +pi/2 = 真上, 0 = 水平, -pi/2 = 真下
            shoulder = math.pi/2 - elev                          # 0 = 真上, pi/2 = 水平
            if dz < 0: shoulder = -shoulder
            bend = math.pi - angle(sh, el, wr)     # 0 = まっすぐ, 大きいほど曲がっている
            wrist = 0.0
            if hr.multi_hand_landmarks:
                for lms, hd in zip(hr.multi_hand_landmarks, hr.multi_handedness):
                    if hd.classification[0].label == HAND_LABEL:
                        wrist = math.pi - angle(el, wr, lms.landmark[9])
            target = [clamp(yaw, -1.57, 1.57),
                      clamp(PITCH_SIGN * -shoulder, -1.57, 1.57),   # 肩の前後の傾き
                      clamp(-bend, -1.57, 0.0),               # 肘を自分の肘に合わせる
                      clamp(-wrist, -1.57, 1.57)]             # 手首を自分の手首に合わせる
            smooth = [s + (t - s) * min(1.0, 0.25 + abs(t - s) * 2.0)
                      for s, t in zip(smooth, target)]
        cv2.putText(frame, 'ARM OK' if visible else 'ARM LOST', (20,80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0) if visible else (0,0,255), 2)
        cv2.putText(frame, ' '.join(f'{v:+.2f}' for v in smooth), (20,120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
        draw.draw_landmarks(frame, pr.pose_landmarks, BODY_CONN)

    is_open = None
    if hr.multi_hand_landmarks:
        for lms, hd in zip(hr.multi_hand_landmarks, hr.multi_handedness):
            if hd.classification[0].label != HAND_LABEL:
                continue
            h = lms.landmark
            draw.draw_landmarks(frame, lms, mp.solutions.hands.HAND_CONNECTIONS)
            size = math.hypot(h[0].x-h[9].x, h[0].y-h[9].y) + 1e-6
            spread = sum(math.hypot(h[i].x-h[0].x, h[i].y-h[0].y) for i in (8,12,16,20)) / 4 / size
            is_open = spread > 1.4
            cv2.putText(frame, 'OPEN' if is_open else 'CLOSE', (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    if time.time() - last > 0.1:   # 10 Hz で送信
        send(arm, ARM_JOINTS, smooth)
        if is_open is not None:
            send(grip, ['crane_plus_joint_hand'], [GRIP_OPEN if is_open else GRIP_CLOSE])
        last = time.time()

    cv2.imshow('arm_mimic (q to quit)', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break
    if key == ord('h'):                     # ホーム姿勢に戻す
        smooth = [0.0, 0.0, 0.0, 0.0]
        send(arm, ARM_JOINTS, smooth)
cap.release(); cv2.destroyAllWindows(); ros.terminate()
