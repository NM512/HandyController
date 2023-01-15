import os
import cv2
import mediapipe as mp
import sys
import time
from multiprocessing import Process, Value, Array, parent_process
from HandyController.utils import Visualizer3D, map_3Dvec_on_plane, angle_between_2vectors
import numpy as np
import yaml

class HandPoseDetector:
    def __init__(self, render_img=False, render_3d=False):
        self.render_3d = render_3d
        if render_3d:
            self.visualizer3d = Visualizer3D()

        file_path = os.path.dirname(__file__)
        relative_path = os.path.join(file_path, './cfg/mediapipe_info.yaml')
        with open(relative_path, 'r') as yml:
            self.cfg = yaml.safe_load(yml)

        # use Array because Pipe (queue only for 2 processes) has delay.
        # MEMO: encounterd a problem of remaining process of Manager even after main process terminated
        self.detected_features = [Array('d', 3) for _ in range(self.cfg['mediapipe']['feature_num'])]
        self.hand_was_detected = Value('b', False)

        self.p = Process(target=self.__detect_hand, args=(self.detected_features,
                                                          self.hand_was_detected,
                                                          render_img))
        self.p.start()
        # use this because daemon option doesn't work with ESC in robogym env
        self.p_check = Process(target=self._kill_daemons)
        self.p_check.start()


    def __detect_hand(self, detected_features, hand_was_detected, render_img):
        cap = cv2.VideoCapture(-1)
        mpHands = mp.solutions.hands
        hands = mpHands.Hands()
        mpDraw = mp.solutions.drawing_utils
        ptime = 0

        while True:
            _, imgBGR = cap.read()
            imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
            result = hands.process(imgRGB)
            if result.multi_hand_landmarks:
                # convert a dict stype result to list of Array style
                for i in range(self.cfg['mediapipe']['feature_num']):
                    # TODO: add function to chose right or left hand
                    detected_features[i][0] = result.multi_hand_landmarks[0].landmark[i].x
                    detected_features[i][1] = result.multi_hand_landmarks[0].landmark[i].y
                    detected_features[i][2] = result.multi_hand_landmarks[0].landmark[i].z
                hand_was_detected.value = True

            # performance measurement
            ctime = time.time()
            fps = 1/(ctime-ptime)
            ptime = ctime

            if render_img:
                if result.multi_hand_landmarks:
                    for handLms in result.multi_hand_landmarks:
                        # Drawing gotten landmarks in the image
                        mpDraw.draw_landmarks(imgBGR, handLms, mpHands.HAND_CONNECTIONS)
                        for id, lm in enumerate(handLms.landmark):
                            h, w, c = imgBGR.shape
                            cx, cy = int(lm.x*w), int(lm.y*h)
                            if id in [4, 8, 12, 16, 20]:
                                cv2.circle(imgBGR, (cx, cy), 15, (255, 0, 255), cv2.FILLED)

                cv2.namedWindow('img', cv2.WINDOW_NORMAL)
                cv2.putText(imgBGR, 'FPS:{:.1f}'.format(fps), (18,50), cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0), 3)
                cv2.imshow("img", imgBGR)
                cv2.waitKey(1)

    def __recv_detection_result(self):
        # pause process if hand has not been detected.
        while not self.hand_was_detected.value:
            if not hasattr(self, 'last_notice_time'):
                self.last_notice_time = time.time()
                print('Hand has not been detected.')
            elif time.time() - self.last_notice_time > 5.0:
                print('Hand has not been detected.')
                self.last_notice_time = time.time()
        # convert to list after receiving
        ret = [list(feature) for feature in self.detected_features]
        return ret

    def __extract_joint_positions(self, result):
        joint_positions = {}
        for finger in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            l = {}
            for joint in ['first', 'second', 'third', 'fourth']:
                l[joint] = result[self.cfg['mediapipe'][finger][joint]['offset']]
            # add wrist coord at last
            l['wrist'] = result[self.cfg['mediapipe']['wrist']['offset']]
            joint_positions[finger] = l
        return joint_positions

    def __extract_joint_angles(self, joint_positions):
        angles = {}

        # used for mapping the points
        plane_vec1 = [joint_positions['thumb']['fourth'][i]
                      - joint_positions['thumb']['wrist'][i]
                      for i in range(3)]
        plane_vec2 = [joint_positions['pinky']['fourth'][i]
                      - joint_positions['pinky']['wrist'][i]
                      for i in range(3)]

        for finger in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            a = {}
            # Z axis joints
            vec1 = [joint_positions[finger]['fourth'][i]
                    - joint_positions[finger]['wrist'][i]
                    for i in range(3)]
            vec2 = [joint_positions[finger]['third'][i]
                    - joint_positions[finger]['fourth'][i]
                    for i in range(3)]
            mapped_vec1 = map_3Dvec_on_plane(plane_vec1, plane_vec2, vec1)
            mapped_vec2 = map_3Dvec_on_plane(plane_vec1, plane_vec2, vec2)
            a['z'] = angle_between_2vectors(mapped_vec1, mapped_vec2)

            # Y axis joints
            for j1, j2, j3 in [['first', 'second', 'third'],
                                ['second', 'third', 'fourth'],
                                ['third', 'fourth', 'wrist']]:
                # processing for x, y and z axis
                vec1 = [joint_positions[finger][j1][i] - joint_positions[finger][j2][i] for i in range(3)]
                vec2 = [joint_positions[finger][j2][i] - joint_positions[finger][j3][i] for i in range(3)]
                # rad is 0.0 ~ 3.141516
                a[j1] = angle_between_2vectors(vec1, vec2)
            angles[finger] = a
        return angles

    def detect(self):
        result = self.__recv_detection_result()
        joint_positions = self.__extract_joint_positions(result)
        joint_angles = self.__extract_joint_angles(joint_positions)
        if self.render_3d:
            self.visualizer3d.render(joint_positions)

        return joint_positions, joint_angles

    def _kill_daemons(self):
        while True:
            if not parent_process().is_alive():
                self.p.terminate()
                sys.exit(0)


if __name__ == "__main__":
    hand_det = HandPoseDetector(render_img=True, render_3d=True)

    while True:
        positions, angles = hand_det.detect()