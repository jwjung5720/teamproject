- 목표
  팔레트 keypoint 6개를 검출하는 YOLOv8-pose 커스텀 모델 학습

----------------------------------------------------
- 데이터셋 (Roboflow 수출, YOLOv8-pose 형식)

  구성     : train 162장 / valid 20장 / test 21장
  kpt_shape: [6, 3]  →  keypoint 6개, 각 (x, y, visible)
  클래스   : pallet 1개

  keypoint 배치
    kp0 : 팔레트 왼쪽 위 모서리
    kp1 : 팔레트 오른쪽 위 모서리
    kp2 : 팔레트 오른쪽 아래 모서리
    kp3 : 팔레트 왼쪽 아래 모서리
    kp4 : 왼쪽 삽입구 기준점
    kp5 : 오른쪽 삽입구 기준점

- 진행
----------------------------------------------------
  [v1] yolov8n-pose / pose_loss=12 / flip_idx=[0,1,2,3,4,5]
       Box  mAP50 = 0.971   Pose mAP50 = 0.447
       → 박스 검출은 우수, keypoint 인식률 낮음

  [v2] yolov8n-pose / pose_loss=24 (가중치 2배)
       Box  mAP50 = 0.992   Pose mAP50 = 0.003
       → pose loss 과도 → keypoint 학습 붕괴. 폐기

- flip_idx가 잘못됬다고함
    수정 전 : [-1, 1, 2, 3, 4, 5] 
    수정 후 : [1, 0, 3, 2, 5, 4]

  [v3] yolov8s-pose / pose_loss=12 / flip_idx=[1,0,3,2,5,4]
       Box  mAP50 = 0.991   Pose mAP50 = 0.991  Pose mAP50-95 = 0.963

====================================================
