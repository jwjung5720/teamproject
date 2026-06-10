from ultralytics import YOLO
import os

# 현재 스크립트 위치 기준 상위 디렉토리의 data/data.yaml 참조
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(BASE_DIR, "data", "data.yaml")
MODEL_BASE = os.path.join(BASE_DIR, "models", "yolov8s-pose.pt")

if __name__ == "__main__":
    model = YOLO(MODEL_BASE)

    results = model.train(
        data=DATA,
        epochs=100,
        imgsz=640,
        device=0,
        project=os.path.join(BASE_DIR, "runs"),
        name="pallet_pose_s",
    )

    print("Training complete:", results.save_dir)
