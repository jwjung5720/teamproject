import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import threading
import os
import cv2

# 현재 스크립트 위치 기준 상위 디렉토리의 models/best.pt 참조
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(BASE_DIR, "models", "best.pt")

model = None

def load_model():
    global model
    from ultralytics import YOLO
    if not os.path.exists(MODEL_PATH):
        messagebox.showerror("오류", f"모델 파일을 찾을 수 없습니다: {MODEL_PATH}")
        return
    model = YOLO(MODEL_PATH)

def select_and_infer():
    path = filedialog.askopenfilename(
        title="이미지 선택",
        filetypes=[("이미지 파일", "*.jpg *.jpeg *.png *.bmp *.webp"), ("모든 파일", "*.*")],
    )
    if not path:
        return

    btn.config(state="disabled", text="추론 중...")
    status_var.set("추론 중...")

    def run():
        try:
            results = model(path, verbose=False)
            result_img = results[0].plot()  # numpy BGR array

            # BGR → RGB
            rgb = cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb)

            # 화면 크기에 맞게 축소
            max_w, max_h = 900, 650
            pil_img.thumbnail((max_w, max_h), Image.LANCZOS)
            tk_img = ImageTk.PhotoImage(pil_img)

            # 키포인트 정보 수집
            kpts = results[0].keypoints
            boxes = results[0].boxes
            n = len(boxes) if boxes is not None else 0

            info_lines = [f"감지된 팔레트: {n}개"]
            if kpts is not None and kpts.xy is not None:
                for i, kp in enumerate(kpts.xy):
                    conf_list = kpts.conf[i] if kpts.conf is not None else ["?"] * len(kp)
                    info_lines.append(f"\n[팔레트 {i+1}]")
                    for j, (xy, cv) in enumerate(zip(kp, conf_list)):
                        info_lines.append(f"  kp{j}: ({xy[0]:.0f}, {xy[1]:.0f})  conf={float(cv):.2f}")

            root.after(0, lambda: update_ui(tk_img, "\n".join(info_lines)))
        except Exception as e:
            root.after(0, lambda: on_error(str(e)))

    threading.Thread(target=run, daemon=True).start()

def update_ui(tk_img, info_text):
    canvas.image = tk_img
    canvas.config(width=tk_img.width(), height=tk_img.height())
    canvas.delete("all")
    canvas.create_image(0, 0, anchor="nw", image=tk_img)
    info_var.set(info_text)
    status_var.set("완료")
    btn.config(state="normal", text="이미지 선택")

def on_error(msg):
    messagebox.showerror("오류", msg)
    status_var.set("오류 발생")
    btn.config(state="normal", text="이미지 선택")

# ── UI 구성 ──────────────────────────────────────
root = tk.Tk()
root.title("팔레트 Pose 추론")
root.resizable(True, True)

top = tk.Frame(root)
top.pack(fill="x", padx=10, pady=8)

btn = tk.Button(top, text="이미지 선택", font=("맑은 고딕", 12),
                command=select_and_infer, padx=12, pady=6)
btn.pack(side="left")

status_var = tk.StringVar(value="모델 로딩 중...")
tk.Label(top, textvariable=status_var, font=("맑은 고딕", 10), fg="gray").pack(side="left", padx=12)

main = tk.Frame(root)
main.pack(fill="both", expand=True, padx=10, pady=(0, 10))

canvas_frame = tk.Frame(main, bg="#222")
canvas_frame.pack(side="left", fill="both", expand=True)

canvas = tk.Canvas(canvas_frame, bg="#222", width=640, height=480)
canvas.pack(fill="both", expand=True)

info_frame = tk.Frame(main, width=220)
info_frame.pack(side="right", fill="y", padx=(8, 0))
info_frame.pack_propagate(False)

tk.Label(info_frame, text="키포인트 정보", font=("맑은 고딕", 11, "bold")).pack(pady=(0, 4))
info_var = tk.StringVar(value="이미지를 선택하세요")
tk.Label(info_frame, textvariable=info_var, font=("맑은 고딕", 9),
         justify="left", anchor="nw", wraplength=210).pack(fill="both", expand=True)

# 모델을 백그라운드에서 미리 로딩
def init_model():
    load_model()
    root.after(0, lambda: status_var.set("준비 완료 — 이미지를 선택하세요"))

threading.Thread(target=init_model, daemon=True).start()

root.mainloop()
