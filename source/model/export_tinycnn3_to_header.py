# export_tinycnn3_to_header.py
import torch
import numpy as np
from train_cnn_3class import TinyCNN  # 같은 폴더에 있다고 가정

CKPT_PATH = "tinycnn3_best.pth"
OUT_HEADER = "tinycnn3_weights.h"

def tensor_to_c_array(f, name, t: torch.Tensor):
    """
    텐서를 1차원 C float 배열로 저장
    """
    arr = t.detach().cpu().numpy().astype(np.float32).ravel()
    f.write(f"static const float {name}[{arr.size}] = {{\n")
    # 보기 편하게 8개씩 줄바꿈
    line = []
    for i, v in enumerate(arr):
        line.append(f"{v:.6f}")
        if (i + 1) % 8 == 0:
            f.write("    " + ", ".join(line) + ",\n")
            line = []
    if line:
        f.write("    " + ", ".join(line) + ",\n")
    f.write("};\n\n")


def main():
    print(f"Loading checkpoint from: {CKPT_PATH}")
    ckpt = torch.load(CKPT_PATH, map_location="cpu")

    state = ckpt["model_state_dict"]
    class_to_idx = ckpt["class_to_idx"]
    num_classes = len(class_to_idx)

    print("class_to_idx:", class_to_idx)

    # 모델 구조는 train_cnn_3class.py의 TinyCNN과 동일해야 함
    model = TinyCNN(num_classes=num_classes)
    model.load_state_dict(state)
    model.eval()

    # state_dict에서 각 레이어 weight/bias 가져오기
    sd = model.state_dict()

    # 키 이름은 train_cnn_3class.py에서 정의한 순서와 일치해야 함
    # conv1, conv2, fc1, fc2 정도일 것
    conv1_w = sd["features.0.weight"]  # [out_c, in_c, kH, kW]
    conv1_b = sd["features.0.bias"]    # [out_c]
    conv2_w = sd["features.3.weight"]
    conv2_b = sd["features.3.bias"]
    fc1_w   = sd["classifier.1.weight"]
    fc1_b   = sd["classifier.1.bias"]
    fc2_w   = sd["classifier.3.weight"]
    fc2_b   = sd["classifier.3.bias"]

    # shape 정보
    C1_O, C1_I, K1H, K1W = conv1_w.shape
    C2_O, C2_I, K2H, K2W = conv2_w.shape
    FC1_O, FC1_I = fc1_w.shape
    FC2_O, FC2_I = fc2_w.shape

    print("conv1 weight shape:", conv1_w.shape)
    print("conv2 weight shape:", conv2_w.shape)
    print("fc1 weight shape  :", fc1_w.shape)
    print("fc2 weight shape  :", fc2_w.shape)

    with open(OUT_HEADER, "w", encoding="utf-8") as f:
        f.write("/* Auto-generated TinyCNN weights for Zynq C inference */\n")
        f.write("/* Do NOT edit manually. */\n\n")
        f.write("#pragma once\n\n")
        f.write("#include <stdint.h>\n\n")

        # 클래스 인덱스 매핑을 주석으로 남겨 놓으면 편함
        f.write("/* class_to_idx mapping:\n")
        for cls, idx in class_to_idx.items():
            f.write(f" *   {cls} -> {idx}\n")
        f.write(" */\n\n")

        # 매크로로 shape 정의
        f.write(f"#define CONV1_OUT_CH {C1_O}\n")
        f.write(f"#define CONV1_IN_CH  {C1_I}\n")
        f.write(f"#define CONV1_KH     {K1H}\n")
        f.write(f"#define CONV1_KW     {K1W}\n\n")

        f.write(f"#define CONV2_OUT_CH {C2_O}\n")
        f.write(f"#define CONV2_IN_CH  {C2_I}\n")
        f.write(f"#define CONV2_KH     {K2H}\n")
        f.write(f"#define CONV2_KW     {K2W}\n\n")

        f.write(f"#define FC1_OUT      {FC1_O}\n")
        f.write(f"#define FC1_IN       {FC1_I}\n")
        f.write(f"#define FC2_OUT      {FC2_O}\n")
        f.write(f"#define FC2_IN       {FC2_I}\n\n")

        # 실제 weight / bias 배열 찍기
        tensor_to_c_array(f, "CONV1_WEIGHT", conv1_w)
        tensor_to_c_array(f, "CONV1_BIAS",   conv1_b)
        tensor_to_c_array(f, "CONV2_WEIGHT", conv2_w)
        tensor_to_c_array(f, "CONV2_BIAS",   conv2_b)
        tensor_to_c_array(f, "FC1_WEIGHT",   fc1_w)
        tensor_to_c_array(f, "FC1_BIAS",     fc1_b)
        tensor_to_c_array(f, "FC2_WEIGHT",   fc2_w)
        tensor_to_c_array(f, "FC2_BIAS",     fc2_b)

    print(f"Export done. Header saved to: {OUT_HEADER}")


if __name__ == "__main__":
    main()
