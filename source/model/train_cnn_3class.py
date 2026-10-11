import os
import random
from collections import defaultdict

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Subset
from torchvision import datasets, transforms

# === 설정 ===
DATA_ROOT   = "./train"   # 이 폴더 안에 background / owner / stranger 폴더가 있다고 가정
IMG_SIZE    = 64
BATCH_SIZE  = 32
VAL_RATIO   = 0.2         # 각 클래스별로 20%를 검증용으로 사용
NUM_EPOCHS  = 20
LR          = 1e-3
SEED        = 42

random.seed(SEED)
torch.manual_seed(SEED)

# === Tiny CNN 정의 (3-class) ===
class TinyCNN(nn.Module):
    def __init__(self, num_classes: int = 3):
        super().__init__()
        # 입력: 1채널 그레이스케일, 64x64
        self.features = nn.Sequential(
            nn.Conv2d(1, 8, kernel_size=3, padding=1),   # 1x64x64 -> 8x64x64
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                             # 8x64x64 -> 8x32x32

            nn.Conv2d(8, 16, kernel_size=3, padding=1),  # 8x32x32 -> 16x32x32
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),                             # 16x32x32 -> 16x16x16
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),                                # 16*16*16 = 4096
            nn.Linear(16 * 16 * 16, 32),
            nn.ReLU(inplace=True),
            nn.Linear(32, num_classes)
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x

# === DataLoader 만들기 (클래스별로 train/val 나누기) ===
def build_dataloaders():
    # 공통 전처리: 그레이스케일 + 리사이즈 + 텐서 변환 + 정규화
    common_tf = transforms.Compose([
        transforms.Grayscale(num_output_channels=1),
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5], std=[0.5]),
    ])

    # ImageFolder: train/background, train/owner, train/stranger
    base_dataset = datasets.ImageFolder(DATA_ROOT, transform=common_tf)
    class_to_idx = base_dataset.class_to_idx
    idx_to_class = {v: k for k, v in class_to_idx.items()}

    print("class_to_idx:", class_to_idx)

    # 클래스별 인덱스 모으기
    idxs_by_class = defaultdict(list)
    for idx, (_, label) in enumerate(base_dataset.samples):
        idxs_by_class[label].append(idx)

    train_indices = []
    val_indices   = []

    print("\n=== Per-class sample split (train / val) ===")
    for cls_idx, idxs in idxs_by_class.items():
        cls_name = idx_to_class[cls_idx]
        n_total = len(idxs)
        if n_total == 0:
            print(f"  - {cls_name}: no samples, skipped")
            continue

        random.shuffle(idxs)
        n_val = max(1, int(n_total * VAL_RATIO)) if n_total > 1 else 1
        val_idx_cls   = idxs[:n_val]
        train_idx_cls = idxs[n_val:]

        train_indices.extend(train_idx_cls)
        val_indices.extend(val_idx_cls)

        print(f"  - {cls_name}: total={n_total}, train={len(train_idx_cls)}, val={len(val_idx_cls)}")

    train_dataset = Subset(base_dataset, train_indices)
    val_dataset   = Subset(base_dataset, val_indices)

    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True,  num_workers=0)
    val_loader   = DataLoader(val_dataset,   batch_size=BATCH_SIZE, shuffle=False, num_workers=0)

    print(f"\nTrain samples : {len(train_dataset)}")
    print(f"Val samples   : {len(val_dataset)}")

    return train_loader, val_loader, class_to_idx

# === 평가 함수 (전체 + 클래스별 정확도) ===
def evaluate(model, loader, device, num_classes, class_to_idx):
    model.eval()
    criterion = nn.CrossEntropyLoss()

    total_loss = 0.0
    total_correct = 0
    total_samples = 0

    # 클래스별 통계
    per_class_correct = [0] * num_classes
    per_class_total   = [0] * num_classes

    with torch.no_grad():
        for inputs, targets in loader:
            inputs  = inputs.to(device)
            targets = targets.to(device)

            outputs = model(inputs)
            loss = criterion(outputs, targets)

            total_loss += loss.item() * inputs.size(0)

            _, preds = torch.max(outputs, 1)
            total_correct += (preds == targets).sum().item()
            total_samples += inputs.size(0)

            for t, p in zip(targets, preds):
                per_class_total[t.item()] += 1
                if t == p:
                    per_class_correct[t.item()] += 1

    avg_loss = total_loss / total_samples if total_samples > 0 else 0.0
    avg_acc  = 100.0 * total_correct / total_samples if total_samples > 0 else 0.0

    # 클래스별 정확도 계산
    idx_to_class = {v: k for k, v in class_to_idx.items()}
    per_class_acc = {}
    for cls_idx in range(num_classes):
        name = idx_to_class[cls_idx]
        tot  = per_class_total[cls_idx]
        cor  = per_class_correct[cls_idx]
        acc  = 100.0 * cor / tot if tot > 0 else 0.0
        per_class_acc[name] = (cor, tot, acc)

    return avg_loss, avg_acc, per_class_acc

# === 학습 메인 ===
def train_model():
    train_loader, val_loader, class_to_idx = build_dataloaders()
    num_classes = len(class_to_idx)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Using device:", device)

    model = TinyCNN(num_classes=num_classes).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=LR)

    best_val_acc = 0.0
    best_epoch   = 0

    for epoch in range(1, NUM_EPOCHS + 1):
        # ---- Train ----
        model.train()
        running_loss = 0.0
        running_correct = 0
        total_samples = 0

        for inputs, targets in train_loader:
            inputs  = inputs.to(device)
            targets = targets.to(device)

            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()

            running_loss += loss.item() * inputs.size(0)
            _, preds = torch.max(outputs, 1)
            running_correct += (preds == targets).sum().item()
            total_samples += inputs.size(0)

        train_loss = running_loss / total_samples if total_samples > 0 else 0.0
        train_acc  = 100.0 * running_correct / total_samples if total_samples > 0 else 0.0

        # ---- Validation ----
        val_loss, val_acc, per_class_acc = evaluate(
            model, val_loader, device, num_classes, class_to_idx
        )

        print(f"[Epoch {epoch:02d}/{NUM_EPOCHS}] "
              f"Train Loss: {train_loss:.4f}, Train Acc: {train_acc:6.2f}% | "
              f"Val Loss: {val_loss:.4f}, Val Acc: {val_acc:6.2f}%")

        # 클래스별 출력
        for cls_name, (cor, tot, acc) in per_class_acc.items():
            print(f"   -> {cls_name:10s} Val Acc: {acc:6.2f}% ({cor}/{tot})")

        # 베스트 모델 저장
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_epoch = epoch
            ckpt = {
                "epoch": epoch,
                "model_state_dict": model.state_dict(),
                "class_to_idx": class_to_idx,
            }
            torch.save(ckpt, "tinycnn3_best.pth")
            print(f"   -> Best model updated (Val Acc={val_acc:.2f}%), saved to tinycnn3_best.pth\n")
        else:
            print()

    print("Training finished.")
    print(f"Best epoch: {best_epoch}, Best Val Acc: {best_val_acc:.2f}%")

if __name__ == "__main__":
    train_model()
