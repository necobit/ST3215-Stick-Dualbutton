# ST3215-Stick-Dualbutton

M5StampS3 + M5Unit-Joystick2 で STS3215 サーボモーター 2 台と PWM サーボを制御するプロジェクト。

## 概要

- ジョイスティックの X/Y 軸で 2 台の STS3215 を連続回転制御
- 2 つのボタンで PWM サーボの角度を切り替え
- ジョイスティックは傾きが大きい軸のみ有効（X/Y 同時操作防止）

## ハードウェア構成

### マイコン

- **M5StampS3** (ESP32-S3)

### サーボモーター

- **Feetech STS3215** x 3 台
  - ID 1, 2, 3
  - TTL シリアル通信（半二重）1Mbps
  - ホイールモード（連続回転）

- **PWM サーボ** x 1 台
  - 標準的な RC サーボ（50Hz PWM）

### 入力デバイス

- **M5Unit-Joystick2** (I2C: 0x63)
- **ボタン** x 2（プルアップ、押下で LOW）

### ピン配置

| 機能 | GPIO |
|------|------|
| STS3215 TX | 15 |
| STS3215 RX | 13 |
| I2C SDA (Joystick) | 2 |
| I2C SCL (Joystick) | 1 |
| PWM サーボ | 10 |
| ボタン 3 | 3 |
| ボタン 4 | 4 |

## 動作仕様

### ジョイスティック制御

| 軸 | 動作 |
|----|------|
| X 軸（左右） | サーボ 1 正転/逆転 |
| Y 軸（上下） | サーボ 2 正転/逆転 |

- センターからのオフセットが大きい軸のみ有効
- デッドゾーン: ±3000（16bit ADC）
- 傾きに応じて速度変化（最大 1500）

### ボタン制御

| ボタン | PWM サーボ角度 | サーボ 1 |
|--------|----------------|----------|
| なし | 90° | ジョイスティック制御 |
| ボタン 3 | 45° | 停止 |
| ボタン 4 | 135° | 停止 |

### ジョイスティックボタン（サーボ 3）

ボタン押下で以下のシーケンスを実行：

1. 500ms 正回転
2. 500ms 待機
3. 500ms 逆回転（元の位置に戻る）

## 使用ライブラリ

- [FTServo](https://github.com/ftservo/FTServo_Arduino) - Feetech バスサーボライブラリ
- [M5Unit-Joystick2](https://github.com/m5stack/M5Unit-Joystick2) - M5Stack ジョイスティックユニット

## ビルド・書き込み

```bash
# ビルド
pio run

# 書き込み
pio run --target upload

# シリアルモニタ
pio device monitor

# ビルド＆書き込み＆モニタ
pio run --target upload && pio device monitor
```
