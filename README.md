# WooDrive SDK (Arduino / Raspberry Pi)

BLDC 모터를 간단한 명령으로 제어할 수 있는 WooDrive SDK입니다.  
FOC 기반 제어를 지원하며, RS-485 통신으로 다양한 환경에서 사용할 수 있습니다.

\---

# 🚀 What you can do

* BLDC / PMSM 모터 제어
* 속도 / 위치 / 전류 제어
* FOC 기반 고성능 제어
* RS-485 다중 제어기 연결

👉 복잡한 제어 구현 없이 바로 사용 가능합니다.

\---

# 🖼️ System Overview

Arduino / Raspberry Pi → RS485 → WooDrive → Motor

\---

# 📦 Project Structure

WooDriveSdk/
├─ arduino/

│   ├─ WooDriveSdk.h

│   ├─ WooDriveSdk.cpp

│   └─ examples/

│       ├─ Example01\_BasicCheck/

│       ├─ Example02\_ReadStatus/

│       ├─ Example03\_AutoMotorSetup/

│       ├─ Example04\_Speed/

│       └─ Example05\_Position/

│
├─ raspberrypi/
│   ├─ Makefile
│   ├─ WooDriveSdk.h
│   ├─ WooDriveSdk.cpp
│   ├─ Example01\_BasicCheck.cpp
│   ├─ Example02\_ReadStatus.cpp
│   ├─ Example03\_AutoMotorSetup.cpp
│   ├─ Example04\_Speed.cpp
│   └─ Example05\_Position.cpp

\---

# 🔵 Arduino Usage

## 사용 방법

Arduino IDE에서:

arduino/examples/Example01\_BasicCheck/Example01\_BasicCheck.ino

👉 열어서 업로드

\---

# 🟢 Raspberry Pi Usage

## 빌드

cd raspberrypi
make

## 실행

./Example01\_BasicCheck

\---

## USB 포트 확인

ls /dev/ttyUSB\*
ls /dev/ttyACM\*

\---

## 권한 설정

sudo chmod 777 /dev/ttyUSB0

또는

sudo chmod 777 /dev/ttyACM0

\---

## 문제 해결

lsusb
dmesg | tail -n 30

\---

# 📌 Example Overview

* Example01\_BasicCheck : 연결 확인
* Example02\_ReadStatus : 상태 읽기
* Example03\_AutoMotorSetup : 자동 셋업
* Example04\_Speed : 속도 제어
* Example05\_Position : 위치 제어

\---

# 🎯 Quick Start

git clone https://github.com/woorobot-dev/WooDriveSdk.git
cd WooDriveSdk/raspberrypi
make
./Example01\_BasicCheck

