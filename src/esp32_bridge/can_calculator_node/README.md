# CAN calculator node

ESP32-S3와 ESP-IDF 6.1용 CAN 계산 노드 펌웨어이다. `0x002` ADD 요청과 `0x003`
SUB 요청을 검증·계산하고 `0x001` 결과 프레임을 전송한다. wire format은 저장소
루트의 `CAN_CALCULATOR_PROTOCOL.md`를 따른다.

배선, GPIO 설정, 빌드 및 플래시 방법은 [HARDWARE_AND_FLASH.md](./HARDWARE_AND_FLASH.md)를
참조한다.
