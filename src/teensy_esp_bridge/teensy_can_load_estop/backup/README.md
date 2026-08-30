# Single-mailbox backup

MB8 하나만 사용하던 수정 전 파일의 백업이다.

- `teensy_can_load_estop_single_mailbox.ino.bak`
- `config_single_mailbox.h.bak`

Arduino IDE가 백업을 추가 스케치로 컴파일하지 않도록 `.bak` 확장자를 사용한다.
원래 버전으로 복원할 때는 두 파일을 상위 폴더의
`teensy_can_load_estop.ino`, `config.h`에 각각 복사한다.
