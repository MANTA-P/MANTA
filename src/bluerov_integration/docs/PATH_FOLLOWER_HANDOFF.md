# 경로 추종 인수인계

데이터 흐름: `/mission/target_position → team_min A* → /uuv/reference_path → PathFollower → tracking_target → team_byung PPID`

병 PPID 목표 입력 한 줄:

```cpp
controller_.setTargetPosition({tracking_target.message.point.x, tracking_target.message.point.y, tracking_target.message.point.z});
```

- 민 수정 위치: `src/team_min/` — A* 경로 생성
- 연결 수정 위치: `src/integration/path_follower.cpp` — 경로에서 추종점 선택
- 병 수정 위치: `src/team_byung/control_module.cpp` — 추종점 PPID 입력
- 핵심 설정: `config/integration.yaml`의 `path_following`
- 경로 없음: `stop_without_valid_path: true`이면 추력 0
- 병 단독 시험: `team_byung_control_test.launch.py`는 A* 없이 최종 목표를 PPID에 직접 입력
