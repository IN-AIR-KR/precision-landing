# Jetson XRCE Agent 서비스

기존 실기체 launch나 수동 Agent를 종료해 UART 포트를 비운 뒤,
Jetson 로그인 계정에서 프로젝트 루트를 기준으로 실행합니다.

```bash
bash scripts/xrce_agent/install.sh
systemctl status precision-xrce-agent.service
journalctl -u precision-xrce-agent.service -f
```

설치 스크립트는 sudo로 서비스를 등록하고 즉시 시작합니다. Jetson이 부팅될
때도 자동 시작합니다. UART 경로, baud와 ROS 도메인은 소스 트리의
`src/pl_bringup/config/real.yaml`에서 읽습니다. 이 값을 바꾸면
`sudo systemctl restart precision-xrce-agent.service`로 적용합니다.

Agent가 시작된 상태에서 Pixhawk를 켜고 저널에서 `session established`를
확인한 뒤 `bash scripts/run_real.sh`로 ROS 노드를 실행합니다. ROS launch를
종료해도 Agent는 유지됩니다. 최초 XRCE 연결이 간헐적으로 실패하는 경우
Agent 저널과 PX4 DDS 클라이언트 상태를 확인합니다.
