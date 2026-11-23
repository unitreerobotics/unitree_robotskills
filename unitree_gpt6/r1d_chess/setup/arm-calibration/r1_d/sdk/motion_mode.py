"""Validate/optionally release the vendor controller before taking LowCmd control."""

import json
import time


def prepare(client, release=False):
    for _ in range(3):
        code, state = client.CheckMode()
        if code != 3104:  # the read-only query sometimes times out on the robot; ask again
            break
        time.sleep(1.0)
    if code or not isinstance(state, dict) or not isinstance(state.get("name"), str):
        if code == 3102:
            raise RuntimeError(
                "MotionSwitcher 3102：DDS 请求发送失败，通常是未发现服务订阅端；请检查 Wi-Fi DDS 发现和机器人回包路由"
            )
        raise RuntimeError(f"MotionSwitcher 查询失败: {code}, {state}")
    print("MotionSwitcher:", json.dumps(state), flush=True)
    if not state["name"]:
        return
    if not release:
        raise RuntimeError(
            "厂商运控仍在运行；准备好低层接管后使用 --release-mode，或通过厂商工具释放运控"
        )
    if state["name"] not in ("ai", "normal"):
        raise RuntimeError(f"拒绝自动释放未知服务 {state['name']!r}；请先通过厂商工具准备低层控制")
    for _ in range(5):
        code, _ = client.ReleaseMode()
        if code not in (0, 7002):
            raise RuntimeError(f"释放运控失败: {code}")
        time.sleep(1.0)
        code, state = client.CheckMode()
        if code == 0 and isinstance(state, dict) and state.get("name") == "":
            return
    raise RuntimeError("未确认运控已释放，禁止启动动作")
