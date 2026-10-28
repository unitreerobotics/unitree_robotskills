# 选招与机器人可达性

[English](move_constraints.md) | 中文

现行规则：不按横跨距离或目标列预先排除棋规合法着法。右手到 a-b、左手到 h-i 均可成为候选；跨盘并不等于实机可达。`scripts/move_constraints.py`保留全部输入的合法着法，标记`policy_id=reachability_gated_cross_board_v2`；实际能否执行由本轮完整路径预检及明确的实体邻子干涉风险判断共同决定，不能把二维遮挡当成已碰撞。

## 规划顺序

1. 从当前确认的棋局获取完整合法着法；被将军时仅考虑合法应将。预测命中的下一步同样不因跨侧边缘而作废。
2. 按 source 列选择首选执行手：a-d 优先左手，e-i 优先右手。用本轮新图和首选手实测起点分别估计 source、target 的 XYZ/RPY；两阶段各自先考虑偏横向。横向抓取可行不要求横向释放；若某阶段横向不可达，或有明确推挤、夹入邻子风险且无法在横向候选消除，该阶段再试偏纵向。抓取时的遮挡、二维投影重叠或接近本身不等于碰撞，不单独作为拒抓依据。
3. source 与 target 的 RPY 分别确定；换释放 RPY 后重新估计 target EE XYZ，不沿用抓取姿态的 EE 坐标，在净空高位转姿并重新 build。完整预检抓取、重试、持物转姿及搬运、释放、上撤和回 init；吃子预检整个联合 sequence。首选手候选均不能通过完整预检或有明确实际干涉风险时，再从另一手实测起点重新估计、读取该手误差参考并完整预检。
4. 两手均无通过完整预检且无明确实际干涉风险的候选时，才放弃该着法，继续搜索下一合法候选。不得用单点 IK、旧坐标或放宽场景保护代替完整预检。

在包根激活已安装依赖的Python环境，设置`CHESS_PYTHON="$(command -v python)"`（见[首次配置](first_game_setup.zh.md)）。搜索辅助脚本接受当前棋规引擎返回的合法 UCI 列表：

```bash
"$CHESS_PYTHON" scripts/move_constraints.py --legal-moves <本轮run>/tmp/legal_moves.json
```

输出包含`policy_id`、`allowed_legal_moves`、`excluded_moves`和`go_command`。除无合法着法外，`searchmoves`包含全部合法候选；脚本不调用引擎或机器人，也不保证动作可达。每轮规划仍需核验引擎`bestmove`属于当前合法列表、按真实图像估计抓放位姿，并运行服务完整预检。预测主变中的未来着法只是预测，不提前按横跨幅度截断；到对应回合再根据实况做可达性预检。
