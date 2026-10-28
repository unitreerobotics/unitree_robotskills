# 抓取判据入口

[English](tip_grasp_policy.md) | 中文

以[round_plan](../prompts/round_plan.zh.md)第3步为准：左右在两指相对纹理面间、前后在前端有效纹理段，能夹持即可，不要求完全居中。低位XY不足retry_high；超过半枚在纹理外但XY可夹、仅高度覆盖不足时考虑adjust_z 5mm。
