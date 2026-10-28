# Grasp Criterion Entry Point

English | [中文](tip_grasp_policy.zh.md)

Step 3 of [round_plan](../prompts/round_plan.md) is authoritative: left-right, the piece is between the two fingers' facing textured surfaces; front-back, it is within the front effective textured segment; it only needs to be graspable, and full centring is not required. If the low-position XY is insufficient, retry_high; if more than half of the piece is outside the texture but the XY is graspable and only the height coverage is insufficient, consider adjust_z 5mm.
