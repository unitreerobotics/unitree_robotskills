# Offline Validation and On-Site Boundaries

English | [中文](validation.zh.md)

For the offline verification commands see the [package usage guide](package_setup.md); all are run from the current deployment's package root. The release validation record is saved in [package_validation.json](package_validation.json); its results describe only the package that was verified and cannot replace actual verification in the target environment.

File verification covers the SHA256 of released files, JSON/Python/shell syntax, in-package documentation links and script paths. Offline tests cover plan generation, checkpoint submission, board marking and the error log; the environment check verifies the actual import locations of the in-package model and the installed SDK, the URDF meshes and the engine handshake. Board marking tests use synthetic images and do not prove accuracy on a real board.

Offline validation does not connect to DDS, call cameras, or start the robot service or motion. For differences between the operating rules and the source implementation see [runtime](runtime.md#implementation-verification-boundary). A new device must complete [first setup](first_game_setup.md), confirming the real cameras, waist hold, geometry and the current complete plan; passing offline checks cannot replace on-site validation.
