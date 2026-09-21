# AI Worker 1044 SH5 Workspace

ROS 2 workspace snapshot for the AI Worker 1044 / SH5 integration, including
Gazebo and Isaac simulation support, Cyclo Intelligence, robot interfaces, and
runtime helpers.

Clone with the large upstream policy dependency:

```bash
git clone --recurse-submodules https://github.com/YWhero/ai_worker_1044_sh5_ws.git
```

The workspace's modified nested repositories are flattened into this repository
so the current working implementation is preserved. The unchanged
`Isaac-GR00T` dependency remains a Git submodule pinned to the revision used by
this snapshot.

Generated build/install/log trees, runtime caches, recorded datasets, and model
checkpoints are intentionally excluded.
