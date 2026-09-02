# sobits_vla_visualization

## Purpose

Reserved for future online VLA debug/visualization nodes (live camera
overlays, action-chunk previews, RViz markers) — it sits alongside the
deploy stage of the pipeline (rosbag → dataset → training → deploy →
eval) as an observability layer, not a pipeline stage itself. Currently an
empty, buildable skeleton: no nodes, no launch files.

Offline episode-log analysis (formerly `vla_eval` in this package) now
lives in `sobits_vla_deploy/eval/` — see that package's README.

## Nodes / executables

None yet. `sobits_vla_visualization/__init__.py` is the only module.

## Parameters

None yet.

## Topics / services

None yet. When a node lands here, it should follow the same owner-private
`~/<channel>` convention as every other package (see `sobits_vla_common`'s
README) and read robot I/O absolutely from the descriptor.

## Outputs

None yet.

## How to run

Nothing to run — the package builds and installs its lint gate only.

## How to test

```
cd /home/rg-station-04-keith/colcon_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select sobits_vla_visualization --test-result-base build/sobits_vla_visualization
colcon test-result --test-result-base build/sobits_vla_visualization

cd src/sobits_vla_tools
pixi run -e gpu python -m pytest sobits_vla_visualization/test/ -q --ignore-glob='*test_flake8.py' --ignore-glob='*test_pep257.py' --ignore-glob='*test_copyright.py'
```

`test_flake8.py` / `test_pep257.py` / `test_copyright.py` — the lint gate is
in place so the first real node has tests to land alongside from day one.
