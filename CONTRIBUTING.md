# Contributing to sobits_vla_tools

Working rules for anyone adding code to this repo. Every PR is expected to
follow these; reviewers will send you back here.

## Node files are wiring only

A `*_node.py` / `*.cpp` node owns: parameter reading, pub/sub/timer/service
creation, and callback dispatch. Every non-trivial computation lives in a
plain, dependency-free module that takes values and returns values.

Test: *can this logic be unit-tested without `rclpy.init()` (or, in C++,
without constructing an `rclcpp::Node`)?* If no, extract it.

## Size caps

No function over 80 lines, no file over 600. This is a hard cap, not a
guideline — exceeding it blocks review. Escape hatch: a
`# refactor-exempt: <reason>` comment on the def line, granted in review, so
a justified exception is possible but always visible and greppable.

## Shared code lives in `sobits_vla_common`

Anything used by 2+ packages belongs in `sobits_vla_common`, not
copy-pasted. `sobits_vla_common` itself must never import from a sibling
package — it is the bottom of the dependency graph, everything else depends
on it.

## Parameters: schema, not hand-declared

New ROS parameters go through `sobits_vla_common.param_schema`:
`declare_from_schema(node, SCHEMA)` / `read_schema(node, SCHEMA)` instead of
hand-written `declare_parameter` calls. `validate_config(SCHEMA, yaml_path)`
sweeps a node's config YAML against its schema — wire it into that package's
tests as a gate against unknown/misspelled keys (see
`sobits_vla_common/test/test_config_schemas.py` for the pattern). A static
schema can't express everything: a *templated section* (a subtree declared
once per item of a list-valued parameter, e.g. one block per robot joint
group) is the escape hatch for genuinely dynamic parameter trees; anything
that still doesn't fit stays hand-declared with a `# schema-exempt` comment.

## Comments: 2 lines max, why only

A comment is at most 2 lines and explains *why*, never *what*. If a comment
merely restates the code, the correct comment is no comment — delete it,
don't compress it.

## Naming

- Python: `snake_case` everywhere; private helpers `_prefixed`.
- C++: `snake_case` members with a trailing underscore; methods stay
  `camelCase` (e.g. `createRosbag()`) — consistent within the C++ packages,
  don't churn it.
- ROS params: `dot.separated.lowercase`, grouped by subsystem (`model.*`,
  `runtime.*`, `logging.*`).

## The lerobot seam

`sobits_vla_common/lerobot_adapter.py` is the *only* module allowed to
import from `lerobot` internals. Every other module — across every package
— imports from the adapter, never from `lerobot` directly. This is what
makes a lerobot version bump a one-file change instead of a repo-wide hunt.

## Namespace model

Every topic/service a node advertises or publishes uses a ROS private name,
`~/<channel>` — it resolves to `/<node_name>/<channel>` bare or
`/<robot_name>/<node_name>/<channel>` under a namespaced launch. Consumers
address it by the owner's relative name, `<owner_node>/<channel>`, which
resolves alongside it under the same namespace. Exceptions that stay
absolute: robot I/O topics from the robot descriptor, external standard
interfaces (`/joy`, `/tf`, controller topics, Gazebo topics), and the
parameter services ROS creates for you. Never introduce a new absolute or
ownerless shared topic/service prefix.

## Output roots

A package that writes generated artifacts (datasets, checkpoints, logs,
rosbags) resolves its output directory with
`sobits_vla_common.output_root.output_root(package, dirname)`, not a
hand-rolled path-walk. The artifact directory itself carries a `.gitignore`
of exactly `*\n!.gitignore\n` — tracked directory, untracked contents.

## Commits

One commit per discrete fix or implementation step — no batching unrelated
changes. Conventional-commit style matching the existing history
(`fix(deploy): ...`, `feat(training): ...`).

## Running the test suites

Inside the development container, from the workspace root:

```sh
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --packages-select <pkg> --allow-overriding <pkg>
colcon test --packages-select <pkg> --test-result-base build/<pkg>
colcon test-result --test-result-base build/<pkg>
```

Non-ROS Python tests (numpy/pandas/torch/lerobot-dependent) run through
`pixi`, from the repo root inside `sobits_vla_tools/`:

```sh
pixi run -e gpu python -m pytest <pkg>/test/ -q \
  --ignore-glob='*test_flake8.py' --ignore-glob='*test_pep257.py' --ignore-glob='*test_copyright.py'
```

(the ignored files run only under `ament_flake8`/`ament_pep257`, which the
pixi env doesn't provide — they're covered by `colcon test` instead.) A pure
C++ package (`sobits_vla_rosbag_collection`) has no pixi step; its gtest
suites run entirely through `colcon test`.

Each package's own README has the exact command for its test suite and a
short description of what's covered.
