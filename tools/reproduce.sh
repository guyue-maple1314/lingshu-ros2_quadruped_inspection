#!/usr/bin/env bash
# 纯逻辑层复现脚本（无需 ROS 2）：Python 单元测试 + C++ 冒烟测试
# 用法：bash tools/reproduce.sh
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/ros2_ws/src"
PY_SRC="$SRC/inspection_planning_py"
TESTS="$SRC/inspection_tests"
CPP_ROOT="$SRC/inspection_execution_cpp"
WORK="$(mktemp -d)"
failed=0

echo
echo "=== 1/2 Python 单元测试（unittest，无需 ROS 2）==="
if command -v python3 >/dev/null 2>&1; then PY=python3; else PY=python; fi
(
  cd "$SRC" || exit 1
  PYTHONPATH="${PY_SRC}${PYTHONPATH:+:$PYTHONPATH}" \
    "$PY" -m unittest discover -s inspection_tests/unit_python -t inspection_tests
)
if [ $? -ne 0 ]; then failed=$((failed + 1)); fi

echo
echo "=== 2/2 C++ 冒烟测试（脱离 ROS 编译并运行）==="
COMPILER="${CXX:-}"
if [ -z "$COMPILER" ]; then
  for c in clang++ g++; do
    if command -v "$c" >/dev/null 2>&1; then COMPILER="$c"; break; fi
  done
fi
if [ -z "$COMPILER" ]; then
  echo "未找到 clang++ / g++，跳过 C++ 冒烟测试"
  failed=$((failed + 1))
else
  echo "使用编译器：$COMPILER"
  SOURCES="$(find "$CPP_ROOT/src" -name '*.cpp' \
      ! -name '*_node*.cpp' ! -name 'bt_task_executor.cpp' \
      ! -name 'task_lifecycle_executor.cpp' | sort)"
  COUNT="$(printf '%s\n' "$SOURCES" | wc -l | tr -d ' ')"
  echo "组件源文件数：$COUNT（另加 smoke_test_core.cpp）"
  if [ "$COUNT" -ne 38 ]; then
    echo "提示：预期 38 个组件源文件，实际 $COUNT 个，请确认是否新增了节点文件"
  fi
  # shellcheck disable=SC2086  # 源码路径不含空格，按空白分词传给编译器
  "$COMPILER" -std=c++17 -Wall -Wextra -Wpedantic \
    -I "$CPP_ROOT/include" \
    "$TESTS/unit_cpp/tech_1_1_to_1_9/smoke_test_core.cpp" \
    $SOURCES -o "$WORK/smoke_test_core"
  if [ $? -ne 0 ]; then
    failed=$((failed + 1))
  else
    "$WORK/smoke_test_core"
    if [ $? -ne 0 ]; then failed=$((failed + 1)); fi
  fi
fi

rm -rf "$WORK"
echo
if [ "$failed" -eq 0 ]; then
  echo "全部通过：Python 单元测试 + C++ 纯逻辑冒烟测试"
  echo "下一步（需 Ubuntu + ROS 2 Humble）：见 README「复现步骤」第 3 节"
else
  echo "有 $failed 项未通过，请对照 README「运行环境」节检查工具链"
  exit 1
fi
