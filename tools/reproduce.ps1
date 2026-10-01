# 纯逻辑层复现脚本（无需 ROS 2）：Python 单元测试 + C++ 冒烟测试
# 用法：powershell -ExecutionPolicy Bypass -File tools\reproduce.ps1
$ErrorActionPreference = "Continue"

$root = Split-Path -Parent $PSScriptRoot
$src = Join-Path $root "ros2_ws\src"
$pySrc = Join-Path $src "inspection_planning_py"
$tests = Join-Path $src "inspection_tests"
$cppRoot = Join-Path $src "inspection_execution_cpp"

$failed = 0

Write-Host "`n=== 1/2 Python 单元测试（unittest，无需 ROS 2）==="
$env:PYTHONPATH = $pySrc
Push-Location $src
python -m unittest discover -s "inspection_tests/unit_python" -t "inspection_tests"
if ($LASTEXITCODE -ne 0) { $failed++ }
Pop-Location

Write-Host "`n=== 2/2 C++ 冒烟测试（脱离 ROS 编译并运行）==="
$compiler = $null
foreach ($c in @("clang++", "g++")) {
  if (Get-Command $c -ErrorAction SilentlyContinue) { $compiler = $c; break }
}
if (-not $compiler) {
  Write-Warning "未找到 clang++ / g++，跳过 C++ 冒烟测试"
  $failed++
} else {
  Write-Host "使用编译器：$compiler"
  $skipNames = @("bt_task_executor.cpp", "task_lifecycle_executor.cpp")
  $sources = Get-ChildItem -Recurse -File -Filter *.cpp -Path (Join-Path $cppRoot "src") |
    Where-Object { $_.Name -notmatch "_node" -and $skipNames -notcontains $_.Name } |
    ForEach-Object { $_.FullName }
  Write-Host "组件源文件数：$($sources.Count)（另加 smoke_test_core.cpp）"
  $exe = Join-Path $env:TEMP "smoke_test_core.exe"
  & $compiler -std=c++17 -Wall -Wextra -Wpedantic `
    "-I$(Join-Path $cppRoot 'include')" `
    (Join-Path $tests "unit_cpp\tech_1_1_to_1_9\smoke_test_core.cpp") `
    @sources -o $exe
  if ($LASTEXITCODE -ne 0) { $failed++ } else { & $exe; if ($LASTEXITCODE -ne 0) { $failed++ } }
}

Write-Host ""
if ($failed -eq 0) {
  Write-Host "全部通过：Python 单元测试 + C++ 纯逻辑冒烟测试"
  Write-Host "下一步（需 Ubuntu + ROS 2 Humble）：见 README「复现步骤」第 3 节"
} else {
  Write-Host "有 $failed 项未通过，请对照 README「运行环境」节检查工具链"
  exit 1
}
