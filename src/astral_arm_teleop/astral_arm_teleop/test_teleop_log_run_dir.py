"""run_log_dir 单元测试：每次 launch 自动建 {root}/{stamp}[_tag]/ 运行目录。

覆盖：空 root 关闭 / 目录创建 / stamp 格式 / tag 追加与消毒 / 多调用并存。
"""

from __future__ import annotations

import os
import re
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from astral_arm_teleop.teleop_log import run_log_dir  # noqa: E402

_STAMP_RE = re.compile(r"^\d{8}-\d{6}(_[A-Za-z0-9_]+)?$")


def test_empty_root_disables() -> None:
    assert run_log_dir("") == ""
    assert run_log_dir("   ") == ""


def test_creates_stamped_dir() -> None:
    with tempfile.TemporaryDirectory() as root:
        d = run_log_dir(root)
        assert os.path.isdir(d)
        assert os.path.dirname(d) == root
        assert _STAMP_RE.match(os.path.basename(d))


def test_tag_appended_and_sanitized() -> None:
    with tempfile.TemporaryDirectory() as root:
        d = run_log_dir(root, "pick/place test 7")
        assert os.path.isdir(d)
        assert os.path.basename(d).endswith("_pick_place_test_7")


def test_separate_calls_exist() -> None:
    with tempfile.TemporaryDirectory() as root:
        a, b = run_log_dir(root), run_log_dir(root)
        # 秒级戳：同一秒内可能同名，但目录都存在且互不影响
        assert os.path.isdir(a) and os.path.isdir(b)


def main() -> int:
    fns = [
        test_empty_root_disables,
        test_creates_stamped_dir,
        test_tag_appended_and_sanitized,
        test_separate_calls_exist,
    ]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"  PASS {fn.__name__}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"  FAIL {fn.__name__}: {exc}")
    print(f"run_log_dir: {len(fns) - failed}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
