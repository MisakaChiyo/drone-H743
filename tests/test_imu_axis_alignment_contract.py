from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_imu_mount_axis_alignment_is_documented() -> None:
    source = read("App/Src/app_sensor.c")
    header = read("App/Inc/app_sensor.h")

    assert "IMU +Y 朝飞机下方" in source
    assert "IMU +Z 朝飞机前方" in source
    assert "IMU +X 朝飞机左方" in source
    assert "机体系采用前右下约定" in source
    assert "当前安装：IMU +Y 朝下，+Z 朝前，+X 朝左" in header


def test_imu_axes_are_rotated_to_filter_forward_right_down_body_frame() -> None:
    source = read("App/Src/app_sensor.c")

    assert "body X =  imu Z" in source
    assert "body Y = -imu X" in source
    assert "body Z =  imu Y" in source
    assert "out[0] =  in[2];" in source
    assert "out[1] = -in[0];" in source
    assert "out[2] =  in[1];" in source
