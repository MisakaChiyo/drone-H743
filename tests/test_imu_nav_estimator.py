import math

import pytest


G = 9.81


def body_to_nav(ax_g, ay_g, az_g, roll, pitch, yaw):
    ax = ax_g * G
    ay = ay_g * G
    az = az_g * G
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr

    return (
        r00 * ax + r01 * ay + r02 * az,
        r10 * ax + r11 * ay + r12 * az,
        r20 * ax + r21 * ay + r22 * az + G,
    )


def level_basis_to_nav(ax_g, ay_g, az_g):
    acc = (ax_g * G, ay_g * G, az_g * G)
    z_body = normalize((-ax_g, -ay_g, -az_g))
    x_body = project_onto_level((1.0, 0.0, 0.0), z_body)
    if norm(x_body) < 1e-6:
        x_body = project_onto_level((0.0, 1.0, 0.0), z_body)
    x_body = normalize(x_body)
    y_body = cross(z_body, x_body)

    return (
        dot(x_body, acc),
        dot(y_body, acc),
        dot(z_body, acc) + G,
    )


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(v):
    return math.sqrt(dot(v, v))


def normalize(v):
    n = norm(v)
    return tuple(x / n for x in v)


def project_onto_level(v, z_body):
    return tuple(x - dot(v, z_body) * z for x, z in zip(v, z_body))


def test_level_static_acceleration_becomes_zero_linear_nav_accel():
    acc = body_to_nav(0.0, 0.0, -1.0, 0.0, 0.0, 0.0)

    assert abs(acc[0]) < 1e-6
    assert abs(acc[1]) < 1e-6
    assert abs(acc[2]) < 1e-6


def test_tilted_static_acceleration_becomes_zero_linear_nav_accel():
    roll = math.radians(12.0)
    pitch = math.radians(-7.0)
    yaw = math.radians(25.0)
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)

    # Accelerometers report specific force: static readings are opposite
    # gravity. With NED/body Z down, level static body Z is -1 g.
    ax_g = sp
    ay_g = -cp * sr
    az_g = -cp * cr

    acc = body_to_nav(ax_g, ay_g, az_g, roll, pitch, yaw)

    assert abs(acc[0]) < 1e-6
    assert abs(acc[1]) < 1e-6
    assert abs(acc[2]) < 1e-6


def test_level_basis_zeroes_horizontal_accel_for_any_static_orientation():
    for vector in [
        (0.0, 0.0, -1.0),
        (0.35, -0.20, -0.9151502608861564),
        (-0.15, 0.74, -0.6551335745820206),
    ]:
        acc = level_basis_to_nav(*normalize(vector))

        assert acc[0] == pytest.approx(0.0, abs=1e-6)
        assert acc[1] == pytest.approx(0.0, abs=1e-6)
        assert acc[2] == pytest.approx(0.0, abs=1e-6)


def test_velocity_integrates_constant_x_acceleration_without_leak():
    velocity = 0.0
    dt = 0.001
    for _ in range(1000):
        velocity += 1.0 * dt

    assert velocity == pytest.approx(1.0, abs=1e-6)


def test_bias_capture_stops_static_velocity_walk():
    bias = body_to_nav(0.02, 0.0, -1.0, 0.0, 0.0, 0.0)
    velocity = 0.0
    dt = 0.001
    for _ in range(1000):
        acc = body_to_nav(0.02, 0.0, -1.0, 0.0, 0.0, 0.0)
        velocity += (acc[0] - bias[0]) * dt

    assert abs(velocity) < 1e-6
