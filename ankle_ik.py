"""
2-RSS-1U 脚踝逆运动学
API: theta_m1, theta_m2 = solve_motor_angles(theta_x, theta_y)
"""
import numpy as np

# ========== 几何常数 ==========
# 请根据实际机器人修改！下面用一组示例值
# 单位：m
B1 = np.array([ 0.028,  0.028, 0.0])   # B1 在固定系
B2 = np.array([-0.028,  0.028, 0.0])   # B2 在固定系
C1 = np.array([ 0.028, -0.028, 0.0])   # C1 在动系 Ow
C2 = np.array([-0.028, -0.028, 0.0])   # C2 在动系 Ow
L1 = L2 = 0.05                       # 连杆长度
d  = 0.04                            # Ow 原点相对 O 的 z 平移

def _build_T(theta_x, theta_y):
    """按文章给出的 Y-X 欧拉角返回 4×4 齐次矩阵"""
    cx, sx = np.cos(theta_x), np.sin(theta_x)
    cy, sy = np.cos(theta_y), np.sin(theta_y)

    T = np.array([
        [ cy,  sx*sy,  cx*sy, 0],
        [ 0,   cx,    -sx,   0],
        [-sy,  sx*cy,  cx*cy, d],
        [ 0,   0,      0,    1]
    ])
    return T

def _solve_single(theta_x, theta_y, B, C_ow, L):
    """对单根连杆求电机角 theta_m"""
    T = _build_T(theta_x, theta_y)
    C_o = (T @ np.r_[C_ow, 1])[:3]          # 3×1 坐标

    # 计算 (1-2) 式中的 k1,k2,a
    k1 = 2 * d * C_ow[0]                    # 2 d xc
    k2 = 2 * d * C_ow[1]                    # 2 d yc
    a  = (np.dot(B, B) + np.dot(C_ow, C_ow) + d**2 - L**2
          + 2 * d * (C_ow[2] - B[2])
          - 2 * B[0] * C_ow[0] - 2 * B[1] * C_ow[1]
          - 2 * C_ow[2] * B[2])

    # 解析解：atan2(k1,k2) ± acos(a / sqrt(k1²+k2²))
    denom = np.hypot(k1, k2)
    if np.abs(a) > denom + 1e-12:
        raise ValueError("无解：姿态超出工作空间")
    phi = np.arccos(np.clip(a / denom, -1, 1))
    theta_m1 = np.arctan2(k1, k2) + phi
    theta_m2 = np.arctan2(k1, k2) - phi
    # 选解：通常取 [-π, π] 内绝对值较小的那个
    return theta_m1 if abs(theta_m1) < abs(theta_m2) else theta_m2

def solve_motor_angles(theta_x, theta_y):
    """
    末端姿态 -> 电机角
    参数
    ----
    theta_x : float   横滚角 roll  (rad)
    theta_y : float   俯仰角 pitch (rad)

    返回
    ----
    theta_m1 : float  电机 1 角 (rad)
    theta_m2 : float  电机 2 角 (rad)
    """
    theta_m1 = _solve_single(theta_x, theta_y, B1, C1, L1)
    theta_m2 = _solve_single(theta_x, theta_y, B2, C2, L2)
    return theta_m1, theta_m2

# ========== 简单 CLI ==========
if __name__ == "__main__":
    import math
    theta_x = math.radians(10)
    theta_y = math.radians(15)
    m1, m2 = solve_motor_angles(theta_x, theta_y)
    print(f"θx={math.degrees(theta_x):.1f}°, θy={math.degrees(theta_y):.1f}°")
    print(f"θ_m1={math.degrees(m1):.2f}°, θ_m2={math.degrees(m2):.2f}°")