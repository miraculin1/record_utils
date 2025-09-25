#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import numpy as np
import cv2
from typing import List, Tuple

# =================== 配置项（也可用命令行参数覆盖） ===================
DEFAULT_FILE   = "poses_log.txt"
ROLE_SWAP      = False  # True 时交换两路角色
METHOD         = "TSAI" # ["TSAI","PARK","HORAUD","DANIILIDIS"]
MIN_SAMPLES    = 8      # 最少样本数（越多越稳）
DEDUP_EPS_ROT  = 1e-6   # 去除连续重复姿态（旋转阈值）
DEDUP_EPS_TRANS= 1e-9   # 去除连续重复姿态（平移阈值）
# ===================================================================


def parse_args():
    ap = argparse.ArgumentParser(
        description="Read poses file and perform hand-eye calibration with OpenCV."
    )
    ap.add_argument("--file", default=DEFAULT_FILE, help="poses_log file path")
    ap.add_argument("--swap", action="store_true", help="swap roles of the two topics")
    ap.add_argument("--method", default=METHOD,
                   choices=["TSAI","PARK","HORAUD","DANIILIDIS"],
                   help="calibrateHandEye method")
    ap.add_argument("--min-samples", type=int, default=MIN_SAMPLES)
    ap.add_argument("--no-dedup", action="store_true",
                    help="do not deduplicate consecutive identical poses")
    return ap.parse_args()


def normalize_quat_xyzw(q):
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([0,0,0,1], dtype=np.float64)
    return q / n


def quat_xyzw_to_R(q):
    """ 四元数(x,y,z,w) -> 3x3 旋转矩阵 """
    x, y, z, w = q
    # 归一化
    n = x*x + y*y + z*z + w*w
    s = 2.0 / n
    wx, wy, wz = s*w*x, s*w*y, s*w*z
    xx, xy, xz = s*x*x, s*x*y, s*x*z
    yy, yz, zz = s*y*y, s*y*z, s*z*z
    R = np.array([
        [1.0 - (yy + zz), xy - wz,        xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)]
    ], dtype=np.float64)
    return R


def R_to_quat_wxyz(R):
    """ 3x3 -> 四元数(w,x,y,z) """
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    else:
        idx = np.argmax([R[0,0], R[1,1], R[2,2]])
        if idx == 0:
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif idx == 1:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def load_poses_file(path: str, role_swap: bool=False):
    """
    读取文件，返回绝对位姿序列：
      R_g2b[i], t_g2b[i], R_t2c[i], t_t2c[i]
    默认 topic1 -> gripper->base, topic2 -> target->cam；role_swap=True 时交换。
    """
    p1_list, q1_list = [], []
    p2_list, q2_list = [], []

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 17:
                # 期望至少 17 列
                continue
            # 解析 topic1
            p1x, p1y, p1z = map(float, parts[2:5])
            q1x, q1y, q1z, q1w = map(float, parts[5:9])
            # 解析 topic2
            p2x, p2y, p2z = map(float, parts[10:13])
            q2x, q2y, q2z, q2w = map(float, parts[13:17])

            p1_list.append([p1x, p1y, p1z])
            q1_list.append([q1x, q1y, q1z, q1w])  # xyzw

            p2_list.append([p2x, p2y, p2z])
            q2_list.append([q2x, q2y, q2z, q2w])

    if len(q1_list) == 0 or len(q2_list) == 0:
        raise RuntimeError("文件内容为空或格式不匹配，未解析到任何姿态。")

    P1 = np.asarray(p1_list, dtype=np.float64)
    Q1 = np.asarray(q1_list, dtype=np.float64)
    P2 = np.asarray(p2_list, dtype=np.float64)
    Q2 = np.asarray(q2_list, dtype=np.float64)

    # 归一化四元数
    Q1 = np.vstack([normalize_quat_xyzw(q) for q in Q1])
    Q2 = np.vstack([normalize_quat_xyzw(q) for q in Q2])

    if role_swap:
        P1, P2 = P2, P1
        Q1, Q2 = Q2, Q1

    # 转旋转矩阵列表 & 列表化平移
    R_g2b = [quat_xyzw_to_R(q) for q in Q1]
    t_g2b = [p.reshape(3,1) for p in P1]
    R_t2c = [quat_xyzw_to_R(q) for q in Q2]
    t_t2c = [p.reshape(3,1) for p in P2]

    return R_g2b, t_g2b, R_t2c, t_t2c


def dedup_consecutive(Rs: List[np.ndarray], ts: List[np.ndarray],
                      eps_rot=1e-6, eps_trans=1e-9):
    """
    去除连续重复（或极近似）的绝对位姿，避免对(A_i,B_i)构造时产生退化。
    """
    keep_R, keep_t = [], []
    last_R, last_t = None, None
    for R, t in zip(Rs, ts):
        if last_R is None:
            keep_R.append(R); keep_t.append(t)
        else:
            # 用旋转向量范数衡量旋转差
            dR = last_R.T @ R
            rvec, _ = cv2.Rodrigues(dR)
            rot_err = np.linalg.norm(rvec)
            trans_err = np.linalg.norm(t - last_t)
            if rot_err > eps_rot or trans_err > eps_trans:
                keep_R.append(R); keep_t.append(t)
        last_R, last_t = R, t
    return keep_R, keep_t


def rel_motions(Rs: List[np.ndarray], ts: List[np.ndarray]) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    由绝对位姿序列构造相邻相对运动：
      A_i = T_i^{-1} * T_{i+1}   （R,t 分量）
    """
    R_rel, t_rel = [], []
    for i in range(len(Rs)-1):
        R0, t0 = Rs[i],   ts[i]
        R1, t1 = Rs[i+1], ts[i+1]
        Rinv = R0.T
        Rinvinv_t = -Rinv @ t0
        # T_rel = T0^{-1} * T1
        Rr = Rinv @ R1
        tr = Rinv @ t1 + Rinvinv_t
        R_rel.append(Rr)
        t_rel.append(tr)
    return R_rel, t_rel


def rotation_only_solve(Rg_rel: List[np.ndarray], Rt_rel: List[np.ndarray]) -> np.ndarray:
    """
    旋转-only解 AX=XB 中的 R_X（最小二乘）。
    """
    def R_to_quat_wxyz_local(R):
        return R_to_quat_wxyz(R)

    def left_mult_quat_mat(q):
        w,x,y,z = q
        return np.array([
            [ w, -x, -y, -z],
            [ x,  w, -z,  y],
            [ y,  z,  w, -x],
            [ z, -y,  x,  w]
        ], dtype=np.float64)

    def right_mult_quat_mat(q):
        w,x,y,z = q
        return np.array([
            [ w, -x, -y, -z],
            [ x,  w,  z, -y],
            [ y, -z,  w,  x],
            [ z,  y, -x,  w]
        ], dtype=np.float64)

    Qa = [R_to_quat_wxyz_local(R) for R in Rg_rel]
    Qb = [R_to_quat_wxyz_local(R) for R in Rt_rel]
    M = []
    for qa, qb in zip(Qa, Qb):
        L = left_mult_quat_mat(qa)
        Rm = right_mult_quat_mat(qb)
        M.append(L - Rm)
    M = np.vstack(M)
    _, _, Vt = np.linalg.svd(M)
    qx = Vt[-1, :]
    qx = qx / np.linalg.norm(qx)
    # wxyz -> R
    w,x,y,z = qx
    # 转 xyzw 再复用上面的函数
    R_X = quat_xyzw_to_R([x,y,z,w])
    return R_X


def method_flag(name: str) -> int:
    name = name.upper()
    if name == "TSAI":
        return cv2.CALIB_HAND_EYE_TSAI
    if name == "PARK":
        return cv2.CALIB_HAND_EYE_PARK
    if name == "HORAUD":
        return cv2.CALIB_HAND_EYE_HORAUD
    if name == "DANIILIDIS":
        return cv2.CALIB_HAND_EYE_DANIILIDIS
    raise ValueError("Unknown method: " + name)


def ax_xb_residual(Rg_rel, tg_rel, Rt_rel, tt_rel, Rcg, tcg):
    """
    用相对运动检验 AX=XB 残差：A_i X 与 X B_i 的 (R,t) 距离。
    返回 (mean_rot_err_rad, mean_trans_err)
    """
    rot_errs, trans_errs = [], []
    for Ra, ta, Rb, tb in zip(Rg_rel, tg_rel, Rt_rel, tt_rel):
        # A X
        R1 = Ra @ Rcg
        t1 = Ra @ tcg + ta
        # X B
        R2 = Rcg @ Rb
        t2 = Rcg @ tb + tcg
        dR = R1.T @ R2
        rvec, _ = cv2.Rodrigues(dR)
        rot_errs.append(np.linalg.norm(rvec))
        trans_errs.append(np.linalg.norm(t1 - t2))
    return float(np.mean(rot_errs)), float(np.mean(trans_errs))


def main():
    args = parse_args()
    R_g2b, t_g2b, R_t2c, t_t2c = load_poses_file(args.file, role_swap=args.swap)

    if not args.no_dedup:
        R_g2b, t_g2b = dedup_consecutive(R_g2b, t_g2b, DEDUP_EPS_ROT, DEDUP_EPS_TRANS)
        R_t2c, t_t2c = dedup_consecutive(R_t2c, t_t2c, DEDUP_EPS_ROT, DEDUP_EPS_TRANS)

    n = min(len(R_g2b), len(R_t2c))
    if n < args.min_samples:
        raise RuntimeError(f"样本不足：有效样本 {n} < {args.min_samples}。请多采点不同姿态。")

    # 相对运动（用于 residual 检验 & 旋转-only 解）
    Rg_rel, tg_rel = rel_motions(R_g2b, t_g2b)
    Rt_rel, tt_rel = rel_motions(R_t2c, t_t2c)

    # 旋转-only 粗解
    R_init = rotation_only_solve(Rg_rel, Rt_rel)
    q_init = R_to_quat_wxyz(R_init)
    print("[Rotation-only] R_X (3x3):\n", R_init)
    print("[Rotation-only] q_X (w,x,y,z):", q_init)

    # 完整手眼（OpenCV）
    meth = method_flag(args.method)
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base=R_g2b,
        t_gripper2base=t_g2b,
        R_target2cam=R_t2c,
        t_target2cam=t_t2c,
        method=meth
    )

    q_cg = R_to_quat_wxyz(R_cam2gripper)
    print(f"\n[OpenCV:{args.method}] cam->gripper 结果：")
    print("R (3x3):\n", R_cam2gripper)
    print("t (3x1):", t_cam2gripper.ravel())
    print("q (w,x,y,z):", q_cg)

    # 基于相对运动的残差评估 AX=XB
    rot_err, trans_err = ax_xb_residual(Rg_rel, tg_rel, Rt_rel, tt_rel,
                                        R_cam2gripper, t_cam2gripper)
    print("\n[Consistency Residual] mean rot err (rad): %.6g | mean trans err: %.6g" %
          (rot_err, trans_err))


if __name__ == "__main__":
    main()
