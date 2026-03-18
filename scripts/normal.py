#!/usr/bin/env python3
import os, csv, argparse
import traci
from common import load_params, W, V_cav, V_h, sat, get_gap, DelayBuffer
from roles import get_role

# ==== 絶対パス固定（あなたの環境） ====
PROJECT_ROOT = "/home/hatada/work/simple-cctc"
SUMO_CFG     = os.path.join(PROJECT_ROOT, "data", "cctc.sumocfg")
LOG_DIR      = os.path.join(PROJECT_ROOT, "logs")

# ==== 実行時間 ====
TEND = 95.0

# one.rou.xml の並び（lead→head→HV群→tail→hv6→hv7→hv8）
# ★ hv6, hv7, hv8 を制御対象に含める
ORDER = ["lead", "head", "hv5", "hv4", "hv3", "hv2", "hv1", "tail", "hv6", "hv7", "hv8"]

# ログ対象（＝制御対象）
LOG_VIDS = ORDER[:]

# ====== 先頭車ブレーキプロファイル ======
LEAD_BRAKE_SEGS = [
    (0.0, 35.0,   0.0),
    (35.0, 45.0, -2.0),
    (45.0, 55.0,  2.0),
    (55.0, TEND,  0.0),
]

def lead_accel(t, segs):
    """先頭車の外生加速度プロファイル"""
    for t0, t1, a in segs:
        if t0 <= t < t1:
            return a
    return 0.0

def leader_of(vid):
    i = ORDER.index(vid)
    return ORDER[i - 1] if i > 0 else None

def main(mode, out_csv, use_gui=False, delay_ms=None, manual_start=False):
    P = load_params()
    dt     = P["common"]["dt"]
    v_max  = P["common"]["v_max"]
    h_st   = P["common"]["h_st"]
    a_min  = P["common"]["a_min"]
    a_max  = P["common"]["a_max"]

    # ==== SUMO 起動 ====
    sumo_bin = "sumo-gui" if use_gui else "sumo"
    cmd = [sumo_bin, "-c", SUMO_CFG]
    if use_gui and delay_ms is not None:
        cmd += ["--delay", str(delay_ms)]
    if not manual_start:
        cmd += ["--start"]

    traci.start(cmd)

    ctrl_vids = ORDER[:]      # 制御対象（hv6-8含む）
    log_vids  = LOG_VIDS[:]   # ログ対象

    # SpeedMode 無効化（制御対象すべて）
    for vid in ctrl_vids:
        traci.vehicle.setSpeedMode(vid, 0)

    if manual_start:
        input("GUIで確認後 Enter を押して開始します…")

    # ==== 遅延バッファ（制御対象のみ） ====
    db = {}
    for vid in ctrl_vids:
        role = get_role(vid)
        if role and role.startswith("CAV"):
            db[vid] = DelayBuffer(P["cav"]["sigma"], dt, 0.0)
        elif role == "HV":
            db[vid] = DelayBuffer(P["hv"]["tau"], dt, 0.0)
        else:
            db[vid] = DelayBuffer(0.0, dt, 0.0)

    t = 0.0

    # ==== ログ ====
    rows = [(
        "t",
        "v_lead", "v_head", "v_hv5", "v_hv4", "v_hv3", "v_hv2", "v_hv1", "v_tail",
        "v_hv6", "v_hv7", "v_hv8",
        "dist_head_tail_center"
    )]

    while t <= TEND and traci.simulation.getMinExpectedNumber() > 0:
        # 現在速度（ログ対象）
        v = {vid: traci.vehicle.getSpeed(vid) for vid in log_vids}

        # ==== lead：外生ブレーキ ====
        a_lead = sat(lead_accel(t, LEAD_BRAKE_SEGS), a_min, a_max)
        v_lead_new = max(0.0, min(v_max, v["lead"] + a_lead * dt))
        traci.vehicle.setSpeed("lead", v_lead_new)

        # ==== gaps（制御対象のみ：hv6-8も含む）====
        gaps = {}
        for vid in ORDER[1:]:
            gaps[vid] = get_gap(leader_of(vid), vid)

        # ==== 制御入力 ====
        u_cmd = {}

        # head（CAV）
        Vh = V_cav(gaps["head"], v_max, P["cav"]["h_go"], h_st)
        u_head = (
            P["cav"]["alpha_head"] * (Vh - v["head"])
            + P["cav"]["beta_head"]  * (W(v["lead"], v_max) - v["head"])
        )
        if mode == "conn":
            u_head += P["cav"]["beta_cross_head_to_tail"] * (W(v["tail"], v_max) - v["head"])
        u_cmd["head"] = u_head

        # HV列（通常追従のみ）＋ hv6-8 も追従制御する
        for up, me in [
            ("head", "hv5"), ("hv5", "hv4"), ("hv4", "hv3"),
            ("hv3", "hv2"), ("hv2", "hv1"),
            ("tail", "hv6"), ("hv6", "hv7"), ("hv7", "hv8"),
        ]:
            Vhv = V_h(gaps[me], v_max, P["hv"]["h_go"], h_st)
            u_cmd[me] = (
                P["hv"]["alpha"] * (Vhv - v[me])
                + P["hv"]["beta"]  * (v[up] - v[me])
            )

        # tail（CAV）
        Vt = V_cav(gaps["tail"], v_max, P["cav"]["h_go"], h_st)
        u_tail = (
            P["cav"]["alpha_tail"] * (Vt - v["tail"])
            + P["cav"]["beta_tail"]  * (W(v["hv1"], v_max) - v["tail"])
        )
        if mode == "conn":
            u_tail += P["cav"]["beta_cross_tail_to_head"] * (W(v["head"], v_max) - v["tail"])
        u_cmd["tail"] = u_tail

        # ==== 遅延・飽和を適用し速度更新（lead以外すべて）====
        for vid in ORDER[1:]:
            u_d = db[vid].push(u_cmd[vid])
            a = sat(u_d, a_min, a_max)
            v_new = max(0.0, min(v_max, v[vid] + a * dt))
            traci.vehicle.setSpeed(vid, v_new)

        # ==== head-tail 距離 ====
        x_head = traci.vehicle.getPosition("head")[0]
        x_tail = traci.vehicle.getPosition("tail")[0]
        dist_head_tail_center = x_head - x_tail

        rows.append((
            t,
            v["lead"], v["head"], v["hv5"], v["hv4"], v["hv3"],
            v["hv2"], v["hv1"], v["tail"],
            v["hv6"], v["hv7"], v["hv8"],
            dist_head_tail_center
        ))

        traci.simulationStep()
        t += dt

    traci.close()

    # ==== CSV 出力 ====
    out_abs = out_csv if os.path.isabs(out_csv) else os.path.join(LOG_DIR, out_csv)
    os.makedirs(os.path.dirname(out_abs), exist_ok=True)
    with open(out_abs, "w", newline="") as f:
        csv.writer(f).writerows(rows)

    print("wrote", out_abs)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["no-conn", "conn"], required=True,
                    help="no-conn: クロス結合なし / conn: クロス結合あり（切断なし）")
    ap.add_argument("--out", required=True)
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--delay-ms", type=int, default=None)
    ap.add_argument("--manual-start", action="store_true")
    args = ap.parse_args()

    main(args.mode, args.out, args.gui, args.delay_ms, args.manual_start)
