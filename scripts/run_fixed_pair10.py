#!/usr/bin/env python3
# run_fixed_pair10.py
import os, csv, argparse
import traci
from common import load_params, W, V_cav, V_h, sat, get_gap, DelayBuffer
from roles import get_role

# ==== 絶対パス固定（あなたの環境） ====
PROJECT_ROOT = "/home/hatada/work/simple-cctc"
SUMO_CFG     = os.path.join(PROJECT_ROOT, "data", "cctc.sumocfg")
LOG_DIR      = os.path.join(PROJECT_ROOT, "logs")

# 並び（lead→head→HV10…HV1→tail）
ORDER = ["lead", "head",
         "hv10","hv9","hv8","hv7","hv6","hv5","hv4","hv3","hv2","hv1",
         "tail"]

def lead_accel(t, segs):
    """先頭車の外生加速度プロファイル"""
    for t0, t1, a in segs:
        if t0 <= t < t1:
            return a
    return 0.0

def leader_of(vid: str):
    i = ORDER.index(vid)
    return ORDER[i - 1] if i > 0 else None

def wait_until_spawned(expected_ids, max_steps=50):
    """depart=0車両がspawnされるまで待機"""
    for _ in range(max_steps):
        present = set(traci.vehicle.getIDList())
        if all(x in present for x in expected_ids):
            return
        traci.simulationStep()
    missing = [x for x in expected_ids if x not in set(traci.vehicle.getIDList())]
    raise RuntimeError(f"Not spawned within {max_steps} steps: {missing}")

def main(mode, out_csv, use_gui=False, delay_ms=None, manual_start=False):
    P = load_params()
    dt     = P["common"]["dt"]
    v_max  = P["common"]["v_max"]
    h_st   = P["common"]["h_st"]
    a_min  = P["common"]["a_min"]
    a_max  = P["common"]["a_max"]

    # ==== SUMO 起動コマンドを構成 ====
    sumo_bin = "sumo-gui" if use_gui else "sumo"
    cmd = [sumo_bin, "-c", SUMO_CFG]
    if use_gui and delay_ms is not None:
        cmd += ["--delay", str(delay_ms)]          # GUI描画遅延（ms）
    if not manual_start:
        cmd += ["--start"]                          # 自動スタート（手動のときは付けない）

    traci.start(cmd)

    # ---- depart=0 の車両をspawnさせる（手動/自動どちらでも先に1歩進める）----
    traci.simulationStep()

    # ---- 手動開始ならGUI確認ののち、spawnが揃うまで進める ----
    if manual_start:
        input("GUI で状況を確認したら Enter を押して開始します…")

    # 期待IDがすべて存在するまで待機
    wait_until_spawned(ORDER, max_steps=50)

    # ここから車両APIを安全に呼ぶ
    vids = ORDER[:]
    for vid in vids:
        traci.vehicle.setSpeedMode(vid, 0)

    # 役割に応じた遅延バッファ（CAV: sigma, HV: tau）
    db = {}
    for vid in vids:
        role = get_role(vid) or ""
        if role.startswith("CAV"):
            db[vid] = DelayBuffer(P["cav"]["sigma"], dt, 0.0)
        elif role == "HV" or role == "":
            db[vid] = DelayBuffer(P["hv"]["tau"], dt, 0.0)
        else:
            db[vid] = DelayBuffer(0.0, dt, 0.0)

    # ====== メインループ ======
    TEND = 90.0
    t = 0.0

    # ログヘッダ（ORDERに追従）
    header = ["t"] + [f"v_{vid}" for vid in vids]
    rows = [tuple(header)]

    try:
        while t <= TEND and traci.simulation.getMinExpectedNumber() > 0:
            # 現在速度
            v = {vid: traci.vehicle.getSpeed(vid) for vid in vids}

            # LEAD：外生 a → v 更新
            a_lead = sat(lead_accel(t, P["lead_profile"]["segments"]), a_min, a_max)
            v_lead_new = max(0.0, min(v_max, v["lead"] + a_lead * dt))
            traci.vehicle.setSpeed("lead", v_lead_new)

            # ギャップ計算（lead以外）
            gaps = {}
            for _vid in vids[1:]:
                gaps[_vid] = get_gap(leader_of(_vid), _vid)

            # 目標加速度計算
            u_cmd = {}

            # head（CAV_HEAD）
            Vh = V_cav(gaps["head"], v_max, P["cav"]["h_go"], h_st)
            u_head = (
                P["cav"]["alpha_head"] * (Vh - v["head"])
                + P["cav"]["beta_head"] * (W(v["lead"], v_max) - v["head"])
            )
            if mode == "conn":
                u_head += P["cav"]["beta_cross_head_to_tail"] * (W(v["tail"], v_max) - v["head"])
            u_cmd["head"] = u_head

            # 中間HV（headの次からtailの直前までを一般化）
            # ORDER = [lead, head, hv10 ... hv1, tail]
            for idx in range(2, len(ORDER)-1):
                me = ORDER[idx]
                up = ORDER[idx-1]
                # HV前提
                Vhv = V_h(gaps[me], v_max, P["hv"]["h_go"], h_st)
                u_cmd[me] = (
                    P["hv"]["alpha"] * (Vhv - v[me]) + P["hv"]["beta"] * (v[up] - v[me])
                )

            # tail（CAV_TAIL）
            leader_of_tail = leader_of("tail")  # 直前車（hv1）
            Vt = V_cav(gaps["tail"], v_max, P["cav"]["h_go"], h_st)
            u_tail = (
                P["cav"]["alpha_tail"] * (Vt - v["tail"])
                + P["cav"]["beta_tail"] * (W(v[leader_of_tail], v_max) - v["tail"])
            )
            if mode == "conn":
                u_tail += P["cav"]["beta_cross_tail_to_head"] * (W(v["head"], v_max) - v["tail"])
            u_cmd["tail"] = u_tail

            # 遅延→飽和→速度更新（lead以外）
            for _vid in vids[1:]:
                u_d = db[_vid].push(u_cmd[_vid])
                a_applied = sat(u_d, a_min, a_max)
                v_now = v[_vid]
                v_new = max(0.0, min(v_max, v_now + a_applied * dt))
                traci.vehicle.setSpeed(_vid, v_new)

            # ログ（ORDER順）
            rows.append(tuple([t] + [v[vid] for vid in vids]))

            traci.simulationStep()
            t += dt
    finally:
        traci.close()

    # ====== CSV出力 ======
    if os.path.isabs(out_csv):
        out_abs = out_csv
    else:
        out_abs = os.path.join(PROJECT_ROOT, out_csv) if os.sep in out_csv else os.path.join(LOG_DIR, out_csv)
    os.makedirs(os.path.dirname(out_abs), exist_ok=True)
    with open(out_abs, "w", newline="") as f:
        csv.writer(f).writerows(rows)
    print("wrote", out_abs)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["no-conn","conn"], required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--gui", action="store_true", help="sumo-gui で起動")
    ap.add_argument("--delay-ms", type=int, default=None, help="GUI描画遅延（ms）例: 80")
    ap.add_argument("--manual-start", action="store_true", help="--start を付けず、Enter待ちで手動開始")
    a = ap.parse_args()
    main(a.mode, a.out, a.gui, a.delay_ms, a.manual_start)
