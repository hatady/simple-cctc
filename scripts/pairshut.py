#!/usr/bin/env python3
import os, csv, argparse
import traci
from common import load_params, W, V_cav, V_h, sat, get_gap, DelayBuffer
from roles import get_role

# ==== 絶対パス固定（あなたの環境） ====
PROJECT_ROOT = "/home/hatada/work/simple-cctc"
SUMO_CFG     = os.path.join(PROJECT_ROOT, "data", "cctc.sumocfg")
LOG_DIR      = os.path.join(PROJECT_ROOT, "logs")

# one.rou.xml の並び（lead→head→HV群→tail）
ORDER = ["lead", "head", "hv5", "hv4", "hv3", "hv2", "hv1", "tail"]

def lead_accel(t, segs):
    """先頭車の外生加速度プロファイル"""
    for t0, t1, a in segs:
        if t0 <= t < t1:
            return a
    return 0.0

def leader_of(vid):
    i = ORDER.index(vid)
    return ORDER[i - 1] if i > 0 else None

# ==== 通信断の設定（48秒から10秒かけて 1→0） ====
COMM_CUT_T  = 48.0
COMM_RAMP_S = 5.0   # 線形減衰

def gate_coeff(t, t_cut, ramp_s):
    """
    通信ゲート係数 g(t) を返す。
    t < t_cut: 1.0
    rampあり: t_cut <= t < t_cut+ramp -> 1→0 線形
    t >= t_cut(+ramp): 0.0
    """
    if t < t_cut:
        return 1.0
    if ramp_s <= 0.0:
        return 0.0
    x = 1.0 - (t - t_cut) / ramp_s
    return max(0.0, min(1.0, x))

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

    vids = ORDER[:]
    for vid in vids:
        traci.vehicle.setSpeedMode(vid, 0)

    # ---- ここが「手動スタート」：Enter を押すまで待機 ----
    if manual_start:
        input("GUI で状況を確認したら Enter を押して開始します…")

    # 遅延バッファ（役割ごとに異なる遅延）
    db = {}
    for vid in vids:
        role = get_role(vid)
        if role and role.startswith("CAV"):
            db[vid] = DelayBuffer(P["cav"]["sigma"], dt, 0.0)
        elif role == "HV":
            db[vid] = DelayBuffer(P["hv"]["tau"], dt, 0.0)
        else:
            db[vid] = DelayBuffer(0.0, dt, 0.0)

    TEND = 90.0
    t = 0.0

    # ログ（解析用に g_conn を追加）
    rows = [("t", "g_conn", "v_lead", "v_head", "v_hv5", "v_hv4", "v_hv3", "v_hv2", "v_hv1", "v_tail")]

    # >>> A: latch start ———— ここから（追加）
    v_head_latched = None
    v_tail_latched = None
    latched = False
    # >>> A: latch end ———— ここまで

    while t <= TEND and traci.simulation.getMinExpectedNumber() > 0:
        v = {vid: traci.vehicle.getSpeed(vid) for vid in vids}

        # 通信ゲート（connモードのみ有効）
        g = gate_coeff(t, COMM_CUT_T, COMM_RAMP_S) if mode == "conn" else 0.0

        # >>> A: latch start ———— ここから（追加）
        # 切断開始時点で一度だけ head/tail の速度をラッチ
        if (not latched) and (t >= COMM_CUT_T):
            v_head_latched = v["head"]
            v_tail_latched = v["tail"]
            latched = True
        # 以降の参照は、t < cut までは最新値、t >= cut ではラッチ値
        ref_head_for_tail = v["head"] if t < COMM_CUT_T else (v_head_latched if v_head_latched is not None else v["head"])
        ref_tail_for_head = v["tail"] if t < COMM_CUT_T else (v_tail_latched if v_tail_latched is not None else v["tail"])
        # >>> A: latch end ———— ここまで

        # LEAD：外生 a → v 更新
        a_lead = sat(lead_accel(t, P["lead_profile"]["segments"]), a_min, a_max)
        v_lead_new = max(0.0, min(v_max, v["lead"] + a_lead * dt))
        traci.vehicle.setSpeed("lead", v_lead_new)

        # 先頭とのヘッドウェイ
        gaps = {}
        for _vid in ORDER[1:]:
            gaps[_vid] = get_gap(leader_of(_vid), _vid)

        u_cmd = {}

        # ---- head（CAV）----
        Vh = V_cav(gaps["head"], v_max, P["cav"]["h_go"], h_st)
        u_head = (
            P["cav"]["alpha_head"] * (Vh - v["head"])
            + P["cav"]["beta_head"] * (W(v["lead"], v_max) - v["head"])
        )
        if mode == "conn":
            # クロス項（tailからの情報）をゲート
            # >>> A: 参照値は t >= cut で固定（ラッチ値）、寄与は g(t) でフェード
            u_head += g * P["cav"]["beta_cross_head_to_tail"] * (W(ref_tail_for_head, v_max) - v["head"])
        u_cmd["head"] = u_head

        # ---- HVチェーン ----
        for up, me in [("head", "hv5"), ("hv5", "hv4"), ("hv4", "hv3"), ("hv3", "hv2"), ("hv2", "hv1")]:
            Vhv = V_h(gaps[me], v_max, P["hv"]["h_go"], h_st)
            u_cmd[me] = (
                P["hv"]["alpha"] * (Vhv - v[me]) + P["hv"]["beta"] * (v[up] - v[me])
            )

        # ---- tail（CAV）----
        Vt = V_cav(gaps["tail"], v_max, P["cav"]["h_go"], h_st)
        u_tail = (
            P["cav"]["alpha_tail"] * (Vt - v["tail"])
            + P["cav"]["beta_tail"] * (W(v["hv1"], v_max) - v["tail"])
        )
        if mode == "conn":
            # クロス項（headからの情報）をゲート
            # >>> A: 参照値は t >= cut で固定（ラッチ値）、寄与は g(t) でフェード
            u_tail += g * P["cav"]["beta_cross_tail_to_head"] * (W(ref_head_for_tail, v_max) - v["tail"])
        u_cmd["tail"] = u_tail

        # 遅延を噛ませつつ加速度飽和→速度更新
        for _vid in ORDER[1:]:
            u_d = db[_vid].push(u_cmd[_vid])
            a_applied = sat(u_d, a_min, a_max)
            v_now = v[_vid]
            v_new = max(0.0, min(v_max, v_now + a_applied * dt))
            traci.vehicle.setSpeed(_vid, v_new)

        # 記録（g_connも）
        rows.append((t, g, v["lead"], v["head"], v["hv5"], v["hv4"], v["hv3"], v["hv2"], v["hv1"], v["tail"]))

        traci.simulationStep()
        t += dt

    traci.close()

    # 出力解決（相対/絶対どちらでもOK）
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
