import csv
import matplotlib.pyplot as plt

NO_CONN = "/home/hatada/work/simple-cctc/logs/no_conn.csv"
CONN    = "/home/hatada/work/simple-cctc/logs/conn.csv"
OUTPNG  = "/home/hatada/work/simple-cctc/plots/fig_all_vehicles_ms_window_5_50.png"

# 時間窓 [s]
T_START = 28.0
T_END   = 90.0

ORDER = ["v_lead","v_head","v_hv5","v_hv4","v_hv3","v_hv2","v_hv1","v_tail"]
LABEL = {
    "v_lead":"Lead",
    "v_head":"Head CAV",
    "v_hv5":"HV5","v_hv4":"HV4","v_hv3":"HV3","v_hv2":"HV2","v_hv1":"HV1",
    "v_tail":"Tail CAV",
}

def load_csv(path):
    """ヘッダ名で列を特定して読み込む（速度は m/s のまま）。"""
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        idx = {name:i for i,name in enumerate(header)}
        have = [k for k in ORDER if k in idx]
        t=[]; series={k:[] for k in have}
        for row in r:
            t.append(float(row[idx["t"]]))
            for k in have:
                series[k].append(float(row[idx[k]]))
    return t, series

def window(t, series_dict, t0, t1):
    mask = [(ti >= t0) and (ti <= t1) for ti in t]
    tw = [ti for ti,m in zip(t,mask) if m]
    sw = {k: [xi for xi,m in zip(xs,mask) if m] for k,xs in series_dict.items()}
    return tw, sw

# 読み込み → 時間窓切り出し
tA, sA = load_csv(NO_CONN); tA, sA = window(tA, sA, T_START, T_END)
tB, sB = load_csv(CONN);    tB, sB = window(tB, sB, T_START, T_END)

def plot_panel(t, s, title):
    plt.title(title)
    for k in ORDER:
        if k not in s: 
            continue
        # HVは黒で統一、他はデフォルト色
        if k.startswith("v_hv"):
            plt.plot(t, s[k], label=LABEL[k], color="black", linewidth=1.0, alpha=0.8)
        else:
            plt.plot(t, s[k], label=LABEL[k])
    plt.ylabel("Speed [m/s]")
    plt.legend(ncol=4)

plt.figure(figsize=(10,8))
plt.subplot(2,1,1)
plot_panel(tA, sA, f"All vehicles — No connection (m/s), {T_START}–{T_END}s")

plt.subplot(2,1,2)
plot_panel(tB, sB, f"All vehicles — Connected pair (m/s), {T_START}–{T_END}s")
plt.xlabel("Time [s]")

plt.tight_layout()
plt.savefig(OUTPNG, dpi=150)
print("saved", OUTPNG)
