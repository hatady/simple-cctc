import csv
import matplotlib.pyplot as plt

NO_CONN = "/home/hatada/work/simple-cctc/logs/normal_no_conn.csv"
CONN    = "/home/hatada/work/simple-cctc/logs/normal_conn.csv"
OUTPNG  = "/home/hatada/work/simple-cctc/plots/fig_all_vehicles_ms_window_5_50.png"

# 時間窓 [s]
T_START = 20.0
T_END   = 90.0

# === 6,7,8 を含む表示順 ===
ORDER = [
    "v_lead","v_head",
    "v_hv5","v_hv4","v_hv3","v_hv2","v_hv1",
    "v_tail",
    "v_hv6","v_hv7","v_hv8"
]

LABEL = {
    "v_lead":"Lead",
    "v_head":"Head CAV",
    "v_tail":"Tail CAV",
}

def load_csv(path):
    """ヘッダ名で列を特定して読み込む（速度は m/s のまま）"""
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        idx = {name: i for i, name in enumerate(header)}

        have = [k for k in ORDER if k in idx]
        t = []
        series = {k: [] for k in have}

        for row in r:
            t.append(float(row[idx["t"]]))
            for k in have:
                series[k].append(float(row[idx[k]]))

    return t, series

def window(t, series_dict, t0, t1):
    mask = [(ti >= t0) and (ti <= t1) for ti in t]
    tw = [ti for ti, m in zip(t, mask) if m]
    sw = {
        k: [xi for xi, m in zip(xs, mask) if m]
        for k, xs in series_dict.items()
    }
    return tw, sw

# === 読み込み → 時間窓切り出し ===
tA, sA = load_csv(NO_CONN)
tA, sA = window(tA, sA, T_START, T_END)

tB, sB = load_csv(CONN)
tB, sB = window(tB, sB, T_START, T_END)

def plot_panel(t, s, title):
    plt.title(title)

    hv_labeled = False  # HV を1回だけ凡例に出す
    for k in ORDER:
        if k not in s:
            continue

        if k.startswith("v_hv"):
            label = "HVs" if not hv_labeled else "_nolegend_"
            hv_labeled = True
            plt.plot(
                t, s[k],
                color="black",
                linewidth=1.0,
                alpha=0.8,
                label=label
            )
        else:
            plt.plot(
                t, s[k],
                linewidth=1.5,
                label=LABEL.get(k, k)
            )

    plt.ylabel("Speed [m/s]")
    plt.legend(
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        fontsize=9,
        frameon=True
    )

# === 描画 ===
plt.figure(figsize=(10, 8))

plt.subplot(2, 1, 1)
plot_panel(
    tA, sA,
    f"All vehicles — No connection (m/s), {T_START}–{T_END}s"
)

plt.subplot(2, 1, 2)
plot_panel(
    tB, sB,
    f"All vehicles — Connected pair (m/s), {T_START}–{T_END}s"
)

plt.xlabel("Time [s]")

# 右側に凡例用スペースを確保
plt.tight_layout(rect=[0, 0, 0.82, 1])
plt.savefig(OUTPNG, dpi=150, bbox_inches="tight")
print("saved", OUTPNG)
