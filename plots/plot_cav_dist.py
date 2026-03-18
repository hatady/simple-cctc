#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAV同士（headとtail）の中心間距離 dist_head_tail_center を時系列プロットするスクリプト。
使い方例:
    python3 plot_cav_distance.py --csv /path/to/conn_cut48.csv --out cav_distance.png --tmin 28 --tmax 90 --ma 1 --vline 48
"""
import argparse
import os
import sys
import math
import pandas as pd
import matplotlib.pyplot as plt

def moving_average(x, w):
    if w is None or w <= 1:
        return x
    return x.rolling(window=w, min_periods=1, center=False).mean()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True, help="入力CSV（t と dist_head_tail_center を含む）")
    ap.add_argument("--out", default=None, help="出力画像ファイル (例: cav_distance.png)。未指定なら CSV と同じ場所に保存")
    ap.add_argument("--tmin", type=float, default=None, help="表示する時刻の最小値 [s]")
    ap.add_argument("--tmax", type=float, default=None, help="表示する時刻の最大値 [s]")
    ap.add_argument("--ma", type=int, default=1, help="移動平均窓幅（サンプル数）。1で平滑化なし")
    ap.add_argument("--vline", type=float, default=None, help="縦線を引く時刻（例: 48.0）")
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        print(f"CSV not found: {args.csv}", file=sys.stderr)
        sys.exit(1)

    # CSVを読み込む
    df = pd.read_csv(args.csv)

    # 列名の候補（ユーザーが名前を少し変えた場合に備える）
    time_cols = ["t", "time", "sec", "seconds"]
    dist_cols = ["dist_head_tail_center", "cav_center_distance", "dist_cav_center"]

    t_col = next((c for c in time_cols if c in df.columns), None)
    d_col = next((c for c in dist_cols if c in df.columns), None)

    if t_col is None:
        print(f"時間列が見つかりません。候補: {time_cols}", file=sys.stderr)
        sys.exit(2)
    if d_col is None:
        print(f"距離列が見つかりません。候補: {dist_cols}", file=sys.stderr)
        sys.exit(3)

    # 時間範囲フィルタ
    if args.tmin is not None:
        df = df[df[t_col] >= args.tmin]
    if args.tmax is not None:
        df = df[df[t_col] <= args.tmax]

    # 平滑化
    df = df.sort_values(by=t_col).reset_index(drop=True)
    y_plot = moving_average(df[d_col], args.ma)

    # プロット（色やスタイルの明示指定はしない）
    plt.figure()
    plt.plot(df[t_col], y_plot)
    plt.xlabel("time [s]")
    plt.ylabel("CAV center distance [m]")
    plt.title("Head–Tail CAV Center Distance vs Time")

    # 縦線（通信断などのイベント時刻）
    if args.vline is not None:
        # デフォルト設定のまま追加（色指定はしない）
        plt.axvline(x=args.vline)

    plt.grid(True)

    # 保存先決定
    if args.out is None:
        base = os.path.splitext(os.path.basename(args.csv))[0]
        out_dir = os.path.dirname(os.path.abspath(args.csv))
        args.out = os.path.join(out_dir, f"{base}_cav_distance.png")

    plt.tight_layout()
    plt.savefig(args.out, dpi=200)
    print(f"Saved: {args.out}")

if __name__ == "__main__":
    main()
