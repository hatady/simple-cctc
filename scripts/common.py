import collections, yaml
import traci

# ← あなたの環境の絶対パスをそのまま記入（必要なら適宜変更）
PARAMS_PATH = "/home/hatada/work/simple-cctc/config/params.yaml"

def load_params(path=None):
    """
    ルートからの絶対パスで params.yaml を読み込む。
    引数 path を明示指定した場合はそちらを優先。
    """
    target = path if path is not None else PARAMS_PATH
    with open(target, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ---- 速度制限・飽和・レンジ関数 ----
def W(v, v_max):               # 式(5) に相当
    return v if v <= v_max else v_max

def sat(u, a_min, a_max):      # 式(2) の飽和
    return max(a_min, min(a_max, u))

def V_cav(h, v_max, h_go, h_st):  # 式(4)
    if h <= h_st: return 0.0
    if h < h_go:  return v_max * (h - h_st) / (h_go - h_st)
    return v_max

def V_h(h, v_max, h_go, h_st):    # 式(8)
    if h <= h_st: return 0.0
    if h < h_go:
        num = (2*h_go - h_st - h)*(h - h_st)
        den = (h_go - h_st)**2
        return v_max * (num/den)
    return v_max

# ---- ヘッドウェイと遅延バッファ ----
def get_gap(leader, follower):
    """先行rear - 後続front の距離（直線1D前提）"""
    xl, _ = traci.vehicle.getPosition(leader)
    xf, _ = traci.vehicle.getPosition(follower)
    ll = traci.vehicle.getLength(leader)
    lf = traci.vehicle.getLength(follower)
    return (xl - ll/2.0) - (xf + lf/2.0)

class DelayBuffer:
    """u(t-τ)などの遅延をΔt単位で実現"""
    def __init__(self, delay_s, dt, init=0.0):
        n = max(1, int(round(delay_s/dt)))
        self.buf = collections.deque([init]*n)
    def push(self, val):
        self.buf.append(val)
        return self.buf.popleft()