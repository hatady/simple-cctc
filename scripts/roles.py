
from typing import Optional
import traci

def get_role(vid: str) -> Optional[str]:
    """
    車両vidのロールを返す。
    優先順位: vehicle param 'role' -> vType/IDヒューリスティクス -> 既定('HV')
    パラメータが読めない/車両が存在しない場合は None も許容。
    """
    # 1) param(role)最優先（SUMO/TraCIは 'param:role' が無難）
    for key in ("param:role", "role"):
        try:
            val = traci.vehicle.getParameter(vid, key)
            if val:  # 空文字でなければ採用
                return val.strip()
        except traci.TraCIException:
            # 車両未生成やID誤り。ここで即return Noneにしてもよい。
            pass

    # 2) vTypeに基づく推測
    try:
        t = traci.vehicle.getTypeID(vid)
    except traci.TraCIException:
        return None  # 不明

    if t == "cav":
        low = vid.lower()
        if "head" in low:
            return "CAV_HEAD"
        if "tail" in low:
            return "CAV_TAIL"
        return "CAV"

    if t == "hv":
        return "HV"

    # 3) 既定
    return "HV"

def is_cav(vid: str) -> bool:
    r = get_role(vid)
    return bool(r) and r.startswith("CAV")
