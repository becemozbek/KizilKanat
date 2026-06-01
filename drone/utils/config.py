"""
Merkezi config yükleyici.
Kullanım:
    from drone.utils.config import cfg
    cfg.rrt.adim_uzunluk_m  → 10
    cfg.batarya.kritik_voltaj_v  → 21.0
"""

import os
import yaml
from types import SimpleNamespace


def _dict_to_ns(d):
    """Dict'i nokta notasyonuyla erişilebilir objeye çevirir."""
    if isinstance(d, dict):
        return SimpleNamespace(**{k: _dict_to_ns(v) for k, v in d.items()})
    if isinstance(d, list):
        return [_dict_to_ns(i) for i in d]
    return d


def _yukle():
    yol = os.path.join(os.path.dirname(__file__), '../../config.yaml')
    yol = os.path.abspath(yol)
    with open(yol, encoding='utf-8') as f:
        return _dict_to_ns(yaml.safe_load(f))


cfg = _yukle()
