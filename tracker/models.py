from __future__ import annotations
from dataclasses import dataclass, asdict
from typing import Optional, Any
import numpy as np


def _vec(v):
    if isinstance(v, np.ndarray): return [float(x) for x in v.tolist()]
    return v

@dataclass
class Observation:
    instance_id: int
    center: np.ndarray
    point_count: int
    diameter: float
    length: float
    yaw_deg: float
    quality: float
    layer_guess: int
    bbox_min: np.ndarray
    bbox_max: np.ndarray
    raw_width_x: float
    raw_width_y: float
    observation_mode: str='MULTI_VIEW'
    topdown_likelihood: float=0.0
    z_span_to_diameter: float=0.0
    center_method: str='ROBUST_BBOX'
    top_z: float=0.0
    axis_min: float=0.0
    axis_max: float=0.0
    normal_min: float=0.0
    normal_max: float=0.0

    def json(self):
        d=asdict(self)
        for k in ('center','bbox_min','bbox_max'): d[k]=_vec(d[k])
        return d

@dataclass
class TrackState:
    global_id: Any
    layer: int
    slot: int
    cluster: str
    stable_diameter: float
    stable_length: float
    center: np.ndarray
    yaw_deg: float
    visibility: str='VISIBLE'
    last_instance_id: Optional[int]=None
    confidence: float=1.0
    reference_point_count: int=0

    def json(self):
        d=asdict(self); d['center']=_vec(self.center); return d

@dataclass
class NewCoilHint:
    hint_id: str
    diameter: float
    length: float
    target_center: np.ndarray
    expected_layer: Optional[int]=None

    def json(self):
        d=asdict(self); d['target_center']=_vec(self.target_center); return d

@dataclass
class Candidate:
    global_id: Any
    instance_id: int
    accepted: bool
    cost: Optional[float]
    reasons: list[str]
    components: dict[str,float]

    def json(self): return asdict(self)

@dataclass
class Decision:
    instance_id: Optional[int]
    global_id: Optional[Any]
    state: str
    confidence: float
    layer: Optional[int]
    slot: Optional[int]
    cluster: Optional[str]
    reason: str
    match_cost: Optional[float]=None
    hint_id: Optional[str]=None

    def json(self): return asdict(self)
