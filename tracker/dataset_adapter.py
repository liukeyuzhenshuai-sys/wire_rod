from __future__ import annotations
from pathlib import Path
import csv
import numpy as np
from .models import TrackState, NewCoilHint
from .ply import split_instances
from .geometry import observe_frame

class SyntheticDatasetAdapter:
    """Benchmark-only adapter.

    Current-frame inference never sees current global_id/layer/slot/visibility.
    GT is used in two isolated roles:
      1) bootstrap a previously *committed* history state for pairwise tests;
      2) emulate robot NEW hints by exposing only target position + nominal D/L.
    Evaluation reads GT after inference in a separate function.
    """
    def __init__(self,root):
        self.root=Path(root); self.labels=self.root/'labels'
        self.objects=self._read(self.labels/'object_gt.csv')
        self.pairs=self._read(self.labels/'frame_pairs.csv')
        self.transitions=self._read(self.labels/'transition_gt.csv')
        self.frame_semantics=self._read(self.labels/'frame_semantics.csv')
    @staticmethod
    def _read(p):
        with open(p,'r',encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
    def pair(self,pair_id):
        return next(r for r in self.pairs if r['pair_id']==pair_id)
    def frame_rows(self,file): return [r for r in self.objects if r['file']==file]
    def load_instances(self,file): return split_instances(self.root/file)
    def bootstrap_history(self,file,cfg,estimated_centers=False):
        rows=self.frame_rows(file)
        obs={}
        if estimated_centers:
            obs=observe_frame(self.load_instances(file),cfg)
        tracks=[]
        for r in rows:
            iid=None if r['instance_id']=='' else int(float(r['instance_id']))
            if estimated_centers and iid is not None and iid in obs: center=obs[iid].center
            else: center=np.array([float(r['center_x']),float(r['center_y']),float(r['center_z'])])
            tracks.append(TrackState(
                global_id=int(r['global_id']), layer=int(r['layer']), slot=int(r['slot']), cluster=r['cluster'],
                stable_diameter=float(r['diameter_nominal']), stable_length=float(r['length_nominal']), center=center,
                yaw_deg=float(r['yaw_deg']), visibility=r['visibility'], last_instance_id=iid, confidence=1.0))
        return tracks
    def robot_hints(self,current_file):
        hints=[]; k=0
        for r in self.frame_rows(current_file):
            if int(r['is_new'])!=1: continue
            # Strip current global_id and instance_id. This is the synthetic equivalent of robot input.
            tx=float(r['target_x']) if r['target_x'] else float(r['center_x'])
            ty=float(r['target_y']) if r['target_y'] else float(r['center_y'])
            tz=float(r['target_z']) if r['target_z'] else float(r['center_z'])
            hints.append(NewCoilHint(f'hint_{k}',float(r['diameter_nominal']),float(r['length_nominal']),np.array([tx,ty,tz]),int(r['layer']))); k+=1
        return hints

    def second_layer_signal(self,current_file):
        """Synthetic equivalent of the user's production layer-2 loading signal.

        This exposes only a boolean operating-mode signal, never current identity/slot GT.
        In production it is supplied by the loading controller/robot.
        """
        rows=self.frame_rows(current_file)
        return any(int(r.get('layer') or 1)==2 for r in rows)

    def gt_transition(self,pair_id): return [r for r in self.transitions if r['pair_id']==pair_id]
