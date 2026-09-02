from pathlib import Path
import csv, json
from ply_io import load_algorithm_input, split_instances

class CoilDataset:
    def __init__(self, root):
        self.root=Path(root)
        self.labels=self.root/'labels'
        self.manifest=self._csv(self.labels/'frame_manifest.csv')
        self.objects=self._csv(self.labels/'object_gt.csv')
        self.pairs=self._csv(self.labels/'frame_pairs.csv')
        self.transitions=self._csv(self.labels/'transition_gt.csv')
    @staticmethod
    def _csv(path):
        with open(path,'r',encoding='utf-8-sig',newline='') as f: return list(csv.DictReader(f))
    def load_frame_input(self, filename):
        return load_algorithm_input(self.root/filename)
    def split_frame_instances(self, filename):
        return split_instances(self.root/filename)
    def object_gt(self, filename):
        return [r for r in self.objects if r['file']==filename]
    def transition_gt(self, pair_id):
        return [r for r in self.transitions if r['pair_id']==pair_id]
