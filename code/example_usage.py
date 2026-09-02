from pathlib import Path
from dataset import CoilDataset

ROOT=Path(__file__).resolve().parents[1]
ds=CoilDataset(ROOT)

# 1) Pick a real temporal pair
pair=next(p for p in ds.pairs if p['sequence_id']=='S06' and p['pair_type']=='temporal_adjacent')
print('PAIR:',pair)

# 2) Legal tracking input: ONLY xyz + instance_id
prev_instances=ds.split_frame_instances(pair['previous_file'])
curr_instances=ds.split_frame_instances(pair['current_file'])
print('previous instance IDs:',sorted(prev_instances))
print('current instance IDs:',sorted(curr_instances))
for iid,pts in curr_instances.items():
    print('instance',iid,'points',pts.shape)

# 3) Run your matcher here. Example expected output format:
# predictions = {current_instance_id: predicted_global_id_or_None}
# IMPORTANT: None means UNCERTAIN. Never read PLY global_id during inference.

# 4) GT is loaded only AFTER prediction for evaluation/debugging.
print('\nGT transition rows:')
for row in ds.transition_gt(pair['pair_id']):
    print(row)
