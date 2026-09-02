from __future__ import annotations

import json, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tracker.engine import load_config
from tracker.session import SessionManager

OUT = ROOT / 'results' / 'double_layer_sequential_regression.json'
EXPECTED_ABNORMAL = {
    'D08': {'D08_F01_foundation_hard_shift.ply': 'ABNORMAL_FIRST_LAYER_SHIFT_HARD'},
    'D09': {'D09_F01_upper_off_support.ply': 'ABNORMAL_LAYER2_SUPPORT_SLOT_MISSING'},
    'D10': {'D10_F05_illegal_ninth_support8.ply': 'ABNORMAL_LAYER2_CAPACITY'},
}
NORMAL_SEQS = {'D01','D02','D03','D04','D05','D06','D07'}


def main():
    cfg = load_config(ROOT / 'tracker_config.json')
    sm = SessionManager(ROOT, cfg)
    report = {'version': 'double_layer_v0.8', 'sequences': [], 'assertions': []}
    total_wrong = total_uncertain = total_frames = 0
    failures = []

    for seq_id in [f'D{i:02d}' for i in range(1, 11)]:
        sess, ev = sm.start(seq_id)
        initial_l1 = {int(t.global_id): [float(x) for x in t.center] for t in sess.tracks if int(t.layer) == 1}
        frames = [{
            'file': sess.current_file,
            'frame_status': 'INITIAL',
            'wrong_id_count': int(ev['wrong_id_count']),
            'uncertain_count': sum(r.get('state') == 'UNCERTAIN' for r in sess.last_rows),
            'reason_codes': [],
            'second_layer_active': bool(sess.second_layer_active),
            'layer1_tracks': sum(int(t.layer) == 1 for t in sess.tracks),
            'layer2_tracks': sum(int(t.layer) == 2 for t in sess.tracks),
        }]

        # Bootstrap-specific structure checks.
        if seq_id in {'D05','D06'}:
            l1 = [t for t in sess.tracks if int(t.layer) == 1]
            l2 = [t for t in sess.tracks if int(t.layer) == 2]
            ok = len(l1) == 10 and len(l2) >= 1 and sess.second_layer_active
            report['assertions'].append({'name': f'{seq_id}_double_layer_bootstrap', 'pass': ok, 'l1': len(l1), 'l2': len(l2)})
            if not ok:
                failures.append(f'{seq_id}: double-layer bootstrap did not create 10 L1 foundation tracks')

        while True:
            sess, ev, done = sm.next(sess.session_id)
            if done:
                break
            row = {
                'file': sess.current_file,
                'frame_status': sess.last_debug.get('frame_status', 'OK'),
                'wrong_id_count': int(ev['wrong_id_count']),
                'uncertain_count': sum(r.get('state') == 'UNCERTAIN' for r in sess.last_rows),
                'reason_codes': list(sess.last_debug.get('semantic_reason_codes', [])),
                'second_layer_active': bool(sess.second_layer_active),
                'layer1_tracks': sum(int(t.layer) == 1 for t in sess.tracks),
                'layer2_tracks': sum(int(t.layer) == 2 for t in sess.tracks),
            }
            frames.append(row)

            # Once L2 is active, committed L1 TrackState is frozen.  This is distinct
            # from the current observation center shown in diagnostics.
            if sess.second_layer_active and initial_l1:
                for t in sess.tracks:
                    if int(t.layer) != 1 or int(t.global_id) not in initial_l1:
                        continue
                    if any(abs(float(a)-float(b)) > 1e-9 for a,b in zip(t.center, initial_l1[int(t.global_id)])):
                        failures.append(f'{seq_id}/{sess.current_file}: locked L1 TrackState changed for ID{t.global_id}')

        wrong = sum(x['wrong_id_count'] for x in frames)
        uncertain = sum(x['uncertain_count'] for x in frames)
        total_wrong += wrong; total_uncertain += uncertain; total_frames += len(frames)

        if seq_id in NORMAL_SEQS:
            bad = [x for x in frames[1:] if x['frame_status'] != 'OK']
            if bad:
                failures.append(f'{seq_id}: normal double-layer sequence produced non-OK frames: {[x["file"] for x in bad]}')

        for fname, code in EXPECTED_ABNORMAL.get(seq_id, {}).items():
            x = next((r for r in frames if r['file'] == fname), None)
            ok = x is not None and x['frame_status'] == 'ABNORMAL' and code in x['reason_codes']
            report['assertions'].append({'name': f'{seq_id}_{code}', 'pass': ok, 'file': fname, 'reason_codes': None if x is None else x['reason_codes']})
            if not ok:
                failures.append(f'{seq_id}/{fname}: expected {code}')

        # Recovery after intentionally abnormal input must return to a normal frame.
        if seq_id in {'D08','D09'}:
            ok = frames[-1]['frame_status'] == 'OK'
            report['assertions'].append({'name': f'{seq_id}_recovery', 'pass': ok, 'last_status': frames[-1]['frame_status']})
            if not ok:
                failures.append(f'{seq_id}: tracker did not recover after abnormal frame')

        report['sequences'].append({
            'sequence_id': seq_id,
            'wrong_total': wrong,
            'uncertain_total': uncertain,
            'frames': frames,
        })

    # Dataset visibility invariant: every D-series physical object must have positive points.
    zero_point = []
    for r in sm.ds.objects:
        if str(r.get('sequence_id','')).startswith('D') and int(float(r.get('point_count') or 0)) <= 0:
            zero_point.append({'file': r.get('file'), 'global_id': r.get('global_id')})
    positive_ok = not zero_point
    report['assertions'].append({'name': 'D_series_positive_visibility', 'pass': positive_ok, 'violations': zero_point})
    if not positive_ok:
        failures.append(f'D-series has zero-point physical objects: {zero_point[:5]}')

    report['summary'] = {
        'sequence_count': 10,
        'frame_count': total_frames,
        'wrong_id_total': total_wrong,
        'uncertain_total': total_uncertain,
        'failures': failures,
        'pass': total_wrong == 0 and not failures,
    }
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report['summary'], ensure_ascii=False, indent=2))
    if not report['summary']['pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
