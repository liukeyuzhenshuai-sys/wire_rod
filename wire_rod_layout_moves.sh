#!/usr/bin/env bash
set -euo pipefail

# Run from the repository root on the current main branch.
mkdir -p doc/requirements doc/design doc/decisions doc/status dataset

# Protected requirements.
git mv docs/TRACKER_REQUIREMENTS_V1.md doc/requirements/
git mv docs/WEB_PRODUCT_REQUIREMENTS_V1.md doc/requirements/

# Protected design assets.
git mv docs/ALGORITHM_TEST_GUIDE.md doc/design/
git mv docs/COMPLEX_DATASET_V4.md doc/design/
git mv docs/COMPLEX_REPRODUCIBILITY.md doc/design/
git mv docs/DATA_FORMAT.md doc/design/
git mv docs/DOUBLE_LAYER_SUPPORT_V0_8.md doc/design/
git mv docs/GENERATOR_LOGIC.md doc/design/
git mv docs/IMPLEMENTATION_AND_TEST_PLAN.md doc/design/
git mv docs/INTERACTIVE_EDITOR_V0_4.md doc/design/
git mv docs/INTERACTIVE_EDITOR_V0_5.md doc/design/
git mv docs/SCENE_RELATIONSHIPS.md doc/design/
git mv docs/TOP_DOWN_Z_SUPPORT_V0_7.md doc/design/
git mv docs/TRACKER_DETAILED_DESIGN_V1.md doc/design/
git mv docs/WEB_INSPECTION_GUIDE.md doc/design/
git mv design/tracker_config.example.json doc/design/

# Decisions.
git mv docs/DECISIONS_AND_TBDS.md doc/decisions/

# Status / handoff / validation records.
git mv docs/COMPLEX_STRESS_FINDINGS.md doc/status/
git mv docs/IMPLEMENTATION_STATUS_V0_1.md doc/status/
git mv docs/IMPLEMENTATION_STATUS_V0_2.md doc/status/
git mv docs/IMPLEMENTATION_STATUS_V0_3.md doc/status/
git mv docs/IMPLEMENTATION_STATUS_V0_4.md doc/status/
git mv docs/IMPLEMENTATION_STATUS_V0_5.md doc/status/
git mv docs/IMPLEMENTATION_STATUS_V0_6.md doc/status/
git mv docs/IMPLEMENTATION_STATUS_V0_7.md doc/status/
git mv docs/IMPLEMENTATION_STATUS_V0_8.md doc/status/
git mv WEB_HANDOFF.md doc/status/WEB_HANDOFF.md
git mv COMPLEX_VALIDATION.txt doc/status/COMPLEX_VALIDATION.txt

# Dataset metadata.
git mv labels dataset/labels

# All root-level point-cloud frames, including current and future sequence names.
# nullglob prevents the literal "*.ply" from being passed when none exist.
shopt -s nullglob
for f in ./*.ply; do
  git mv "$f" dataset/
done
shopt -u nullglob

# Remove legacy directories if now empty.
rmdir docs 2>/dev/null || true
rmdir design 2>/dev/null || true

echo "Repository moves complete."
echo "Next: git apply wire_rod_layout_v1.patch"
echo "Then: python tests/check_repository_governance.py"
