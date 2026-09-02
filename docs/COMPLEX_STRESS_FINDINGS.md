# Complex Stress Findings V0.3

## Why this dataset exists

The original V3 sequences are intentionally simple and remain useful as unit/regression tests. C01-C10 are not replacements; they combine several realistic loading effects in the same temporal sequence to expose failures that simple sequences can hide.

## Failure actually found: C08 full-chain identity shift

The first run of C01-C10 did **not** pass. In C08, all six historical layer-1 coils translated together by about 1.05 m while object count stayed unchanged. The old ordered DP was monotone, but monotonicity alone was insufficient: it selected a lower local-position-cost solution similar to:

```text
SKIP old ID0
old ID1 -> current physical ID0
old ID2 -> current physical ID1
...
INSERT one current object
```

This preserves left-to-right order but shifts the complete identity chain, which is unacceptable. The initial complex regression reported 8 accumulated wrong-ID assignments in C08.

## Fix: semantic rank lock under conservative preconditions

When all of the following are true for a layer:

1. every committed historical track was visible;
2. current visible object count equals historical count;
3. there is no robot NEW hint;
4. every rank-by-X pair passes normal Layer / motion ROI / diameter / length gates;
5. rank-pair X displacements are collectively coherent;

then the business model gives stronger information than nearest position: old removal is unsupported and same-layer physical order cannot reverse, so rank-by-X is the identity correspondence. Those pairs become semantic anchors.

If any condition fails, the matcher does **not** force this rule; it falls back to the standard sparse candidate + geometric anchor + ordered DP path, retaining UNCERTAIN as a legal result.

## Regression after the fix

- C01-C10: 10 sequences / 52 temporal frames / **WRONG_ID=0**.
- Pairwise recommended regression across original and complex data: 87 pairs / **WRONG_ID=0**, validator failure=0.
- Pairwise NEW: TP=39, FP=0, FN=0.
- Pairwise OCCLUDED: TP=8, FP=0, FN=0.
- Conservative UNCERTAIN: 1.

The purpose of recording this failure is to prevent a future maintainer from removing the semantic rank rule just because an easier dataset still passes.
