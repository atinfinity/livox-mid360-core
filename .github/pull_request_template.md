Closes #

## Summary

<!-- What changes and why. Link the design decisions taken in the issue or in docs/. -->

## Testing

<!-- New or changed tests, and what you ran: ctest, sanitizers, the Docker CI reproduction,
     a real Mid-360 (firmware version). -->

## Checklist

<!-- See CONTRIBUTING.md. Strike through or remove items that do not apply. -->

- [ ] The title follows the conventional style (`feat(device): ...`, `fix: ...`, `docs: ...`); it becomes the squash commit subject.
- [ ] `scripts/lint.sh` passes.
- [ ] Transport / session / device changes have a simulator-driven test in `tests/`.
- [ ] Protocol changes regenerate the golden vectors with `tools/gen_golden_vectors.py`.
- [ ] The docs that describe the change are updated in this PR (`docs/`, `docs/architecture.md`, `docs/roadmap.md`, README).
- [ ] Clean room: nothing is copied or paraphrased from Livox-SDK2 or other drivers.
