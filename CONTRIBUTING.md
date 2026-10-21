# Branches and merge requests

## Long-lived branches

| Branch | Holds | Changes by |
|---|---|---|
| `main` | robot-verified states only, each tagged | MR from `develop` or `release/*` (maintainer merges) |
| `develop` | integration; the GitLab default branch | MR from a topic branch (maintainer merges) |
| `develop_ai` | integration for AI-agent work; replaces ad-hoc merge branches | agents' topic branches merge here directly; no MR needed |

Nobody pushes to `main` or `develop` directly; force-push is off on both.

`develop_ai` is the agents' own integration branch: AI topic branches branch
off it and merge back into it, so agent work never needs a throwaway merge
branch. Humans promote it to `develop` with a single MR (tests, build and
robot evidence in the description) when a state is worth keeping. Merge
`develop` into `develop_ai` when `develop` moves.

## Topic branches: `type/topic`

Branch off `develop`, one topic per branch, lowercase kebab-case:

| Type | For | Example |
|---|---|---|
| `feature/` | new behaviour | `feature/esc-speed-capture` |
| `fix/` | a bug | `fix/fwu-done-restart` |
| `hw/` | hardware bring-up, tuning, measurement | `hw/ground-speed-tuning` |
| `docs/` | documentation only | `docs/can-fwu-procedure` |
| `chore/` | build, CI, tooling, cleanup | `chore/ci-runner` |
| `release/` | stabilising a version for `main` | `release/0.2` |

Keep branches short-lived: rebase on or merge `develop` when it moves, and
GitLab deletes the source branch after merge. Experiments that may be thrown
away still get a branch (`hw/...`); delete it when done instead of merging.

## Merge requests

- Target `develop` (or `main` from `release/*`).
- Before opening: `scripts/test.sh` passes and `scripts/build.sh`
  builds without warnings (CI runs the same once a runner is enabled).
- Anything that ran on the robot: put the evidence in the MR description
  (board, firmware hash/CRC, battery voltage, numbers measured). Docs describe
  the current state only; see `AGENTS.md`.
- One reviewer approval. Merge commits, no squash (history keeps the evidence
  in the commit messages).

## Releases and tags

- `develop` → `main` when a state has been verified on the robot, via
  `release/x.y` if it needs stabilising first.
- Tag every `main` update `vX.Y.Z`. Notable robot results get an annotated
  `milestone/<name>` tag (e.g. `milestone/first-ground-drive`) with the
  evidence in the tag message.
- A fix needed on `main` urgently: `fix/...` from `main`, MR into `main`,
  then merge `main` back into `develop`.
