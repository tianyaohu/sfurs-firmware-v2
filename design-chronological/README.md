# Design log

Why things were done, in date order, so a later reader (human or Claude) can
see the reasoning behind a choice without digging through git.

- `MASTER.md`: decisions that affect the whole system. Append only; newest at
  the bottom.
- `<name>.md` (one per person, e.g. `frank-h.md`): that person's design
  choices, experiments and dead ends. Same entry format, append only.

Entry format:

```
## YYYY-MM-DD  Title  (who: person or person+claude)
- Decision: what was chosen
- Why: the reason, with the number or measurement that drove it
- Rejected: alternatives and why not
- Touches: files, docs, tests
```

Rules: never rewrite an old entry; if a decision is reversed, add a new entry
that says so. The log holds reasoning, not state: current behaviour still
belongs in [ARCHITECTURE](../architecture/index.md) and [DRIVE](../architecture/drive.md), and a
one-line takeaway in [LESSONS](../docs/lessons.md). Hardware numbers go in
[test-results](../hardware-test-results/).
