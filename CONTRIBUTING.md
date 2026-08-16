# Contributing

## Making a change

1. Branch off `main`, one branch per change (`feature/...`, `fix/...`, `docs/...`).
2. Open a PR against `main`. Include a short description of what changed and why.
3. Get a review before merging. Don't merge your own PR.
4. Keep PRs scoped, a docs restructure and a node behavior change should be separate PRs.

## Documentation

If you touch a node, service, or config file's behavior, update the matching page under [`docs/`](docs/README.md) in the same PR. Docs that drift from the code are worse than no docs, they actively mislead the next person.

Found something in the docs that's wrong, stale, or confusing? Flag it or fix it directly, documentation fixes are cheap and don't need the same scrutiny as code changes.

## Code

- Match the existing style in the file you're editing.
- If you're adding a new node, service, or message type, it needs to show up in [`docs/architecture.md`](docs/architecture.md)'s interface tables.
- Don't hand-edit generated or install-time files (`install/`, `build/`, `log/`), those aren't tracked.

## Questions

Flag it to the team.
