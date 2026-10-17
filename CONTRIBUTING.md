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

## Docstrings

Document what isn't obvious from the name and signature: public interfaces, non-trivial behaviour, and anything a caller could get wrong. Trivial helpers don't need one.

Style: Google-style docstrings (`Args:`, `Returns:`, `Raises:` sections). Skip a section that doesn't apply, don't write `Args: None` or `Returns: None` for a function that takes no arguments or returns nothing. Continuation lines under an `Args:`/`Raises:` entry get an extra level of indent under the name they continue.

If the existing code already has a comment explaining *why* something is done a particular way (not just what it does), fold that into the docstring rather than deleting it, that context is usually the most valuable part.

## Questions

Flag it to the team.
