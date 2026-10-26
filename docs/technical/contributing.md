# Contributing

## Workflow

1. Create a branch from `main` for each change. Name it `feature/<name>`, `fix/<name>` or `docs/<name>`.
2. Open a pull request against `main`. Describe what changed and why.
3. A reviewer approves the pull request before it is merged. The author does not merge it.
4. Keep each pull request focused on one change. Submit documentation restructures and node behaviour changes separately.

Run the [unit tests](testing.md#unit-tests) before opening a pull request. Do not edit or commit `build/`, `install/` or `log/`. They are generated and ignored by git.

## Code

Match the style of the file you edit. A new node, service or message type is added to [Middleware nodes](middleware-nodes.md) and [ROS interfaces](ros-interfaces.md).

## Documentation

A change to the behaviour of a node, service, action or configuration file updates the matching page of this manual in the same pull request. The pages are in `docs/`, and `mkdocs.yml` lists them.

To build the manual locally:

```bash
pip install -r docs/requirements.txt
mkdocs serve
```

MkDocs serves the site at `http://127.0.0.1:8000` and rebuilds it when a file changes. The pages under Proxy come from the proxy repository. They exist only in the build of the repository's workflow, so a local build warns that they are missing.

The workflow in `.github/workflows/docs.yml` builds the site for every pull request, and deploys it to GitHub Pages when a change reaches `main`. To refresh the pages from the proxy repository without a change in this repository, run the workflow from the Actions tab. The workflow copies the folder `docs` of the proxy repository to `docs/proxy/` and generates the Go API reference in `docs/proxy/api/`. Git ignores `docs/proxy/`. Do not commit those files.

## Docstrings

Docstrings are the source of the API reference. mkdocstrings reads them from the source files without importing the code, so building the manual does not need ROS 2.

Write a docstring for what the name and signature do not show: public interfaces, behaviour that is not obvious and anything a caller can get wrong. A trivial helper does not need one. If a comment explains why the code does something, move it into the docstring.

Use the Google style with the sections `Args:`, `Returns:` and `Raises:`. Leave out a section that does not apply. Continuation lines of an `Args:` or `Raises:` entry have one more level of indent than the entry:

```python
"""Check one position and orientation, and describe the result.

Args:
    actions (ArmActions): Action context used to call the move services.
    args (argparse.Namespace): Parsed options (``ik_only``, ``speed``,
        ``tolerance_deg``).

Returns:
    tuple[str, bool]: The text for the results line, and whether the
    check passed.
"""
```
