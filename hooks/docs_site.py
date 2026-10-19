"""MkDocs hook: docs/README.md is the GitHub landing page and docs/index.md is the site's, so on the site
drop the 'Docs Home' breadcrumb line and point any other README.md link at index.md."""
import re

BREADCRUMB = re.compile(r"\A\[Docs Home\]\([^)]*README\.md\)\n+")
README_LINK = re.compile(r"\]\(((?:\.\./|\./)*)README\.md")


def on_page_markdown(markdown, **kwargs):
    markdown = BREADCRUMB.sub("", markdown)
    return README_LINK.sub(r"](\1index.md", markdown)
