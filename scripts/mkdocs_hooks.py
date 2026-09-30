"""
MkDocs hooks for the project site (issue #104).

The Markdown files are written for GitHub and stay unchanged; these hooks adapt them to the
site at build time:

- README.md, CONTRIBUTING.md, examples/README.md and tools/README.md become pages of the site
  (ROOT_PAGES) without copies or stubs in docs/.
- Relative links are resolved from the file's place in the repository. A link to one of the
  site's pages is rewritten to that page; a link to any other file of the repository (LICENSE,
  sources) points to the file on GitHub.
- GitHub alerts (``> [!NOTE]``) become Material admonitions.
"""

from pathlib import Path
import posixpath
import re

from mkdocs.structure.files import File

REPO_URL = 'https://github.com/atinfinity/livox-mid360-core'
DOCS_DIR = 'docs'

# Repository path -> page of the site.
ROOT_PAGES = {
    'README.md': 'index.md',
    'CONTRIBUTING.md': 'contributing.md',
    'examples/README.md': 'examples.md',
    'tools/README.md': 'tools.md',
}
SITE_TO_REPO = {page: path for path, page in ROOT_PAGES.items()}

# GitHub alert type -> Material admonition type and title.
ALERTS = {
    'NOTE': ('note', 'Note'),
    'TIP': ('tip', 'Tip'),
    'IMPORTANT': ('info', 'Important'),
    'WARNING': ('warning', 'Warning'),
    'CAUTION': ('danger', 'Caution'),
}

_FENCE = re.compile(r'^\s*(```|~~~)')
_CODE_SPAN = re.compile(r'(`+).*?\1')
_LINK = re.compile(r'(\]\()([^)\s]+)((?:\s+"[^"]*")?\))')
_ALERT = re.compile(r'^>\s*\[!(\w+)\]\s*$')


def on_files(files, config):
    """Add the Markdown files outside docs/ as pages."""
    root = Path(config['config_file_path']).parent
    for path, page in ROOT_PAGES.items():
        content = (root / path).read_text(encoding='utf-8')
        files.append(File.generated(config, page, content=content))
    return files


def on_page_markdown(markdown, page, config, files):
    """Rewrite links and alerts of one page."""
    src = page.file.src_uri
    repo_path = SITE_TO_REPO.get(src, posixpath.join(DOCS_DIR, src))
    out = []
    in_fence = False
    lines = markdown.split('\n')
    i = 0
    while i < len(lines):
        line = lines[i]
        if _FENCE.match(line):
            in_fence = not in_fence
        elif not in_fence:
            alert = _ALERT.match(line)
            if alert and alert.group(1) in ALERTS:
                kind, title = ALERTS[alert.group(1)]
                out.append(f'!!! {kind} "{title}"')
                i += 1
                while i < len(lines) and lines[i].startswith('>'):
                    body = lines[i][1:].removeprefix(' ')
                    out.append('    ' + _rewrite_links(body, repo_path, src) if body else '')
                    i += 1
                continue
            line = _rewrite_links(line, repo_path, src)
        out.append(line)
        i += 1
    return '\n'.join(out)


def _rewrite_links(line, repo_path, src):
    """Rewrite the relative link targets of one line, skipping code spans."""
    parts = []
    pos = 0
    for code in _CODE_SPAN.finditer(line):
        end = code.start()
        parts.append(_LINK.sub(lambda m: _link(m, repo_path, src), line[pos:end]))
        parts.append(code.group(0))
        pos = code.end()
    parts.append(_LINK.sub(lambda m: _link(m, repo_path, src), line[pos:]))
    return ''.join(parts)


def _link(match, repo_path, src):
    target = match.group(2)
    if re.match(r'^[a-z][a-z0-9+.-]*:', target) or target.startswith('#'):
        return match.group(0)
    path, sep, anchor = target.partition('#')
    resolved = posixpath.normpath(posixpath.join(posixpath.dirname(repo_path), path))
    if resolved in ROOT_PAGES:
        page = ROOT_PAGES[resolved]
    elif resolved.startswith(DOCS_DIR + '/'):
        prefix = len(DOCS_DIR) + 1
        page = resolved[prefix:]
    else:
        kind = 'tree' if path.endswith('/') else 'blob'
        new = f'{REPO_URL}/{kind}/main/{resolved}{sep}{anchor}'
        return match.group(1) + new + match.group(3)
    new = posixpath.relpath(page, posixpath.dirname(src) or '.') + sep + anchor
    return match.group(1) + new + match.group(3)
