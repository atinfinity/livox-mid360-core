"""
Stage the Markdown sources of the project site (issues #104, #185).

The Markdown files are written for GitHub and stay unchanged. Zensical has no MkDocs hooks, so
this script writes the adapted copies into site-src/ (git-ignored), the docs_dir of
zensical.toml; scripts/build-docs.sh runs it before Zensical:

- docs/ is copied, including the Doxygen reference in docs/reference/.
- README.md, CONTRIBUTING.md, examples/README.md and tools/README.md become pages of the site
  (ROOT_PAGES) without copies or stubs in docs/.
- Relative links are resolved from the file's place in the repository. A link to one of the
  site's pages is rewritten to that page; a link to any other file of the repository (LICENSE,
  sources) points to the file on GitHub.
- GitHub alerts (``> [!NOTE]``) become admonitions.

It also checks the nav of zensical.toml against the pages, which Zensical does not: every page
needs a nav entry, and every nav entry needs a page (MkDocs' ``validation: omitted_files``).

Only changed files are written, and files that no longer have a source are removed, so that
``--watch`` (used by ``scripts/build-docs.sh --serve``) triggers a rebuild only for real edits.

Usage: python3 scripts/stage_docs.py [--watch]
"""

import argparse
from pathlib import Path
import posixpath
import re
import sys
import time
import tomllib

REPO_URL = 'https://github.com/atinfinity/livox-mid360-core'
ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / 'zensical.toml'
DOCS_DIR = 'docs'
OUT_DIR = ROOT / 'site-src'

# Repository path -> page of the site.
ROOT_PAGES = {
    'README.md': 'index.md',
    'CONTRIBUTING.md': 'contributing.md',
    'examples/README.md': 'examples.md',
    'tools/README.md': 'tools.md',
}

# GitHub alert type -> admonition type and title.
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


def sources():
    """Map each page or file of the site to its path in the repository."""
    pages = {page: path for path, page in ROOT_PAGES.items()}
    for path in sorted((ROOT / DOCS_DIR).rglob('*')):
        if path.is_file():
            pages[path.relative_to(ROOT / DOCS_DIR).as_posix()] = path.relative_to(ROOT).as_posix()
    return pages


def stage():
    """Bring OUT_DIR up to date and return the number of files written or removed."""
    changed = 0
    wanted = sources()
    for page, repo_path in wanted.items():
        data = (ROOT / repo_path).read_bytes()
        if page.endswith('.md'):
            data = adapt(data.decode('utf-8'), repo_path, page).encode('utf-8')
        out = OUT_DIR / page
        if not out.is_file() or out.read_bytes() != data:
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_bytes(data)
            changed += 1
    for out in sorted(OUT_DIR.rglob('*'), reverse=True):
        rel = out.relative_to(OUT_DIR).as_posix()
        if out.is_file() and rel not in wanted:
            out.unlink()
            changed += 1
        elif out.is_dir() and not any(out.iterdir()):
            out.rmdir()
    return changed


def check_nav():
    """Return the errors between the nav of CONFIG and the staged pages."""
    targets = set()
    entries = tomllib.loads(CONFIG.read_text(encoding='utf-8'))['project']['nav']
    while entries:
        entry = entries.pop()
        if isinstance(entry, dict):
            entries.extend(entry.values())
        elif isinstance(entry, list):
            entries.extend(entry)
        elif not re.match(r'^[a-z][a-z0-9+.-]*:', entry):
            targets.add(entry)
    pages = set(sources())
    errors = [f'nav entry without a page: {t}' for t in sorted(targets - pages)]
    for page in sorted(p for p in pages - targets if p.endswith('.md')):
        errors.append(f'page without a nav entry: {page}')
    return errors


def adapt(markdown, repo_path, page):
    """Rewrite the links and alerts of one page."""
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
                    out.append('    ' + _rewrite_links(body, repo_path, page) if body else '')
                    i += 1
                continue
            line = _rewrite_links(line, repo_path, page)
        out.append(line)
        i += 1
    return '\n'.join(out)


def _rewrite_links(line, repo_path, page):
    """Rewrite the relative link targets of one line, skipping code spans."""
    parts = []
    pos = 0
    for code in _CODE_SPAN.finditer(line):
        end = code.start()
        parts.append(_LINK.sub(lambda m: _link(m, repo_path, page), line[pos:end]))
        parts.append(code.group(0))
        pos = code.end()
    parts.append(_LINK.sub(lambda m: _link(m, repo_path, page), line[pos:]))
    return ''.join(parts)


def _link(match, repo_path, page):
    target = match.group(2)
    if re.match(r'^[a-z][a-z0-9+.-]*:', target) or target.startswith('#'):
        return match.group(0)
    path, sep, anchor = target.partition('#')
    resolved = posixpath.normpath(posixpath.join(posixpath.dirname(repo_path), path))
    if resolved in ROOT_PAGES:
        dest = ROOT_PAGES[resolved]
    elif resolved.startswith(DOCS_DIR + '/'):
        prefix = len(DOCS_DIR) + 1
        dest = resolved[prefix:]
    else:
        kind = 'tree' if path.endswith('/') else 'blob'
        new = f'{REPO_URL}/{kind}/main/{resolved}{sep}{anchor}'
        return match.group(1) + new + match.group(3)
    new = posixpath.relpath(dest, posixpath.dirname(page) or '.') + sep + anchor
    return match.group(1) + new + match.group(3)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--watch', action='store_true', help='restage once a second until killed')
    args = parser.parse_args()
    print(f'stage_docs: {stage()} files updated in {OUT_DIR.relative_to(ROOT)}/', flush=True)
    errors = check_nav()
    for error in errors:
        print(f'stage_docs: {CONFIG.name}: {error}', file=sys.stderr)
    if errors and not args.watch:
        sys.exit(1)
    while args.watch:
        time.sleep(1)
        changed = stage()
        if changed:
            print(f'stage_docs: {changed} files updated', flush=True)


if __name__ == '__main__':
    main()
