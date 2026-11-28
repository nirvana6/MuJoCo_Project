#!/usr/bin/env python3
"""Build a single-file HTML view of tutorial-01 from its Markdown source."""

import html
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MD_NAME = "tutorial-01-MJCF with ur5e.md"
TITLE = "tutorial-01-MJCF with UR5e"
PAIR_RE = re.compile(r'<a id="(sec-[A-Za-z0-9-]+)"></a>\n\n## ([^\n]+)\n')
MATHML_XMLNS = ' xmlns="http://www.w3.org/1998/Math/MathML"'

CSS = """
html { scroll-behavior: smooth; }
[id] { scroll-margin-top: 16px; }
body {
  margin: 0;
  padding-left: 304px;
  color: #1b1b1b;
  background: #fff;
  font-family: sans-serif;
  line-height: 1.7;
}
.toc {
  position:fixed;
  left:0;
  top: 0;
  bottom: 0;
  width: 280px;
  overflow-y: auto;
  box-sizing: border-box;
  padding: 20px 14px 32px;
  background: #f4f4f4;
  border-right: 1px solid #d0d0d0;
}
.toc a {
  display: block;
  margin: 2px 0;
  padding: 6px 8px;
  color: #1b1b1b;
  text-decoration: none;
  line-height: 1.45;
  border-radius: 4px;
}
.toc a:hover { background: #e6e6e6; }
.toc a.active {
  background: #fff;
  font-weight: 600;
}
main {
  margin: 0 auto;
  max-width: 1040px;
  padding: 40px 48px;
}
table { border-collapse: collapse; margin: 1em 0; }
th, td {
  border: 1px solid #ccc;
  padding: 6px 10px;
  vertical-align: top;
}
pre {
  background: #f6f8fa;
  overflow-x: auto;
  padding: 12px 16px;
}
code { font-family: ui-monospace, monospace; }
blockquote {
  margin: 1em 0;
  padding: 0.1em 1em;
  border-left: 4px solid #c5c5c5;
}
ul.task-list, ul.task-list li { list-style: none; }
ul.task-list { padding-left: 0; }
@media (max-width: 900px) {
  .toc { display: none; }
  body { padding-left:0; }
}
""".strip()

JS = """
(function () {
  var links = document.querySelectorAll("nav.toc a");
  var byId = new Map();
  links.forEach(function (link) {
    var href = link.getAttribute("href") || "";
    if (href.charAt(0) === "#") byId.set(href.slice(1), link);
  });
  function activate(id) {
    links.forEach(function (link) {
      var on = link.getAttribute("href") === "#" + id;
      link.classList.toggle("active", on);
      if (on) link.setAttribute("aria-current", "true");
      else link.removeAttribute("aria-current");
    });
  }
  if (typeof IntersectionObserver !== "function") return;
  var observer = new IntersectionObserver(function (entries) {
    entries.forEach(function (entry) {
      if (entry.isIntersecting) activate(entry.target.id);
    });
  }, { rootMargin: "-16px 0px -70% 0px", threshold: 0 });
  byId.forEach(function (_link, id) {
    var el = document.getElementById(id);
    if (el) observer.observe(el);
  });
})();
""".strip()


def parse_sections(md_text: str) -> list[tuple[str, str]]:
    found = PAIR_RE.findall(md_text)
    ids = [item[0] for item in found]
    if len(found) != 11 or len(set(ids)) != 11:
        sys.stderr.write(
            f"expected 11 distinct <a id> + blank line + ## pairs, found {len(found)}\n"
        )
        sys.exit(1)
    return [(sid, title.strip()) for sid, title in found]


def pandoc_fragment(md_path: Path) -> str:
    cmd = [
        "pandoc",
        "-f",
        "gfm+tex_math_dollars+raw_html",
        "--mathml",
        "--no-highlight",
        "-t",
        "html",
        str(md_path),
    ]
    try:
        proc = subprocess.run(cmd, check=False, capture_output=True)
    except FileNotFoundError:
        sys.stderr.write("pandoc not found\n")
        sys.exit(1)
    if proc.returncode != 0:
        sys.stderr.buffer.write(proc.stderr)
        sys.exit(proc.returncode)
    fragment = proc.stdout.decode("utf-8").replace(MATHML_XMLNS, "")
    if "http://" in fragment or "https://" in fragment:
        sys.stderr.write("pandoc fragment contains a URL\n")
        sys.exit(1)
    return fragment


def render(sections: list[tuple[str, str]], fragment: str) -> str:
    links = "\n".join(
        f'<a href="#{html.escape(sid, quote=True)}">{html.escape(title)}</a>'
        for sid, title in sections
    )
    return (
        "<!DOCTYPE html>\n"
        '<html lang="zh-CN">\n'
        "<head>\n"
        '<meta charset="utf-8">\n'
        f"<title>{html.escape(TITLE)}</title>\n"
        "<style>\n"
        f"{CSS}\n"
        "</style>\n"
        "</head>\n"
        "<body>\n"
        '<nav class="toc">\n'
        f"{links}\n"
        "</nav>\n"
        "<main>\n"
        f"{fragment}"
        "</main>\n"
        "<script>\n"
        f"{JS}\n"
        "</script>\n"
        "</body>\n"
        "</html>\n"
    )


def main() -> None:
    md_path = ROOT / MD_NAME
    html_path = md_path.with_suffix(".html")
    sections = parse_sections(md_path.read_text(encoding="utf-8"))
    document = render(sections, pandoc_fragment(md_path))
    data = document.encode("utf-8")
    html_path.write_bytes(data)
    print(html_path)
    print(len(data))


if __name__ == "__main__":
    main()
