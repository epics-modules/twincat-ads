#
#    This file is part of twincat-ads.
#
#    twincat-ads is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
#
#    twincat-ads is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License along with twincat-ads. If not, see <https://www.gnu.org/licenses/>.
#
# Configuration file for the Sphinx documentation builder.

import datetime
import pathlib
import re

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

project = "EPICS TwinCAT-ADS"
author = "epics-modules"
copyright = f"{datetime.date.today().year}, {author}"


def _latest_release() -> str:
    """Read the most recent version from RELEASE.md."""
    release_notes = REPO_ROOT / "RELEASE.md"
    match = re.search(r"^## Release (v[\d.]+)", release_notes.read_text(), re.M)
    return match.group(1) if match else "unreleased"


release = _latest_release()
version = release

extensions = [
    "myst_parser",
    "sphinx_copybutton",
    "sphinx_design",
]

source_suffix = {
    ".md": "markdown",
    ".rst": "restructuredtext",
}

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "linkify",
    "substitution",
]
myst_heading_anchors = 3

html_theme = "sphinx_rtd_theme"
html_title = f"{project} documentation"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_theme_options = {
    "navigation_depth": 3,
    "collapse_navigation": False,
    "style_external_links": True,
}

html_context = {
    "display_github": True,
    "github_user": "epics-modules",
    "github_repo": "twincat-ads",
    "github_version": "master",
    "conf_py_path": "/docs/",
}

# Files that live outside docs/ and are pulled in with MyST `include`.
myst_substitutions = {
    "repo_url": "https://github.com/epics-modules/twincat-ads",
}
