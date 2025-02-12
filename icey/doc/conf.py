import subprocess, os

project = 'icey'
copyright = '2025, Ivo Ivanov'
author = 'Ivo Ivanov'
release = '0.1.0'


extensions = [
    "breathe",
    "myst_parser",
    "sphinx_copybutton",
    "sphinxcontrib.spelling",
]

breathe_projects = {"icey": ".build/doxygenxml/"}
breathe_default_project = "icey"
breathe_domain_by_extension = {"hpp": "cpp"}
breathe_default_members = ('members', 'undoc-members')

templates_path = ['_templates']
exclude_patterns = []

subprocess.call('doxygen Doxyfile', shell=True)

highlight_language = 'c++'

html_title = "ICEY Documentation"
html_theme = 'sphinx_book_theme'
html_static_path = ['_static']
html_css_files = ["icey.css", "book_custom.css"]
html_context = {"default_mode": "light"}

html_theme_options = {
   "pygments_light_style": "tango",
   "pygments_dark_style": "github-dark"
}
