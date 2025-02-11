# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'icey'
copyright = '2025, Ivo Ivanov'
author = 'Ivo Ivanov'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "breathe",
]

breathe_projects = {"icey": ".build/doxygenxml/"}
breathe_default_project = "icey"
breathe_domain_by_extension = {"hpp": "cpp"}

templates_path = ['_templates']
exclude_patterns = []

import subprocess, os

# Doxygen
subprocess.call('doxygen Doxyfile', shell=True)

highlight_language = 'c++'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'furo'
html_static_path = ['_static']
