# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys
from sphinx.ext import todo

sys.path.insert(0, os.path.abspath("../../src"))
print("Current sys.path:", sys.path)

project = 'Embedded Control Lab'
copyright = '2024, Universität Siegen'
author = 'Utkarsh Raj'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.coverage",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "myst_parser",
    "sphinx.ext.todo",
]

todo_include_todos = True

myst_url_schemes = [
    "http",
    "https",
]  # Ensures that standard web protocols are recognized

myst_enable_extensions = [
    "dollarmath",  # Enables dollar symbol for inline math
    "amsmath",  # Allows more complex math formatting
]
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

templates_path = ['_templates']
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ['_static']
html_favicon = "_static/unisiegen.png"

# Function to convert absolute paths to relative paths
# Mock third-party modules that are not available during the build
autodoc_mock_imports = [
    "coppeliasim_zmqremoteapi_client",
    "cv2",
]


def convert_todo_path(app, doctree, fromdocname):
    # Iterate over all nodes in the document tree
    for node in doctree.traverse(todo.todo_node):
        source_file = node.source
        if source_file:
            # Convert the absolute path to a relative path
            rel_path = os.path.relpath(source_file, start=app.srcdir)
            # Update the node's source attribute to the relative path
            node.source = rel_path


# Add the above function to the Sphinx setup
def setup(app):
    app.connect("doctree-resolved", convert_todo_path)
