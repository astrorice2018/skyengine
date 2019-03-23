import os
import sys

import pkg_resources
import sphinx_rtd_theme


PROJECT_NAME = 'skyengine'

sys.path.insert(0, os.path.abspath('.'))

extensions = ['sphinx.ext.autodoc']
templates_path = ['_templates']
source_suffix = '.rst'
master_doc = 'index'
project = PROJECT_NAME
copyright = u'2017, Skynet Engineering'
author = u'Skynet Engineering'
version = pkg_resources.get_distribution(PROJECT_NAME).version
release = pkg_resources.get_distribution(PROJECT_NAME).version
language = None
exclude_patterns = ['env']
pygments_style = 'sphinx'
todo_include_todos = False
html_theme = 'sphinx_rtd_theme'
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]
html_static_path = ['_static']
html_sidebars = {
    '**': [
        'relations.html',  # needs 'show_related': True theme option to display
        'searchbox.html',
    ]
}
htmlhelp_basename = '{}doc'.format(PROJECT_NAME)
