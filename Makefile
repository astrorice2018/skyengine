MODULE=skyengine


bootstrap:
	pip install -r requirements.txt

lint:
	flake8 skyengine

test:
	python -m unittest discover -s test -v

docs:
	sphinx-apidoc -o docs ${MODULE}
	mv docs/${MODULE}.rst docs/index.rst
	PYTHONPATH=. sphinx-build -c docs docs docs/build

.PHONY: bootstrap lint test docs
