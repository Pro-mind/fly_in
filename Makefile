PYTHON = python3
MAP ?= map.txt

.PHONY: install run debug clean lint lint-strict

## Install project dependencies
install:
	pip install flake8 mypy --break-system-packages pygame

## Run the simulation (use MAP=<path> to specify a map file)
run:
	$(PYTHON) main.py $(MAP)

## Run in debug mode using pdb
debug:
	$(PYTHON) -m pdb main.py $(MAP)

## Remove caches and temporary files
clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name .mypy_cache -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true

## Run flake8 and mypy linting
lint:
	flake8 .
	mypy . \
		--warn-return-any \
		--warn-unused-ignores \
		--ignore-missing-imports \
		--disallow-untyped-defs \
		--check-untyped-defs

## Run strict mypy check
lint-strict:
	flake8 .
	mypy . --strict
