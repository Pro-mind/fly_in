PYTHON = python3
MAP ?= map.txt

.PHONY: install run debug clean lint lint-strict

install:
	$(PYTHON) -m pip install flake8 mypy pygame

run:
	$(PYTHON) main.py $(MAP)

debug:
	$(PYTHON) -m pdb main.py $(MAP)

clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null
	find . -type d -name .mypy_cache -exec rm -rf {} + 2>/dev/null
	find . -name "*.pyc" -delete 2>/dev/null


lint:
	flake8 .
	mypy . \
		--warn-return-any \
		--warn-unused-ignores \
		--ignore-missing-imports \
		--disallow-untyped-defs \
		--check-untyped-defs


lint-strict:
	flake8 .
	mypy . --strict
