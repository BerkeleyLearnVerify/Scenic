.PHONY: format
format:
	poetry run black .

.PHONY: format-check
format-check:
	poetry run black . --check
