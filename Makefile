GRAMMAR = ./src/scenic/syntax/scenic.gram
PARSER = ./src/scenic/syntax/parser.py

.PHONY: all
all: $(PARSER)

$(PARSER): $(GRAMMAR)
	poetry run python -m pegen $(GRAMMAR) -o $(PARSER)

.PHONY: clean
clean:
	-rm $(PARSER)
