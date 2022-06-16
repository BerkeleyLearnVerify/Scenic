GRAMMAR = ./grammar/scenic.gram
PARSER = ./src/scenic/parser/parser.py

.PHONY: all
all: $(PARSER)

$(PARSER): $(GRAMMAR)
	poetry run python -m pegen $(GRAMMAR) -o $(PARSER)

.PHONY: clean
clean:
	-rm $(PARSER)
