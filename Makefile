GRAMMAR = ./grammar/scenic.gram
PARSER = ./src/scenic/parser/parser.py

all: $(PARSER)

$(PARSER): $(GRAMMAR)
	poetry run python -m pegen $(GRAMMAR) -o $(PARSER)

clean:
	-rm $(PARSER)
