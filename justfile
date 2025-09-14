init:
    poetry install

build:
    poetry run mkdocs build

serve:
    poetry run mkdocs serve

clean:
    rm -rf site
