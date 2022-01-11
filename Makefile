all: release

clean:
	@echo Cleaning...
	-rm -r ./build
	-rm -r ./dist

build_wheel:
	python3 setup.py sdist bdist_wheel

release: build_wheel

upload:
	@echo Uploading to PyPi...
	twine upload dist/*
