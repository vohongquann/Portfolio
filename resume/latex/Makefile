# Builds every template twice into pdf/:
#   <name>.pdf     one-page resume (MIT resume rules)
#   <name>-cv.pdf  long version with the extra project bullets (\def\LONG{1})
# Needs TeX Live with latexmk (pdflatex).  `make` builds all, `make jake` builds one pair.
TEMPLATES := jake mit-a mit-b moderncv altacv
LATEXMK   := latexmk -pdf -interaction=nonstopmode -halt-on-error -outdir=build

.PHONY: all clean $(TEMPLATES)
all: $(TEMPLATES)

$(TEMPLATES): %: pdf/%.pdf pdf/%-cv.pdf

pdf/%.pdf: %.tex content.tex
	@mkdir -p pdf build
	$(LATEXMK) $<
	cp build/$*.pdf $@

pdf/%-cv.pdf: %.tex content.tex
	@mkdir -p pdf build
	$(LATEXMK) -jobname=$*-cv -pretex='\def\LONG{1}' -usepretex $<
	cp build/$*-cv.pdf $@

clean:
	rm -rf build
