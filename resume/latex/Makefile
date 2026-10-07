# Builds every template twice into pdf/:
#   <name>.pdf     one-page resume (MIT resume rules)
#   <name>-cv.pdf  long version with the extra project bullets (\def\LONG{1})
# Needs TeX Live with latexmk (pdflatex).  `make` builds all, `make jake` builds one pair.
TEMPLATES := jake mit-a mit-b moderncv altacv
LATEXMK   := latexmk -pdf -interaction=nonstopmode -halt-on-error -outdir=build

.PHONY: all clean cv portfolio $(TEMPLATES)
all: $(TEMPLATES) cv portfolio

$(TEMPLATES): %: pdf/%.pdf pdf/%-cv.pdf

pdf/%.pdf: %.tex content.tex project-data.tex
	@mkdir -p pdf build
	$(LATEXMK) $<
	cp build/$*.pdf $@

pdf/%-cv.pdf: %.tex content.tex project-data.tex
	@mkdir -p pdf build
	$(LATEXMK) -jobname=$*-cv -pretex='\def\LONG{1}' -usepretex $<
	cp build/$*-cv.pdf $@

cv: pdf/altacv.pdf
	cp $< ../CV-VoHongQuan.pdf
	cp $< ../word/CV-VoHongQuan.pdf
	cp $< pdf/CV-VoHongQuan.pdf

portfolio: ../Portfolio-VoHongQuan.pdf

../Portfolio-VoHongQuan.pdf: portfolio.tex project-data.tex assets/portfolio/crazyflie.png assets/portfolio/balancing-robot.jpg assets/portfolio/rotary-pendulum.jpg assets/portfolio/portfolio-qr.png
	@mkdir -p build
	$(LATEXMK) portfolio.tex
	cp build/portfolio.pdf $@

clean:
	rm -rf build
