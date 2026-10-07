# Resume, CV and portfolio

| File | What it is |
|---|---|
| [CV-VoHongQuan.pdf](CV-VoHongQuan.pdf) | One-page resume, the file the website links to |
| [Portfolio-VoHongQuan.pdf](Portfolio-VoHongQuan.pdf) | 5-page technical portfolio with three case studies |
| [word/](word/) | Editable Word versions: `CV_VoHongQuan.docx`, `Template-A.docx` and `Template-B.docx` (MIT CAPD layouts A and B), `Portfolio-VoHongQuan.docx` |
| [latex/](latex/) | Five resume layouts plus the portfolio; shared project facts live in [project-data.tex](latex/project-data.tex) |
| [MIT_GUIDE.md](MIT_GUIDE.md) | MIT CAPD advice these documents follow, and what is still to do |

## LaTeX layouts

| Template | Look | ATS-safe (MIT CAPD) |
|---|---|---|
| `jake.tex` | [Jake's Resume](https://github.com/jakegut/resume), one column | yes |
| `mit-a.tex` | MIT CAPD Template A (Times, ruled titles) | yes |
| `mit-b.tex` | MIT CAPD Template B (sans serif, skills first) | yes |
| `moderncv.tex` | moderncv classic, coloured titles, date column | no (date column) |
| `altacv.tex` | AltaCV two columns with a sidebar | no (columns, icons) |

```bash
cd latex
make            # every template -> pdf/<name>.pdf (1 page) and pdf/<name>-cv.pdf (long: extra project bullets)
make jake       # one template
make cv         # canonical CV -> ../CV-VoHongQuan.pdf and ../word/CV-VoHongQuan.pdf
make portfolio  # canonical portfolio -> ../Portfolio-VoHongQuan.pdf
make clean
```

Needs TeX Live with `latexmk`. Build files go to `latex/build/` (ignored by git).
`altacv.cls` is by LianTze Lim (LaTeX Project Public License); `latex/samples/roberto-altacv/` is the AltaCV sample it came from.
