# LaTeX Project Template
A clean and reproducible LaTeX project template for local development using **VSCode** and **latexmk**.

This template is designed to:

- Keep the project root clean (only `main.tex`, `main.pdf`, configs, and README)
- Separate build artifacts into the `build/` directory
- Support modular document structure (`sections/`, `packages/`, `asset/`)
- Work consistently across machines and team members

## Features

- 📁 Clean project structure
- 🧱 Modular sections (`sections/`)
- 🎨 Custom styles and macros (`packages/`)
- 📦 External templates isolated (`asset/`)
- 🗂 Build artifacts separated (`build/`)
- 🧪 Reproducible builds with `latexmk`
- 🧑‍💻 VSCode integration (LaTeX Workshop)

## Project Structure

```
root/
├─ main.tex
├─ main.pdf
├─ README.md
├─ .gitignore
├─ .latexmkrc
├─ .vscode/
│ └─ settings.json
├─ build/
├─ sections/
├─ figures/
├─ bibliography/
│ └─ reference.bib
├─ packages/
│ └─ general.sty
└─ asset/
```

## Requirements

- TeX distribution:
  - TeX Live (recommended) or MiKTeX
- VSCode
- VSCode extension:
  - **LaTeX Workshop**

## Build Instructions

### Using VSCode

1. Open the project folder in VSCode
2. Install **LaTeX Workshop**
3. Press: Ctrl + Alt + B

or click **Build LaTeX project**

Output:

- Final PDF → `main.pdf`
- Build files → `build/`

### Using command line

``` bash
latexmk
```

## Bibliography

- BibTeX file: bibliography/reference.bib
- Optional:
    store reference PDFs in: bibliography/pdf/

## Customization

### Common styles
Edit:

```
packages/general.sty
```

for:

- Package loading
- Macros
- Formatting rules
- Hyperlinks
- Captions

### Sections
Add or edit chapters in:

```
sections/
```

and include them in `main.tex`.

### External templates
Store external templates (IEEE, ACM, etc.) in:

```
asset/
```

Do not modify them directly.  
Apply customizations in `packages/`.

## Git Notes
Build artifacts are ignored:

```
build/
*.aux
*.log
*.out
*.bbl
*.blg
*.fdb_latexmk
*.fls
*.synctex.gz
```

## Author
Beomjun Chung (@wjdqjawns)

## License
MIT License

## Notes
This template is suitable for:

- Research reports
- Academic papers
- Project documentation
- Team collaboration
- Personal technical writing