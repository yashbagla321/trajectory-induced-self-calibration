from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "figures"
TEX = OUT_DIR / "acc_geometry.tex"
PDF = OUT_DIR / "acc_geometry.pdf"


TIKZ_SOURCE = r"""\documentclass[tikz,border=2pt]{standalone}
\usepackage{amsmath}
\usetikzlibrary{arrows.meta,calc}

\begin{document}
\begin{tikzpicture}[
  x=1cm,
  y=1cm,
  >=Latex,
  point/.style={circle,fill=#1,draw=#1,inner sep=1.9pt},
  packet/.style={line width=1.25pt,-Latex},
  note/.style={font=\scriptsize,text=black!65},
  every node/.style={font=\small}
]
\definecolor{vehicleA}{RGB}{37,99,235}
\definecolor{vehicleB}{RGB}{29,78,216}
\definecolor{targetG}{RGB}{22,163,74}

\draw[rounded corners=3pt, fill=white, draw=black!15] (-0.20,-0.12) rectangle (7.20,3.45);

\coordinate (qa) at (0.75,2.78);
\coordinate (qb) at (2.05,0.48);
\coordinate (x) at (3.55,1.48);
\coordinate (p) at (6.55,2.72);

\draw[gray!70, dashed, -Latex, line width=0.85pt]
  (qa) -- node[pos=0.54,left=2pt,align=center,note] {vehicle motion\\[-1pt]$q_b-q_a$} (qb);

\draw[packet, vehicleA] (x) -- node[above, sloped, text=vehicleA] {$R(\psi)\ell_a^v$} (qa);
\draw[packet, vehicleB] (x) -- node[below, sloped, text=vehicleB] {$R(\psi)\ell_b^v$} (qb);
\draw[packet, targetG] (x) -- node[above, sloped, text=targetG] {$R(\psi)\ell^{t,*}$} (p);

\draw[gray!55, dashed, line width=0.55pt] (x) -- ++(1.02,0);
\draw[-Latex, black!75, line width=0.75pt] (x) -- ++(0.88,-0.32);
\draw[-Latex, black!75, line width=0.75pt] (x) -- ++(0.32,0.88);
\draw[-Latex, black!75, line width=0.65pt] ($(x)+(0.62,0)$) arc[start angle=0,end angle=-20,radius=0.62];
\node[black!80] at ($(x)+(0.82,-0.16)$) {$\psi$};
\node[anchor=west,align=left,note] at ($(x)+(0.82,-0.66)$) {unknown\\beacon frame};

\node[point=vehicleA,label={[text=vehicleA]above left:$q_a$}] at (qa) {};
\node[point=vehicleB,label={[text=vehicleB]below:$q_b$}] at (qb) {};
\node[point=black,label={[text=black]below left:$x,\psi$}] at (x) {};
\node[point=targetG,label={[text=targetG]right:$p$}] at (p) {};
\end{tikzpicture}
\end{document}
"""


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    TEX.write_text(TIKZ_SOURCE, encoding="ascii")
    subprocess.run(
        ["pdflatex", "-interaction=nonstopmode", TEX.name],
        cwd=OUT_DIR,
        check=True,
    )
    print(PDF)


if __name__ == "__main__":
    main()
