---
### YAML metadata block ###
title: ORB-SLAM2 Notes
author: Charlie Li
date: 2019.05.05
# set document properties
papersize: a4
documentclass: article
geometry: "left=2cm, right=2cm, top=2cm, bottom=2cm"
linestretch: 1.25
fontsize: 11pt
# classoption: twocolumn # for 2 columns rendering
# generate table of contents with customized depth
toc: true
toc-title: Table of Contents
toc-depth: 10
colorlinks: true
# bibliography configurations
bibliography: main.bib
# additional csl file for bib formatting
#csl: acm-siggraph.csl 
link-citations: true
# font specification for XeLaTeX (default: Latin Modern family)
#mainfont: TeX Gyre Schola
#mathfont: TeX Gyre Schola Math
# include LaTex packages & macros
header-includes:
- |
  ```{=latex}
  % math library
  \usepackage{mathtools}
  
  % for subsections with more depths
  \usepackage{enumitem}
  \setlistdepth{20}
  \renewlist{itemize}{itemize}{20}
  \setlist[itemize]{label=$\cdot$}
  
  % latex macros
  % \V: boldface characters for vector (use \symbfup instead of 
  %     \mathbf as the latter does not fit with unicode-math package)
  \def\V#1{{\symbfup{#1}}} 
  \def\BG#1{{\symbf{#1}}} % \BG: bold greek letters
  \def\F#1{{\mathbb{#1}}} % \F: blackboard bold for fields/special sets
  \newcommand{\argmax}{\operatornamewithlimits{arg\,max}} % argmax
  \newcommand{\argmin}{\operatornamewithlimits{arg\,min}} % argmin
  
  % add internal link to "top"
  \hypertarget{top} {}
  \label{top}
  
  % add bookmarks
  \usepackage{bookmark}
  \pdfbookmark[section]{Title Page}{title}
  
  % vector graphic (TikZ)
  \usepackage{tikz}
  ```
include-before:
- |
  ```{=latex}
  % add bookmark to toc
  \pdfbookmark[section]{Table of Contents}{toc}
  ```
---

\newpage
