#!/bin/sh

filename=orb_slam2_notes
ext=pdf
hl_style=tango # {pygments, tango, espresso, zenburn, kate, monochrome, breezedark, haddock}

pandoc -s -f markdown+tex_math_single_backslash -t latex main.md chapters/*.md --pdf-engine=xelatex --highlight-style=$hl_style --filter pandoc-citeproc -o $filename.$ext

#cp $filename.$ext ~/sf_D_DRIVE

## sync to windows
rsync -avh ../orb_slam2_notes ~/Workspace/cygwinWorkspace/markdown
