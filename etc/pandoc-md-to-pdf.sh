#!/bin/bash
# Script to convert a directory of .md files to pdf using pandoc.
# Based on code by: https://gist.github.com/hugorodgerbrown/5317616
#
# 1. Install pandoc from http://johnmacfarlane.net/pandoc/
# 2. Copy this script and listings-setup.tex into the directory containing the .md files
# 3. Ensure that the script has execute permissions
# 4. Run the script
#
# Uses a listings-setup.tex file based on an answer found here:
# https://tex.stackexchange.com/questions/323329/pandoc-code-blocks-in-markdown-with-very-long-lines-get-cut-off-when-outputting


FILES=*.md

for f in $FILES
do
	filename="${f%.*}"	
	echo "Converting $f to $filename.pdf"	
	pandoc --listings -H listings-setup.tex -f markdown_github -o $filename.pdf $f --latex-engine=xelatex
done
