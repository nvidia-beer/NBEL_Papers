#!/bin/bash

echo "Cleaning up old generated files..."
rm -f NBEL_Papers_Complete.pdf NBEL_Papers_Complete.tex temp_*.md *.aux *.log *.out *.toc

echo "Preparing markdown files with page breaks and image fixes..."

# Process ALL files with page break at the start (including README)
# NOTE: NEF.md is now integrated into README.md, so we exclude it
for file in README.md papers/2021.md papers/2022.md papers/2023.md papers/2024.md papers/Control.md papers/KBM.md papers/PES.md; do
    basename=$(basename "$file" .md)
    
    # Add clearpage at the beginning, then convert HTML img tags
    {
        echo ""
        echo "\\clearpage"
        echo ""
        sed 's|<img src="\([^"]*\)" alt="\([^"]*\)" width="\([0-9]*\)"[^>]*>|![\2](\1){width=0.75\\textwidth}|g' "$file"
    } > "temp_${basename}.md"
done

echo "Generating LaTeX file..."

# Generate LaTeX first (not PDF) - excluding NEF.md since it's now in README
pandoc \
  temp_README.md \
  temp_2021.md \
  temp_2022.md \
  temp_2023.md \
  temp_2024.md \
  temp_Control.md \
  temp_KBM.md \
  temp_PES.md \
  -o NBEL_Papers_Complete.tex \
  -t latex \
  -s \
  --include-in-header=preamble.tex \
  --toc \
  --toc-depth=3 \
  --number-sections \
  --resource-path=.:papers:figures \
  --highlight-style=tango \
  -V documentclass=article \
  -V fontsize=11pt \
  -V papersize=letter \
  -V title="Neuromorphic Brain Engineering Lab (NBEL)\\\\2021--2024 Research Papers" \
  -V author="Ilan Beer" \
  -V date="2025" \
  -V geometry:margin=1in

echo "Fixing LaTeX issues..."

# Fix dollar signs and image sizes
sed -i 's/\\$ /$/g; s/ \\$/ $/g; s/\\$\\$/$/g' NBEL_Papers_Complete.tex
sed -i 's/width=[0-9.]*in,height=\\textheight/width=0.75\\textwidth/g' NBEL_Papers_Complete.tex
sed -i 's|\\includegraphics{../figures|\\includegraphics[width=0.7\\textwidth]{../figures|g' NBEL_Papers_Complete.tex

echo "Compiling PDF (pass 1/3)..."
pdflatex -interaction=nonstopmode NBEL_Papers_Complete.tex > /dev/null 2>&1

echo "Compiling PDF (pass 2/3)..."
pdflatex -interaction=nonstopmode NBEL_Papers_Complete.tex > /dev/null 2>&1

echo "Compiling PDF (pass 3/3)..."
pdflatex -interaction=nonstopmode NBEL_Papers_Complete.tex > /dev/null 2>&1

# Check result
if [ -f "NBEL_Papers_Complete.pdf" ]; then
    echo ""
    echo "✓ PDF generated successfully!"
    ls -lh NBEL_Papers_Complete.pdf
    echo ""
    echo "📄 DOCUMENT STRUCTURE:"
    echo "  ✓ README (Introduction + NEF Framework)"
    echo "  ✓ 2021 Paper"
    echo "  ✓ 2022 Paper"
    echo "  ✓ 2023 Paper"
    echo "  ✓ 2024 Paper"
    echo "  ✓ Control Systems"
    echo "  ✓ Kinematic Bicycle Model"
    echo "  ✓ Prescribed Error Sensitivity"
    echo ""
    echo "📝 NEF content is now integrated in the README introduction"
    echo ""
    
    # Cleanup
    rm -f temp_*.md *.aux *.log *.out *.toc
    echo "✓ Cleanup complete"
else
    echo ""
    echo "✗ PDF generation failed - check errors"
    tail -50 NBEL_Papers_Complete.log
fi
