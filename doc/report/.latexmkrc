# .latexmkrc
$out_dir = '.';
$aux_dir = 'build';

$pdf_mode = 1;  # pdflatex

# if do
if (!-d 'build') { mkdir 'build'; }