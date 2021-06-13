for f in *.txt; do
    file=$(echo $f | cut -d'.' -f 1)
    echo "set terminal png
set output '$file.png'
set xlabel 'chunk\_size'
set ylabel 'Time (ms)'
plot '$f' using 1:2 title 'PathCache' with lines, '$f' using 1:3 title 'A*' with lines" | gnuplot
done