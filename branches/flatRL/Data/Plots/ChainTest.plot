set autoscale;
set terminal png;
set output @OUTPUT;
set title @TITLE;
plot @DATA0 using 1:2 title 'PD' with lines, @DATA0 using 1:3 title 'PD GC' with lines, @DATA0 using 1:4 title 'Velocity driven PD' with lines;