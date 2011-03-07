set autoscale;
set terminal png;
set output @OUTPUT;
set title @TITLE;
plot @DATA0 using 1:2 title 'PD' with lines;