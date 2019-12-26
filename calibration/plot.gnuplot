set view equal xyz

set xtics axis nomirror
set ytics axis nomirror
set ztics axis nomirror

set border 0
set xyplane at 0
set xzeroaxis lt -1
set yzeroaxis lt -1
set zzeroaxis lt -1

splot \
    'mag.txt' using 1:2:3 with points pointsize 1 pointtype 7 lc rgb "red",\
    'mag_cal.txt' using 1:2:3 with points pointsize 1 pointtype 7 lc rgb "green"

pause -1
