set multiplot layout 5,1  #multiplotの開始、縦3横1自動配置
set grid 

plot filename u 1:2 w l,"" u 1:3 w l, "" u 1:4 w l,"" u 1:5 w l
plot filename u 1:($9*180/pi) w l,"" u 1:($10*180/pi) w l, "" u 1:($11*180/pi) w l
plot filename u 1:14 w l,"" u 1:15 w l, "" u 1:16 w l
plot filename u 1:($17-$6) w l,"" u 1:($18-$7) w l, "" u 1:($19-$8) w l
plot filename u 1:20 w l,"" u 1:21 w l, "" u 1:22 w l

unset multiplot           #multiplotの終了
