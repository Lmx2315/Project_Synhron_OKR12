rbf2svf.exe %1 1.svf
IF EXIST 1.svf svf2pof.exe 1.svf %2
IF EXIST %2 del 1.svf
:EXIT