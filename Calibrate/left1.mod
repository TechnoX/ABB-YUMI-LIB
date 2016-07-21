MODULE MainModule

    CONST speeddata vSpeed := v200;
    VAR intnum irLogoIsMoving;
    PERS robtarget pLogoTarget:=[[395.833,137.02,263.213],[0.998195,0.0195428,-0.0567784,0.000971802],[-1,-1,2,11],[127.878,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    PROC main()
        !MoveAbsJ [[-96.8411,-120.914,31.206,-41.7261,99.4567,187.525],[93.9614,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, z50, tool0;
        SetDO doLogoIsMoving,0;
        CONNECT irLogoIsMoving WITH ShouldMove;
        ISignalDO doLogoIsMoving,high,irLogoIsMoving;
        
        WaitTime 1000;
    ENDPROC
    
    TRAP ShouldMove
        ! This is not used anywhere, just to throw error if position is not reachable. The error thrown by MoveL didn't seem catchable
        !jtDummy := CalcJointT(pTarget, tool0, \WObj:=wobj0);
        
        MoveL pLogoTarget, vSpeed, fine, tool0\WObj:=wobj0;
        
        ! Give execution back to other thread
        SetDO doLogoIsMoving, 0;
    ENDTRAP
ENDMODULE