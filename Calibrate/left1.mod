MODULE MainModule

    CONST speeddata vSpeed := v200;
    VAR intnum irLogoIsMoving;
    PERS robtarget pLogoTarget:=[[332.286,13.2479,238.699],[0.85446,-0.368638,0.0880639,-0.355315],[-2,-1,2,11],[131.783,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    PROC main()
        MoveAbsJ [[-96.8411,-120.914,31.206,-41.7261,99.4567,187.525],[93.9614,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, z50, tool0;
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