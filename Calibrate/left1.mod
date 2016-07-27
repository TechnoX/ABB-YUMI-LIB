MODULE MainModule

    CONST speeddata vSpeed := v200;
    VAR intnum irLogoIsMoving;
    PERS robtarget pLogoTarget:=[[428.802,107.913,336.046],[0.969237,0.186976,0.159937,-0.0062316],[-1,-1,-2,11],[157.51,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    PROC main()
        MoveAbsJ [[-78.7476,-84.8618,33.6877,-36.8878,98.6226,-121.75],[80.7304,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, z50, tool0;
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