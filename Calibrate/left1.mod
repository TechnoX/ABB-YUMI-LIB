MODULE MainModule

    CONST speeddata vSpeed := v200;
    VAR intnum irLogoIsMoving;
    PERS robtarget pLogoTarget:=[[417.118,160.257,270.536],[0.999792,0.00422953,-0.011057,-0.0166347],[-2,-3,0,1010],[133.733,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    PROC main()
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