MODULE MainModule

    ! Coordinate system for the ABB Marker. With the orientation of the system as the corresponding camera (to simplify some calculations)
    CONST pose peLogoWRTWrist:=[[0,-35,30],[0.5,-0.5,-0.5,-0.5]];
    
    PERS bool bSync:=TRUE;
    PERS pose peLogoEnd:=[[-100,-100,200],[0.364705,0.279848,0.115917,-0.880476]];
    PERS robtarget pCamera;
    PERS wobjdata wCamera :=[ FALSE, TRUE, "", [ [400, -200, 300], [0.5, -0.5, -0.5 ,-0.5] ], [ [0, 0, 0], [1, 0, 0 ,0] ] ];
    
    VAR intnum irLogoIsMoving;
    
    PROC main()
        CONNECT irLogoIsMoving WITH ShouldMove;
        ISignalDO doLogoIsMoving,high,irLogoIsMoving;
        
        !VAR robtarget pCenterPoint;
        ! The position to place the ABB marker in base coordinate frame
        wCamera.uframe.trans := pCamera.trans;
        wCamera.uframe.rot := pCamera.rot;
        wCamera.oframe := [[0,0,0],[1,0,0,0]];!peCameraWRTWrist;
        
        bSync:=TRUE;
        tool_YuMiGripper_S_C.tFrame:=peLogoWRTWrist;
        MoveAbsJ [[-80,-75,10,-45,120,-130],[80,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, z50, tool_YuMiGripper_S_C\WObj:=wCamera;
    
        WaitTime 1000;
    ENDPROC
    
    TRAP ShouldMove
        VAR robtarget pCurrentGoal := [[0,0,0],[1,0,0,0],[-1,-1,-2,11],[170,9E+09,9E+09,9E+09,9E+09,9E+09]];
        pCurrentGoal.rot := peLogoEnd.rot;
        pCurrentGoal.trans := peLogoEnd.trans;
        MoveL pCurrentGoal, v50, fine, tool_YuMiGripper_S_C\WObj:=wCamera;
    
        ! Give execution back to other thread
        SetDO doLogoIsMoving,0;
    ENDTRAP
ENDMODULE