MODULE MainModule
    
    RECORD pixel
        num u;
        num v;
    ENDRECORD
    
    PERS robtarget pCamera:=[[400,-200,300],[0.5,-0.5,-0.5,-0.5],[0,0,1,11],[-170,9E+09,9E+09,9E+09,9E+09,9E+09]];

    PERS bool bSync:=TRUE;
    PERS pose peLogoEnd;
    
    ! Coordinate system in the camera center. With Z out of image center, and X & Y aligned to pixel coord frame
    CONST pose peCameraWRTWrist:=[[-10,30,35],[0.5,-0.5,-0.5,-0.5]];
    
    
    PROC main()
        bSync:=TRUE;
        tool_YuMiGripper_S_C.tFrame:=peCameraWRTWrist;

        ! Moves to a initial position with correct configuration. 
        MoveAbsJ [[80,-75,10,45,120,130],[-80,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, fine, tool_YuMiGripper_S_C;
        
        
        MoveL pCamera, v50, fine, tool_YuMiGripper_S_C;
        
        !moveLogoToPose [[0,0,100],[1,0,0,0]];
        moveLogoToPos [-100,-100,200];
        moveLogoToPos [100,-100,200];
        moveLogoToPos [100,100,200];
        moveLogoToPos [-100,100,200];
        Stop;
	ENDPROC
    
    
    ! Rotates the logo automatically toward the camera, so the Z axis of the logo will point toward the camera
    PROC moveLogoToPos(pos psPosition)
        ! The vector from the camera to this position is just psPosition; in camera centered coords
        VAR polar angles;
        VAR pose pePosition;
        angles := pos2polar(psPosition);
        pePosition.trans := psPosition;
        pePosition.rot := OrientZYX(angles.phi, angles.theta, 0);
        moveLogoToPose(pePosition);
    ENDPROC
    
    
    
    ! Moves the logo to the position given in pPosition, in the coordinate system given by the camera (Z is out of image center). 
    PROC moveLogoToPose(pose pePosition)
        peLogoEnd:=pePosition;
        SetDO doLogoIsMoving,1;
        WaitDO doLogoIsMoving,0;
    ENDPROC
ENDMODULE