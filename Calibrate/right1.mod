MODULE MainModule
    
    RECORD pixel
        num u;
        num v;
        ! If scale is used for a detection it can be saved in the pixel record. Otherwise uninitialized; 
        num scale; 
    ENDRECORD
    ! "Enum" for the directions
    CONST num DIRECTION_X:=0;
    CONST num DIRECTION_Y:=1;
    CONST num DIRECTION_Z:=2;
    
    ! The general speed used during calibration
    CONST speeddata vSpeed := v200;
    VAR cameradev cameraToUse;
    VAR intnum err_int;
        
    PROC main()
        VAR num sak;
        CONNECT err_int WITH err_trap;
        IError COMMON_ERR, TYPE_ALL, err_int;
        
        MotionSup \Off;


        ! Moves to a initial position with correct configuration. 
        MoveAbsJ [[80,-75,10,45,120,130],[-80,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, fine, tool_YuMiGripper_S_C;    
        
        !sak := 412 / 0;
        
        setUpCameraDevice; 
        calibIntrinsicAndRotation;
        Stop;
	ENDPROC
    
    TRAP err_trap
        TPWrite "Buuuuu!";
        !Stop;
    ENDTRAP
    
    
    LOCAL PROC setupCameraDevice()
        VAR string markerFinderJob:="PatternABB.job";
        
        CheckIfVisionExists VisionSystemName, cameraToUse;
        !Get jobs from controller
        RestoreCtrlToCamera cameraToUse,FALSE;
        !Initialize the camera
        InitCamera cameraToUse,markerFinderJob;
    ENDPROC
    
    ! Whole phase A of the paper. 
    LOCAL PROC calibIntrinsicAndRotation()
        ! The m value in the paper
        CONST num nNumPoses:=15;
        VAR pixel pixels{ nNumPoses}; 
        
        getPixelValuesFromPoses nNumPoses, pixels;
    ENDPROC
    
    
    LOCAL PROC getPixelValuesFromPoses(num nNumPoses, INOUT pixel pixels{*})
        VAR pose pePoses{6};
        ! Get poses that is spread inside the view of the camera
        getSpreadPosesInView nNumPoses, pePoses;
        
        ! Move the ABB marker to these positions and get the pixel values from them. 
        FOR i FROM 1 TO nNumPoses DO
            !MoveLogoToPose pePoses{i};
        ENDFOR
        
    ENDPROC

    ! Return a bunch of poses that all is visible in the camera; 
    ! Note: All of them needs to have the same orientation!
    LOCAL PROC getSpreadPosesInView(num nNumPoses, INOUT pose pePoses{*})
        VAR num nTransform{3,3};

        calculateMovementTransform nTransform;
        
        printMatrix nTransform;
        
    ENDPROC
    
    LOCAL PROC printMatrix(num matrix{*,*})
        VAR string stRow;
        FOR r FROM 1 TO DIM(matrix,1) DO
            stRow:="[";
            FOR c FROM 1 TO DIM(matrix,2) DO
                stRow := stRow + NumToStr(matrix{r,c},3);
                IF c+1 <> DIM(matrix,2) THEN
                    stRow := stRow + " ";
                ENDIF
            ENDFOR
            TPWrite stRow;
        ENDFOR
    ENDPROC
    
    
    LOCAL PROC calculateMovementTransform(INOUT num nTransform{*,*})
        VAR num nDistances{3};
        VAR pixel nPixelDistances{3};
        IF DIM(nTransform,1) <> 3 OR DIM(nTransform,1) <> 3 THEN
            TPWrite "The transform must be of size 3x3";
            RAISE ERR_NOTEQDIM;
        ENDIF
        
        ! Moves in X direction of robot-frame and get moved distance in X and pixels. 
        getMaxVisibleDistance DIRECTION_X, nDistances{1}, nPixelDistances{1};
        ! Moves in Y direction of robot-frame and get moved distance in Y and pixels. 
        getMaxVisibleDistance DIRECTION_Y, nDistances{2}, nPixelDistances{2};
        ! Moves in Z direction of robot-frame and get moved distance in Z and pixels. 
        getMaxVisibleDistance DIRECTION_Z, nDistances{3}, nPixelDistances{3};
        
        ! Probably possible to make this to a Matrix multiplication?
        ! But this is at least working. And pretty clear? :)
        FOR c FROM 1 TO 3 DO
            nTransform{1,c} := nPixelDistances{c}.u / nDistances{c};
            nTransform{2,c} := nPixelDistances{c}.v / nDistances{c};
            nTransform{3,c} := nPixelDistances{c}.scale / nDistances{c};
        ENDFOR
    ENDPROC
    
    ! Moves in the arm along the specified axis to the most negative and then most positive to find out the biggest difference in pixel distance
    LOCAL PROC getMaxVisibleDistance(num nDirection, INOUT num traveledDistanceAlongAxis, INOUT pixel pixelDifferences)
        VAR pos psMaxDetectedMarker;
        VAR pos psMinDetectedMarker;
        VAR num nMaxDistance;
        VAR num nMinDistance;
        VAR robtarget pStartPos;
        
        pStartPos := CRobT(\Tool:=tool0,\WObj:=wobj0);
        
        ! Moves in positive direction
        movesFurthestDistance nDirection, 1, psMaxDetectedMarker, nMaxDistance;
        ! Moves directly back to start
        MoveL pStartPos, vSpeed, z50, tool0, \WObj:=wobj0;        
        ! Moves in negative direction
        movesFurthestDistance nDirection, -1, psMinDetectedMarker, nMinDistance;
        ! Moves back to start
        MoveL pStartPos, vSpeed, z50, tool0, \WObj:=wobj0;
        
        ! Returns the differencies 
        traveledDistanceAlongAxis := nMaxDistance - nMinDistance;
        
        IF traveledDistanceAlongAxis=0 THEN
            TPWrite "Couldn't travel along axis "\Num:=nDirection;
            traveledDistanceAlongAxis:=0.1;
        ENDIF
        
        pixelDifferences.u := psMaxDetectedMarker.x - psMinDetectedMarker.x;
        pixelDifferences.v := psMaxDetectedMarker.y - psMinDetectedMarker.y;
        pixelDifferences.scale := psMaxDetectedMarker.z - psMinDetectedMarker.z;
    ERROR
        TPWrite "!!!!!!!Error in getMaxVisibleDistance";
        TRYNEXT;
    ENDPROC
    
    ! Tries to move as long as possible in specified direction, moves until the marker is lost. 
    LOCAL PROC movesFurthestDistance(num nDirection, num nSign, INOUT pos lastDetectedMarker, INOUT num nDistance)
        VAR robtarget pTarget;
        VAR pos markerInfo;
        VAR jointtarget jtDummy;
        VAR num nStepLength := 100;
        nDistance := 0;
        
        ! Moves in specified direction until the marker is lost. 
        WHILE getMarkerInfo(markerInfo) DO
            lastDetectedMarker := markerInfo;
            nDistance := nDistance + nStepLength;
            ! Moves the robot to a new target
            pTarget := getMotionOffset(nStepLength, nDirection,nSign);
            MotionSup \Off;

            
            ! This is not used anywhere, just to throw error if position is not reachable. The error thrown by MoveL didn't seem catchable
            jtDummy := CalcJointT(pTarget, tool0, \WObj:=wobj0);
            MoveL pTarget, vSpeed, fine, tool0,\WObj:=wobj0;
        ENDWHILE
    ERROR
        TPWrite "movesFurthestDistance(): Target not reachable";
        ! It was not possible to reach the new location, it is then safe to just return
        ! It is not a real error
        RETURN;
    ENDPROC
    
    LOCAL FUNC robtarget getMotionOffset(num nDistance, num nDirection, num nSign)
        VAR robtarget pTarget;
        TEST nDirection
        CASE DIRECTION_X:
            pTarget := Offs(CRobT(\Tool:=tool0,\WObj:=wobj0),nSign*nDistance,0,0);
        CASE DIRECTION_Y:
            pTarget := Offs(CRobT(\Tool:=tool0,\WObj:=wobj0),0,nSign*nDistance,0);
        CASE DIRECTION_Z:
            pTarget := Offs(CRobT(\Tool:=tool0,\WObj:=wobj0),0,0,nSign*nDistance);
        DEFAULT: 
            TPWrite "Illegal direction axis specified";
            RAISE ERR_ARGVALERR;
        ENDTEST
        
        RETURN pTarget;
    ENDFUNC
    
    
    ! Tries to find the logo, returns true if success, false otherwise. psCameraResult contains the data about the logo
    ! Fix and trix with the exposure here to get a good detection!
    LOCAL FUNC bool getMarkerInfo(INOUT pos psCameraResult)
        VAR num nNoOffPicture:=0;
        CONST num nMaxTries := 3;
        WHILE nNoOffPicture < nMaxTries DO
            psCameraResult:=RequestImageAndGetResult(cameraToUse);
            IF psCameraResult=[0,0,0] THEN
                Incr nNoOffPicture;
                TPWrite "Picture not OK. Try no = "\Num:=nNoOffPicture;
            ELSE
                RETURN TRUE;
            ENDIF
        ENDWHILE
        RETURN FALSE;
    ENDFUNC
    
ENDMODULE