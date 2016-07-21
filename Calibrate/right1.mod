MODULE MainModule
    
    LOCAL RECORD pixel
        num u;
        num v;
        ! If scale is used for a detection it can be saved in the pixel record. Otherwise uninitialized; 
        num scale; 
    ENDRECORD
    ! "Enum" for the directions
    LOCAL CONST num DIRECTION_X:=0;
    LOCAL CONST num DIRECTION_Y:=1;
    LOCAL CONST num DIRECTION_Z:=2;
    
    LOCAL CONST bool bMoveCamera := FALSE;
    ! The general speed used during calibration
    LOCAL CONST speeddata vSpeed := v200;
    LOCAL VAR cameradev cameraToUse;
    
    LOCAL PERS num transform{3,3} := [[-0.000905999,0.163839,-0.202577],   [0.0227405,-0.00622846,-3.83902],   [0.1626,0.00163294,0.320635]];
    
    PERS robtarget pLogoTarget;
    LOCAL VAR robtarget pStart;
    
    PROC main()
        VAR num sak;
        
        
        ! Moves to a initial position with correct configuration. 
        MoveAbsJ [[74.077,-105.906,28.11,38.7805,94.3583,158.079],[-54.3312,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, fine, tool_YuMiGripper_S_C;    
        
        WaitTime 5;
        IF bMoveCamera THEN
            pStart := CRobT(\Tool:=tool0,\WObj:=wobj0);
        ELSE
            ! TOOD: Use WaitSyncTask to be sure the other arm is standing still after its MoveAbsJ instruction
            pStart := CRobT(\TaskName:="T_ROB_L",\Tool:=tool0,\WObj:=wobj0);
        ENDIF
        setUpCameraDevice; 
        calibIntrinsicAndRotation;
        Stop \AllMoveTasks;
        
	ENDPROC
    
    
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
        VAR num nCoords{3}; 
        VAR bool bDummy;
        VAR pos psRelPos;
        VAR pixel markerInfo;
        
        calculateMovementTransform nTransform;
        
        printMatrix nTransform;
        
        invert3x3Matrix nTransform, transform;
        
        ! Relative image coords from the original coord
        ! [30,30,0]
        markerInfo := getMarkerInfoAtPos(pStart, [0,0,0]);
        !markerInfo.u := markerInfo.u + 100;
        !markerInfo.v := markerInfo.v + 100;
        
        
        !printMatrix transform;
        MatrixMultiply2 transform, [0,0,-20], nCoords;
        psRelPos.x := nCoords{1};
        psRelPos.y := nCoords{2};
        psRelPos.z := nCoords{3};
        TPWrite "RelPos: "\Pos:=psRelPos;
        
        markerInfo := getMarkerInfoAtPos(pStart, psRelPos);
        
        
        
    ENDPROC
    
    
    LOCAL PROC calculateMovementTransform(INOUT num nTransform{*,*})
        VAR num nDistances{3};
        VAR pixel nPixelDistances{3};
        IF DIM(nTransform,1) <> 3 OR DIM(nTransform,1) <> 3 THEN
            TPWrite "The transform must be of size 3x3";
            RAISE ERR_NOTEQDIM;
        ENDIF
        
        ! Moves in X direction of robot-frame and get moved distance in X and pixels. 
        getMarkerData DIRECTION_X, nDistances{1}, nPixelDistances{1};
        ! Moves in Y direction of robot-frame and get moved distance in Y and pixels. 
        getMarkerData DIRECTION_Y, nDistances{2}, nPixelDistances{2};
        ! Moves in Z direction of robot-frame and get moved distance in Z and pixels. 
        getMarkerData DIRECTION_Z, nDistances{3}, nPixelDistances{3};
        
        
        ! Probably possible to make this to a Matrix multiplication?
        ! But this is at least working. And pretty clear? :)
        FOR c FROM 1 TO 3 DO
            nTransform{1,c} := nPixelDistances{c}.u / nDistances{c};
            nTransform{2,c} := nPixelDistances{c}.v / nDistances{c};
            nTransform{3,c} := nPixelDistances{c}.scale / nDistances{c};
        ENDFOR
        
        !Stop;
    ENDPROC
    
    
    
    
    ! Moves in the arm along the specified axis to the most negative and then most positive to find out the biggest difference in pixel distance
    LOCAL PROC getMarkerData(num nDirection, INOUT num traveledDistanceAlongAxis, INOUT pixel pixelDifferences)
        CONST num nDistance:=30;
        VAR pixel psMaxDetectedMarker;
        VAR pixel psMinDetectedMarker;
        
        
        TEST nDirection
            CASE DIRECTION_X:
                psMaxDetectedMarker := getMarkerInfoAtPos(pStart,[nDistance,0,0]);
                psMinDetectedMarker := getMarkerInfoAtPos(pStart,[-nDistance,0,0]);
            CASE DIRECTION_Y:
                psMaxDetectedMarker := getMarkerInfoAtPos(pStart,[0,nDistance,0]);
                psMinDetectedMarker := getMarkerInfoAtPos(pStart,[0,-nDistance,0]);
            CASE DIRECTION_Z:
                psMaxDetectedMarker := getMarkerInfoAtPos(pStart,[0,0,nDistance]);
                psMinDetectedMarker := getMarkerInfoAtPos(pStart,[0,0,-nDistance]);
            DEFAULT: 
                TPWrite "Illegal direction axis specified";
                RAISE ERR_ARGVALERR;
        ENDTEST

        
        
        ! Moves back to start
        Move(pStart);
        
        ! Returns the differencies 
        traveledDistanceAlongAxis := 2 * nDistance;
        pixelDifferences.u := psMaxDetectedMarker.u - psMinDetectedMarker.u;
        pixelDifferences.v := psMaxDetectedMarker.v - psMinDetectedMarker.v;
        pixelDifferences.scale := psMaxDetectedMarker.scale - psMinDetectedMarker.scale;
    ERROR
        TPWrite "!!!!!!!Error in getMaxVisibleDistance";
        TRYNEXT;
    ENDPROC
    

    LOCAL PROC Move(robtarget pTarget)
        IF bMoveCamera THEN
            MoveCamera(pTarget);
        ELSE
            MoveLogo(pTarget);
        ENDIF
    ENDPROC
    
    LOCAL PROC MoveCamera(robtarget pTarget)
        VAR jointtarget jtDummy;
        ! This is not used anywhere, just to throw error if position is not reachable. The error thrown by MoveL didn't seem catchable
        jtDummy := CalcJointT(pTarget, tool0, \WObj:=wobj0);
        ! Moves the robot
        MoveL pTarget, vSpeed, fine, tool0, \WObj:=wobj0;
    ERROR
        TPWrite "Destination not reachable";
        RETURN;
    ENDPROC

    LOCAL PROC MoveLogo(robtarget pTarget)
        pLogoTarget:=pTarget;
        SetDO doLogoIsMoving,1;
        WaitDO doLogoIsMoving,0;
    ENDPROC
    
    
    LOCAL FUNC pixel getMarkerInfoAtPos(robtarget pStartPos, pos psRelPos)
        VAR robtarget pTarget;
        VAR bool bDummy;
        VAR pixel markerInfo;
        ! Moves the robot to a new target
        pTarget := Offs(pStartPos, psRelPos.x,psRelPos.y,psRelPos.z);
        
        Move(pTarget);
        bDummy := getMarkerInfo(markerInfo);
        TPWrite "Found marker at: ";
        TPWrite "U: " \Num:=markerInfo.u;
        TPWrite "V: " \Num:=markerInfo.v;
        TPWrite "Scale: " \Num:=markerInfo.scale;
        
        RETURN markerInfo;
    ENDFUNC
    
    
    ! Tries to find the logo, returns true if success, false otherwise. psCameraResult contains the data about the logo
    ! Fix and trix with the exposure here to get a good detection!
    LOCAL FUNC bool getMarkerInfo(INOUT pixel psCameraResult)
        VAR num nNoOffPicture:=0;
        CONST num nMaxTries := 3;

        !TPWrite "Tries to take picture";
        WHILE nNoOffPicture < nMaxTries DO
            IF NOT RequestImage(cameraToUse, psCameraResult) THEN
                Incr nNoOffPicture;
                TPWrite "Picture not OK. Try no = "\Num:=nNoOffPicture;
            ELSE
                RETURN TRUE;
            ENDIF
        ENDWHILE
        RETURN FALSE;
    ENDFUNC

    LOCAL FUNC bool RequestImage(VAR cameradev cameraToUse, INOUT pixel px)
        VAR cameratarget tgt;
        CamReqImage CameraToUse;
        CamGetResult CameraToUse,tgt\MaxTime:=5;
        !WaitTime 2.0;
        px.u := tgt.cframe.trans.x;
        px.v := tgt.cframe.trans.y;
        px.scale := tgt.val1;
        RETURN TRUE;
    ERROR
        IF ERRNO=ERR_CAM_MAXTIME THEN
            TPWrite "Could not find the target";
            RETURN FALSE;
        ELSEIF ERRNO=ERR_CAM_COM_TIMEOUT THEN
            WaitTime 2.0;
            RETRY;
        ENDIF
    ENDFUNC

    
    
ENDMODULE



