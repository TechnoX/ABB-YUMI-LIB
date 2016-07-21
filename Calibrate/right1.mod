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
    
    LOCAL CONST bool bMoveCamera := TRUE;
    ! The general speed used during calibration
    LOCAL CONST speeddata vSpeed := v400;
    LOCAL VAR cameradev cameraToUse;
    
    
    LOCAL CONST pixel pxImageSize := [1280,960,0];
    LOCAL CONST num nMargin := 200;
    LOCAL CONST num nScales{3} := [90, 70, 50];
    LOCAL CONST num nMaxPoints:=2*9 + 1;
    LOCAL CONST pixel pxGoodPointsToVisit{nMaxPoints} := [[pxImageSize.u/2             , pxImageSize.v/2             , nScales{1}],
                                                          [                    nMargin ,                     nMargin , nScales{2}],
                                                          [pxImageSize.u/2             ,                 1.5*nMargin , nScales{2}],
                                                          [pxImageSize.u -     nMargin ,                     nMargin , nScales{2}],
                                                          [pxImageSize.u - 1.5*nMargin , pxImageSize.v/2             , nScales{2}],
                                                          [pxImageSize.u -     nMargin , pxImageSize.v -     nMargin , nScales{2}],
                                                          [pxImageSize.u/2             , pxImageSize.v - 1.5*nMargin , nScales{2}],
                                                          [                1.5*nMargin , pxImageSize.v -     nMargin , nScales{2}],
                                                          [                    nMargin , pxImageSize.v/2             , nScales{2}],
                                                          [pxImageSize.u/2             , pxImageSize.v/2             , nScales{2}],
                                                          [                    nMargin ,                     nMargin , nScales{3}],
                                                          [pxImageSize.u/2             ,                 1.5*nMargin , nScales{3}],
                                                          [pxImageSize.u -     nMargin ,                     nMargin , nScales{3}],
                                                          [pxImageSize.u - 1.5*nMargin , pxImageSize.v/2             , nScales{3}],
                                                          [pxImageSize.u -     nMargin , pxImageSize.v -     nMargin , nScales{3}],
                                                          [pxImageSize.u/2             , pxImageSize.v - 1.5*nMargin , nScales{3}],
                                                          [                1.5*nMargin , pxImageSize.v -     nMargin , nScales{3}],
                                                          [                    nMargin , pxImageSize.v/2             , nScales{3}],
                                                          [pxImageSize.u/2             , pxImageSize.v/2             , nScales{3}]];
    
    
    !LOCAL PERS num transform{3,3} := [[0.00734952,-0.186346,0.0676703],   [0.0166307,0.00824375,4.93447],   [-0.186678,-0.00741658,-0.0260662]];
    
    PERS robtarget pLogoTarget;
    LOCAL VAR robtarget pStart;
    
    PROC main()
        VAR num sak;
        
        
        ! Moves to a initial position with correct configuration. 
        MoveAbsJ [[73.7158,-90.4035,20.6941,36.0716,113.105,138.297],[-75.8492,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, fine, tool_YuMiGripper_S_C;    
        
        WaitTime 1;
        ! TOOD: Use WaitSyncTask to be sure the other arm is standing still after its MoveAbsJ instruction
        pStart := getCurrentPosition();
        setUpCameraDevice; 
        calibIntrinsicAndRotation;
        Stop \AllMoveTasks;
        
	ENDPROC
    
    
    LOCAL FUNC robtarget getCurrentPosition()
        IF bMoveCamera THEN
            RETURN CRobT(\Tool:=tool0,\WObj:=wobj0);
        ELSE
            RETURN CRobT(\TaskName:="T_ROB_L",\Tool:=tool0,\WObj:=wobj0);
        ENDIF
    ENDFUNC
    
    
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
        VAR num nNumPoses;
        VAR robtarget S{nMaxPoints};
        VAR pixel U{nMaxPoints};
        VAR pixel pxCalibOffset;
        VAR num nTransform{3,3};
        
        preCalibration nTransform, pxCalibOffset;
        !Stop \AllMoveTasks;
        getSAndU pxCalibOffset, nTransform, nNumPoses, S, U;
    ENDPROC
    

    ! Get the S and U datasets. 
    LOCAL PROC getSAndU(pixel pxCalibOffset, num nTransform{*,*}, num nNumPoses, INOUT robtarget S{*}, INOUT pixel U{*})
        VAR num nIndex := 1;
        VAR pixel pxDetectedMarker;
        IF Dim(nTransform, 1) <> 3 OR Dim(nTransform,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        ! Loop through all good points to look at
        FOR px FROM 1 TO DIM(pxGoodPointsToVisit,1) DO
            ! If the marker is visible from this point (and point is reachable)
            IF placeMarkerAtPixel(pxGoodPointsToVisit{px}, pxCalibOffset, nTransform, pxDetectedMarker) THEN
                S{nIndex} := getCurrentPosition();
                TPWrite "Saves pixel "\Num:=px;
                U{nIndex} := pxDetectedMarker;
            ENDIF
        ENDFOR
        
    ENDPROC
    
    LOCAL PROC preCalibration(INOUT num transform{*,*}, INOUT pixel offset)
                VAR num nTransform{3,3};
        VAR bool bDummy;
        
        IF Dim(nTransform, 1) <> 3 OR Dim(nTransform,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        ! Image coords from the original marker (that is at the "pStart" position)
        bDummy := getMarkerInfoAtPos(pStart, [0,0,0], offset);
        
        calculateMovementTransform nTransform;
        invert3x3Matrix nTransform, transform;
    ENDPROC
    
    LOCAL FUNC bool placeMarkerAtPixel(pixel pxTarget, pixel pxCalibOffset, num nTransform{*,*}, INOUT pixel pxDetectedMarker)
        VAR num nCoords{3}; 
        VAR pos psRelPos;
        VAR num nRelPixelTarget{3};
        
        nRelPixelTarget{1} := pxTarget.u - pxCalibOffset.u;
        nRelPixelTarget{2} := pxTarget.v - pxCalibOffset.v;
        nRelPixelTarget{3} := pxTarget.scale - pxCalibOffset.scale;
        
        MatrixMultiply2 nTransform, nRelPixelTarget, nCoords;
        psRelPos.x := nCoords{1};
        psRelPos.y := nCoords{2};
        psRelPos.z := nCoords{3};
        
        IF NOT getMarkerInfoAtPos(pStart, psRelPos, pxDetectedMarker) THEN
            RETURN FALSE;
        ENDIF
        
        RETURN TRUE;
    ENDFUNC
    
    
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
        VAR bool bDummy;
        
        TEST nDirection
            CASE DIRECTION_X:
                bDummy := getMarkerInfoAtPos(pStart,[nDistance,0,0], psMaxDetectedMarker);
                bDummy := getMarkerInfoAtPos(pStart,[-nDistance,0,0], psMinDetectedMarker);
            CASE DIRECTION_Y:
                bDummy := getMarkerInfoAtPos(pStart,[0,nDistance,0], psMaxDetectedMarker);
                bDummy := getMarkerInfoAtPos(pStart,[0,-nDistance,0], psMinDetectedMarker);
            CASE DIRECTION_Z:
                bDummy := getMarkerInfoAtPos(pStart,[0,0,nDistance], psMaxDetectedMarker);
                bDummy := getMarkerInfoAtPos(pStart,[0,0,-nDistance], psMinDetectedMarker);
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
    
    LOCAL PROC moveToPos(robtarget pStartPos, pos psRelPos)
        VAR robtarget pTarget;
        ! Moves the robot to a new target
        pTarget := Offs(pStartPos, psRelPos.x,psRelPos.y,psRelPos.z);
        
        Move(pTarget);
    ENDPROC
    
    LOCAL FUNC bool getMarkerInfoAtPos(robtarget pStartPos, pos psRelPos, INOUT pixel detectedMarker)
        
        moveToPos pStartPos, psRelPos;
        IF NOT getMarkerInfo(detectedMarker) THEN
            RETURN FALSE;
        ENDIF
        !TPWrite "Found marker at: ";
        !TPWrite "U: " \Num:=detectedMarker.u;
        !TPWrite "V: " \Num:=detectedMarker.v;
        !TPWrite "Scale: " \Num:=detectedMarker.scale;
        
        RETURN TRUE;
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



