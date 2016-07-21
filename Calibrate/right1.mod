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
    
    !PERS num transform{3,3} := [[0.00723867,-0.0786696,0.0164772],   [-0.00277539,0.00338421,1.86149],   [-0.0785732,-0.00727967,-0.0161841]];
    !PERS num transform{3,3} := [[0.00722126,-0.0787567,0.0167684],   [-0.00346829,0.000816222,1.85072],   [-0.0785935,-0.00720426,-0.0158495]];
    !PERS num transform{3,3} := [[0.00723406,-0.0786497,0.0141874],   [-0.00598537,0.00274677,1.86199],   [-0.0785122,-0.0072436,-0.0168165]];
    !PERS num transform{3,3} := [[0.00729401,-0.0787129,0.0173469],   [0.0009239,0.0021448,1.8942],   [-0.0787221,-0.00722021,-0.0166163]];
    PERS num transform{3,3} := [[0.0144255,-0.15707,0.0304456],   [0.00192358,0.00524922,3.73649],   [-0.157166,-0.0143517,-0.0445262]];
    
    
    PROC main()
        VAR num sak;
        
        
        ! Moves to a initial position with correct configuration. 
        MoveAbsJ [[80,-75,10,45,120,130],[-80,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, fine, tool_YuMiGripper_S_C;    
        
        !sak := 412 / 0;
        
        setUpCameraDevice; 
        calibIntrinsicAndRotation;
        Stop;
        
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
        VAR robtarget pStart;
        VAR pos psRelPos;
        VAR pixel markerInfo;
        
        pStart := CRobT(\Tool:=tool0,\WObj:=wobj0);
        calculateMovementTransform nTransform;
        
        printMatrix nTransform;
        
        invert3x3Matrix nTransform, transform;
        
        ! Relative image coords from the original coord
        ! [30,30,0]
        markerInfo := getMarkerInfoAtPos(pStart, [0,0,0]);
        !markerInfo.u := markerInfo.u + 100;
        !markerInfo.v := markerInfo.v + 100;
        
        
        !printMatrix transform;
        MatrixMultiply2 transform, [100,0,0], nCoords;
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
        CONST num nDistance:=50;
        VAR pixel psMaxDetectedMarker;
        VAR pixel psMinDetectedMarker;
        VAR robtarget pStart;
        
        pStart := CRobT(\Tool:=tool0,\WObj:=wobj0);
        
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
        MoveL pStart, vSpeed, fine, tool0, \WObj:=wobj0;
        
        
        ! Returns the differencies 
        traveledDistanceAlongAxis := 2 * nDistance;
        pixelDifferences.u := psMaxDetectedMarker.u - psMinDetectedMarker.u;
        pixelDifferences.v := psMaxDetectedMarker.v - psMinDetectedMarker.v;
        pixelDifferences.scale := psMaxDetectedMarker.scale - psMinDetectedMarker.scale;
    ERROR
        TPWrite "!!!!!!!Error in getMaxVisibleDistance";
        TRYNEXT;
    ENDPROC
    
    
    
    FUNC pixel getMarkerInfoAtPos(robtarget pStartPos, pos psRelPos)
        VAR robtarget pTarget;
        VAR jointtarget jtDummy;
        VAR bool bDummy;
        VAR pixel markerInfo;
        ! Moves the robot to a new target
        pTarget := Offs(pStartPos, psRelPos.x,psRelPos.y,psRelPos.z);
        
        ! This is not used anywhere, just to throw error if position is not reachable. The error thrown by MoveL didn't seem catchable
        jtDummy := CalcJointT(pTarget, tool0, \WObj:=wobj0);
        MoveL pTarget, vSpeed, fine, tool0, \WObj:=wobj0;
        bDummy := getMarkerInfo(markerInfo);
        TPWrite "Found marker at: ";
        TPWrite "U: " \Num:=markerInfo.u;
        TPWrite "V: " \Num:=markerInfo.v;
        TPWrite "Scale: " \Num:=markerInfo.scale;
        
        RETURN markerInfo;
    ERROR
        TPWrite "Destination not reachable";
        RETURN markerInfo;
    ENDFUNC
    
    
    
    ! Tries to move as long as possible in specified direction, moves until the marker is lost. 
    ! DO NOT USE THIS FUNCTION SINCE IT IS PRONE TO GET 
    ! Event Message 50431
    ! Predicted Collision
    ! Predicted a vollision bteween objects 'ROB_R_Link6' and 'ROB_L_Link6'
    ! The robots stop immediately. 
    !
    ! Couldn't find a way to catch this error! 
    LOCAL PROC movesFurthestDistance(num nDirection, num nSign, INOUT pixel lastDetectedMarker, INOUT num nDistance)
        VAR robtarget pTarget;
        VAR pixel markerInfo;
        VAR jointtarget jtDummy;
        VAR bool first := TRUE;
        CONST num nStepLength := 100;
        ! Maximum allowed distance to travel
        CONST num nMaxDistance := 100;
        nDistance := 0;
        
        
        ! Moves in specified direction until the marker is lost. 
        WHILE first OR getMarkerInfo(markerInfo) DO
            first := FALSE;
            lastDetectedMarker := markerInfo;
            IF nDistance >= nMaxDistance THEN
                RETURN;
            ENDIF
            
            nDistance := nDistance + nStepLength;
            ! Moves the robot to a new target
            pTarget := getMotionOffset(nStepLength, nDirection,nSign);
            
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

    FUNC bool RequestImage(VAR cameradev cameraToUse, INOUT pixel px)
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



