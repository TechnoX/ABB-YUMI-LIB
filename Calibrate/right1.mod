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
        VAR robtarget pS{nMaxPoints};
        VAR pixel pxU{nMaxPoints};
        VAR pixel pxCalibOffset;
        VAR num nTransform{3,3};
        VAR dnum dnP{3,4};
        
        preCalibration nTransform, pxCalibOffset;

        getSAndU pxCalibOffset, nTransform, nNumPoses, pS, pxU;
        
        findP nNumPoses, pS, pxU, dnP;

        QRFactorization;
        
    ENDPROC
    
    LOCAL PROC QRFactorization(dnum dnP{*,*}, INOUT num K)
        VAR bool bDummy;
        VAR num signChecks;
        
        bDummy:=PMatDecompQR(P,K,Kinv,camR,camT,1);
        signChecks:=0;
        WHILE signChecks<2 DO
            p1.x:=dnumtonum(camR{1,1});
            p1.y:=dnumtonum(camR{2,1});
            p1.z:=dnumtonum(camR{3,1});
            p2.x:=dnumtonum(camR{1,2});
            p2.y:=dnumtonum(camR{2,2});
            p2.z:=dnumtonum(camR{3,2});
            p3.x:=dnumtonum(camR{1,3});
            p3.y:=dnumtonum(camR{2,3});
            p3.z:=dnumtonum(camR{3,3});
            camTrob.rot:=vec2quat(p1,p2,p3);
            camTrob.trans:=[dnumtonum(camT{1,1}),dnumtonum(camT{2,1}),dnumtonum(camT{3,1})];

            !Check if "pixel" values match x,y axis of camera wobj
            p1:=PoseVect([[0,0,0],camTrob.rot],[dnumtonum(robPos{1,1}),dnumtonum(robPos{1,2}),dnumtonum(robPos{1,3})]);
            p2:=PoseVect([[0,0,0],camTrob.rot],[dnumtonum(robPos{2,1}),dnumtonum(robPos{2,2}),dnumtonum(robPos{2,3})]);
            p3:=PoseVect([[0,0,0],camTrob.rot],[dnumtonum(robPos{3,1}),dnumtonum(robPos{3,2}),dnumtonum(robPos{3,3})]);
            p1.z:=0;
            p2.z:=0;
            p3.z:=0;
            d12:=DotProd(NormalizePos(p2-p1),NormalizePos([dnumtonum(camPos{2,1}),dnumtonum(camPos{2,2}),0]-[dnumtonum(camPos{1,1}),dnumtonum(camPos{1,2}),0]));
            d13:=DotProd(NormalizePos(p3-p1),NormalizePos([dnumtonum(camPos{3,1}),dnumtonum(camPos{3,2}),0]-[dnumtonum(camPos{1,1}),dnumtonum(camPos{1,2}),0]));
            !0.05 for poorly calibrated robot, 0.01 for properly calibrated robot
            IF Present(HandHeldCamera) THEN
                Zsign:=-1;
            ELSE
                Zsign:=1;
            ENDIF

            IF abs(d12-Zsign)>0.05 OR abs(d13-Zsign)>0.05 THEN
                !Axes do not match, redo pMatDecomp with reversed Z sign
                IF signChecks=0 bDummy:=PMatDecompQR(P,K,Kinv,camR,camT,-1);
                Incr signChecks;
            ELSE
                !Signs are OK, break loop
                signChecks:=10;
            ENDIF

        ENDWHILE
        IF signChecks=2 THEN
            ErrWrite "Error in CalibIntrinsic","unable to match signs in CalibIntrinsic";
            Stop;
        ENDIF

    ENDPROC
    

    LOCAL PROC findP(num nNumPoses, robtarget pS{*}, pixel pxU{*}, INOUT dnum dnP{*,*})
        VAR dnum dnMatrix{2*nMaxPoints,12};
        VAR dnum dnU{nMaxPoints*2,12};
        VAR dnum dnS{12};
        VAR dnum dnV{12,12};
        VAR num nMinIndexS;
        VAR dnum dnMinValueS;
        
        !TODO: normalize the data! 
        normalize;
        
        
        ! Create big matrix used for finding P using SVD, see eq. 6 in paper. 
        FOR r2 FROM 1 TO nNumPoses DO
            ! First row
            dnMatrix{r2,1} := NumToDnum(pS{r2}.trans.x);
            dnMatrix{r2,2} := NumToDnum(pS{r2}.trans.y);
            dnMatrix{r2,3} := NumToDnum(pS{r2}.trans.z);
            dnMatrix{r2,4} := 1;
            dnMatrix{r2,5} := 0;
            dnMatrix{r2,6} := 0;
            dnMatrix{r2,7} := 0;
            dnMatrix{r2,8} := 0;
            dnMatrix{r2,9} :=  - NumToDnum(pxU{r2}.u * pS{r2}.trans.x);
            dnMatrix{r2,10} := - NumToDnum(pxU{r2}.u * pS{r2}.trans.y);
            dnMatrix{r2,11} := - NumToDnum(pxU{r2}.u * pS{r2}.trans.z);
            dnMatrix{r2,12} := - NumToDnum(pxU{r2}.u);
            ! Second row
            dnMatrix{r2+1,1} := 0;
            dnMatrix{r2+1,2} := 0;
            dnMatrix{r2+1,3} := 0;
            dnMatrix{r2+1,4} := 0;
            dnMatrix{r2+1,5} := NumToDnum(pS{r2}.trans.x);
            dnMatrix{r2+1,6} := NumToDnum(pS{r2}.trans.y);
            dnMatrix{r2+1,7} := NumToDnum(pS{r2}.trans.z);
            dnMatrix{r2+1,8} := 1;
            dnMatrix{r2+1,9} :=  - NumToDnum(pxU{r2}.v * pS{r2}.trans.x);
            dnMatrix{r2+1,10} := - NumToDnum(pxU{r2}.v * pS{r2}.trans.y);
            dnMatrix{r2+1,11} := - NumToDnum(pxU{r2}.v * pS{r2}.trans.z);
            dnMatrix{r2+1,12} := - NumToDnum(pxU{r2}.v);
        ENDFOR
        
        
        ! Apply SVD for this! 
        MatrixSVD dnMatrix\A_m:=nMaxPoints,\A_n:=12, dnU, dnS, dnV\Econ;

        !Find smallest singular value
        dnMinValueS := MinDnumArray(dnS, \idx:=nMinIndexS);

        
        
        dnP:=[[dnV{1,nMinIndexS}, dnV{2,nMinIndexS},  dnV{3,nMinIndexS},  dnV{4,nMinIndexS}],
              [dnV{5,nMinIndexS}, dnV{6,nMinIndexS},  dnV{7,nMinIndexS},  dnV{8,nMinIndexS}],
              [dnV{9,nMinIndexS}, dnV{10,nMinIndexS}, dnV{11,nMinIndexS}, dnV{12,nMinIndexS}]];
        
        
        denormalize;
        
        
    ENDPROC
    
    LOCAL PROC denormalize()
!        Scale_robPos:=[[1/srobPos{1},0,0,-trobPos{1}/srobPos{1}],[0,1/srobPos{2},0,-trobPos{2}/srobPos{2}],[0,0,1/srobPos{3},-trobPos{3}/srobPos{3}],[0,0,0,1]];
!        Scale_camPos:=[[1/scamPos{1},0,-tcamPos{1}/scamPos{1}],[0,1/scamPos{2},-tcamPos{2}/scamPos{2}],[0,0,1]];
!        MatrixMultiplyDnum P,Scale_robPos,result4;
!        status:=invert3DMatrix(Scale_camPos,result3);
!        MatrixMultiplyDnum result3,result4,P;
!        abs_lambda:=SqrtDnum(P{3,1}*P{3,1}+P{3,2}*P{3,2}+P{3,3}*P{3,3});
!        !Scale P with the scale factor
!        FOR i FROM 1 TO 3 DO
!            FOR j FROM 1 TO 4 DO
!                P{i,j}:=P{i,j}/abs_lambda;
!            ENDFOR
!        ENDFOR
    ENDPROC
    
    LOCAL PROC normalize()
        !Calculate normalization scaling
!        FOR i FROM 1 TO n DO
!            trobPos{1}:=trobPos{1}+robPos{i,1}/numtodnum(n);
!            trobPos{2}:=trobPos{2}+robPos{i,2}/numtodnum(n);
!            trobPos{3}:=trobPos{3}+robPos{i,3}/numtodnum(n);
!            tcamPos{1}:=tcamPos{1}+camPos{i,1}/numtodnum(n);
!            tcamPos{2}:=tcamPos{2}+camPos{i,2}/numtodnum(n);
!        ENDFOR
!        FOR i FROM 1 TO n DO
!            srobPos{1}:=srobPos{1}+(robPos{i,1}-trobPos{1})*(robPos{i,1}-trobPos{1})/numtodnum(n-1);
!            srobPos{2}:=srobPos{2}+(robPos{i,2}-trobPos{2})*(robPos{i,2}-trobPos{2})/numtodnum(n-1);
!            srobPos{3}:=srobPos{3}+(robPos{i,3}-trobPos{3})*(robPos{i,3}-trobPos{3})/numtodnum(n-1);
!            scamPos{1}:=scamPos{1}+(camPos{i,1}-tcamPos{1})*(camPos{i,1}-tcamPos{1})/numtodnum(n-1);
!            scamPos{2}:=scamPos{2}+(camPos{i,2}-tcamPos{2})*(camPos{i,2}-tcamPos{2})/numtodnum(n-1);
!        ENDFOR
!        srobPos{1}:=SqrtDnum(srobPos{1});
!        srobPos{2}:=SqrtDnum(srobPos{2});
!        srobPos{3}:=SqrtDnum(srobPos{3});
!        scamPos{1}:=SqrtDnum(scamPos{1});
!        scamPos{2}:=SqrtDnum(scamPos{2});
!        FOR i FROM 1 TO n DO
!            robPos{i,1}:=(robPos{i,1}-trobPos{1})/srobPos{1};
!            robPos{i,2}:=(robPos{i,2}-trobPos{2})/srobPos{2};
!            robPos{i,3}:=(robPos{i,3}-trobPos{3})/srobPos{3};
!            camPos{i,1}:=(camPos{i,1}-tcamPos{1})/scamPos{1};
!            camPos{i,2}:=(camPos{i,2}-tcamPos{2})/scamPos{2};
!        ENDFOR
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
        ! Max distance to be considered a true detection
        CONST num nMaxDistance := 200; 
        
        nRelPixelTarget{1} := pxTarget.u - pxCalibOffset.u;
        nRelPixelTarget{2} := pxTarget.v - pxCalibOffset.v;
        nRelPixelTarget{3} := pxTarget.scale - pxCalibOffset.scale;
        
        MatrixMultiply2 nTransform, nRelPixelTarget, nCoords;
        psRelPos.x := nCoords{1};
        psRelPos.y := nCoords{2};
        psRelPos.z := nCoords{3};
        
        ! If not detected at all
        IF NOT getMarkerInfoAtPos(pStart, psRelPos, pxDetectedMarker) THEN
            RETURN FALSE;
        ENDIF
        
        ! If detected at wrong position in the image, we are discarding this detection
        IF getPixelDistance(pxDetectedMarker, pxTarget) > 200 THEN
            RETURN FALSE;
        ENDIF
        
        RETURN TRUE;
    ENDFUNC
    
    
    LOCAL FUNC num getPixelDistance(pixel px1, pixel px2)
        RETURN Sqrt((px1.u - px2.u) * (px1.u - px2.u) + (px1.v - px2.v) * (px1.v - px2.v) + (px1.scale - px2.scale) * (px1.scale - px2.scale));
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



