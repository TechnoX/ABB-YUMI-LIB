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
    LOCAL CONST speeddata vSpeed := v400;
    LOCAL VAR cameradev cameraToUse;
    
    
    LOCAL CONST pixel pxImageSize := [1280,960,0];
    LOCAL CONST num nMargin := 200;
    LOCAL CONST num nScales{3} := [90, 70, 50];
    
    ! Can't handle more than 18 points because of some arbitrary limit inside MatrixSVD. "Report the problem to ABB Robotics"
    LOCAL CONST num nMaxPoints:=18;
    LOCAL CONST pixel pxGoodPointsToVisit{nMaxPoints} := [[pxImageSize.u/2             , pxImageSize.v/2             , nScales{1}],
                                                          [                    nMargin ,                     nMargin , nScales{2}],
                                                          [pxImageSize.u/2             ,                 1.5*nMargin , nScales{2}],
                                                          [pxImageSize.u -     nMargin ,                     nMargin , nScales{2}],
                                                          [pxImageSize.u - 1.5*nMargin , pxImageSize.v/2             , nScales{2}],
                                                          [pxImageSize.u -     nMargin , pxImageSize.v -     nMargin , nScales{2}],
                                                          [pxImageSize.u/2             , pxImageSize.v - 1.5*nMargin , nScales{2}],
                                                          [                1.5*nMargin , pxImageSize.v -     nMargin , nScales{2}],
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
        VAR pose peRobTCam;
        VAR dnum dnK{3,3};
        VAR dnum dnKinv{3,3};
        
        ! Moves to a initial position with correct configuration. 
        !MoveAbsJ [[73.7158,-90.4035,20.6941,36.0716,113.105,138.297],[-75.8492,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, fine, tool_YuMiGripper_S_C;    
        
        WaitTime 1;
        ! TOOD: Use WaitSyncTask to be sure the other arm is standing still after its MoveAbsJ instruction
        pStart := getCurrentRobtarget();
        
        ! Setup camera device
        setUpCameraDevice; 
        
        ! Phase one of the paper
        calibIntrinsicAndRotation peRobTCam, dnK, dnKinv,\print;
        !peRobTCam := [[386.23,125.164,235.203],[0.498083,-0.475105,-0.510217,-0.515623]];
        !dnK := [[-0.157644977384217,0.00374791804962304,639.404475621505],[0,-0.15959236409683,479.917088423164],[0,0,1]];
        !dnKinv := [[-6.34336733458515,-0.148969664452468,4127.4705518724],[0,-6.26596394921328,3007.14317467095],[0,0,1.00000000000045]];
        
        TPWrite "Done with part 1"; 
        
        ! Phase two and three of the paper
        calibExtrinsic peRobTCam;
        
        
        Stop \AllMoveTasks;
        
	ENDPROC
    
    
    LOCAL FUNC robtarget getCurrentRobtarget()
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
    
    
    ! Phase two and three of the paper
    LOCAL PROC calibExtrinsic(pose peRob2Cam)
        getExtSAndU peRob2Cam;
        
        ! Equation (15) in paper
        !getPointsInCameraFrame nNumPoses, nZ;
        
        
    ENDPROC
    
    
    
    LOCAL PROC getExtSAndU(pose peRob2Cam)
        VAR robtarget pTarget;
        ! Align marker to cam axes (using peRob2Cam)
        pTarget := pStart;
        ! Rotate up the angle
        !pTarget.rot := [0.70710678118, 0.70710678118, 0, 0] * peRob2Cam.rot;
        pTarget.rot := peRob2Cam.rot;
        
        Move(pTarget);
        
    ENDPROC
    
    
    
    
    
    
    ! Whole phase one of the paper.
    LOCAL PROC calibIntrinsicAndRotation(INOUT pose peRob2Cam, INOUT dnum dnK{*,*}, INOUT dnum dnKinv{*,*},\switch print)
        ! The number of detected markers, the m value in the paper
        VAR num nNumPoses;
        ! The first S dataset, from paper
        VAR pos psS{nMaxPoints};
        ! The first U dataset, from paper
        VAR pixel pxU{nMaxPoints};
        ! The pixel where the first marker was detected (during precalibration)
        VAR pixel pxCalibOffset;
        ! The precalibration transform, used to place the logo at specific coordinates in the image
        VAR num nTransform{3,3};
        ! The P matrix, from paper
        VAR dnum dnP{3,4};
        ! The rotation matrix
        VAR dnum dnR{3,3};
        ! The translation matrix, not correct values since it is calculated from the wrist
        VAR dnum dnT{3,1};
        ! The transformation from camera frame to(2) robot frame
        VAR pose peCam2Rob;
        
        ! Check dimensions of input arguments
        IF DIM(dnK,1) <> 3 OR DIM(dnK,2) <> 3 OR DIM(dnKinv,1) <> 3 OR DIM(dnKinv,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        ! Find an initial transformation from pixel coordinates to world coordinates
        preCalibration nTransform, pxCalibOffset;

        ! Get measurements of pixels (U_int dataset) and their correspondences in real world (S_int dataset)
        getIntSAndU pxCalibOffset, nTransform, nNumPoses, psS, pxU;
        
        ! Create a P (transformation matrix) from the S and U datasets
        findP nNumPoses, psS, pxU, dnP;
        IF Present(print) THEN
            printP nNumPoses, dnP, psS, pxU;
        ENDIF
        
        ! Creates a transform from robot to camera, and the intrinsic parameters dnK
        ! (as a side effect from the algorithm we also obtain the inverted K matrix, which can be good for future use). 
        IF NOT PMatDecompQR(dnP,dnK,dnKinv,dnR,dnT,1) THEN 
            ErrWrite "calibIntrinsicAndRotation", "Wrong matrix dimension";
            Stop;
        ENDIF
        createPoseFromRAndT dnR, dnT, peCam2Rob;
        
        ! If the axis was wrong, change direction of Z-axis
        IF NOT axesCoincide(peCam2rob, psS, pxU) THEN
            IF NOT PMatDecompQR(dnP,dnK,dnKinv,dnR,dnT,-1) THEN 
                ErrWrite "calibIntrinsicAndRotation", "Wrong matrix dimension";
                Stop;
            ENDIF
            createPoseFromRAndT dnR, dnT, peCam2Rob;
            
            ! If they still don't coincide, something is broken! 
            IF NOT axesCoincide(peCam2rob, psS, pxU) THEN
                ErrWrite "calibIntrinsicAndRotation","Unable to match sign of axes";
                Stop;
            ENDIF
        ENDIF

        ! TODO: Analyze dnP before PMatDecompQR beforehand to select positive or negative sign?? So we don't need to check if the axes coincide. 
        
        
        ! Return the inverse transform, from robot base to camera coordinate system
        peRob2Cam:=PoseInv(peCam2Rob);
        ! Note that only the R part of peRobTCam is correct so far! 
        
        
        ! TODO: Refine calibration result with non linear optimization, including distortion
        !CalibIntNonLin psS,psU,nNumPoses,peRob2Cam,dnK;
        
        
        ! Returns and peRobTCam and K (and Kinv) values. Done with phase one.  
    ENDPROC
    
    LOCAL PROC printP(num nNumPoses, dnum dnP{*,*}, pos psS{*}, pixel pxU{*})
        ! The back projected point
        VAR dnum nBackProjected{3,1};
        VAR pixel pxBackProjectedError;
        
        !Calculate back projection error
        TPWrite "Back Projection Error=";
        FOR i FROM 1 TO nNumPoses DO
            MatrixMultiplyDnum dnP,[[NumToDnum(psS{i}.x)],[NumToDnum(psS{i}.y)],[NumToDnum(psS{i}.z)],[1]],nBackProjected;
            nBackProjected{1,1}:=nBackProjected{1,1}/nBackProjected{3,1};
            nBackProjected{2,1}:=nBackProjected{2,1}/nBackProjected{3,1};
            pxBackProjectedError.u := DnumToNum(nBackProjected{1,1}) - pxU{i}.u;
            pxBackProjectedError.v := DnumToNum(nBackProjected{2,1}) - pxU{i}.v;
            TPWrite "error u: " + NumToStr(pxBackProjectedError.u,3) + ", v: " + NumToStr(pxBackProjectedError.v,3);
            WaitTime 1;
        ENDFOR
    ENDPROC
    
    
    
    LOCAL FUNC bool axesCoincide(pose peCam2Rob, pos psS{*}, pixel pxU{*})
        VAR num d12;
        VAR num d13;
        VAR num Zsign;
        VAR pos ps1;
        VAR pos ps2;
        VAR pos ps3;
        IF Dim(psS,1) < 3 OR Dim(pxU,1) < 3 THEN
            ErrWrite "Error in axesCoincide", "Too few detected positions";
            Stop;
        ENDIF
        
        !Check if "pixel" values match x,y axis of camera wobj
        ps1:=PoseVect([[0,0,0],peCam2Rob.rot],[psS{1}.x,psS{1}.y,psS{1}.z]);
        ps2:=PoseVect([[0,0,0],peCam2Rob.rot],[psS{2}.x,psS{2}.y,psS{2}.z]);
        ps3:=PoseVect([[0,0,0],peCam2Rob.rot],[psS{3}.x,psS{3}.y,psS{3}.z]);
        ps1.z:=0;
        ps2.z:=0;
        ps3.z:=0;
        d12:=DotProd(NormalizePos(ps2-ps1),NormalizePos([pxU{2}.u,pxU{2}.v,0]-[pxU{1}.u,pxU{1}.v,0]));
        d13:=DotProd(NormalizePos(ps3-ps1),NormalizePos([pxU{3}.u,pxU{3}.v,0]-[pxU{1}.u,pxU{1}.v,0]));
        !0.05 for poorly calibrated robot, 0.01 for properly calibrated robot
        IF bMoveCamera THEN
            Zsign:=-1;
        ELSE
            Zsign:=1;
        ENDIF

        IF abs(d12-Zsign)>0.05 OR abs(d13-Zsign)>0.05 THEN
            !Axes do not match, redo pMatDecomp with reversed Z sign
            RETURN FALSE;
        ELSE
            !Signs are OK
            RETURN TRUE;
        ENDIF
    ENDFUNC
    
    
    
    LOCAL PROC createPoseFromRAndT(dnum dnR{*,*}, dnum dnT{*,*}, INOUT pose peCam2Rob)
        VAR pos ps1;
        VAR pos ps2;
        VAR pos ps3;
        
        ! Create rotation matrix
        ps1.x:=dnumtonum(dnR{1,1});
        ps1.y:=dnumtonum(dnR{2,1});
        ps1.z:=dnumtonum(dnR{3,1});
        ps2.x:=dnumtonum(dnR{1,2});
        ps2.y:=dnumtonum(dnR{2,2});
        ps2.z:=dnumtonum(dnR{3,2});
        ps3.x:=dnumtonum(dnR{1,3});
        ps3.y:=dnumtonum(dnR{2,3});
        ps3.z:=dnumtonum(dnR{3,3});
        peCam2Rob.rot:=vec2quat(ps1,ps2,ps3);
        
        ! Create translation vector
        peCam2Rob.trans:=[dnumtonum(dnT{1,1}),dnumtonum(dnT{2,1}),dnumtonum(dnT{3,1})];
    ENDPROC
    

    LOCAL PROC findP(num nNumPoses, pos psS{*}, pixel pxU{*}, INOUT dnum dnP{*,*})
        VAR dnum dnMatrix{2*nMaxPoints,12};
        VAR dnum dnU{2*nMaxPoints,12};
        VAR dnum dnS{12};
        VAR dnum dnV{12,12};
        VAR num nMinIndexS;
        VAR dnum dnMinValueS;
        ! The mean values of the datasets
        VAR pos psMeanS;
        VAR pixel pxMeanU;
        ! The standard deviation of the datasets
        VAR pos psStdevS;
        VAR pixel pxStdevU;
        
        psStdevS := getPosStdev(nNumPoses, psS, \psMean:=psMeanS);
        pxStdevU := getPixelStdev(nNumPoses, pxU, \pxMean:=pxMeanU);
        
        normalize nNumPoses, psMeanS, psStdevS,pxMeanU,pxStdevU,psS,pxU; 
        
        
        ! Create big matrix used for finding P using SVD, see eq. 6 in paper. 
        FOR r2 FROM 1 TO nNumPoses DO
            ! First row
            dnMatrix{2*r2-1,1} := NumToDnum(psS{r2}.x);
            dnMatrix{2*r2-1,2} := NumToDnum(psS{r2}.y);
            dnMatrix{2*r2-1,3} := NumToDnum(psS{r2}.z);
            dnMatrix{2*r2-1,4} := 1;
            dnMatrix{2*r2-1,5} := 0;
            dnMatrix{2*r2-1,6} := 0;
            dnMatrix{2*r2-1,7} := 0;
            dnMatrix{2*r2-1,8} := 0;
            dnMatrix{2*r2-1,9} :=  - NumToDnum(pxU{r2}.u * psS{r2}.x);
            dnMatrix{2*r2-1,10} := - NumToDnum(pxU{r2}.u * psS{r2}.y);
            dnMatrix{2*r2-1,11} := - NumToDnum(pxU{r2}.u * psS{r2}.z);
            dnMatrix{2*r2-1,12} := - NumToDnum(pxU{r2}.u);
            ! Second row
            dnMatrix{2*r2,1} := 0;
            dnMatrix{2*r2,2} := 0;
            dnMatrix{2*r2,3} := 0;
            dnMatrix{2*r2,4} := 0;
            dnMatrix{2*r2,5} := NumToDnum(psS{r2}.x);
            dnMatrix{2*r2,6} := NumToDnum(psS{r2}.y);
            dnMatrix{2*r2,7} := NumToDnum(psS{r2}.z);
            dnMatrix{2*r2,8} := 1;
            dnMatrix{2*r2,9} :=  - NumToDnum(pxU{r2}.v * psS{r2}.x);
            dnMatrix{2*r2,10} := - NumToDnum(pxU{r2}.v * psS{r2}.y);
            dnMatrix{2*r2,11} := - NumToDnum(pxU{r2}.v * psS{r2}.z);
            dnMatrix{2*r2,12} := - NumToDnum(pxU{r2}.v);
        ENDFOR
        
        
        ! Apply SVD for this! 
        MatrixSVD dnMatrix\A_m:=2*nNumPoses,\A_n:=12, dnU, dnS, dnV\Econ;

        !Find smallest singular value
        dnMinValueS := MinDnumArray(dnS, \idx:=nMinIndexS);
        
        ! TODO: I suspect that dnS is always sorted from largest to smallest number, so it should be fine to just take the last value from dnV and use below?? 
        dnP:=[[dnV{1,nMinIndexS}, dnV{2,nMinIndexS},  dnV{3,nMinIndexS},  dnV{4,nMinIndexS}],
              [dnV{5,nMinIndexS}, dnV{6,nMinIndexS},  dnV{7,nMinIndexS},  dnV{8,nMinIndexS}],
              [dnV{9,nMinIndexS}, dnV{10,nMinIndexS}, dnV{11,nMinIndexS}, dnV{12,nMinIndexS}]];
        
        denormalize psMeanS, psStdevS, pxMeanU, pxStdevU, dnP;
    ENDPROC
    
    LOCAL PROC denormalize(pos psMean, pos psStdev, pixel pxMean, pixel pxStdev, INOUT dnum dnP{*,*})
        VAR dnum dnScale_robPos{4,4};
        VAR dnum dnScale_camPos{3,3};
        VAR dnum result3{3,3};
        VAR dnum result4{4,4};
        VAR bool bDummy;
        VAR dnum abs_lambda;
        
        numMatrix2dnumMatrix [[1/psStdev.x,0,0,-psMean.x/psStdev.x],
                              [0,1/psStdev.y,0,-psMean.y/psStdev.y],
                              [0,0,1/psStdev.z,-psMean.z/psStdev.z],[0,0,0,1]],
                            dnScale_robPos;
                       
        numMatrix2dnumMatrix [[1/pxStdev.u,0,-pxMean.u/pxStdev.u],
                              [0,1/pxStdev.v,-pxMean.v/pxStdev.v],
                              [0,0,1]],
                            dnScale_camPos;

        MatrixMultiplyDnum dnP,dnScale_robPos,result4;
        bDummy:=invert3DMatrix(dnScale_camPos,result3);
        MatrixMultiplyDnum result3,result4,dnP;
        abs_lambda:=SqrtDnum(dnP{3,1}*dnP{3,1}+dnP{3,2}*dnP{3,2}+dnP{3,3}*dnP{3,3});
        !Scale P with the scale factor
        FOR i FROM 1 TO 3 DO
            FOR j FROM 1 TO 4 DO
                dnP{i,j}:=dnP{i,j}/abs_lambda;
            ENDFOR
        ENDFOR
    ENDPROC
    
    LOCAL PROC normalize(num nNumPoses, pos psMeanS, pos psStdevS, pixel pxMeanU, pixel pxStdevU, INOUT pos psS{*}, INOUT pixel pxU{*})
        !Calculate normalization scaling
        FOR i FROM 1 TO nNumPoses DO
            psS{i}.x:=(psS{i}.x-psMeanS.x)/psStdevS.x;
            psS{i}.y:=(psS{i}.y-psMeanS.y)/psStdevS.y;
            psS{i}.z:=(psS{i}.z-psMeanS.z)/psStdevS.z;
            pxU{i}.u:=(pxU{i}.u-pxMeanU.u)/pxStdevU.u;
            pxU{i}.v:=(pxU{i}.v-pxMeanU.v)/pxStdevU.v;
        ENDFOR
    ENDPROC
    
    LOCAL FUNC pos getPosMean(num nNumPoses, pos ps{*})
        VAR pos psMean;
        FOR i FROM 1 TO nNumPoses DO
            psMean.x:=psMean.x + ps{i}.x/nNumPoses;
            psMean.y:=psMean.y + ps{i}.y/nNumPoses;
            psMean.z:=psMean.z + ps{i}.z/nNumPoses;
        ENDFOR
        RETURN psMean;
    ENDFUNC
    
    LOCAL FUNC pos getPosStdev(num nNumPoses, pos ps{*}, \INOUT pos psMean)
        VAR pos psStdev;
        
        psMean := getPosMean(nNumPoses, ps);
        FOR i FROM 1 TO nNumPoses DO
            psStdev.x:=psStdev.x+(ps{i}.x-psMean.x)*(ps{i}.x-psMean.x)/(nNumPoses-1);
            psStdev.y:=psStdev.y+(ps{i}.y-psMean.y)*(ps{i}.y-psMean.y)/(nNumPoses-1);
            psStdev.z:=psStdev.z+(ps{i}.z-psMean.z)*(ps{i}.z-psMean.z)/(nNumPoses-1);
        ENDFOR
        psStdev.x:=Sqrt(psStdev.x);
        psStdev.y:=Sqrt(psStdev.y);
        psStdev.z:=Sqrt(psStdev.z);
        
        RETURN psStdev;
    ENDFUNC
    
    LOCAL FUNC pixel getPixelMean(num nNumPoses, pixel px{*})
        VAR pixel pxMean;
        FOR i FROM 1 TO nNumPoses DO
            pxMean.u:=pxMean.u + px{i}.u/nNumPoses;
            pxMean.v:=pxMean.v + px{i}.v/nNumPoses;
        ENDFOR
        RETURN pxMean;
    ENDFUNC
    
    LOCAL FUNC pixel getPixelStdev(num nNumPoses, pixel px{*}, \INOUT pixel pxMean)
        VAR pixel pxStdev;
        pxMean := getPixelMean(nNumPoses, px);
        
        FOR i FROM 1 TO nNumPoses DO
            pxStdev.u:=pxStdev.u+(px{i}.u-pxMean.u)*(px{i}.u-pxMean.u)/(nNumPoses-1);
            pxStdev.v:=pxStdev.v+(px{i}.v-pxMean.v)*(px{i}.v-pxMean.v)/(nNumPoses-1);
        ENDFOR
        pxStdev.u:=Sqrt(pxStdev.u);
        pxStdev.v:=Sqrt(pxStdev.v);
        RETURN pxStdev;
    ENDFUNC
    
    
    ! Get the S_int and U_int datasets, for intrinsic calibration. 
    LOCAL PROC getIntSAndU(pixel pxCalibOffset, num nTransform{*,*}, INOUT num nNumPoses, INOUT pos psS{*}, INOUT pixel pxU{*})
        VAR num nIndex := 1;
        VAR pixel pxDetectedMarker;
        VAR robtarget temp;
        
        IF Dim(nTransform, 1) <> 3 OR Dim(nTransform,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        ! Loop through all good points to look at
        FOR px FROM 1 TO DIM(pxGoodPointsToVisit,1) DO
            ! If the marker is visible from this point (and point is reachable)
            IF placeMarkerAtPixel(pxGoodPointsToVisit{px}, pxCalibOffset, nTransform, pxDetectedMarker) THEN
                temp := getCurrentRobtarget();
                psS{nIndex} := temp.trans;
                TPWrite "Saves pixel "\Num:=px;
                pxU{nIndex} := pxDetectedMarker;
                Incr nIndex;
            ENDIF
        ENDFOR
        
        ! Minus one to compensate for last incrementation
        nNumPoses := nIndex - 1;
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
        CONST num nMaxDistance:=30;
        CONST num nDecreaseFactor := 0.5;
        VAR pixel psMaxDetectedMarker;
        VAR pixel psMinDetectedMarker;
        VAR num nPositiveDistance:=nMaxDistance;
        VAR num nNegativeDistance:=nMaxDistance;
        
        TEST nDirection
            CASE DIRECTION_X:
                WHILE NOT getMarkerInfoAtPos(pStart,[nPositiveDistance,0,0], psMaxDetectedMarker) DO
                    nPositiveDistance := nDecreaseFactor * nPositiveDistance;
                ENDWHILE
                WHILE NOT getMarkerInfoAtPos(pStart,[-nNegativeDistance,0,0], psMinDetectedMarker) DO
                    nNegativeDistance := nDecreaseFactor * nNegativeDistance;
                ENDWHILE
            CASE DIRECTION_Y:
                WHILE NOT getMarkerInfoAtPos(pStart,[0,nPositiveDistance,0], psMaxDetectedMarker) DO
                    nPositiveDistance := nDecreaseFactor * nPositiveDistance;
                ENDWHILE
                WHILE NOT getMarkerInfoAtPos(pStart,[0,-nNegativeDistance,0], psMinDetectedMarker) DO
                    nNegativeDistance := nDecreaseFactor * nNegativeDistance;
                ENDWHILE
            CASE DIRECTION_Z:
                WHILE NOT getMarkerInfoAtPos(pStart,[0,0,nPositiveDistance], psMaxDetectedMarker) DO
                    nPositiveDistance := nDecreaseFactor * nPositiveDistance;
                ENDWHILE
                WHILE NOT getMarkerInfoAtPos(pStart,[0,0,-nNegativeDistance], psMinDetectedMarker) DO
                    nNegativeDistance := nDecreaseFactor * nNegativeDistance;
                ENDWHILE
            DEFAULT: 
                TPWrite "Illegal direction axis specified";
                RAISE ERR_ARGVALERR;
        ENDTEST

        
        
        ! Moves back to start
        Move(pStart);
        
        ! Returns the differencies 
        traveledDistanceAlongAxis := nPositiveDistance + nNegativeDistance;
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



