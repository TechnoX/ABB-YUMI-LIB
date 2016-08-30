MODULE MainModule
    
    LOCAL RECORD pixel
        num u; ! Horizontal in computer viewed image
        num v; ! Vertical in computer viewed image
        ! If scale is used for a detection it can be saved in the pixel record. Otherwise uninitialized; 
        num scale; 
        ! angle around Z axis in camera coordinate system
        !num Rz;
    ENDRECORD
    
    LOCAL RECORD target
        pixel pxFrom;
        pixel pxTo;
        pos psAngle; !NOTE THIS IS EULER ANGLES; NOT ORIENT; TO BE ABLE TO SET IT INTIUTIVELY
    ENDRECORD
    
    
    ! "Enum" for the directions
    LOCAL CONST num DIRECTION_X:=0;
    LOCAL CONST num DIRECTION_Y:=1;
    LOCAL CONST num DIRECTION_Z:=2;
    
    ! True of moving the this arm, false if moving the other arm
    LOCAL VAR bool bMoveThisArm := FALSE;
    ! The camera to be used.
    LOCAL VAR cameradev cameraToUse;
    ! The position of the movable wrist when the calibration routine starts
    LOCAL VAR robtarget pStart;
    
    ! The general speed used during calibration
    LOCAL CONST speeddata vSpeed := v400;
    
    LOCAL CONST pixel pxImageSize := [1280,960,0];
    LOCAL CONST num nMargin := 200;
    LOCAL CONST num nScales{3} := [90, 70, 50];
    
    ! Can't handle more than 18 points because of some arbitrary limit inside MatrixSVD. "Report the problem to ABB Robotics"
    ! This is the maximum value for m in the paper.
    LOCAL CONST num nMaxIntPoints:=18;
    ! This is the maximum value for 2*n in the paper. Must be even for obv reasons. 
    LOCAL CONST num nMaxExtPoints:=8;
    
    ! Maximum number of pairs in (18) - (22).
    LOCAL CONST num nMaxPairs := 6;
    
    ! Temp tooldata used when passing around variables. Can't create pers inside routine, and tooldata is required to be PERS...
    LOCAL PERS tooldata tTempTool :=  [TRUE, [[0, 0, 0], [0.509439, -0.51102, -0.504796, -0.473826]], [0.001, [0, 0, 0.001], [1, 0, 0, 0], 0, 0, 0]];
    
    ! Used to pass position data between this and the other Motion Task for moving the other arm. 
    PERS robtarget pLogoTarget;
    
    
    
    
    
    
    ! This is for the example code in main(), not used by the calibration routines themselves.  
    PERS tooldata tMarker := [TRUE, [[247.271, -1619.63, -142.362], [1, 0, 0, 0]],
                             [0.001, [247.271, -1619.63, -142.362],[1, 0, 0, 0], 0, 0, 0]];
    PERS tooldata tCamera := [TRUE, [[36.8569, -1695.44, -174.353], [0.460121, -0.504528, -0.488578, -0.543168]],
                             [0.001, [36.8569, -1695.44, -174.353],[1, 0, 0, 0], 0, 0, 0]];
    PERS wobjdata wCamera := [FALSE, TRUE, "", [[368.467, -111.038, 195.797],[0.506313, -0.448169, 0.522104, 0.519807]],
                             [[0, 0, 0],[1, 0, 0, 0]]];
    PERS dnum dnK{3,3} :=    [[1653.61235861371,-3.87061033149386,626.08474435721],[0,1652.56549023444,496.636737396745],[0,0,1]];

    PROC main()
        
        ! Moves to a initial position with correct configuration. 
        MoveAbsJ [[85.7181,-100.266,26.4111,27.8509,94.6338,140.358],[-70.538,9E+09,9E+09,9E+09,9E+09,9E+09]]\NoEOffs, v1000, fine, tool_YuMiGripper_S_C;    
        
        ! Wait for other task
        WaitTime 1;
        ! TOOD: Use WaitSyncTask to be sure the other arm is standing still after its MoveAbsJ instruction
        
        ! Important that both arms have been moved to their initial positions before you call one of the routines below. 
        
        ! Run ONE of the below examples
        !CalibrateExternalCamera wCamera, tMarker, dnK;
        !CalibrateHandCamera tCamera, tMarker, dnK;
        CalibrateHandCamera tCamera, tMarker, dnK, \moveCamera;
        !bMoveThisArm := TRUE;
        
        
        !pStart := getCurrentRobtarget(\tTool:=tTempTool);
        
        !MoveInCameraFramePlane [0,0,100];
        !MoveInCameraFramePlane [0,0,-100];
        !MoveInCameraFramePlane [0,100,0];
        !MoveInCameraFramePlane [0,-100,0];
        !MoveInCameraFramePlane [100,0,0];
        !MoveInCameraFramePlane [-100,0,0];
        
        
        !MoveInCameraFramePlane [0,100,100],[0.5,-0.5,0.5,0.5],\psEulerAngle:=[0,0,20];
        !MoveInCameraFramePlane [0,-200,0],[0.5,-0.5,0.5,0.5],\psEulerAngle:=[0,0,20];
        !MoveInCameraFramePlane [100,100,0],[0.5,-0.5,0.5,0.5],\psEulerAngle:=[0,0,-20];
        !MoveInCameraFramePlane [-200,0,0],[0.5,-0.5,0.5,0.5],\psEulerAngle:=[0,0,-20];
        
        Stop \AllMoveTasks;
    ENDPROC

    
    ! Calibrate a fix camera (for example a camrea fixture above yumi). Movable marker 
    ! Needs the marker to be fasten to a movable arm, which is visible in the view in the beginning
    ! Resultatet ska vara en wobj för kamerans extrinsiska parametrar, samt en tcp eller tooldata för markören. 
    PROC CalibrateExternalCamera(INOUT wobjdata wCamera, INOUT tooldata tMarker, INOUT dnum dnK{*,*})
        VAR pose peRob2Cam;
        VAR pose peMarkerWrist2Marker;
        IF DIM(dnK,1) <> 3 OR Dim(dnK,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        PrintLog "Calibrate fix camera rig";
        
        ! Setup camera device
        setUpCameraDevice; 
        
        ! Should be the marker arm that call the calibration routine. 
        bMoveThisArm := TRUE;
        
        
        CalibrateInternal peRob2Cam, peMarkerWrist2Marker, dnK;
        
        tMarker.robhold := TRUE;
        tMarker.tframe := peMarkerWrist2Marker;
        tMarker.tload := [0.001, peMarkerWrist2Marker.trans, [1, 0, 0, 0], 0, 0, 0];
        
        wCamera.robhold := FALSE;
        wCamera.ufprog := TRUE;
        
        wCamera.uframe := peRob2Cam;
        wCamera.oframe := [[0,0,0],[1,0,0,0]];
    ENDPROC

    ! Calibrate a HandHeld Camera. 
    ! Move camera or move Calibration marker. Default is to move calibration marker. 
    PROC CalibrateHandCamera(INOUT tooldata tCamera, INOUT tooldata tMarker, INOUT dnum dnK{*,*}, \switch moveCamera)
        VAR pose peRob2Cam;
        ! Robtarget to the wrist that is holding the camera hand that is being calibrated
        VAR robtarget pRob2CamWrist;
        
        VAR pose peMarkerWrist2Marker;
        VAR pose peCamWrist2Cam;
        VAR pose peCamWrist2Rob;
        
        IF DIM(dnK,1) <> 3 OR Dim(dnK,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        PrintLog "Calibrate hand held camera";
        IF Present(moveCamera) THEN
            PrintLog "While moving the camera";
            bMoveThisArm := TRUE;
        ELSE
            PrintLog "While moving the calibration marker";
            bMoveThisArm := FALSE;
        ENDIF
        
        ! Setup camera device
        setUpCameraDevice; 
        
        pRob2CamWrist := getCurrentRobtarget(\other);
        CalibrateInternal peRob2Cam, peMarkerWrist2Marker, dnK, \moveCamera?moveCamera;

        peCamWrist2Rob := PoseInv([pRob2CamWrist.trans, pRob2CamWrist.rot]);
        peCamWrist2Cam := PoseMult(peCamWrist2Rob, peRob2Cam);
        
        tMarker.robhold := TRUE;
        tMarker.tframe := peMarkerWrist2Marker;
        tMarker.tload := [0.001, peMarkerWrist2Marker.trans, [1, 0, 0, 0], 0, 0, 0];
        
        tCamera.robhold := TRUE;
        tCamera.tframe := peCamWrist2Cam;
        tCamera.tload := [0.001, peCamWrist2Cam.trans, [1, 0, 0, 0], 0, 0, 0];
    ENDPROC
    
    
    LOCAL PROC CalibrateInternal(INOUT pose peRob2Cam, INOUT pose peMarkerWrist2Marker, INOUT dnum dnK{*,*}, \switch moveCamera)
        VAR orient orRob2Cam;
        VAR pos psRob2CHat;
        VAR pos psWrist2Marker;
        VAR orient orMarkerWrist2Rob;
        VAR dnum dnKInv{3,3};
        VAR num  nKInv{3,3};
        
        ! Get the start position of the moving arm (either the camera or the marker, depending on bMoveThisArm)
        pStart := getCurrentRobtarget();
        ! TODO: Can't have pStart here if the camera is moving.... Important!
        orMarkerWrist2Rob := QuatInv(pStart.rot);
        
        ! Phase one of the paper
        !calibIntrinsicAndRotation orRob2Cam, psRob2CHat, dnK, dnKinv;
        orRob2Cam := [0.487218,-0.517394,-0.520737,-0.473027];
        psRob2CHat := [413.957,177.691,255.447];
        dnKinv := [[0.000604736651120786,1.41640373316101E-06,-0.379319829749267],[0,0.000605119740130927,-0.30052469347299],[0,0,1]];
        !orRob2Cam := [0.573205,0.397208,0.431512,0.57224];
        !psRob2CHat := [155.257,24.6881,66.2292];
        !dnK := [[1649.52212675067,-2.14907467250056,635.975918352858],[0,1640.33360150003,498.794668040982],[0,0,1]];
        !dnKinv := [[0.000606236184275903,7.94257234009788E-07,-0.385947785306977],[0,0.000609632088915044,-0.30408123541751],[0,0,1]];
        
        
        TPWrite "Done with part 1";
        
        createTool orRob2Cam, \moveCamera?moveCamera;
        pStart := getCurrentRobtarget(\tTool:=tTempTool);
        
        
        
        dnumMatrix2numMatrix dnKinv, nKInv;
        ! Phase two and three of the paper
        calibExtrinsic orRob2Cam, nKInv, peRob2Cam, \moveCamera?moveCamera;
        
        ! The translation C-hat to C is the same as the translation Wrist to Marker, in compliance with Fig. 6, on page 5 of paper
        ! This is the distance / transformation from wrist to marker, in the base robot coordinate system.
        psWrist2Marker := peRob2Cam.trans - psRob2CHat;

        ! Rotate it around to get the translation in the wrist coordinate system
        peMarkerWrist2Marker.trans := PoseVect([[0,0,0],orMarkerWrist2Rob], psWrist2Marker);
        ! Just the identity matrix for rotation, as indicated on page 2 of paper. 
        peMarkerWrist2Marker.rot := [1,0,0,0];
        
	ENDPROC
    
    
    LOCAL PROC createTool(orient orRob2Cam, \switch moveCamera)
        ! Rotation of the wrist (of the arm with camrea) to the camera center. 
        VAR orient orCamWrist2Cam;
        ! Robtarget of the wrist with the camera
        VAR robtarget pRob2CamWrist;
        ! Transformation from camwrist to robot base frame coordinate system
        VAR pose peCamWrist2Rob;
        ! Transofrmation from camwrist to camera center.
        VAR pose peCamWrist2Cam;
        
        
        IF Present(moveCamera) THEN
            pRob2CamWrist := getCurrentRobtarget();
            peCamWrist2Rob := PoseInv([pRob2CamWrist.trans, pRob2CamWrist.rot]);
            peCamWrist2Cam := PoseMult(peCamWrist2Rob, [[0,0,0],orRob2Cam]);
            orCamWrist2Cam := peCamWrist2Cam.rot;
        ELSE ! TODO: Maybe this if-else-statement is unecessary even for external cams or moving ABB logo. But keep it for now to prevent anything from breaking
            orCamWrist2Cam := orRob2Cam;
        ENDIF
        tTempTool := [TRUE, [[0, 0, 0], orCamWrist2Cam], [0.001, [0, 0, 0.001], [1, 0, 0, 0], 0, 0, 0]];
        
    ENDPROC
    
    
    ! Normally it returns the arm that is moving (sometimes it is the hand with marker, other times it is the one with the camera)
    ! If you specify "other" you will get the robtarget for the stationary arm. 
    LOCAL FUNC robtarget getCurrentRobtarget(\PERS tooldata tTool, \switch other)
        IF Present(other) XOR bMoveThisArm THEN
            IF Present(tTool) THEN
                PrintLog "Get incorrect";
                RETURN CRobT(\Tool:=tTool,\WObj:=wobj0);
            ELSE
                PrintLog "Get correct position";
                RETURN CRobT(\Tool:=tool0,\WObj:=wobj0);
            ENDIF

        ELSE
            PrintLog "Get (left) incorrect";
            ! TODO: Fix this to work for both left and right arm. 
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
    LOCAL PROC calibExtrinsic(orient orRob2Cam, num nKInv{*,*}, INOUT pose peRob2Cam, \switch moveCamera)
        ! The pixel where the first marker was detected (during precalibration)
        VAR pixel pxPreOffset;
        ! The precalibration transform, used to place the logo at specific coordinates in the image
        VAR num nPreTransform{3,3};
        ! The angle precalibration transform, used to place the logo at specific coordinates in the image
        VAR num nAnglePreTransform{3,3};
        ! This is called S_ext in the paper on page 5. 
        VAR Pose peRob2Wrist{nMaxExtPoints};
        ! This is called U_ext in the paper on page 5.
        VAR pixel pxU{nMaxExtPoints};
        ! Called ct in paper
        VAR pos psCam2Marker{nMaxExtPoints};
        ! The number of points used for second phase (reorientations), this is number 2*n in the paper
        ! Needs to be even for obvious reasons (n is an integer).
        VAR num nNumPoints;
        ! Called t_ext in paper
        VAR pos psRob2Cam;
        VAR pixel pxDummy;
        VAR bool bDummy;

        VAR pos psLookAtMarkerAtEulerAngle;
        
        IF Dim(nKInv,1) <> 3 OR Dim(nKInv,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        ! Precalibration to get a sense of what direction is up, down, etc of the image
        ! The result here can be used to positionate the logo at specific pixels in the image
        ! Find an initial transformation from pixel coordinates to world coordinates
        !preCalibration nPreTransform, pxPreOffset, \moveCamera?moveCamera;
        nPreTransform := [[-0.186389,-0.00892854,0.343306],[0.00672944,-0.183832,0.193036],[-0.0180619,0.000956219,5.26818]];
        pxPreOffset := [700.733,578.631,60.0001];
        
        ! Do a angle precalibration as well
        preCalibration nAnglePreTransform, pxPreOffset, \moveCamera?moveCamera, \angular;
        
        
        getExtSAndU pxPreOffset, nPreTransform, nAnglePreTransform, peRob2Wrist, pxU, nNumPoints, \moveCamera?moveCamera;
        
        ! Equation (14) and (15) in paper
        getPointsInCameraFrame nNumPoints, peRob2Wrist, pxU, nKinv, psCam2Marker;
        
        
        ! Part three!! 
        TPWrite "Done with phase 2";
        
        SolveHandEyeEquation nNumPoints, orRob2Cam, peRob2Wrist, psCam2Marker, psRob2Cam;
        
        peRob2Cam.rot := orRob2Cam;
        peRob2Cam.trans := psRob2Cam;
        
        
    ENDPROC
    
    
    LOCAL FUNC pixel getNewOffset(pos psEulerAngle, pixel pxOldCalibOffset, num nAngleTransform{*,*}, \switch moveCamera)
        VAR pixel pxNewCalibOffset;
        VAR num nCoords{3};
        VAR num nRelAngleTarget{3};
        VAR num nAngleInvTransform{3,3};
        
        ! Do not rotate around Z axis, since it will not change the positon of the image (and is likely not calibrated at all). 
        IF psEulerAngle.z <> 0 THEN
            PrintLog "Z should be 0, ignores the Z rotation.";
            psEulerAngle.z := 0;
        ENDIF
        
        nRelAngleTarget{1} := psEulerAngle.x;
        nRelAngleTarget{2} := psEulerAngle.y;
        nRelAngleTarget{3} := psEulerAngle.z;
        
        invert2DMatrix nAngleTransform, nAngleInvTransform;
        
        MatrixMultiply2 nAngleInvTransform, nRelAngleTarget, nCoords;
        pxNewCalibOffset.u := nCoords{1} + pxOldCalibOffset.u;
        pxNewCalibOffset.v := nCoords{2} + pxOldCalibOffset.v;
        pxNewCalibOffset.scale := nCoords{3} + pxOldCalibOffset.scale;
                    
        RETURN pxNewCalibOffset;
    ENDFUNC
    
    
    
    
    
    LOCAL PROC SolveHandEyeEquation(num nNumPoses, orient orRob2Cam, pose peRob2Wrist{*}, pos psCam2Marker{*}, INOUT pos psRob2Cam)
        VAR pose peCam2Marker{nMaxExtPoints};
        ! A in the paper on p. 6 (TODO: Should be given a more descriptive name)
        ! Wrist movement (in robot frame) between two images
        VAR pose peA{nMaxPairs};
        ! B in the paper on p. 6 (TODO: Should be given a more descriptive name)
        ! Marker movement (in camera frame) between two images
        VAR pose peB{nMaxPairs};
        ! Number of pairs used to solve the hand eye equation, number of valid elements of nPairs. 
        VAR num nNumPairs := 6;
        ! Maybe take nPairs as an argument?? As it depends on the number of avaliable images/poses and may vary... 
        CONST num nPairs{nMaxPairs,2} := [[1,3],[2,4],[7,6],[3,8],[3,8],[1,6]];
        !!!!!!  TODO: Pair 4 and 5 are the same!! Fix it! 
        
        ! Equation (17)
        ! TODO: Refactor this up one level to the function before, i.e. inside getPointsInCameraFrame. I think it belongs more there...
        createCam2Marker nNumPoses, orRob2Cam, peRob2Wrist, psCam2Marker, peCam2Marker;
        
        ! Equation (18) and (19)
        createAB nNumPairs, nPairs, peRob2Wrist, peCam2Marker, peA, peB;
        
        ! Equation (22)
        getTExt nNumPairs, orRob2Cam, peA, peB, psRob2Cam;
        
        PrintLog "Done with part 3";
    ENDPROC

    
    ! psRob2Cam is denoted t_ext in the paper
    LOCAL PROC getTExt(num nNumPairs, orient orRext, pose peA{*}, pose peB{*}, INOUT pos psText)
        VAR dnum dnLeftMatrix{3*nMaxPairs,3};
        VAR dnum dnRightMatrix{3*nMaxPairs};
        VAR num nT{3,1};
        VAR num nTempA{4,4};
        VAR pose peRight;
        VAR dnum dnRob2Cam{3};

        IF Dim(peA,1) < nNumPairs OR  Dim(peB,1) < nNumPairs THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        ! Create the matrixes for the equation system (22)
        FOR p FROM 1 TO nNumPairs DO
            
            ! Create left side matrix
            posetoxform peA{p}, nTempA;
            FOR r FROM 1 TO 3 DO
                FOR c FROM 1 TO 3 DO
                    IF r = c THEN
                        dnLeftMatrix{3*p + r - 3, c} := NumToDnum(nTempA{r,c} - 1);
                    ELSE
                        dnLeftMatrix{3*p + r - 3, c} := NumToDnum(nTempA{r,c});
                    ENDIF
                ENDFOR
            ENDFOR
            
            ! Create right side matrix
            ! Rext * tB
            peRight := PoseMult([[0,0,0],orRext], [peB{p}.trans,[1,0,0,0]]);
            ! result from above minus tA
            peRight.trans := peRight.trans - peA{p}.trans;! := PoseMult([peRight.trans,[1,0,0,0]], [-peA{p}.trans,[1,0,0,0]]);
            
            dnRightMatrix{3*p - 2} := NumToDnum(peRight.trans.x);
            dnRightMatrix{3*p - 1} := NumToDnum(peRight.trans.y);
            dnRightMatrix{3*p - 0} := NumToDnum(peRight.trans.z);
        ENDFOR
        
        ! Solve the equation using least squares
        MatrixSolve dnLeftMatrix\A_m:=3*nNumPairs\A_n:=3,dnRightMatrix,dnRob2Cam;
        
        
        psText.x := DnumToNum(dnRob2Cam{1});
        psText.y := DnumToNum(dnRob2Cam{2});
        psText.z := DnumToNum(dnRob2Cam{3});
        
    ENDPROC
    
    
    LOCAL PROC createAB(num nNumPairs, num nPairs{*,*}, pose peRob2Wrist{*}, pose peCam2Marker{*}, INOUT pose peA{*}, INOUT pose peB{*})
        
        
        FOR i FROM 1 TO nNumPairs DO
            peA{i} := PoseMult(peRob2Wrist{nPairs{i,1}}, PoseInv(peRob2Wrist{nPairs{i,2}}));
            peB{i} := PoseMult(peCam2Marker{nPairs{i,1}}, PoseInv(peCam2Marker{nPairs{i,2}}));
        ENDFOR
        
        
    ENDPROC
    
    
    ! Augment the position data from the camera to include orientation, assigning the same orientation to the calibration marker as measured for the robot wrist
    LOCAL PROC createCam2Marker(num nNumPoses, orient orRob2Cam, pose peRob2Wrist{*}, pos psCam2Marker{*}, INOUT pose peCam2Marker{*})
        VAR orient orCam2Rob;
        VAR orient orCam2Wrist;
        
        orCam2Rob := QuatInv(orRob2Cam);
        FOR i FROM 1 TO nNumPoses DO
            orCam2Wrist := orCam2Rob * peRob2Wrist{i}.rot;
            ! Set the marker coordinate system to the same as the wrist have. 
            peCam2Marker{i}.rot := orCam2Wrist;
            peCam2Marker{i}.trans := psCam2Marker{i};
        ENDFOR
    ENDPROC
        
    ! Equation (14) and (15) in paper
    LOCAL PROC getPointsInCameraFrame(num nNumPoses, pose peRob2Wrist{*}, pixel pxU{*}, num nKinv{*,*}, INOUT pos psCam2Marker{*})
        VAR num nDistance; 
        
        IF Dim(peRob2Wrist,1) < nNumPoses OR Dim(pxU,1) < nNumPoses OR Dim(psCam2Marker,1) < nNumPoses THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        ! For all detected points
        FOR i FROM 1 TO nNumPoses/2 DO
            
            ! Get distance. Equation (14)
            ! TODO: Take several readings and do a mean value of them. Three was used for each height, in the old code...
            nDistance := getDistanceToCam(nKinv, peRob2Wrist{2*i-1}.trans, peRob2Wrist{2*i}.trans, 
                                                         pxU{2*i-1},               pxU{2*i});

            ! Project the pixel back to a 3D coordinate, equation (15) 
            psCam2Marker{2*i-1} := backProjectPixel(pxU{2*i-1}, nKInv, nDistance);
            psCam2Marker{2*i}   := backProjectPixel(pxU{2*i},   nKInv, nDistance);
            
        ENDFOR
    ENDPROC
    
    LOCAL FUNC pos backProjectPixel(pixel px, num nKinv{*,*}, num nDistance)
        VAR pos psRes;
        VAR num nCoord{3,1};
        VAR num nPixel{3,1};
        
        
        nPixel:=[[px.u],[px.v],[1]];
        MatrixMultiply nKInv, nPixel, nCoord;
        
        psRes.x := nCoord{1,1};
        psRes.y := nCoord{2,1};
        psRes.z := nCoord{3,1};
        
        ! Normalize vector (I think this is necessary? Was not that clear in the former code)
        psRes := NormalizePos(psRes);
        
        psRes := psRes * nDistance;
        
        RETURN psRes;
    ENDFUNC
    
    
    LOCAL FUNC num getDistanceToCam(num nKInv{*,*}, pos ps1, pos ps2, pixel px1, pixel px2)
        VAR pos psDelta;
        VAR pixel pxDelta;
        VAR num nRes{3,1};
        VAR pos psRes;
        VAR num nDistance; 
        
        psDelta:=ps1-ps2;
        pxDelta.u:=px1.u-px2.u;
        pxDelta.v:=px1.v-px2.v;
        
        ! TODO: Why have 0 as last element?? It should be 1 as a homogenous coordinate?
        ! TODO: ... or is it because the delta is of two homogenous coords and 1 - 1 = 0 ? :) 
        MatrixMultiply nKInv,[[pxDelta.u],[pxDelta.v],[0]], nRes;
        psRes:=[nRes{1,1}, nRes{2,1}, nRes{3,1}];
        
        nDistance := VectMagn(psDelta)/VectMagn(psRes);
        
        RETURN nDistance;
    ENDFUNC
    
    LOCAL PROC getExtSAndU(pixel pxPreOffset, num nPreTransform{*,*}, num nPreAngleTransform{*,*}, INOUT pose peS{*}, INOUT pixel pxU{*}, INOUT num nNumPoints, \switch moveCamera)
        VAR pixel pxFirstMarker;
        VAR pixel pxSecondMarker;
        VAR robtarget pFirstTarget;
        VAR robtarget pSecondTarget;
        VAR bool bFound;
        VAR target targets{4} := [[[pxImageSize.u/2, nMargin*1.5, 0],[pxImageSize.u/2, pxImageSize.v - nMargin*1.5, 0],[-30,0,0]],
                                  [[nMargin*1.5, pxImageSize.v/2, 0],[pxImageSize.u - nMargin*1.5, pxImageSize.v/2, 0],[0,-25,0]],
                                  [[nMargin*1.5, nMargin*1.5, 0],[pxImageSize.u - nMargin*1.5, pxImageSize.v - nMargin*1.5, 0],[-20,-20,0]],
                                  [[nMargin*1.5, pxImageSize.v/2, 0],[pxImageSize.u - nMargin*1.5, pxImageSize.v/2, 0],[0,0,90]]];
        CONST num decreaseFactor := 0.75;
        
        IF Dim(nPreTransform, 1) <> 3 OR Dim(nPreTransform,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        
        ! This will increase everytime we add a new pose
        nNumPoints := 0;
        ! Loop through all good points to look at
        FOR i FROM 1 TO Dim(targets,1) DO
            PrintLog "Search for angle #"\n:=i;
            bFound := FALSE;
            WHILE NOT bFound DO
                ! TODO: Fix so this not enters an infinite loop. This should be skipped after N tries. Important!!
                bFound := findMarkers(targets{i}, pxPreOffset, nPreAngleTransform, nPreTransform, \moveCamera?moveCamera, pxFirstMarker, pxSecondMarker, pFirstTarget, pSecondTarget);
                IF NOT bFound THEN
                    PrintLog "Can't see first marker! Decrease angle";
                    ! Decrease angle
                    targets{i}.psAngle := targets{i}.psAngle * decreaseFactor;
                ENDIF
            ENDWHILE
            
            ! Saves these positions
            !TPWrite "Saves pixel "\Num:=i;
            peS{2*i-1} := [pFirstTarget.trans, pFirstTarget.rot];
            peS{2*i} := [pSecondTarget.trans, pSecondTarget.rot];
            pxU{2*i-1} := pxFirstMarker;
            pxU{2*i} := pxSecondMarker;
            ! Increase number of saved poses with 2. 
            nNumPoints := nNumPoints + 2;
        ENDFOR
        
    ENDPROC
    
    
    ! Place marker in the middle of the image, while looking at it with the angle orAngle (relative to the base coordinate system). 
    LOCAL FUNC bool findMarkers(target t, pixel pxPreOffset, num nPreAngleTransform{*,*}, num nPreTransTransform{*,*}, \switch moveCamera, INOUT pixel pxFirstMarker, INOUT pixel pxSecondMarker, INOUT robtarget pFirstTarget, INOUT robtarget pSecondTarget)
        VAR pixel pxTempOffset;
        VAR num nDist := 1.0;
        VAR num nDecreaseFactor := 0.75;
        VAR robtarget pOldStart;
        ! Place logo outside image with angle
        pxTempOffset := getNewOffset(t.psAngle, pxPreOffset, nPreAngleTransform, \moveCamera?moveCamera);
        !Moves the hand
        MoveInCameraFramePlane [0,0,0], \psEulerAngle:=t.psAngle;

        ! Move the translation to get the image inside of view again. Without turning the angle. 
        
        ! Reset the new working plane....  TODO: It is not good to change start! 
        pOldStart := pStart; ! Backup old start;
        pStart := getCurrentRobtarget(\tTool:=tTempTool);
        
        ! IF Z ANGLE ADJUSTMENT! 
        !bDummy := getMarkerInfo(pxDummy);!placeMarkerAtPixel([640,480,70], pxPreOffset, nPreTransform, pxDummy, \moveCamera?moveCamera, \restrictZ);
        !pxPreOffset.u := pxPreOffset.u - pxDummy.u;
        !pxPreOffset.v := pxPreOffset.v - pxDummy.v;
        !pxPreOffset.scale := pxPreOffset.scale - pxDummy.scale;
        IF NOT placeMarkerAtPixel(t.pxFrom, pxTempOffset, nPreTransTransform, pxFirstMarker, \moveCamera?moveCamera, \restrictZ, \nMaxDistance:=400) THEN
            pStart := pOldStart;
            RETURN FALSE;
        ENDIF
        pFirstTarget := getCurrentRobtarget();
        PrintLog "First marker found";
        
        
        ! NOTE: When going to next point we are restricting the Z (optical axis) to not move at all.
        WHILE NOT placeMarkerAtPixel(t.pxTo, pxTempOffset, nPreTransTransform, pxSecondMarker, \moveCamera?moveCamera, \restrictZ, \nMaxDistance:=400) DO
            ! TODO: Fix so this not enters an infinite loop. This translation should be skipped after N tries. Important!!
            PrintLog "Can't see second marker, decrease distance";
            nDist := nDist * nDecreaseFactor;
            t.pxTo.u := t.pxFrom.u + nDist * (t.pxTo.u - t.pxFrom.u);
            t.pxTo.v := t.pxFrom.v + nDist * (t.pxTo.v - t.pxFrom.v);
            t.pxTo.scale := t.pxFrom.scale + nDist * (t.pxTo.scale - t.pxFrom.scale);
        ENDWHILE
        pSecondTarget := getCurrentRobtarget();
        PrintLog "Second marker found";
        pStart := pOldStart;
        RETURN TRUE;
    ENDFUNC
    
    ! Moves to a relative position (translation) in the camera frame
    ! With an optional rotation of the wrist relative camera center coordinate system
    ! NOTE: pStart needs to be calculated with the orientation from orCamWrist2Cam somewhere before (and the same orientation should be in tTempTool!)
    LOCAL PROC MoveInCameraFramePlane(pos relativeMovement, \pos psEulerAngle, \switch restrictZ)
        VAR num Rx := 0;
        VAR num Ry := 0;
        VAR num Rz := 0;
        IF Present(psEulerAngle) THEN
            Rx := psEulerAngle.x;
            Ry := psEulerAngle.x;
            Rz := psEulerAngle.z;
        ENDIF
        IF Present(restrictZ) THEN
            relativeMovement.z := 0;
        ENDIF
        
        MoveL reltool(pStart, relativeMovement.x, relativeMovement.y, relativeMovement.z, \Rx?psEulerAngle.x, \Ry?psEulerAngle.y, \Rz?psEulerAngle.z), vSpeed, fine, tTempTool, \WObj:=wobj0;
    ENDPROC
    
    
    ! Whole phase one of the paper.
    ! psCHat2Rob == The translation from C-hat to robot frame. This is actually the translation from Camera to Robot frame MINUS the translation from marker to wrist. 
    LOCAL PROC calibIntrinsicAndRotation(INOUT orient orRob2Cam, INOUT pos psRob2CHat, INOUT dnum dnK{*,*}, INOUT dnum dnKinv{*,*},\switch print)
        ! The pixel where the first marker was detected (during precalibration)
        VAR pixel pxPreOffset;
        ! The precalibration transform, used to place the logo at specific coordinates in the image
        VAR num nPreTransform{3,3};
        ! The number of detected markers, the m value in the paper
        VAR num nNumPoses;
        ! The first S dataset, from paper. Should maybe be called psRob2Wrist???
        VAR pos psRob2Wrist{nMaxIntPoints};
        ! The first U dataset, from paper
        VAR pixel pxU{nMaxIntPoints};
        ! The P matrix, from paper
        VAR dnum dnPhat{3,4};
        ! The rotation matrix
        VAR dnum dnR{3,3};
        ! The translation matrix, not correct values since it is calculated from the wrist
        VAR dnum dnT{3,1};
        ! Translation from CHat to robot in CHat coordinate system (I think...)
        VAR pose peRob2CHat;
        ! Translation from Robot to CHat in robot coordinate system
        VAR pose peCHat2Rob;
        
        ! Check dimensions of input arguments
        IF DIM(dnK,1) <> 3 OR DIM(dnK,2) <> 3 OR DIM(dnKinv,1) <> 3 OR DIM(dnKinv,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        
        ! Precalibration to get a sense of what direction is up, down, etc of the image
        ! The result here can be used to positionate the logo at specific pixels in the image
        ! Find an initial transformation from pixel coordinates to world coordinates
        preCalibration nPreTransform, pxPreOffset;

        
        ! Get measurements of pixels (U_int dataset) and their correspondences in real world (S_int dataset)
        getIntSAndU pxPreOffset, nPreTransform, nNumPoses, psRob2Wrist, pxU;
        
        ! Create a P (transformation matrix) from the S and U datasets
        findP nNumPoses, psRob2Wrist, pxU, dnPhat;
        IF Present(print) THEN
            printP nNumPoses, dnPhat, psRob2Wrist, pxU;
        ENDIF
        
        ! Creates a transform from robot to camera, and the intrinsic parameters dnK
        ! (as a side effect from the algorithm we also obtain the inverted K matrix, which can be good for future use). 
        IF NOT PMatDecompQR(dnPhat,dnK,dnKinv,dnR,dnT,1) THEN 
            ErrWrite "calibIntrinsicAndRotation", "Wrong matrix dimension";
            Stop;
        ENDIF
        ! Split this into two calls, one for R and another for R. They are not affecting each other. 
        createPoseFromRAndT dnR, dnT, peCHat2Rob;
        
        ! TODO: Analyze dnP before PMatDecompQR beforehand to select positive or negative sign?? So we don't need to check if the axes coincide afterwards. 
        
        ! If the axis was wrong, change direction of Z-axis
        IF NOT axesCoincide(peCHat2Rob.rot, psRob2Wrist, pxU) THEN
            IF NOT PMatDecompQR(dnPHat,dnK,dnKinv,dnR,dnT,-1) THEN 
                ErrWrite "calibIntrinsicAndRotation", "Wrong matrix dimension";
                Stop;
            ENDIF
            createPoseFromRAndT dnR, dnT, peCHat2Rob;
            
            ! If they still don't coincide, something is broken! 
            IF NOT axesCoincide(peCHat2Rob.rot, psRob2Wrist, pxU) THEN
                ErrWrite "calibIntrinsicAndRotation","Unable to match sign of axes";
                Stop;
            ENDIF
        ENDIF
        
        IF Present(print) THEN
            !Check that PmatdecompQR return values indeed compose P, P=K*[R t]
            checkP dnPhat, dnK, dnR, dnT;
        ENDIF
        
        
        ! Return the inverse transform, from robot base to camera coordinate system
        ! NOTE: This is called Rext in the paper! 
        !orRob2Cam:=QuatInv(orCam2Rob); ! Don't do it here, it will be done below within PoseInv()
        
        ! NOTE: Important to also invert the translation part, otherwise it gives the Rob position in the CHat coordinate system (rotated and translated)
        peRob2CHat := PoseInv(peCHat2Rob);
        Move(pStart);! Not neccesary!?
        orRob2Cam := peRob2CHat.rot;
        psRob2CHat := peRob2CHat.trans;
        
        ! TODO: Refine calibration result with non linear optimization, including distortion
        !CalibIntNonLin psS,psU,nNumPoses,peRob2Cam,dnK;
        
        
        ! Returns psCHat2Rob, orRob2Cam and K (and Kinv) values. Done with phase one.  
    ENDPROC
    
    !Check that PmatdecompQR return values indeed compose P, P=K*[R t]
    LOCAL PROC checkP(dnum dnP{*,*}, dnum dnK{*,*}, dnum dnR{*,*}, dnum dnT{*,*})
        VAR dnum dnP_test{3,4};
        
        MatrixMultiplyDnum dnK,[[dnR{1,1},dnR{1,2},dnR{1,3},dnT{1,1}],[dnR{2,1},dnR{2,2},dnR{2,3},dnT{2,1}],[dnR{3,1},dnR{3,2},dnR{3,3},dnT{3,1}]],dnP_test;
        PrintLog "P Error=";
        FOR i FROM 1 TO 3 DO
            PrintLog "["+dnumtostr(dnP{i,1}-dnP_test{i,1},3)+","+dnumtostr(dnP{i,2}-dnP_test{i,2},3)+","+dnumtostr(dnP{i,3}-dnP_test{i,3},3)+","+dnumtostr(dnP{i,4}-dnP_test{i,4},3)+"]";
        ENDFOR
    ENDPROC
    
    LOCAL PROC printP(num nNumPoses, dnum dnP{*,*}, pos psRob2Wrist{*}, pixel pxU{*})
        ! The back projected point
        VAR dnum dnHomoPixel{3,1};
        VAR dnum dnHomoCoord{4,1};
        VAR pixel pxBackProjectedError;
        
        !Calculate back projection error
        PrintLog "Back Projection Error=";
        FOR i FROM 1 TO nNumPoses DO
            dnHomoCoord := [[NumToDnum(psRob2Wrist{i}.x)],[NumToDnum(psRob2Wrist{i}.y)],[NumToDnum(psRob2Wrist{i}.z)],[1]];
            MatrixMultiplyDnum dnP,dnHomoCoord,dnHomoPixel;
            ! Normalize resulting backprojected pixel
            dnHomoPixel{1,1}:=dnHomoPixel{1,1}/dnHomoPixel{3,1};
            dnHomoPixel{2,1}:=dnHomoPixel{2,1}/dnHomoPixel{3,1};
            ! Calc error 
            pxBackProjectedError.u := DnumToNum(dnHomoPixel{1,1}) - pxU{i}.u;
            pxBackProjectedError.v := DnumToNum(dnHomoPixel{2,1}) - pxU{i}.v;
            ! Print error
            PrintLog "error u: " + NumToStr(pxBackProjectedError.u,3) + ", v: " + NumToStr(pxBackProjectedError.v,3);
            WaitTime 1;
        ENDFOR
    ENDPROC
    
    
    
    LOCAL FUNC bool axesCoincide(orient orCam2Rob, pos psS{*}, pixel pxU{*})
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
        ps1:=PoseVect([[0,0,0],orCam2Rob],[psS{1}.x,psS{1}.y,psS{1}.z]);
        ps2:=PoseVect([[0,0,0],orCam2Rob],[psS{2}.x,psS{2}.y,psS{2}.z]);
        ps3:=PoseVect([[0,0,0],orCam2Rob],[psS{3}.x,psS{3}.y,psS{3}.z]);
        ps1.z:=0;
        ps2.z:=0;
        ps3.z:=0;
        d12:=DotProd(NormalizePos(ps2-ps1),NormalizePos([pxU{2}.u,pxU{2}.v,0]-[pxU{1}.u,pxU{1}.v,0]));
        d13:=DotProd(NormalizePos(ps3-ps1),NormalizePos([pxU{3}.u,pxU{3}.v,0]-[pxU{1}.u,pxU{1}.v,0]));
        !0.05 for poorly calibrated robot, 0.01 for properly calibrated robot
        ! TODO: Check this for external fixed camera, it may be incorrect now....  I'm unsure about what this checks actually. 
        IF bMoveThisArm THEN 
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
    
    
    
    LOCAL PROC createPoseFromRAndT(dnum dnR{*,*}, dnum dnT{*,*}, INOUT pose pePose)
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
        pePose.rot := vec2quat(ps1,ps2,ps3);
        
        ! Create translation vector
        pePose.trans := [dnumtonum(dnT{1,1}),dnumtonum(dnT{2,1}),dnumtonum(dnT{3,1})];
    ENDPROC
    

    LOCAL PROC findP(num nNumPoses, pos psRob2Wrist{*}, pixel pxU{*}, INOUT dnum dnP{*,*})
        VAR dnum dnMatrix{2*nMaxIntPoints,12};
        VAR dnum dnU{2*nMaxIntPoints,12};
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
        
        psStdevS := getPosStdev(nNumPoses, psRob2Wrist, \psMean:=psMeanS);
        pxStdevU := getPixelStdev(nNumPoses, pxU, \pxMean:=pxMeanU);
        
        normalize nNumPoses, psMeanS, psStdevS,pxMeanU,pxStdevU,psRob2Wrist,pxU; 
        
        
        ! Create big matrix used for finding P using SVD, see eq. 6 in paper. 
        FOR r2 FROM 1 TO nNumPoses DO
            ! First row
            dnMatrix{2*r2-1,1} := NumToDnum(psRob2Wrist{r2}.x);
            dnMatrix{2*r2-1,2} := NumToDnum(psRob2Wrist{r2}.y);
            dnMatrix{2*r2-1,3} := NumToDnum(psRob2Wrist{r2}.z);
            dnMatrix{2*r2-1,4} := 1;
            dnMatrix{2*r2-1,5} := 0;
            dnMatrix{2*r2-1,6} := 0;
            dnMatrix{2*r2-1,7} := 0;
            dnMatrix{2*r2-1,8} := 0;
            dnMatrix{2*r2-1,9} :=  - NumToDnum(pxU{r2}.u * psRob2Wrist{r2}.x);
            dnMatrix{2*r2-1,10} := - NumToDnum(pxU{r2}.u * psRob2Wrist{r2}.y);
            dnMatrix{2*r2-1,11} := - NumToDnum(pxU{r2}.u * psRob2Wrist{r2}.z);
            dnMatrix{2*r2-1,12} := - NumToDnum(pxU{r2}.u);
            ! Second row
            dnMatrix{2*r2,1} := 0;
            dnMatrix{2*r2,2} := 0;
            dnMatrix{2*r2,3} := 0;
            dnMatrix{2*r2,4} := 0;
            dnMatrix{2*r2,5} := NumToDnum(psRob2Wrist{r2}.x);
            dnMatrix{2*r2,6} := NumToDnum(psRob2Wrist{r2}.y);
            dnMatrix{2*r2,7} := NumToDnum(psRob2Wrist{r2}.z);
            dnMatrix{2*r2,8} := 1;
            dnMatrix{2*r2,9} :=  - NumToDnum(pxU{r2}.v * psRob2Wrist{r2}.x);
            dnMatrix{2*r2,10} := - NumToDnum(pxU{r2}.v * psRob2Wrist{r2}.y);
            dnMatrix{2*r2,11} := - NumToDnum(pxU{r2}.v * psRob2Wrist{r2}.z);
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
        CONST pixel pxGoodPointsToVisit{nMaxIntPoints} := [[pxImageSize.u/2             , pxImageSize.v/2             , nScales{1}],
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
        
        
        IF Dim(nTransform, 1) <> 3 OR Dim(nTransform,2) <> 3 THEN
            RAISE ERR_ILLDIM;
        ENDIF
        
        ! Loop through all good points to look at
        FOR px FROM 1 TO DIM(pxGoodPointsToVisit,1) DO
            ! If the marker is visible from this point (and point is reachable)
            IF placeMarkerAtPixel(pxGoodPointsToVisit{px}, pxCalibOffset, nTransform, pxDetectedMarker\nMaxDistance:=220) THEN
                temp := getCurrentRobtarget();
                psS{nIndex} := temp.trans;
                PrintLog "Saves pixel "\n:=px;
                pxU{nIndex} := pxDetectedMarker;
                Incr nIndex;
            ENDIF
        ENDFOR
        
        ! Minus one to compensate for last incrementation
        nNumPoses := nIndex - 1;
    ENDPROC
    
    ! If a moveCamera is provided the movements will be done using the camera frame instead of absolute coordinates. 
    LOCAL PROC preCalibration(INOUT num transform{*,*}, INOUT pixel offset, \switch moveCamera, \switch angular)
        VAR num nTransform{3,3};
        VAR bool bDummy;
        
        IF DIM(transform,1) <> 3 OR DIM(transform,1) <> 3 THEN
            PrintLog "The transform must be of size 3x3";
            RAISE ERR_ILLDIM;
        ENDIF
        
        ! Image coords from the original marker (that is at the "pStart" position)
        bDummy := getMarkerInfoAtPos([0,0,0], offset, \moveCamera?moveCamera);
        calculateMovementTransform nTransform, \moveCamera?moveCamera, \angular?angular;
        IF Present(angular) THEN
            invert2DMatrix nTransform, transform;
        ELSE
            invert3x3Matrix nTransform, transform;
        ENDIF
    ENDPROC
    
    
    ! If restrictZ = True, don't move the hand anything along the Z axis. 
    ! psEulerAngle: Relative angle to turn the wrist (around the camera coordinate system axes)
    LOCAL FUNC bool placeMarkerAtPixel(pixel pxTarget, pixel pxCalibOffset, num nTransform{*,*}, INOUT pixel pxDetectedMarker, \switch moveCamera, \switch restrictZ, \pos psEulerAngle, \num nMaxDistance, \switch angular)
        VAR num nCoords{3};
        VAR pos psRelPos;
        VAR num nRelPixelTarget{3};
        VAR num nDist;
        VAR num nMaxDist;
        
        ! Max distance to be considered a true detection
        IF Present(nMaxDistance) THEN
            nMaxDist := nMaxDistance;
        ELSE
            nMaxDist := 200; 
        ENDIF
        
        nRelPixelTarget{1} := pxTarget.u - pxCalibOffset.u;
        nRelPixelTarget{2} := pxTarget.v - pxCalibOffset.v;
        nRelPixelTarget{3} := pxTarget.scale - pxCalibOffset.scale;
        
        MatrixMultiply2 nTransform, nRelPixelTarget, nCoords;
        psRelPos.x := nCoords{1};
        psRelPos.y := nCoords{2};
        psRelPos.z := nCoords{3};
        
        !If not detected at all
        IF Present(angular) THEN
            ! Do not rotate around Z axis, since it will not change the positon of the image (and is not calibrated at all). 
            psRelPos.z := 0; ! TODO: Isn't this already 0 in all cases, because last row in transform will be 0 0 0???
            IF NOT getMarkerInfoAtPos([0,0,0], pxDetectedMarker, \moveCamera?moveCamera, \psEulerAngle:=psRelPos) THEN
                PrintLog "Marker not detected! Not saved!";
                RETURN FALSE;
            ENDIF
        ELSE
            IF NOT getMarkerInfoAtPos(psRelPos, pxDetectedMarker, \moveCamera?moveCamera, \psEulerAngle?psEulerAngle, \restrictZ?restrictZ) THEN
                PrintLog "Marker not detected! Not saved!";
                RETURN FALSE;
            ENDIF
        ENDIF
        
        ! If detected at wrong position in the image, we are discarding this detection
        nDist := getPixelDistance(pxDetectedMarker, pxTarget);
        !TPWrite "Pixel distance: "\Num:=nDist;
        IF nDist > nMaxDist THEN
            PrintLog "Marker detected at wrong location in image!! "\n:=nDist;
            RETURN FALSE;
        ENDIF
        
        RETURN TRUE;
    ENDFUNC
    
    
    LOCAL FUNC num getPixelDistance(pixel px1, pixel px2)
        RETURN Sqrt((px1.u - px2.u) * (px1.u - px2.u) + (px1.v - px2.v) * (px1.v - px2.v) + (px1.scale - px2.scale) * (px1.scale - px2.scale));
    ENDFUNC
    
    LOCAL PROC calculateMovementTransform(INOUT num nTransform{*,*}, \switch moveCamera, \switch angular)
        VAR num nDistances{3};
        VAR pixel nPixelDistances{3};
        IF DIM(nTransform,1) <> 3 OR DIM(nTransform,1) <> 3 THEN
            PrintLog "The transform must be of size 3x3";
            RAISE ERR_ILLDIM;
        ENDIF
        ! Moves in X direction of robot-frame and get moved distance in X and pixels. 
        getMarkerData DIRECTION_X, nDistances{1}, nPixelDistances{1},\moveCamera?moveCamera, \angular?angular;
        
        ! Moves in Y direction of robot-frame and get moved distance in Y and pixels. 
        getMarkerData DIRECTION_Y, nDistances{2}, nPixelDistances{2},\moveCamera?moveCamera, \angular?angular;

        IF NOT Present(angular) THEN
            ! Moves in Z direction of robot-frame and get moved distance in Z and pixels. 
            getMarkerData DIRECTION_Z, nDistances{3}, nPixelDistances{3},\moveCamera?moveCamera, \angular?angular;
        ENDIF
        
        IF Present(angular) THEN
            FOR c FROM 1 TO 2 DO
                nTransform{1,c} := nPixelDistances{c}.u / nDistances{c};
                nTransform{2,c} := nPixelDistances{c}.v / nDistances{c};
            ENDFOR
        ELSE
            ! Probably possible to make this to a Matrix multiplication?
            ! But this is at least working. And pretty clear? :)
            FOR c FROM 1 TO 3 DO
                nTransform{1,c} := nPixelDistances{c}.u / nDistances{c};
                nTransform{2,c} := nPixelDistances{c}.v / nDistances{c};
                nTransform{3,c} := nPixelDistances{c}.scale / nDistances{c};
            ENDFOR
        ENDIF
    ENDPROC
    
    
    
    
    ! Moves in the arm along the specified axis to the most negative and then most positive to find out the biggest difference in pixel distance
    LOCAL PROC getMarkerData(num nDirection, INOUT num traveledDistanceAlongAxis, INOUT pixel pixelDifferences, \switch moveCamera, \switch angular)
        CONST num nMaxDistance:=30;
        CONST num nMaxAngle:=10;
        CONST num nDecreaseFactor := 0.5;
        VAR pixel psMaxDetectedMarker;
        VAR pixel psMinDetectedMarker;
        VAR num nPositiveDistance;
        VAR num nNegativeDistance;
        IF Present(angular) THEN
            nPositiveDistance:=nMaxAngle;
            nNegativeDistance:=nMaxAngle;
        ELSE
            nPositiveDistance:=nMaxDistance;
            nNegativeDistance:=nMaxDistance;
        ENDIF
        
        TEST nDirection
            CASE DIRECTION_X:
                IF Present(angular) THEN
                    WHILE NOT getMarkerInfoAtPos([0,0,0], psMaxDetectedMarker,\moveCamera?moveCamera,\psEulerAngle:=[nPositiveDistance,0,0]) DO
                        nPositiveDistance := nDecreaseFactor * nPositiveDistance;
                    ENDWHILE
                    WHILE NOT getMarkerInfoAtPos([0,0,0], psMinDetectedMarker,\moveCamera?moveCamera,\psEulerAngle:=[-nNegativeDistance,0,0]) DO
                        nNegativeDistance := nDecreaseFactor * nNegativeDistance;
                    ENDWHILE
                ELSE
                    WHILE NOT getMarkerInfoAtPos([nPositiveDistance,0,0], psMaxDetectedMarker,\moveCamera?moveCamera) DO
                        nPositiveDistance := nDecreaseFactor * nPositiveDistance;
                    ENDWHILE
                    WHILE NOT getMarkerInfoAtPos([-nNegativeDistance,0,0], psMinDetectedMarker,\moveCamera?moveCamera) DO
                        nNegativeDistance := nDecreaseFactor * nNegativeDistance;
                    ENDWHILE
                ENDIF
            CASE DIRECTION_Y:
                IF Present(angular) THEN
                    WHILE NOT getMarkerInfoAtPos([0,0,0], psMaxDetectedMarker,\moveCamera?moveCamera,\psEulerAngle:=[0,nPositiveDistance,0]) DO
                        nPositiveDistance := nDecreaseFactor * nPositiveDistance;
                    ENDWHILE
                    WHILE NOT getMarkerInfoAtPos([0,0,0], psMinDetectedMarker,\moveCamera?moveCamera,\psEulerAngle:=[0,-nNegativeDistance,0]) DO
                        nNegativeDistance := nDecreaseFactor * nNegativeDistance;
                    ENDWHILE
                ELSE
                    WHILE NOT getMarkerInfoAtPos([0,nPositiveDistance,0], psMaxDetectedMarker,\moveCamera?moveCamera) DO
                        nPositiveDistance := nDecreaseFactor * nPositiveDistance;
                    ENDWHILE
                    WHILE NOT getMarkerInfoAtPos([0,-nNegativeDistance,0], psMinDetectedMarker,\moveCamera?moveCamera) DO
                        nNegativeDistance := nDecreaseFactor * nNegativeDistance;
                    ENDWHILE
                ENDIF
            CASE DIRECTION_Z:
                IF Present(angular) THEN
                    WHILE NOT getMarkerInfoAtPos([0,0,0], psMaxDetectedMarker,\moveCamera?moveCamera,\psEulerAngle:=[0,0,nPositiveDistance]) DO
                        nPositiveDistance := nDecreaseFactor * nPositiveDistance;
                    ENDWHILE
                    WHILE NOT getMarkerInfoAtPos([0,0,0], psMinDetectedMarker,\moveCamera?moveCamera,\psEulerAngle:=[0,0,-nNegativeDistance]) DO
                        nNegativeDistance := nDecreaseFactor * nNegativeDistance;
                    ENDWHILE
                ELSE
                    WHILE NOT getMarkerInfoAtPos([0,0,nPositiveDistance], psMaxDetectedMarker,\moveCamera?moveCamera) DO
                        nPositiveDistance := nDecreaseFactor * nPositiveDistance;
                    ENDWHILE
                    WHILE NOT getMarkerInfoAtPos([0,0,-nNegativeDistance], psMinDetectedMarker,\moveCamera?moveCamera) DO
                        nNegativeDistance := nDecreaseFactor * nNegativeDistance;
                    ENDWHILE
                ENDIF
            DEFAULT: 
                PrintLog "Illegal direction axis specified";
                RAISE ERR_ARGVALERR;
        ENDTEST

        
        !! TODO:  Try to detect false detections here somehow!
        ! TODO: Up down, right, left needs to form a valid rectangle, right? Otherwise we have a false detection somewhere!!
        
        
        ! Moves back to start
        !! Move(pStart); ! Shouldn't be here! At least not without correct tool! Makes robot turn around very bad. 
        
        ! Returns the differencies 
        traveledDistanceAlongAxis := nPositiveDistance + nNegativeDistance;
        pixelDifferences.u := psMaxDetectedMarker.u - psMinDetectedMarker.u;
        pixelDifferences.v := psMaxDetectedMarker.v - psMinDetectedMarker.v;
        pixelDifferences.scale := psMaxDetectedMarker.scale - psMinDetectedMarker.scale;
    ERROR
        PrintLog "!!!!!!!Error in getMaxVisibleDistance";
        TRYNEXT;
    ENDPROC
    

    LOCAL PROC Move(robtarget pTarget)
        IF bMoveThisArm THEN
            MoveThisArm pTarget;
        ELSE
            MoveOtherArm pTarget;
        ENDIF
    ENDPROC
    
    LOCAL PROC MoveThisArm(robtarget pTarget)
        VAR jointtarget jtDummy;
        
        ! This is not used anywhere, just to throw error if position is not reachable. The error thrown by MoveL didn't seem catchable
        jtDummy := CalcJointT(pTarget, tool0, \WObj:=wobj0);
        ! Moves the robot
        MoveL pTarget, vSpeed, fine, tool0, \WObj:=wobj0;
    ERROR
        PrintLog "Destination not reachable";
        RETURN;
    ENDPROC

    LOCAL PROC MoveOtherArm(robtarget pTarget)
        pLogoTarget:=pTarget;
        SetDO doLogoIsMoving,1;
        WaitDO doLogoIsMoving,0;
    ENDPROC
    
    LOCAL PROC moveToPos(pos psRelPos)
        VAR robtarget pTarget;
        ! Moves the robot to a new target
        pTarget := Offs(pStart, psRelPos.x,psRelPos.y,psRelPos.z);
        
        Move(pTarget);
    ENDPROC
    
    LOCAL FUNC bool getMarkerInfoAtPos(pos psRelPos, INOUT pixel detectedMarker, \switch moveCamera, \pos psEulerAngle, \switch restrictZ)
        IF Present(moveCamera) THEN
            PrintLog "Position: "\p:=psRelPos;
            PrintLog "Angle: "\p?psEulerAngle;
            MoveInCameraFramePlane psRelPos, \psEulerAngle?psEulerAngle, \restrictZ?restrictZ;
        ELSE
            moveToPos psRelPos;
        ENDIF
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
            IF NOT RequestImage(psCameraResult) THEN
                Incr nNoOffPicture;
                PrintLog "Picture not OK. Try no = "\n:=nNoOffPicture;
            ELSE
                RETURN TRUE;
            ENDIF
        ENDWHILE
        RETURN FALSE;
    ENDFUNC

    LOCAL FUNC bool RequestImage(INOUT pixel px)
        VAR cameratarget tgt;
        CamReqImage cameraToUse;
        CamGetResult cameraToUse,tgt\MaxTime:=5;
        !WaitTime 2.0;
        px.u := tgt.cframe.trans.x;
        px.v := tgt.cframe.trans.y;
        px.scale := tgt.val1;
        !px.Rz := EulerZYX (\Z, tgt.cframe.rot);
        RETURN TRUE;
    ERROR
        IF ERRNO=ERR_CAM_MAXTIME THEN
            PrintLog "Could not find the target";
            RETURN FALSE;
        ELSEIF ERRNO=ERR_CAM_COM_TIMEOUT THEN
            PrintLog "Camera communication timeout";
            WaitTime 2.0;
            RETRY;
        ENDIF
    ENDFUNC
    
    LOCAL PROC PrintLog(string text,\num n,\pos p)
        TPWrite text\Num?n\Pos?p;
        ErrWrite \I, text, "a";
    ENDPROC
    
ENDMODULE
