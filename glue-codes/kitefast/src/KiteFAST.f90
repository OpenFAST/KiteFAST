module KiteFAST
   use, intrinsic :: ISO_C_Binding 
   use NWTC_Library 
   use MoorDyn_Types
   use MoorDyn
   use InflowWind_Types
   use InflowWind
   use KiteAeroDyn_Types
   use KiteAeroDyn
   use KiteFAST_Types
   
   implicit none 

private

   type(ProgDesc), parameter  :: KFAST_Ver = ProgDesc( 'KiteFAST', '', '' )
   integer,        parameter  :: IntfStrLen  = 1025       ! length of strings through the C interface
   type(KFAST_ParameterType)  :: p
   type(KFAST_MiscVarType)  :: m
   public :: KFAST_Init
   public :: KFAST_AssRes
   public :: KFAST_End


contains
subroutine CreateMBDynL2MotionsMesh(origin, numNodes, positions, alignDCM, nodeDCMs, mesh, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numNodes          !< Number of nodes in the mesh
   real(ReKi),                   intent(in   )  :: positions(:,:)    !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the Kite axes
   real(R8Ki),                   intent(in   )  :: nodeDCMs(3,3,numNodes) !< DCM needed to transform into a node's axes
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   integer(intKi)                               :: j                 ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateL2MotionsMesh'

      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   call MeshCreate ( BlankMesh = mesh     &
                     ,IOS       = COMPONENT_INPUT &
                     ,Nnodes    = numNodes        &
                     ,errStat   = errStat2        &
                     ,ErrMess   = errMsg2         &
                     ,Orientation     = .true.    &
                     ,TranslationDisp = .true.    &
                     ,TranslationVel = .true.    &
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation
   position = 0.0_ReKi
   do j=1,numNodes       
      position = positions(:,j) - origin(:) 
      position = matmul(alignDCM, position) 
      orientation = nodeDCMs(:,:,j)
      orientation = matmul(orientation, transpose(alignDCM))
      
      call MeshPositionNode(mesh, j, position, errStat2, errMsg2, orientation)  
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
   end do !j
         
      ! create line2 elements
   do j=1,numNodes-1
      call MeshConstructElement( mesh, ELEMENT_LINE2, errStat2, errMsg2, p1=j, p2=j+1 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   end do 
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return

      
   mesh%Orientation     = mesh%RefOrientation
   mesh%TranslationDisp = 0.0_R8Ki
   mesh%TranslationVel  = 0.0_ReKi
   
   
end subroutine CreateMBDynL2MotionsMesh  

subroutine CreateMBDynPtMotionsMesh(origin, positionIn, alignDCM, nodeDCM, mesh, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   real(ReKi),                   intent(in   )  :: positionIn(3)     !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the Kite axes
   real(R8Ki),                   intent(in   )  :: nodeDCM(3,3)      !< DCM needed to transform into a node's axes
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateL2MotionsMesh'

      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   call MeshCreate ( BlankMesh = mesh     &
                     ,IOS       = COMPONENT_INPUT &
                     ,Nnodes    = 1               &
                     ,errStat   = errStat2        &
                     ,ErrMess   = errMsg2         &
                     ,Orientation     = .true.    &
                     ,TranslationDisp = .true.    &
                     ,TranslationVel = .true.    &
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation
   position = 0.0_ReKi
        
   position = positionIn - origin
   position = matmul(alignDCM, position) 
   orientation = matmul(nodeDCM, transpose(alignDCM))
      
   call MeshPositionNode(mesh, 1, position, errStat2, errMsg2, orientation)  
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

         
      ! create point element
  
      call MeshConstructElement( mesh, ELEMENT_POINT, errStat2, errMsg2, p1=1 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return

      
   mesh%Orientation     = mesh%RefOrientation
   mesh%TranslationDisp = 0.0_R8Ki
   mesh%TranslationVel  = 0.0_ReKi
   
   
end subroutine CreateMBDynPtMotionsMesh  
   
subroutine KFAST_Init(dt, numFlaps, numPylons, numComp, numCompNds, KAD_FileName_c, IfW_FileName_c, MD_FileName_c, &
                       outFileRoot_c, gravity, FusODCM_c, numRtrPtsElem_c, rtrPts_c, numRefPtElem_c, refPts_c, numDCMElem_c, nodeDCMs_c, errStat_c, errMsg_c ) BIND (C, NAME='KFAST_Init')
   IMPLICIT NONE
#ifndef IMPLICIT_DLLEXPORT
!DEC$ ATTRIBUTES DLLEXPORT :: KFAST_Init
!GCC$ ATTRIBUTES DLLEXPORT :: KFAST_Init
#endif

   real(C_DOUBLE),         intent(in   ) :: dt                         ! simulation time step size (s)
   integer(C_INT),         intent(in   ) :: numFlaps                   ! number of flaps per wing
   integer(C_INT),         intent(in   ) :: numPylons                  ! number of pylons per wing
   integer(C_INT),         intent(in   ) :: numComp                    ! number of individual components in the kite
   integer(C_INT),         intent(in   ) :: numCompNds(numComp)        ! number of nodes per kite component
   character(kind=C_CHAR), intent(in   ) :: KAD_FileName_c(IntfStrLen) ! name of KiteAeroDyn input file
   character(kind=C_CHAR), intent(in   ) :: IfW_FileName_c(IntfStrLen) ! name of InflowWind input file
   character(kind=C_CHAR), intent(in   ) :: MD_FileName_c(IntfStrLen)  ! name of MoorDyn input file
   character(kind=C_CHAR), intent(in   ) :: outFileRoot_c(IntfStrLen)  ! root name of any output file generated by KiteFAST
   real(C_DOUBLE),         intent(in   ) :: gravity                    ! gravitational constant (m/s^2)
!   real(C_DOUBLE),         intent(in   ) :: FusO(3)                    ! fuselage principal reference point (MIP of kite)
   real(C_DOUBLE),         intent(in   ) :: FusODCM_c(9)               ! fuselage principal reference point DCM (MIP of kite)
   integer(C_INT),         intent(in   ) :: numRtrPtsElem_c            ! total number of rotor point elements
   real(C_DOUBLE),         intent(in   ) :: rtrPts_c(numRtrPtsElem_c)  ! location of the rotor points
   integer(C_INT),         intent(in   ) :: numRefPtElem_c             ! total number of array elements in the reference points array
   real(C_DOUBLE),         intent(in   ) :: refPts_c(numRefPtElem_c)   ! 1D array containing all the reference point data
   integer(C_INT),         intent(in   ) :: numDCMElem_c               ! total number of array elements in the nodal DCM data
   real(C_DOUBLE),         intent(in   ) :: nodeDCMs_c(numDCMElem_c)   ! 1D array containing all the nodal DCM data
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

      ! Local variables
   real(ReKi)                      :: FusO(3), SWnO(3), PWnO(3), VSPO(3), SHSO(3), PHSO(3), FusODCM(3,3)
   real(ReKi)                      :: SPyO(3,numPylons), PPyO(3,numPylons), SPyRtrO(3,2,numPylons), PPyRtrO(3,2,numPylons)
   integer(IntKi)                  :: numFusNds, numSwnNds, numPWnNds, numVSPNds, numSHSNds, numPHSNds, numSPyNds(numPylons), numPPyNds(numPylons)
   real(R8Ki), allocatable         :: FusNdDCMs(:,:,:),SWnNdDCMs(:,:,:),PWnNdDCMs(:,:,:),VSPNdDCMs(:,:,:),SHSNdDCMs(:,:,:),PHSNdDCMs(:,:,:)
   real(R8Ki), allocatable         :: SPyNdDCMs(:,:,:,:), PPyNdDCMs(:,:,:,:)
   type(KAD_InitInputType)         :: KAD_InitInp
   type(KAD_InitOutputType)        :: KAD_InitOut
   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   type(InflowWind_InitInputType)  :: IfW_InitInp 
   type(InflowWind_InitOutputType) :: IfW_InitOut
   type(MD_InitInputType)          :: MD_InitInp
   type(MD_InitOutputType)         :: MD_InitOut
   character(*), parameter         :: routineName = 'KFAST_Init'
   integer(IntKi)                  :: i,j,c
   real(DbKi)                      :: interval
   
   ! Initialize all the sub-modules : MoorDyn, KiteAeroDyn, Controller, and InflowWind
   ! Set KiteFAST-level parameters, misc vars
   ! Open an Output file

      

      ! Set KiteFAST parameters
   p%dt = dt
!   p%outFileRoot = outFileRoot_c
   p%numFlaps    = numFlaps
   p%numPylons   = numPylons
   
      ! Break out the number of structural nodes for a given component
   numFusNds = numCompNds(1)
   numSwnNds = numCompNds(2)
   numPWnNds = numCompNds(3)
   numVSPNds = numCompNds(4)
   numSHSNds = numCompNds(5)
   numPHSNds = numCompNds(6)
   c=6
   do i = 1, numPylons
      numSPyNds(i) = numCompNds(c)
      c = c + 1
   end do
   do i = 1, numPylons
      numPPyNds(i) = numCompNds(c)
      c = c + 1
   end do
   
      ! Decode rotor positions
   c=1
   do i = 1, numPylons
      SPyRtrO(:,1,i) = rtrPts_c(c:c+3)
      c = c + 4
      SPyRtrO(:,2,i) = rtrPts_c(c:c+3)
      c = c + 4
   end do
   do i = 1, numPylons
      PPyRtrO(:,1,i) = rtrPts_c(c:c+3)
      c = c + 4
      PPyRtrO(:,2,i) = rtrPts_c(c:c+3)
      c = c + 4
   end do
    
      ! Convert 1D float array data into specific quantities
      
   FusO = refPts_c(1:3)  
   SWnO = refPts_c(4:6)
   PWnO = refPts_c(7:9)
   VSPO = refPts_c(10:12)
   SHSO = refPts_c(13:15)
   PHSO = refPts_c(16:18)
   c = 19
   do i = 1, numPylons
      SPyO(:,i) = refPts_c(c:c+3)
      c = c + 4
   end do
   do i = 1, numPylons
      PPyO(:,i) = refPts_c(c:c+3)
      c = c + 4
   end do
 
   if ( (c-1) /= numRefPtElem_c ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of reference point elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numRefPtElem_c))//'.'
   end if
   
      ! Decode the nodal DCMs
   
   call AllocAry( FusNdDCMs, 3, 3, numFusNds, 'FusNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( SWnNdDCMs, 3, 3, numSWnNds, 'SWnNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( PWnNdDCMs, 3, 3, numPWnNds, 'PWnNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( VSPNdDCMs, 3, 3, numVSPNds, 'VSPNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( SHSNdDCMs, 3, 3, numSHSNds, 'SHSNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( PHSNdDCMs, 3, 3, numFusNds, 'PHSNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
      
   ! TODO: Probably will need to take the transpose of all the following DCMs because of memory storage issues between C and Fortran.   
      
 
   FusODCM = reshape(FusODCM_c,(/3,3/))
   
   c=1
   do i = 1,numFusNds
      FusNdDCMs(:,:,i) = reshape(nodeDCMs_c(c:c+9),(/3,3/))
      c = c + 9
   end do
   do i = 1,numSwnNds
      SwnNdDCMs(:,:,i) = reshape(nodeDCMs_c(c:c+9),(/3,3/))
      c = c + 9
   end do
   do i = 1,numPwnNds
      PwnNdDCMs(:,:,i) = reshape(nodeDCMs_c(c:c+9),(/3,3/))
      c = c + 9
   end do
 
   do j = 1, numPylons
      do i = 1, numSPyNds(j)
         SPyNdDCMs(:,:,i,j) = reshape(nodeDCMs_c(c:c+9),(/3,3/))
         c = c + 9
      end do
   end do
   do j = 1, numPylons
      do i = 1, numPPyNds(j)
         PPyNdDCMs(:,:,i,j) = reshape(nodeDCMs_c(c:c+9),(/3,3/))
         c = c + 9
      end do
   end do
   
   if ( (c-1) /= numDCMElem_c ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of DCM elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numDCMElem_c))//'.'
   end if
  
!----------------------------------------------------------------
! Initialize the Motions meshes which correspond to the motions 
!   coming from MBDyn.  The points coming from MBDyn are assumed
!   to be in the global, inertial coordinate system and the DCMs
!   transform from global to local.
!----------------------------------------------------------------
   !   ! Fuselage
   !call CreateMBDynL2MotionsMesh(FusO, numFusNds, FusPts, FusODCM, FusNdDCMs, mbdFusMotions, errStat, errMsg)
   !   ! Starboard Wing
   !call CreateMBDynL2MotionsMesh(FusO, numSWnNds, SWnPts, FusODCM, SWnNdDCMs, mbdSWnMotions, errStat, errMsg)
   !   ! Port Wing
   !call CreateMBDynL2MotionsMesh(FusO, numPWnNds, PWnPts, FusODCM, PWnNdDCMs, mbdPWnMotions, errStat, errMsg)
   !   ! Vertical Stabilizer
   !call CreateMBDynL2MotionsMesh(FusO, numVSPNds, VSPPts, FusODCM, VSPNdDCMs, mbdVSPMotions, errStat, errMsg)
   !   ! Starboard Horizontal Stabilizer
   !call CreateL2MotionsMesh(FusO, numSHSNds, SHSPts, FusODCM, SHSNdDCMs, mbdSHSMotions, errStat, errMsg)
   !   ! Port Horizontal Stabilizer
   !call CreateL2MotionsMesh(FusO, numPHSNds, PHSPts, FusODCM, PHSNdDCMs, mbdPHSMotions, errStat, errMsg)
   !   ! Starboard Pylons
   !do i = 1, numPylons
   !   call CreateMBDynL2MotionsMesh(FusO, numSPyNds(i), SPyPts(:,:,i), FusODCM, SPyNdDCMs(:,:,:,i), mbdSPyMotions(i), errStat, errMsg)
   !end do
   !   ! Port Pylons
   !do i = 1, numPylons
   !   call CreateMBDynL2MotionsMesh(FusO, numPPyNds(i), PPyPts(:,:,i), FusODCM, PPyNdDCMs(:,:,:,i), mbdPPyMotions(i), errStat, errMsg)
   !end do   
   !   ! Rotors [ Point meshes ]
   !do i = 1, numPylons
   !   call CreateMBDynRtrPtMotionsMesh(FusO, SPyRtrO(:,1,i), FusODCM,  mbdSPyRtrMotions(i), errStat, errMsg)
   !   call CreateMBDynRtrPtMotionsMesh(FusO, SPyRtrO(:,2,i), FusODCM,  mbdSPyRtrMotions(i), errStat, errMsg)
   !end do   
   !do i = 1, numPylons
   !   call CreateMBDynRtrPtMotionsMesh(FusO, PPyRtrO(:,1,i), FusODCM,  mbdPPyRtrMotions(i), errStat, errMsg)
   !   call CreateMBDynRtrPtMotionsMesh(FusO, PPyRtrO(:,2,i), FusODCM,  mbdPPyRtrMotions(i), errStat, errMsg)
   !end do
   !
   !! Create point load meshes for the inputs to MBDyn
   !
   !   ! Fuselage
   !call CreateMBDynPtLoadsMesh(FusO, numFusNds, FusPts, FusODCM, FusNdDCMs, mbdFusLoads, errStat, errMsg)
   !   ! Starboard Wing
   !call CreateMBDynPtLoadsMesh(FusO, numSWnNds, SWnPts, FusODCM, SWnNdDCMs, mbdSWnLoads, errStat, errMsg)
   !   ! Port Wing
   !call CreateMBDynPtLoadsMesh(FusO, numPWnNds, PWnPts, FusODCM, PWnNdDCMs, mbdPWnLoads, errStat, errMsg)
   !   ! Vertical Stabilizer
   !call CreateMBDynPtLoadsMesh(FusO, numVSPNds, VSPPts, FusODCM, VSPNdDCMs, mbdVSPLoads, errStat, errMsg)
   !   ! Starboard Horizontal Stabilizer
   !call CreateMBDynPtLoadsMesh(FusO, numSHSNds, SHSPts, FusODCM, SHSNdDCMs, mbdSHSLoads, errStat, errMsg)
   !   ! Port Horizontal Stabilizer
   !call CreateMBDynPtLoadsMesh(FusO, numPHSNds, PHSPts, FusODCM, PHSNdDCMs, mbdPHSLoads, errStat, errMsg)
   !   ! Starboard Pylons
   !do i = 1, numPylons
   !   call CreateMBDynPtLoadsMesh(FusO, numSPyNds(i), SPyPts(:,:,i), FusODCM, SPyNdDCMs(:,:,:,i), mbdSPyLoads(i), errStat, errMsg)
   !end do
   !   ! Port Pylons
   !do i = 1, numPylons
   !   call CreateMBDynPtLoadsMesh(FusO, numPPyNds(i), PPyPts(:,:,i), FusODCM, PPyNdDCMs(:,:,:,i), mbdPPyLoads(i), errStat, errMsg)
   !end do   
   !   ! Rotors [ Point meshes ]
   !do i = 1, numPylons
   !   call CreateMBDynRtrPtLoadsMesh(FusO, SPyRtrO(:,1,i), FusODCM,  mbdSPyRtrLoads(i), errStat, errMsg)
   !   call CreateMBDynRtrPtLoadsMesh(FusO, SPyRtrO(:,2,i), FusODCM,  mbdSPyRtrLoads(i), errStat, errMsg)
   !end do   
   !do i = 1, numPylons
   !   call CreateMBDynRtrPtLoadsMesh(FusO, PPyRtrO(:,1,i), FusODCM,  mbdPPyRtrLoads(i), errStat, errMsg)
   !   call CreateMBDynRtrPtLoadsMesh(FusO, PPyRtrO(:,2,i), FusODCM,  mbdPPyRtrLoads(i), errStat, errMsg)
   !end do
   
!----------------------------------------------------------------
! Initialize the KiteAeroDyn Module
!----------------------------------------------------------------

      ! Set KiteAeroDyn initialization input data
   !KAD_InitInp%FileName  = KAD_FileName_c
   KAD_InitInp%NumFlaps  = numFlaps
   KAD_InitInp%NumPylons = numPylons
   interval              = dt
   
      ! Determine Kite component reference point locations in the kite coordinate system
              
   call AllocAry( KAD_InitInp%SPyOR, 3, numPylons, 'InitInp%SPyOR', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( KAD_InitInp%PPyOR, 3, numPylons, 'InitInp%PPyOR', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( KAD_InitInp%SPyRtrOR, 3, 2, numPylons, 'InitInp%SPyRtrOR', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( KAD_InitInp%PPyRtrOR, 3, 2, numPylons, 'InitInp%PPyRtrOR', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

   KAD_InitInp%SWnOR =  SwnO - FusO
   KAD_InitInp%SWnOR = matmul(FusODCM,KAD_InitInp%SWnOR)
   KAD_InitInp%PWnOR =  PwnO - FusO
   KAD_InitInp%PWnOR = matmul(FusODCM,KAD_InitInp%PWnOR)
   KAD_InitInp%VSPOR =  VSPO - FusO
   KAD_InitInp%VSPOR = matmul(FusODCM,KAD_InitInp%VSPOR)
   KAD_InitInp%SHSOR =  SHSO - FusO
   KAD_InitInp%SHSOR = matmul(FusODCM,KAD_InitInp%SHSOR)
   KAD_InitInp%PHSOR =  PHSO - FusO
   KAD_InitInp%PHSOR = matmul(FusODCM,KAD_InitInp%PHSOR)
   KAD_InitInp%SWnOR =  SwnO - FusO
   KAD_InitInp%SWnOR = matmul(FusODCM,KAD_InitInp%SWnOR)
   do i = 1, numPylons
      KAD_InitInp%SPyOR(:,i) = SPyO(:,i) - FusO
      KAD_InitInp%SPyOR(:,i) = matmul(FusODCM,KAD_InitInp%SPyOR(:,i))
   end do
   do i = 1, numPylons
      KAD_InitInp%PPyOR(:,i) = PPyO(:,i) - FusO
      KAD_InitInp%PPyOR(:,i) = matmul(FusODCM,KAD_InitInp%PPyOR(:,i))
   end do
  
   do i = 1, numPylons
      KAD_InitInp%SPyRtrOR(:,1,i) = SPyRtrO(:,1,i) - FusO   
      KAD_InitInp%SPyRtrOR(:,1,i) = matmul(FusODCM,KAD_InitInp%SPyRtrOR(:,1,i))
      KAD_InitInp%SPyRtrOR(:,2,i) = SPyRtrO(:,2,i) - FusO   
      KAD_InitInp%SPyRtrOR(:,2,i) = matmul(FusODCM,KAD_InitInp%SPyRtrOR(:,2,i))     
   end do
   do i = 1, numPylons
      KAD_InitInp%PPyRtrOR(:,1,i) = PPyRtrO(:,1,i) - FusO   
      KAD_InitInp%PPyRtrOR(:,1,i) = matmul(FusODCM,KAD_InitInp%PPyRtrOR(:,1,i))
      KAD_InitInp%PPyRtrOR(:,2,i) = PPyRtrO(:,2,i) - FusO   
      KAD_InitInp%PPyRtrOR(:,2,i) = matmul(FusODCM,KAD_InitInp%PPyRtrOR(:,2,i))     
   end do
   

  
   
   call KAD_Init(KAD_InitInp, m%KAD%u, m%KAD%p, m%KAD%y, interval, m%KAD%m, KAD_InitOut, errStat2, errMsg2 )
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   
!----------------------------------------------------------------
! Initialize the InflowWind Module
!----------------------------------------------------------------

   IfW_InitInp%Linearize         = .false.   
!   IfW_InitInp%InputFileName     = IfW_FileName
!   IfW_InitInp%RootName          = TRIM(outFileRoot)//'.'//TRIM(y_FAST%Module_Abrev(Module_IfW))
   IfW_InitInp%UseInputFile      = .TRUE.
   IfW_InitInp%NumWindPoints     = KAD_InitOut%nIfWPts   
   IfW_InitInp%lidar%Tmax        = 0.0_ReKi
   IfW_InitInp%lidar%HubPosition = 0.0_ReKi
   IfW_InitInp%lidar%SensorType  = SensorType_None
   IfW_InitInp%Use4Dext          = .false.
   interval                      = dt
   call InflowWind_Init( IfW_InitInp, m%IfW%u, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z,  &
                         m%IfW%OtherSt, m%IfW%y, m%IfW%m, interval, IfW_InitOut, errStat2, errMsg2 )
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      
!----------------------------------------------------------------
! Initialize the MoorDyn Module
!----------------------------------------------------------------

!   MD_InitInp%FileName  = MD_FileName         ! This needs to be set according to what is in the FAST input file. 
!   MD_InitInp%RootName  = outFileRoot  
      ! TODO: Modify PtfmInit to accept a vector and DCM instead of a vector and 3 angles
   !MD_InitInp%PtfmInit  = InitOutData_ED%PlatformPos ! The platform in this application is the location of the Kite's fuselage reference point in the inertial coordinate system
   MD_InitInp%g         = gravity                    ! 
   MD_InitInp%rhoW      = KAD_InitOut%AirDens        ! This needs to be set according to air density at the Kite      
   MD_InitInp%WtrDepth  = 0.0_ReKi                   ! No water depth in this application
   interval             = dt
   
   call MD_Init( MD_InitInp, m%MD%u, m%MD%p, m%MD%x, m%MD%xd, m%MD%z, &
                  m%MD%OtherSt, m%MD%y, m%MD%m, interval, MD_InitOut, errStat2, errMsg2 )
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         
!----------------------------------------------------------------
! Initialize the Controller Module
!----------------------------------------------------------------

! -------------------------------------------------------------------------
! Initialize mesh-mapping data
! -------------------------------------------------------------------------

   !call InitModuleMappings(u, y, m%KAD%u, m%KAD%y, m%WingLoads, m%MD%u, m%MD%y, m%WingMeshMapData, errStat2, errMsg2)
   !   call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)

      if (errStat >= AbortErrLev) then
       !  call Cleanup()
       !  return
      end if      

            ! transfer Fortran variables to C:  
   errStat_c = errStat
   errMsg    = trim(errMsg)//C_NULL_CHAR
   errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )

end subroutine KFAST_Init

subroutine KFAST_AssRes() BIND (C, NAME='KFAST_AssRes')
   IMPLICIT NONE
#ifndef IMPLICIT_DLLEXPORT
!DEC$ ATTRIBUTES DLLEXPORT :: KFAST_AssRes
!GCC$ ATTRIBUTES DLLEXPORT :: KFAST_AssRes
#endif
 
   

end subroutine KFAST_AssRes
   
subroutine KFAST_End() BIND (C, NAME='KFAST_End')
   IMPLICIT NONE
#ifndef IMPLICIT_DLLEXPORT
!DEC$ ATTRIBUTES DLLEXPORT :: KFAST_End
!GCC$ ATTRIBUTES DLLEXPORT :: KFAST_End
#endif
   
   ! Call the End subroutines for KiteAeroDyn, MoorDyn, InflowWind, and the Controller
   ! Close the output file
   
end subroutine KFAST_End
   
end module KiteFAST
   