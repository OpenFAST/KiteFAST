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
   real(ReKi),                   intent(inout)  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numNodes          !< Number of nodes in the mesh
   real(ReKi),                   intent(in   )  :: positions(:,:)    !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the Kite axes
   real(R8Ki),                   intent(in   )  :: nodeDCMs(:,:,:) !< DCM needed to transform into a node's axes
   type(MeshType),               intent(inout)  :: mesh              !< The resulting mesh 
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

subroutine CreateMBDynPtLoadsMesh(origin, numNodes, positions, alignDCM, nodeDCMs, mesh, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numNodes          !< Number of nodes in the mesh
   real(ReKi),                   intent(in   )  :: positions(:,:)    !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the Kite axes
   real(R8Ki),                   intent(in   )  :: nodeDCMs(3,3, numNodes) !< DCM needed to transform into a node's axes
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   integer(intKi)                               :: j                 ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateMBDynPtLoadsMesh'

      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   call MeshCreate ( BlankMesh = mesh     &
                     ,IOS         = COMPONENT_INPUT &
                     ,Nnodes      = 1               &
                     ,errStat     = errStat2        &
                     ,ErrMess     = errMsg2         &
                     ,force       = .true.          &
                     ,moment      = .true.          &
                     ,orientation = .true.          &
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
   do j=1,numNodes
      call MeshConstructElement( mesh, ELEMENT_POINT, errStat2, errMsg2, p1=j )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   end do 
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return
      
   mesh%Orientation     = mesh%RefOrientation
   mesh%Force           = 0.0_R8Ki
   mesh%Moment          = 0.0_ReKi
   
   
end subroutine CreateMBDynPtLoadsMesh  

subroutine CreateMeshMappings(m, p, KAD, MD, errStat, errMsg)

   type(KFAST_MiscVarType),     intent(inout) :: m
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KAD_Data),           intent(in   ) :: KAD
   type(MD_Data),            intent(in   ) :: MD
   integer(IntKi),           intent(  out)  :: errStat           !< Error status of the operation
   character(*),             intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None
   
   character(*), parameter  :: RoutineName = 'CreateMeshMappings'
   integer(IntKi)           :: i
   integer(intKi)           :: errStat2          ! temporary Error status
   character(ErrMsgLen)     :: errMsg2           ! temporary Error message
   
   
      ! Mappings between MBDyn input motions meshes and the KiteAeroDyn motions meshes
   call MeshMapCreate( m%mbdFusMotions, KAD%u%FusMotions, m%Fus_L2_L2, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%Fus_L2_L2' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( m%mbdSWnMotions, KAD%u%SWnMotions, m%SWn_L2_L2, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SWn_L2_L2' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( m%mbdPWnMotions, KAD%u%PWnMotions, m%PWn_L2_L2, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PWn_L2_L2' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( m%mbdVSPMotions, KAD%u%VSPMotions, m%VSP_L2_L2, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%VSP_L2_L2' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( m%mbdSHSMotions, KAD%u%SHSMotions, m%SHS_L2_L2, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SHS_L2_L2' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( m%mbdPHSMotions, KAD%u%PHSMotions, m%PHS_L2_L2, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PHS_L2_L2' )     
         if (ErrStat>=AbortErrLev) return

   allocate(m%SPy_L2_L2(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPy_L2_L2', errStat, errMsg, RoutineName )     
   allocate(m%PPy_L2_L2(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPy_L2_L2', errStat, errMsg, RoutineName )     
   
   do i = 1 , p%NumPylons           
      call MeshMapCreate( m%mbdSPyMotions(i), KAD%u%SPyMotions(i), m%SPy_L2_L2(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SPy_L2_L2('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
            
      call MeshMapCreate( m%mbdPPyMotions(i), KAD%u%PPyMotions(i), m%PPy_L2_L2(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PPy_L2_L2('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
   end do
   
   
      ! Mappings between KiteAeroDyn loads point meshes and MBDyn loads point meshes 

   call MeshMapCreate( KAD%y%FusLoads, m%mbdFusLoads, m%Fus_P_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%Fus_P_P' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( KAD%y%SWnLoads, m%mbdSWnLoads, m%SWn_P_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SWn_P_P' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( KAD%y%PWnLoads, m%mbdPWnLoads, m%PWn_P_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PWn_P_P' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( KAD%y%VSPLoads, m%mbdVSPLoads, m%VSP_P_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%VSP_P_P' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( KAD%y%SHSLoads, m%mbdSHSLoads, m%SHS_P_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SHS_P_P' )     
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( KAD%y%PHSLoads, m%mbdPHSLoads, m%PHS_P_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PHS_P_P' )     
         if (ErrStat>=AbortErrLev) return

   allocate(m%SPy_P_P(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPy_P_P', errStat, errMsg, RoutineName )     
   allocate(m%PPy_P_P(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPy_P_P', errStat, errMsg, RoutineName )     
   
   do i = 1 , p%NumPylons           
      call MeshMapCreate( KAD%y%SPyLoads(i), m%mbdSPyLoads(i), m%SPy_P_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SPy_P_P('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
            
      call MeshMapCreate( KAD%y%PPyLoads(i), m%mbdPPyLoads(i), m%PPy_P_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PPy_P_P('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
   end do

      ! Need to transfer the MBDyn bridle point motions to MoorDyn
   ! MD%u%PtFairleadDisplacement 
   ! MD%y%PtFairleadLoad         

end subroutine CreateMeshMappings
   
subroutine KFAST_Init(dt, numFlaps, numPylons, numComp, numCompNds, KAD_FileName_c, IfW_FileName_c, MD_FileName_c, &
                       outFileRoot_c, gravity, FusODCM_c, numRtrPtsElem_c, rtrPts_c, numRefPtElem_c, refPts_c, numNodePtElem_c, nodePts_c, numDCMElem_c, nodeDCMs_c, errStat_c, errMsg_c ) BIND (C, NAME='KFAST_Init')
   IMPLICIT NONE


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
   integer(C_INT),         intent(in   ) :: numNodePtElem_c            ! total number of array elements in the nodal points array
   real(C_DOUBLE),         intent(in   ) :: nodePts_c(numNodePtElem_c) ! 1D array containing all the nodal point coordinate data
   integer(C_INT),         intent(in   ) :: numDCMElem_c               ! total number of array elements in the nodal DCM data
   real(C_DOUBLE),         intent(in   ) :: nodeDCMs_c(numDCMElem_c)   ! 1D array containing all the nodal DCM data
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

      ! Local variables
   real(ReKi)                      :: FusO(3), SWnO(3), PWnO(3), VSPO(3), SHSO(3), PHSO(3)
   real(R8Ki)                      :: FusODCM(3,3)
   real(ReKi)                      :: SPyO(3,numPylons), PPyO(3,numPylons), SPyRtrO(3,2,numPylons), PPyRtrO(3,2,numPylons)
   real(R8Ki), allocatable         :: FusNdDCMs(:,:,:),SWnNdDCMs(:,:,:),PWnNdDCMs(:,:,:),VSPNdDCMs(:,:,:),SHSNdDCMs(:,:,:),PHSNdDCMs(:,:,:)
   real(R8Ki), allocatable         :: SPyNdDCMs(:,:,:,:), PPyNdDCMs(:,:,:,:)
   real(ReKi), allocatable         :: FusPts(:,:),SWnPts(:,:),PWnPts(:,:),VSPPts(:,:),SHSPts(:,:),PHSPts(:,:)
   real(ReKi), allocatable         :: SPyPts(:,:,:), PPyPts(:,:,:)
   type(KAD_InitInputType)         :: KAD_InitInp
   type(KAD_InitOutputType)        :: KAD_InitOut
   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   type(InflowWind_InitInputType)  :: IfW_InitInp 
   type(InflowWind_InitOutputType) :: IfW_InitOut
   type(MD_InitInputType)          :: MD_InitInp
   type(MD_InitOutputType)         :: MD_InitOut
   character(*), parameter         :: routineName = 'KFAST_Init'
   integer(IntKi)                  :: i,j,c, n, maxSPyNds, maxPPyNds
   real(DbKi)                      :: interval
   
   ! Initialize all the sub-modules : MoorDyn, KiteAeroDyn, Controller, and InflowWind
   ! Set KiteFAST-level parameters, misc vars
   ! Open an Output file
   errStat = ErrID_None
   errMsg  = ''
  
   ! Validate some of the input data
   if ( numFlaps /=  3 )  call SetErrStat( ErrID_FATAL, "Due to the Controller interface requirements, numFlaps must be set to 3", errStat, errMsg, routineName )
   if ( numPylons /=  2 ) call SetErrStat( ErrID_FATAL, "Due to the Controller interface requirements, numPylons must be set to 2", errStat, errMsg, routineName )
   if ( .not. EqualRealNos(dt,0.01_DbKi) )  call SetErrStat( ErrID_FATAL, "Due to the Controller requirements, dt must be set to 0.01", errStat, errMsg, routineName )

   if (errStat >= AbortErrLev) then
      !errStat_c = errStat
      errMsg    = trim(errMsg)//C_NULL_CHAR
      errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )
      return
   end if
      
      
      ! Set KiteFAST parameters
   p%dt = dt
   p%outFileRoot = transfer(outFileRoot_c(1:IntfStrLen-1),p%outFileRoot)
   call RemoveNullChar(p%outFileRoot)
   p%numFlaps    = numFlaps
   p%numPylons   = numPylons
   
   
      ! Break out the number of structural nodes for a given component
   p%numFusNds = numCompNds(1)
   p%numSwnNds = numCompNds(2)
   p%numPWnNds = numCompNds(3)
   p%numVSPNds = numCompNds(4)
   p%numSHSNds = numCompNds(5)
   p%numPHSNds = numCompNds(6)
   c=7
   
   call AllocAry( p%numSPyNds, numPylons, 'p%numSPyNds', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( p%numPPyNds, numPylons, 'p%numPPyNds', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

   maxSPyNds = 0
   maxPPyNds = 0
   do i = 1, numPylons
      p%numSPyNds(i) = numCompNds(c)
      if ( p%numSPyNds(i) >  maxSPyNds ) maxSPyNds = p%numSPyNds(i)
      c = c + 1
   end do
   do i = 1, numPylons    
      p%numPPyNds(i) = numCompNds(c)
      if ( p%numPPyNds(i) >  maxPPyNds ) maxPPyNds = p%numPPyNds(i)
      c = c + 1
   end do
   
      ! Decode rotor positions
   c=1
   do i = 1, numPylons
      SPyRtrO(:,1,i) = rtrPts_c(c:c+2)
      c = c + 3
      SPyRtrO(:,2,i) = rtrPts_c(c:c+2)
      c = c + 3
   end do
   do i = 1, numPylons
      PPyRtrO(:,1,i) = rtrPts_c(c:c+2)
      c = c + 3
      PPyRtrO(:,2,i) = rtrPts_c(c:c+2)
      c = c + 3
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
      SPyO(:,i) = refPts_c(c:c+2)
      c = c + 3
   end do
   do i = 1, numPylons
      PPyO(:,i) = refPts_c(c:c+2)
      c = c + 3
   end do
 
   if ( (c-1) /= numRefPtElem_c ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of reference point elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numRefPtElem_c))//'.'
   end if
   
      ! Decode the nodal DCMs
   
   call AllocAry( FusNdDCMs, 3, 3, p%numFusNds, 'FusNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( SWnNdDCMs, 3, 3, p%numSWnNds, 'SWnNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( PWnNdDCMs, 3, 3, p%numPWnNds, 'PWnNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( VSPNdDCMs, 3, 3, p%numVSPNds, 'VSPNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( SHSNdDCMs, 3, 3, p%numSHSNds, 'SHSNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( PHSNdDCMs, 3, 3, p%numFusNds, 'PHSNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( SPyNdDCMs, 3, 3, maxSPyNds, numPylons, 'SPyNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( PPyNdDCMs, 3, 3, maxPPyNds, numPylons, 'PPyNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
   call AllocAry( FusPts, 3, p%numFusNds, 'FusPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( SWnPts, 3, p%numSWnNds, 'SWnPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( PWnPts, 3, p%numPWnNds, 'PWnPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( VSPPts, 3, p%numVSPNds, 'VSPPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( SHSPts, 3, p%numSHSNds, 'SHSPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( PHSPts, 3, p%numFusNds, 'PHSPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( SPyPts, 3, maxSPyNds, numPylons, 'SPyPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( PPyPts, 3, maxPPyNds, numPylons, 'PPyPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
   ! TODO: Probably will need to take the transpose of all the following DCMs because of memory storage issues between C a Fortran.   
      
 
   FusODCM = reshape(FusODCM_c,(/3,3/))
   
   c=1
   n=1
   do i = 1,p%numFusNds
      FusNdDCMs(:,:,i) = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      FusPts(:,i)      = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSwnNds
      SWnNdDCMs(:,:,i) = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      SWnPts(:,i)      = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPwnNds
      PWnNdDCMs(:,:,i) = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      PWnPts(:,i)      = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numVSPNds
      VSPNdDCMs(:,:,i) = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      VSPPts(:,i)      = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSHSNds
      SHSNdDCMs(:,:,i) = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      SHSPts(:,i)      = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPHSNds
      PHSNdDCMs(:,:,i) = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      PHSPts(:,i)      = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do j = 1, numPylons
      do i = 1, p%numSPyNds(j)
         SPyNdDCMs(:,:,i,j) = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         SPyPts(:,i,j)      = nodePts_c(n:n+2)
         c = c + 9
         n = n + 3
      end do
   end do
   do j = 1, numPylons
      do i = 1, p%numPPyNds(j)
         PPyNdDCMs(:,:,i,j) = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         PPyPts(:,i,j)      = nodePts_c(n:n+2)
         c = c + 9
         n = n + 3
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

      ! Fuselage
   call CreateMBDynL2MotionsMesh(FusO, p%numFusNds, FusPts, FusODCM, FusNdDCMs, m%mbdFusMotions, errStat, errMsg)
      ! Starboard Wing
   call CreateMBDynL2MotionsMesh(FusO, p%numSWnNds, SWnPts, FusODCM, SWnNdDCMs, m%mbdSWnMotions, errStat, errMsg)
      ! Port Wing
   call CreateMBDynL2MotionsMesh(FusO, p%numPWnNds, PWnPts, FusODCM, PWnNdDCMs, m%mbdPWnMotions, errStat, errMsg)
      ! Vertical Stabilizer
   call CreateMBDynL2MotionsMesh(FusO, p%numVSPNds, VSPPts, FusODCM, VSPNdDCMs, m%mbdVSPMotions, errStat, errMsg)
      ! Starboard Horizontal Stabilizer
   call CreateMBDynL2MotionsMesh(FusO, p%numSHSNds, SHSPts, FusODCM, SHSNdDCMs, m%mbdSHSMotions, errStat, errMsg)
      ! Port Horizontal Stabilizer
   call CreateMBDynL2MotionsMesh(FusO, p%numPHSNds, PHSPts, FusODCM, PHSNdDCMs, m%mbdPHSMotions, errStat, errMsg)
   
   allocate(m%mbdSPyMotions(numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdSPyMotions', errStat, errMsg, RoutineName )     
   allocate(m%mbdPPyMotions(numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdPPyMotions', errStat, errMsg, RoutineName )  

      ! Starboard Pylons
   do i = 1, numPylons
      call CreateMBDynL2MotionsMesh(FusO, p%numSPyNds(i), SPyPts(:,:,i), FusODCM, SPyNdDCMs(:,:,:,i), m%mbdSPyMotions(i), errStat, errMsg)
   end do
      ! Port Pylons
   do i = 1, numPylons
      call CreateMBDynL2MotionsMesh(FusO, p%numPPyNds(i), PPyPts(:,:,i), FusODCM, PPyNdDCMs(:,:,:,i), m%mbdPPyMotions(i), errStat, errMsg)
   end do   
   
      ! Rotors [ Point meshes ]
   !do i = 1, numPylons
   !   call CreateMBDynRtrPtMotionsMesh(FusO, SPyRtrO(:,1,i), FusODCM,  m%mbdSPyRtrMotions(i), errStat, errMsg)
   !   call CreateMBDynRtrPtMotionsMesh(FusO, SPyRtrO(:,2,i), FusODCM,  m%mbdSPyRtrMotions(i), errStat, errMsg)
   !end do  
   
   !TODO: Why are these meshes even needed?  We are not haveing to transfer loads for the rotors, don't the simply copy 
   !       straight to the MBDyn equivalent node?
   !do i = 1, numPylons
   !   call CreateMBDynRtrPtMotionsMesh(FusO, PPyRtrO(:,1,i), FusODCM,  m%mbdPPyRtrMotions(i), errStat, errMsg)
   !   call CreateMBDynRtrPtMotionsMesh(FusO, PPyRtrO(:,2,i), FusODCM,  m%mbdPPyRtrMotions(i), errStat, errMsg)
   !end do
   
   ! Create point load meshes for the inputs to MBDyn
   
      ! Fuselage
   call CreateMBDynPtLoadsMesh(FusO, p%numFusNds, FusPts, FusODCM, FusNdDCMs, m%mbdFusLoads, errStat, errMsg)
      ! Starboard Wing
   call CreateMBDynPtLoadsMesh(FusO, p%numSWnNds, SWnPts, FusODCM, SWnNdDCMs, m%mbdSWnLoads, errStat, errMsg)
      ! Port Wing
   call CreateMBDynPtLoadsMesh(FusO, p%numPWnNds, PWnPts, FusODCM, PWnNdDCMs, m%mbdPWnLoads, errStat, errMsg)
      ! Vertical Stabilizer
   call CreateMBDynPtLoadsMesh(FusO, p%numVSPNds, VSPPts, FusODCM, VSPNdDCMs, m%mbdVSPLoads, errStat, errMsg)
      ! Starboard Horizontal Stabilizer
   call CreateMBDynPtLoadsMesh(FusO, p%numSHSNds, SHSPts, FusODCM, SHSNdDCMs, m%mbdSHSLoads, errStat, errMsg)
      ! Port Horizontal Stabilizer
   call CreateMBDynPtLoadsMesh(FusO, p%numPHSNds, PHSPts, FusODCM, PHSNdDCMs, m%mbdPHSLoads, errStat, errMsg)
      ! Starboard Pylons
   allocate(m%mbdSPyLoads(numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdSPyLoads', errStat, errMsg, RoutineName )     
   allocate(m%mbdPPyLoads(numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdPPyLoads', errStat, errMsg, RoutineName )  
   do i = 1, numPylons
      call CreateMBDynPtLoadsMesh(FusO, p%numSPyNds(i), SPyPts(:,:,i), FusODCM, SPyNdDCMs(:,:,:,i), m%mbdSPyLoads(i), errStat, errMsg)
   end do
      ! Port Pylons
   do i = 1, numPylons
      call CreateMBDynPtLoadsMesh(FusO, p%numPPyNds(i), PPyPts(:,:,i), FusODCM, PPyNdDCMs(:,:,:,i), m%mbdPPyLoads(i), errStat, errMsg)
   end do  
   
      ! Rotors [ Point meshes ]
   !do i = 1, numPylons
   !   call CreateMBDynRtrPtLoadsMesh(FusO, SPyRtrO(:,1,i), FusODCM,  m%mbdSPyRtrLoads(i), errStat, errMsg)
   !   call CreateMBDynRtrPtLoadsMesh(FusO, SPyRtrO(:,2,i), FusODCM,  m%mbdSPyRtrLoads(i), errStat, errMsg)
   !end do   
   !do i = 1, numPylons
   !   call CreateMBDynRtrPtLoadsMesh(FusO, PPyRtrO(:,1,i), FusODCM,  m%mbdPPyRtrLoads(i), errStat, errMsg)
   !   call CreateMBDynRtrPtLoadsMesh(FusO, PPyRtrO(:,2,i), FusODCM,  m%mbdPPyRtrLoads(i), errStat, errMsg)
   !end do
   
!----------------------------------------------------------------
! Initialize the KiteAeroDyn Module
!----------------------------------------------------------------

      ! Set KiteAeroDyn initialization input data
   KAD_InitInp%FileName  = transfer(KAD_FileName_c(1:IntfStrLen-1),KAD_InitInp%FileName)
   call RemoveNullChar(KAD_InitInp%FileName)
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
   IfW_InitInp%InputFileName  = transfer(IfW_FileName_c(1:IntfStrLen-1),IfW_InitInp%InputFileName)
   call RemoveNullChar(IfW_InitInp%InputFileName)
   IfW_InitInp%RootName          = TRIM(p%outFileRoot)//'.IfW'
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
         
   MD_InitInp%FileName  = transfer(MD_FileName_c(1:IntfStrLen-1),MD_InitInp%FileName)
   call RemoveNullChar(MD_InitInp%FileName)
   MD_InitInp%RootName  = TRIM(p%outFileRoot)//'.MD'
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

   call CreateMeshMappings(m, p, m%KAD, m%MD, errStat2, errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)


            ! transfer Fortran variables to C:  
   errStat_c = errStat
   errMsg    = trim(errMsg)//C_NULL_CHAR
   errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )

end subroutine KFAST_Init

subroutine KFAST_AssRes() BIND (C, NAME='KFAST_AssRes')
   IMPLICIT NONE

 
   

end subroutine KFAST_AssRes
   
subroutine KFAST_End() BIND (C, NAME='KFAST_End')
   IMPLICIT NONE

   
   ! Call the End subroutines for KiteAeroDyn, MoorDyn, InflowWind, and the Controller
   ! Close the output file
   
end subroutine KFAST_End
   
end module KiteFAST
   