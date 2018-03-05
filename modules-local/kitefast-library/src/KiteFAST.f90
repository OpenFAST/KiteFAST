module KiteFAST
   use, intrinsic :: ISO_C_Binding 
   use NWTC_Library 
   use MoorDyn_Types
   use MoorDyn
   use InflowWind_Types
   use InflowWind
   use KiteAeroDyn_Types
   use KiteAeroDyn
   use KiteFastController
   use KiteFAST_Types
   
   implicit none 

private

   type(ProgDesc), parameter  :: KFAST_Ver = ProgDesc( 'KiteFAST', '', '' )
   integer,        parameter  :: IntfStrLen  = 1025       ! length of strings through the C interface
   type(KFAST_ParameterType)  :: p
   type(KFAST_MiscVarType)    :: m
   !type(KFAST_InputType)      :: u
   !type(KFAST_OutputType)     :: y
   
   
   public :: KFAST_Init
   public :: KFAST_AssRes
   public :: KFAST_Output
   public :: KFAST_AfterPredict
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
                     ,TranslationVel  = .true.    &
                     ,RotationVel     = .true.    &
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
   mesh%TranslationDisp = 0.0_ReKi
   mesh%TranslationVel  = 0.0_ReKi
   mesh%RotationVel     = 0.0_ReKi
   
   
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
   
   if ( p%useKAD ) then
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
      call MeshMapCreate( m%mbdVSMotions, KAD%u%VSMotions, m%VS_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%VS_L2_L2' )     
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
      call MeshMapCreate( KAD%y%VSLoads, m%mbdVSLoads, m%VS_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%VS_P_P' )     
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
   end if ! if ( p%useKAD )
   
      ! Need to transfer the MBDyn bridle point motions to MoorDyn
   ! MD%u%PtFairleadDisplacement 
   ! MD%y%PtFairleadLoad         

end subroutine CreateMeshMappings

subroutine TransferMBDynInputs2Meshes( p, m, errStat, errMsg )
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None

   integer(IntKi)   :: i,j
   
      ! Use the current point locations and the mesh node reference positions to compute the current displacements
      ! Then set the displacements, velocities, and DCMs to the MBDyn-based meshes

   do i = 1,p%numFusNds
      m%mbdFusMotions%Orientation  (:,:,i) = m%FusNdDCMs(:,:,i)
      m%mbdFusMotions%TranslationDisp(:,i) = m%FusPts(:,i) - m%mbdFusMotions%Position(:,i)
      m%mbdFusMotions%TranslationVel (:,i) = m%FusVels(:,i)
      m%mbdFusMotions%RotationVel    (:,i) = m%FusOmegas(:,i)
   end do
   do i = 1,p%numSwnNds
      m%mbdSWnMotions%Orientation  (:,:,i) = m%SWnNdDCMs(:,:,i)
      m%mbdSWnMotions%TranslationDisp(:,i) = m%SWnPts(:,i) - m%mbdSWnMotions%Position(:,i)
      m%mbdSWnMotions%TranslationVel (:,i) = m%SWnVels(:,i)
      m%mbdSWnMotions%RotationVel    (:,i) = m%SWnOmegas(:,i)
   end do
   do i = 1,p%numPwnNds
      m%mbdPWnMotions%Orientation  (:,:,i) = m%PWnNdDCMs(:,:,i)
      m%mbdPWnMotions%TranslationDisp(:,i) = m%PWnPts(:,i) - m%mbdPWnMotions%Position(:,i)
      m%mbdPWnMotions%TranslationVel (:,i) = m%PWnVels(:,i)
      m%mbdPWnMotions%RotationVel    (:,i) = m%PWnOmegas(:,i)
   end do
   do i = 1,p%numVSNds
      m%mbdVSMotions%Orientation  (:,:,i) = m%VSNdDCMs(:,:,i)
      m%mbdVSMotions%TranslationDisp(:,i) = m%VSPts(:,i) - m%mbdVSMotions%Position(:,i)
      m%mbdVSMotions%TranslationVel (:,i) = m%VSVels(:,i)
      m%mbdVSMotions%RotationVel    (:,i) = m%VSOmegas(:,i)
   end do
   do i = 1,p%numSHSNds
      m%mbdSHSMotions%Orientation  (:,:,i) = m%SHSNdDCMs(:,:,i)
      m%mbdSHSMotions%TranslationDisp(:,i) = m%SHSPts(:,i) - m%mbdSHSMotions%Position(:,i)
      m%mbdSHSMotions%TranslationVel (:,i) = m%SHSVels(:,i)
      m%mbdSHSMotions%RotationVel    (:,i) = m%SHSOmegas(:,i)
   end do
   do i = 1,p%numPHSNds
      m%mbdPHSMotions%Orientation  (:,:,i) = m%PHSNdDCMs(:,:,i)
      m%mbdPHSMotions%TranslationDisp(:,i) = m%PHSPts(:,i) - m%mbdPHSMotions%Position(:,i)
      m%mbdPHSMotions%TranslationVel (:,i) = m%PHSVels(:,i)
      m%mbdPHSMotions%RotationVel    (:,i) = m%PHSOmegas(:,i)
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)
         m%mbdSPyMotions(j)%Orientation  (:,:,i) = m%SPyNdDCMs(:,:,i,j)
         m%mbdSPyMotions(j)%TranslationDisp(:,i) = m%SPyPts(:,i,j) - m%mbdSPyMotions(j)%Position(:,i)
         m%mbdSPyMotions(j)%TranslationVel (:,i) = m%SPyVels(:,i,j)
         m%mbdSPyMotions(j)%RotationVel    (:,i) = m%SPyOmegas(:,i,j)
      end do
   end do   
   do j = 1, p%numPylons
      do i = 1, p%numPPyNds(j)
         m%mbdPPyMotions(j)%Orientation  (:,:,i) = m%PPyNdDCMs(:,:,i,j)
         m%mbdPPyMotions(j)%TranslationDisp(:,i) = m%PPyPts(:,i,j) - m%mbdPPyMotions(j)%Position(:,i)
         m%mbdPPyMotions(j)%TranslationVel (:,i) = m%PPyVels(:,i,j)
         m%mbdPPyMotions(j)%RotationVel    (:,i) = m%PPyOmegas(:,i,j)
      end do
   end do
   
      ! Rotors
   
      ! Bridle Connections
   
   
end subroutine TransferMBDynInputs2Meshes


subroutine TransferMBDynInitInputs( WindPt_c, FusO, numNodePtElem_c, nodePts_c, numDCMElem_c, nodeDCMs_c, rtrPts_c, p, m, errStat, errMsg )
   real(C_DOUBLE),            intent(in   ) :: WindPt_c(3)                      ! The location of the ground station point where the freestream wind is measured [global coordinates] (m)
   real(ReKi),                intent(in   ) :: FusO(3)                          ! Location of principal kite reference point in global coordinates
   integer(C_INT),            intent(in   ) :: numNodePtElem_c                  ! total number of array elements in the nodal points array
   real(C_DOUBLE),            intent(in   ) :: nodePts_c(numNodePtElem_c)       ! 1D array containing all the nodal point coordinate data
   integer(C_INT),            intent(in   ) :: numDCMElem_c                     ! total number of array elements in the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: nodeDCMs_c(numDCMElem_c)         ! 1D array containing all the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: rtrPts_c(:)                      ! 1D array of the rotor positions in global coordinates (m)
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None


   integer(IntKi)   :: i, j, c, n, v

   
   
   
   c=1
   n=1
   
   do i = 1,p%numFusNds
      m%FusNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%FusPts(:,i)            = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSwnNds
      m%SWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SWnPts(:,i)            = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPwnNds
      m%PWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PWnPts(:,i)            = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numVSNds
      m%VSNdDCMs(:,:,i)        = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%VSPts(:,i)             = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSHSNds
      m%SHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SHSPts(:,i)            = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPHSNds
      m%PHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PHSPts(:,i)            = nodePts_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)
         m%SPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%SPyPts(:,i,j)            = nodePts_c(n:n+2)
         c = c + 9
         n = n + 3
      end do
   end do
   do j = 1, p%numPylons
      do i = 1, p%numPPyNds(j)
         m%PPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%PPyPts(:,i,j)            = nodePts_c(n:n+2)
         c = c + 9
         n = n + 3
      end do
   end do
   
   if ( (c-1) /= numDCMElem_c ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of DCM elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numDCMElem_c))//'.'
   end if
   
      ! Decode rotor positions
   c=1
   do i = 1, p%numPylons
      m%SPyRtrO(:,1,i)         = rtrPts_c(c:c+2)
      c = c + 3
      m%SPyRtrO(:,2,i)         = rtrPts_c(c:c+2)
      c = c + 3
   end do
   do i = 1, p%numPylons
      m%PPyRtrO(:,1,i)         = rtrPts_c(c:c+2)
      c = c + 3
      m%PPyRtrO(:,2,i)         = rtrPts_c(c:c+2)
      c = c + 3
   end do
   
end subroutine TransferMBDynInitInputs


subroutine TransferMBDynToKAD( FusO, numNodePtElem_c, nodePts_c, numNodeVelElem_c, nodeVels_c, numNodeOmegaElem_c, nodeOmegas_c, numDCMElem_c, nodeDCMs_c, rtrPts_c, p, m, errStat, errMsg )
   real(ReKi),                intent(in   ) :: FusO(3)                          ! Location of principal kite reference point in global coordinates
   integer(C_INT),            intent(in   ) :: numNodePtElem_c                  ! total number of array elements in the nodal points array
   real(C_DOUBLE),            intent(in   ) :: nodePts_c(numNodePtElem_c)       ! 1D array containing all the nodal point coordinate data
   integer(C_INT),            intent(in   ) :: numNodeVelElem_c                 ! total number of array elements in the nodal translationalvelocities array
   real(C_DOUBLE),            intent(in   ) :: nodeVels_c(numNodeVelElem_c)     ! 1D array containing all the nodal translational velocities data
   integer(C_INT),            intent(in   ) :: numNodeOmegaElem_c               ! total number of array elements in the nodal angular velocities array
   real(C_DOUBLE),            intent(in   ) :: nodeOmegas_c(numNodeOmegaElem_c) ! 1D array containing all the nodal angular velocities data
   integer(C_INT),            intent(in   ) :: numDCMElem_c                     ! total number of array elements in the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: nodeDCMs_c(numDCMElem_c)         ! 1D array containing all the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: rtrPts_c(:)                      ! 1D array of the rotor positions in global coordinates (m)
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None


   integer(IntKi)   :: i, j, c, n

   
   
   c=1
   n=1
   
   do i = 1,p%numFusNds
      m%FusNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%FusPts(:,i)            = nodePts_c(n:n+2)
      m%FusVels(:,i)           = nodeVels_c(n:n+2)
      m%FusOmegas(:,i)         = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSwnNds
      m%SWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SWnPts(:,i)            = nodePts_c(n:n+2)
      m%SWnVels(:,i)           = nodeVels_c(n:n+2)
      m%SWnOmegas(:,i)         = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPwnNds
      m%PWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PWnPts(:,i)            = nodePts_c(n:n+2)
      m%PWnVels(:,i)           = nodeVels_c(n:n+2)
      m%PWnOmegas(:,i)         = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numVSNds
      m%VSNdDCMs(:,:,i)        = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%VSPts(:,i)             = nodePts_c(n:n+2)
      m%VSVels(:,i)            = nodeVels_c(n:n+2)
      m%VSOmegas(:,i)          = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSHSNds
      m%SHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SHSPts(:,i)            = nodePts_c(n:n+2)
      m%SHSVels(:,i)           = nodeVels_c(n:n+2)
      m%SHSOmegas(:,i)         = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPHSNds
      m%PHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PHSPts(:,i)            = nodePts_c(n:n+2)
      m%PHSVels(:,i)           = nodeVels_c(n:n+2)
      m%PHSOmegas(:,i)         = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)
         m%SPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%SPyPts(:,i,j)            = nodePts_c(n:n+2)
         m%SPyVels(:,i,j)           = nodeVels_c(n:n+2)
         m%SPyOmegas(:,i,j)         = nodeOmegas_c(n:n+2)
         c = c + 9
         n = n + 3
      end do
   end do
   do j = 1, p%numPylons
      do i = 1, p%numPPyNds(j)
         m%PPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%PPyPts(:,i,j)            = nodePts_c(n:n+2)
         m%PPyVels(:,i,j)           = nodeVels_c(n:n+2)
         m%PPyOmegas(:,i,j)         = nodeOmegas_c(n:n+2)
         c = c + 9
         n = n + 3
      end do
   end do
   
   if ( (c-1) /= numDCMElem_c ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of DCM elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numDCMElem_c))//'.'
   end if
   
      ! Decode rotor positions
   c=1
   do i = 1, p%numPylons
      m%SPyRtrO(:,1,i)         = rtrPts_c(c:c+2)
      c = c + 3
      m%SPyRtrO(:,2,i)         = rtrPts_c(c:c+2)
      c = c + 3
   end do
   do i = 1, p%numPylons
      m%PPyRtrO(:,1,i)         = rtrPts_c(c:c+2)
      c = c + 3
      m%PPyRtrO(:,2,i)         = rtrPts_c(c:c+2)
      c = c + 3
   end do
   
   
  
end subroutine TransferMBDynToKAD
 
subroutine TransferMBDynToIfW( WindPt_c, FusO, numNodePtElem_c, nodePts_c, rtrPts_c, p, m, errStat, errMsg )
   real(C_DOUBLE),            intent(in   ) :: WindPt_c(3)                      ! The location of the ground station point where the freestream wind is measured [global coordinates] (m)
   real(ReKi),                intent(in   ) :: FusO(3)                          ! Location of principal kite reference point in global coordinates
   integer(C_INT),            intent(in   ) :: numNodePtElem_c                  ! total number of array elements in the nodal points array
   real(C_DOUBLE),            intent(in   ) :: nodePts_c(numNodePtElem_c)       ! 1D array containing all the nodal point coordinate data
   real(C_DOUBLE),            intent(in   ) :: rtrPts_c(:)                      ! 1D array of the rotor positions in global coordinates (m)
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None


   integer(IntKi)   :: i, j, n, c, v

   m%IfW%u%PositionXYZ(:,1) = WindPt_c
   m%IfW%u%PositionXYZ(:,2) = FusO
   
   n=1
   v=3
   
   do i = 1,p%numFusNds
      m%IfW%u%PositionXYZ(:,v) = nodePts_c(n:n+2)
      n = n + 3
      v = v + 1
   end do
   do i = 1,p%numSwnNds
      m%IfW%u%PositionXYZ(:,v) = nodePts_c(n:n+2)
      n = n + 3
      v = v + 1
   end do
   do i = 1,p%numPwnNds
      m%IfW%u%PositionXYZ(:,v) = nodePts_c(n:n+2)
      n = n + 3
      v = v + 1
   end do
   do i = 1,p%numVSNds
      m%IfW%u%PositionXYZ(:,v) = nodePts_c(n:n+2)
      n = n + 3
      v = v + 1
   end do
   do i = 1,p%numSHSNds
      m%IfW%u%PositionXYZ(:,v) = nodePts_c(n:n+2)
      n = n + 3
      v = v + 1
   end do
   do i = 1,p%numPHSNds
      m%IfW%u%PositionXYZ(:,v) = nodePts_c(n:n+2)
      n = n + 3
      v = v + 1
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)
         m%IfW%u%PositionXYZ(:,v)   = nodePts_c(n:n+2)
         n = n + 3
         v = v + 1
      end do
   end do
   do j = 1, p%numPylons
      do i = 1, p%numPPyNds(j)
         m%IfW%u%PositionXYZ(:,v)   = nodePts_c(n:n+2)
         n = n + 3
         v = v + 1
      end do
   end do
   
      ! Decode rotor positions
   c=1
   do i = 1, p%numPylons
      m%IfW%u%PositionXYZ(:,v) = rtrPts_c(c:c+2)
      c = c + 3
      v = v + 1
      m%IfW%u%PositionXYZ(:,v) = rtrPts_c(c:c+2)
      c = c + 3
      v = v + 1
   end do
   do i = 1, p%numPylons
      m%IfW%u%PositionXYZ(:,v) = rtrPts_c(c:c+2)
      c = c + 3
      v = v + 1
      m%IfW%u%PositionXYZ(:,v) = rtrPts_c(c:c+2)
      c = c + 3
      v = v + 1
   end do
   
   
  
end subroutine TransferMBDynToIfW

subroutine KFAST_Init(dt, numFlaps, numPylons, numComp, numCompNds, modFlags, KAD_FileName_c, IfW_FileName_c, MD_FileName_c, KFC_FileName_c, &
                       outFileRoot_c, gravity, WindPt_c, FusODCM_c, numRtrPtsElem_c, rtrPts_c, numRefPtElem_c, refPts_c, &
                       numNodePtElem_c, nodePts_c, numDCMElem_c, nodeDCMs_c, errStat_c, errMsg_c ) BIND (C, NAME='KFAST_Init')
   IMPLICIT NONE


   real(C_DOUBLE),         intent(in   ) :: dt                         ! simulation time step size (s)
   integer(C_INT),         intent(in   ) :: numFlaps                   ! number of flaps per wing
   integer(C_INT),         intent(in   ) :: numPylons                  ! number of pylons per wing
   integer(C_INT),         intent(in   ) :: numComp                    ! number of individual components in the kite
   integer(C_INT),         intent(in   ) :: numCompNds(numComp)        ! number of nodes per kite component
   integer(C_INT),         intent(in   ) :: modFlags(4)                ! flags indicating which modules are being used [ Index: 1=KAD, 2=IfW, 3=MD, 4=KFC; Value: 0=off,1=on]
   character(kind=C_CHAR), intent(in   ) :: KAD_FileName_c(IntfStrLen) ! name of KiteAeroDyn input file
   character(kind=C_CHAR), intent(in   ) :: IfW_FileName_c(IntfStrLen) ! name of InflowWind input file
   character(kind=C_CHAR), intent(in   ) :: MD_FileName_c(IntfStrLen)  ! name of MoorDyn input file
   character(kind=C_CHAR), intent(in   ) :: KFC_FileName_c(IntfStrLen) ! name of KiteFAST controller Shared Library file
   character(kind=C_CHAR), intent(in   ) :: outFileRoot_c(IntfStrLen)  ! root name of any output file generated by KiteFAST
   real(C_DOUBLE),         intent(in   ) :: gravity                    ! gravitational constant (m/s^2)
   real(C_DOUBLE),         intent(in   ) :: WindPt_c(3)                ! The location of the ground station point where the freestream wind is measured [global coordinates] (m)
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
   real(ReKi)                      :: FusO(3), SWnO(3), PWnO(3), VSO(3), SHSO(3), PHSO(3)
   real(R8Ki)                      :: FusODCM(3,3)
   real(ReKi)                      :: SPyO(3,numPylons), PPyO(3,numPylons)
   type(KAD_InitInputType)         :: KAD_InitInp
   type(KAD_InitOutputType)        :: KAD_InitOut
   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   type(InflowWind_InitInputType)  :: IfW_InitInp 
   type(InflowWind_InitOutputType) :: IfW_InitOut
   type(MD_InitInputType)          :: MD_InitInp
   type(MD_InitOutputType)         :: MD_InitOut
   type(KFC_InitInputType)         :: KFC_InitInp
   type(KFC_InitOutputType)        :: KFC_InitOut
   character(*), parameter         :: routineName = 'KFAST_Init'
   integer(IntKi)                  :: i,j,c, n, maxSPyNds, maxPPyNds
   real(DbKi)                      :: interval
   
   ! Initialize all the sub-modules : MoorDyn, KiteAeroDyn, Controller, and InflowWind
   ! Set KiteFAST-level parameters, misc vars
   ! Open an Output file
   errStat = ErrID_None
   errMsg  = ''
  
   ! If a module's input file/dll file is empty, then turn off that module
   
   if ( modFlags(1) > 0 ) then
      p%useKAD = .true.
   else
      p%useKAD = .false.
   end if
   
   
   if ( modFlags(2) > 0 ) then
      p%useIfW = .true.
   else
      p%useIfW = .false.
   end if
   
   if ( modFlags(3) > 0 ) then
      p%useMD = .true.
   else
      p%useMD = .false.
   end if
   
   if ( modFlags(4) > 0 ) then
      p%useKFC = .true.
         ! Validate some of the input data
      if ( numFlaps /=  3 )  call SetErrStat( ErrID_FATAL, "Due to the Controller interface requirements, numFlaps must be set to 3", errStat, errMsg, routineName )
      if ( numPylons /=  2 ) call SetErrStat( ErrID_FATAL, "Due to the Controller interface requirements, numPylons must be set to 2", errStat, errMsg, routineName )
   else
      p%useKFC = .false.
   end if
   
   if ( .not. EqualRealNos(REAL(dt, DbKi) ,0.01_DbKi) )  call SetErrStat( ErrID_FATAL, "Due to the Controller requirements, dt must be set to 0.01", errStat, errMsg, routineName )

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
   
   call SetupMBDynMotionData()
   
!----------------------------------------------------------------
! Initialize the KiteAeroDyn Module
!----------------------------------------------------------------

      ! Set KiteAeroDyn initialization input data

   if (p%useKAD) then
      
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
      KAD_InitInp%SWnOR = matmul(m%FusODCM,KAD_InitInp%SWnOR)
      KAD_InitInp%PWnOR =  PwnO - FusO
      KAD_InitInp%PWnOR = matmul(m%FusODCM,KAD_InitInp%PWnOR)
      KAD_InitInp%VSOR =  VSO - FusO
      KAD_InitInp%VSOR = matmul(m%FusODCM,KAD_InitInp%VSOR)
      KAD_InitInp%SHSOR =  SHSO - FusO
      KAD_InitInp%SHSOR = matmul(m%FusODCM,KAD_InitInp%SHSOR)
      KAD_InitInp%PHSOR =  PHSO - FusO
      KAD_InitInp%PHSOR = matmul(m%FusODCM,KAD_InitInp%PHSOR)
      KAD_InitInp%SWnOR =  SwnO - FusO
      KAD_InitInp%SWnOR = matmul(m%FusODCM,KAD_InitInp%SWnOR)
      do i = 1, numPylons
         KAD_InitInp%SPyOR(:,i) = SPyO(:,i) - FusO
         KAD_InitInp%SPyOR(:,i) = matmul(m%FusODCM,KAD_InitInp%SPyOR(:,i))
      end do
      do i = 1, numPylons
         KAD_InitInp%PPyOR(:,i) = PPyO(:,i) - FusO
         KAD_InitInp%PPyOR(:,i) = matmul(m%FusODCM,KAD_InitInp%PPyOR(:,i))
      end do
  
      do i = 1, numPylons
         KAD_InitInp%SPyRtrOR(:,1,i) = m%SPyRtrO(:,1,i) - FusO   
         KAD_InitInp%SPyRtrOR(:,1,i) = matmul(m%FusODCM,KAD_InitInp%SPyRtrOR(:,1,i))
         KAD_InitInp%SPyRtrOR(:,2,i) = m%SPyRtrO(:,2,i) - FusO   
         KAD_InitInp%SPyRtrOR(:,2,i) = matmul(m%FusODCM,KAD_InitInp%SPyRtrOR(:,2,i))     
      end do
      do i = 1, numPylons
         KAD_InitInp%PPyRtrOR(:,1,i) = m%PPyRtrO(:,1,i) - FusO   
         KAD_InitInp%PPyRtrOR(:,1,i) = matmul(m%FusODCM,KAD_InitInp%PPyRtrOR(:,1,i))
         KAD_InitInp%PPyRtrOR(:,2,i) = m%PPyRtrO(:,2,i) - FusO   
         KAD_InitInp%PPyRtrOR(:,2,i) = matmul(m%FusODCM,KAD_InitInp%PPyRtrOR(:,2,i))     
      end do
   
      call KAD_Init(KAD_InitInp, m%KAD%u, m%KAD%p, m%KAD%y, interval, m%KAD%x, m%KAD%xd, m%KAD%z, m%KAD%OtherSt, m%KAD%m, KAD_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      
      p%AirDens = KAD_InitOut%AirDens
   else
      p%AirDens = 0.0_ReKi
      KAD_InitOut%nIfWPts = 0
   end if
                       
!----------------------------------------------------------------
! Initialize the InflowWind Module
!----------------------------------------------------------------
   
   
   if (p%useIfW) then
      IfW_InitInp%InputFileName  = transfer(IfW_FileName_c(1:IntfStrLen-1),IfW_InitInp%InputFileName)
      call RemoveNullChar(IfW_InitInp%InputFileName)
      IfW_InitInp%Linearize         = .false.     
      IfW_InitInp%RootName          = TRIM(p%outFileRoot)//'.IfW'
      IfW_InitInp%UseInputFile      = .TRUE.
      IfW_InitInp%NumWindPoints     = KAD_InitOut%nIfWPts  + 2 
      IfW_InitInp%lidar%Tmax        = 0.0_ReKi
      IfW_InitInp%lidar%HubPosition = 0.0_ReKi
      IfW_InitInp%lidar%SensorType  = SensorType_None
      IfW_InitInp%Use4Dext          = .false.
      interval                      = dt
      call InflowWind_Init( IfW_InitInp, m%IfW%u, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z,  &
                            m%IfW%OtherSt, m%IfW%y, m%IfW%m, interval, IfW_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   end if
   
!----------------------------------------------------------------
! Initialize the MoorDyn Module
!----------------------------------------------------------------
         
   
   
   if (p%useMD) then
      
      MD_InitInp%FileName  = transfer(MD_FileName_c(1:IntfStrLen-1),MD_InitInp%FileName)
      call RemoveNullChar(MD_InitInp%FileName)
   
      MD_InitInp%RootName  = TRIM(p%outFileRoot)//'.MD'
         ! The platform in this application is the location of the Kite's fuselage reference point in the inertial coordinate system
      MD_InitInp%PtfmPos   = FusO
      MD_InitInp%PtfmDCM   = m%FusODCM
      MD_InitInp%g         = gravity                    ! 
      MD_InitInp%rhoW      = KAD_InitOut%AirDens        ! This needs to be set according to air density at the Kite      
      MD_InitInp%WtrDepth  = 0.0_ReKi                   ! No water depth in this application
      interval             = dt
   
      call MD_Init( MD_InitInp, m%MD%u(1), m%MD%p, m%MD%x, m%MD%xd, m%MD%z, &
                     m%MD%OtherSt, m%MD%y, m%MD%m, interval, MD_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      
      if ( MD_InitOut%NAnchs > 1 ) then
         call SetErrStat(ErrID_Fatal,'KiteFAST can only support a single anchor point in the MoorDyn model',errStat,errMsg,routineName)
      end if
   
      p%anchorPt = MD_InitOut%Anchs(:,1)  
   else
      p%anchorPt = 0.0_ReKi
   end if
   
!----------------------------------------------------------------
! Initialize the Controller Module
!----------------------------------------------------------------
   
   
   if (p%useKFC) then
      
      KFC_InitInp%DLL_FileName = transfer(KFC_FileName_c(1:IntfStrLen-1),KFC_InitInp%DLL_FileName)
      call RemoveNullChar(KFC_InitInp%DLL_FileName)

         ! Set the DCM between FAST inertial frame and the Controller ground frame
      p%DCM_Fast2Ctrl = reshape((/-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0/),(/3,3/))
      KFC_InitInp%DLL_FileName = transfer(KFC_FileName_c(1:IntfStrLen-1),KFC_InitInp%DLL_FileName)
      KFC_InitInp%numPylons    = numPylons
      KFC_InitInp%numFlaps     = numFlaps
      interval = dt
      call KFC_Init(KFC_InitInp, m%KFC%p, KFC_InitOut, interval, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   end if
   
   m%NewTime = .true.  ! This flag is needed to tell us when we have advanced time, and hence need to call the Controller's Step routine
   
! -------------------------------------------------------------------------
! Initialize mesh-mapping data
! -------------------------------------------------------------------------

   call CreateMeshMappings(m, p, m%KAD, m%MD, errStat2, errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)

! -------------------------------------------------------------------------
! Open the Output file and generate header data
! -------------------------------------------------------------------------      

      
! -------------------------------------------------------------------------
! Call the t = 0.0 CalcOutput() routines
! -------------------------------------------------------------------------
      
      
      
      
            ! transfer Fortran variables to C:  
   errStat_c = errStat
   errMsg    = trim(errMsg)//C_NULL_CHAR
   errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )

contains
                       
   subroutine SetupMBDynMotionData( )

      ! Break out the number of structural nodes for a given component
   p%numFusNds = numCompNds(1)
   p%numSwnNds = numCompNds(2)
   p%numPWnNds = numCompNds(3)
   p%numVSNds  = numCompNds(4)
   p%numSHSNds = numCompNds(5)
   p%numPHSNds = numCompNds(6)
   c=7
   
   call AllocAry( p%numSPyNds, p%numPylons, 'p%numSPyNds', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( p%numPPyNds, p%numPylons, 'p%numPPyNds', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

   maxSPyNds = 0
   maxPPyNds = 0
   do i = 1, p%numPylons
      p%numSPyNds(i) = numCompNds(c)
      if ( p%numSPyNds(i) >  maxSPyNds ) maxSPyNds = p%numSPyNds(i)
      c = c + 1
   end do
   do i = 1, p%numPylons    
      p%numPPyNds(i) = numCompNds(c)
      if ( p%numPPyNds(i) >  maxPPyNds ) maxPPyNds = p%numPPyNds(i)
      c = c + 1
   end do
   
      ! Allocate rotor positions
 
   call AllocAry( m%SPyRtrO, 3,2,p%numPylons, 'm%SPyRtrO', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PPyRtrO, 3,2,p%numPylons, 'm%PPyRtrO', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
      
      ! Convert 1D float array data into specific quantities
      
   FusO = refPts_c(1:3)  
   SWnO = refPts_c(4:6)
   PWnO = refPts_c(7:9)
   VSO = refPts_c(10:12)
   SHSO = refPts_c(13:15)
   PHSO = refPts_c(16:18)
   c = 19
   do i = 1, p%numPylons
      SPyO(:,i) = refPts_c(c:c+2)
      c = c + 3
   end do
   do i = 1, p%numPylons
      PPyO(:,i) = refPts_c(c:c+2)
      c = c + 3
   end do
 
   if ( (c-1) /= numRefPtElem_c ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of reference point elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numRefPtElem_c))//'.'
   end if
   
      ! Decode the nodal DCMs
   
   call AllocAry( m%FusNdDCMs, 3, 3, p%numFusNds, 'FusNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SWnNdDCMs, 3, 3, p%numSWnNds, 'SWnNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PWnNdDCMs, 3, 3, p%numPWnNds, 'PWnNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%VSNdDCMs, 3, 3, p%numVSNds, 'VSNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SHSNdDCMs, 3, 3, p%numSHSNds, 'SHSNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PHSNdDCMs, 3, 3, p%numFusNds, 'PHSNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SPyNdDCMs, 3, 3, maxSPyNds, p%numPylons, 'SPyNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PPyNdDCMs, 3, 3, maxPPyNds, p%numPylons, 'PPyNdDCMs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
   call AllocAry( m%FusPts, 3, p%numFusNds, 'FusPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SWnPts, 3, p%numSWnNds, 'SWnPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PWnPts, 3, p%numPWnNds, 'PWnPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%VSPts, 3, p%numVSNds, 'VSPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SHSPts, 3, p%numSHSNds, 'SHSPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PHSPts, 3, p%numFusNds, 'PHSPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SPyPts, 3, maxSPyNds, p%numPylons, 'SPyPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PPyPts, 3, maxPPyNds, p%numPylons, 'PPyPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   
   call AllocAry( m%FusVels, 3, p%numFusNds, 'FusVels', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SWnVels, 3, p%numSWnNds, 'SWnVels', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PWnVels, 3, p%numPWnNds, 'PWnVels', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%VSVels, 3, p%numVSNds, 'VSVels', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SHSVels, 3, p%numSHSNds, 'SHSVels', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PHSVels, 3, p%numFusNds, 'PHSVels', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SPyVels, 3, maxSPyNds, p%numPylons, 'SPyVels', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PPyVels, 3, maxPPyNds, p%numPylons, 'PPyVels', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

   call AllocAry( m%FusOmegas, 3, p%numFusNds, 'FusOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SWnOmegas, 3, p%numSWnNds, 'SWnOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PWnOmegas, 3, p%numPWnNds, 'PWnOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%VSOmegas, 3, p%numVSNds, 'VSOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SHSOmegas, 3, p%numSHSNds, 'SHSOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PHSOmegas, 3, p%numFusNds, 'PHSOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SPyOmegas, 3, maxSPyNds, p%numPylons, 'SPyOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PPyOmegas, 3, maxPPyNds, p%numPylons, 'PPyOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

   ! TODO: Probably will need to take the transpose of all the following DCMs because of memory storage issues between C a Fortran.   
      
 
   m%FusODCM = reshape(FusODCM_c,(/3,3/))
   
   call TransferMBDynInitInputs( WindPt_c, FusO, numNodePtElem_c, nodePts_c, numDCMElem_c, nodeDCMs_c, rtrPts_c, p, m, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
   
   
   
!----------------------------------------------------------------
! Initialize the Motions meshes which correspond to the motions 
!   coming from MBDyn.  The points coming from MBDyn are assumed
!   to be in the global, inertial coordinate system and the DCMs
!   transform from global to local.
!----------------------------------------------------------------

      ! Fuselage
   call CreateMBDynL2MotionsMesh(FusO, p%numFusNds, m%FusPts, m%FusODCM, m%FusNdDCMs, m%mbdFusMotions, errStat, errMsg)
      ! Starboard Wing
   call CreateMBDynL2MotionsMesh(FusO, p%numSWnNds, m%SWnPts, m%FusODCM, m%SWnNdDCMs, m%mbdSWnMotions, errStat, errMsg)
      ! Port Wing
   call CreateMBDynL2MotionsMesh(FusO, p%numPWnNds, m%PWnPts, m%FusODCM, m%PWnNdDCMs, m%mbdPWnMotions, errStat, errMsg)
      ! Vertical Stabilizer
   call CreateMBDynL2MotionsMesh(FusO, p%numVSNds, m%VSPts, m%FusODCM, m%VSNdDCMs, m%mbdVSMotions, errStat, errMsg)
      ! Starboard Horizontal Stabilizer
   call CreateMBDynL2MotionsMesh(FusO, p%numSHSNds, m%SHSPts, m%FusODCM, m%SHSNdDCMs, m%mbdSHSMotions, errStat, errMsg)
      ! Port Horizontal Stabilizer
   call CreateMBDynL2MotionsMesh(FusO, p%numPHSNds, m%PHSPts, m%FusODCM, m%PHSNdDCMs, m%mbdPHSMotions, errStat, errMsg)
   
   allocate(m%mbdSPyMotions(p%numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdSPyMotions', errStat, errMsg, RoutineName )     
   allocate(m%mbdPPyMotions(p%numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdPPyMotions', errStat, errMsg, RoutineName )  

      ! Starboard Pylons
   do i = 1, p%numPylons
      call CreateMBDynL2MotionsMesh(FusO, p%numSPyNds(i), m%SPyPts(:,:,i), m%FusODCM, m%SPyNdDCMs(:,:,:,i), m%mbdSPyMotions(i), errStat, errMsg)
   end do
      ! Port Pylons
   do i = 1, p%numPylons
      call CreateMBDynL2MotionsMesh(FusO, p%numPPyNds(i), m%PPyPts(:,:,i), m%FusODCM, m%PPyNdDCMs(:,:,:,i), m%mbdPPyMotions(i), errStat, errMsg)
   end do   
   
      ! Rotors [ Point meshes ]
   !do i = 1, p%numPylons
   !   call CreateMBDynRtrPtMotionsMesh(FusO, m%SPyRtrO(:,1,i), m%FusODCM,  m%mbdSPyRtrMotions(i), errStat, errMsg)
   !   call CreateMBDynRtrPtMotionsMesh(FusO, m%SPyRtrO(:,2,i), m%FusODCM,  m%mbdSPyRtrMotions(i), errStat, errMsg)
   !end do  
   
   !TODO: Why are these meshes even needed?  We are not having to transfer loads for the rotors, don't they simply copy 
   !       straight to the MBDyn equivalent node?
   !do i = 1, p%numPylons
   !   call CreateMBDynRtrPtMotionsMesh(FusO, m%PPyRtrO(:,1,i), m%FusODCM,  m%mbdPPyRtrMotions(i), errStat, errMsg)
   !   call CreateMBDynRtrPtMotionsMesh(FusO, m%PPyRtrO(:,2,i), m%FusODCM,  m%mbdPPyRtrMotions(i), errStat, errMsg)
   !end do
   
   ! Create point load meshes for the inputs to MBDyn
   
      ! Fuselage
   call CreateMBDynPtLoadsMesh(FusO, p%numFusNds, m%FusPts, m%FusODCM, m%FusNdDCMs, m%mbdFusLoads, errStat, errMsg)
      ! Starboard Wing
   call CreateMBDynPtLoadsMesh(FusO, p%numSWnNds, m%SWnPts, m%FusODCM, m%SWnNdDCMs, m%mbdSWnLoads, errStat, errMsg)
      ! Port Wing
   call CreateMBDynPtLoadsMesh(FusO, p%numPWnNds, m%PWnPts, m%FusODCM, m%PWnNdDCMs, m%mbdPWnLoads, errStat, errMsg)
      ! Vertical Stabilizer
   call CreateMBDynPtLoadsMesh(FusO, p%numVSNds, m%VSPts, m%FusODCM, m%VSNdDCMs, m%mbdVSLoads, errStat, errMsg)
      ! Starboard Horizontal Stabilizer
   call CreateMBDynPtLoadsMesh(FusO, p%numSHSNds, m%SHSPts, m%FusODCM, m%SHSNdDCMs, m%mbdSHSLoads, errStat, errMsg)
      ! Port Horizontal Stabilizer
   call CreateMBDynPtLoadsMesh(FusO, p%numPHSNds, m%PHSPts, m%FusODCM, m%PHSNdDCMs, m%mbdPHSLoads, errStat, errMsg)
      ! Starboard Pylons
   allocate(m%mbdSPyLoads(p%numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdSPyLoads', errStat, errMsg, RoutineName )     
   allocate(m%mbdPPyLoads(p%numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdPPyLoads', errStat, errMsg, RoutineName )  
   do i = 1, p%numPylons
      call CreateMBDynPtLoadsMesh(FusO, p%numSPyNds(i), m%SPyPts(:,:,i), m%FusODCM, m%SPyNdDCMs(:,:,:,i), m%mbdSPyLoads(i), errStat, errMsg)
   end do
      ! Port Pylons
   do i = 1, p%numPylons
      call CreateMBDynPtLoadsMesh(FusO, p%numPPyNds(i), m%PPyPts(:,:,i), m%FusODCM, m%PPyNdDCMs(:,:,:,i), m%mbdPPyLoads(i), errStat, errMsg)
   end do  
   
      ! Rotors [ Point meshes ]
   !do i = 1, p%numPylons
   !   call CreateMBDynRtrPtLoadsMesh(FusO, m%SPyRtrO(:,1,i), m%FusODCM,  m%mbdSPyRtrLoads(i), errStat, errMsg)
   !   call CreateMBDynRtrPtLoadsMesh(FusO, m%SPyRtrO(:,2,i), m%FusODCM,  m%mbdSPyRtrLoads(i), errStat, errMsg)
   !end do   
   !do i = 1, p%numPylons
   !   call CreateMBDynRtrPtLoadsMesh(FusO, m%PPyRtrO(:,1,i), m%FusODCM,  m%mbdPPyRtrLoads(i), errStat, errMsg)
   !   call CreateMBDynRtrPtLoadsMesh(FusO, m%PPyRtrO(:,2,i), m%FusODCM,  m%mbdPPyRtrLoads(i), errStat, errMsg)
   !end do
   
   end subroutine SetupMBDynMotionData
                       
end subroutine KFAST_Init

subroutine KFAST_AssRes(t, numRtSpdRtrElem_c, RtSpd_PyRtr_c, WindPt_c, FusO_c, FusODCM_c, FusOv_c, FusOomegas_c, FusOacc_c, numNodePtElem_c, nodePts_c, &
                          numNodeVelElem_c, nodeVels_c, numNodeOmegaElem_c, nodeOmegas_c, numDCMElem_c, nodeDCMs_c, numRtrPtsElem_c, rtrPts_c, &
                          errStat_c, errMsg_c ) BIND (C, NAME='KFAST_AssRes')
   IMPLICIT NONE

   real(C_DOUBLE),         intent(in   ) :: t                                 ! simulation time  (s)
   integer(C_INT),         intent(in   ) :: numRtSpdRtrElem_c                 ! total number of array elements in the rotor rotor speeds array
   real(C_DOUBLE),         intent(in   ) :: RtSpd_PyRtr_c(numRtSpdRtrElem_c)  ! 1D array containing all the rotor speeds for the kite 
   real(C_DOUBLE),         intent(in   ) :: WindPt_c(3)                       ! The location of the ground station point where the freestream wind is measured [global coordinates] (m)
   real(C_DOUBLE),         intent(in   ) :: FusO_c(3)                         ! The location of the principal Kite reference point [global coordinates] (m)
   real(C_DOUBLE),         intent(in   ) :: FusODCM_c(9)                      ! Principal reference point DCM (MIP of kite)
   real(C_DOUBLE),         intent(in   ) :: FusOv_c(3)                        ! Translational velocities at the principal reference point [global coordinates] (m/s)
   real(C_DOUBLE),         intent(in   ) :: FusOomegas_c(3)                   ! Angular velocities at the principal reference point [global coordinates] (rad/s)
   real(C_DOUBLE),         intent(in   ) :: FusOacc_c(3)                      ! Accelerations at the principal reference point  [global coordinates] (m/s^2)
   integer(C_INT),         intent(in   ) :: numNodePtElem_c                   ! total number of array elements in the nodal points array
   real(C_DOUBLE),         intent(in   ) :: nodePts_c(numNodePtElem_c)        ! 1D array containing all the nodal point coordinate data
   integer(C_INT),         intent(in   ) :: numNodeVelElem_c                  ! total number of array elements in the nodal translational velocities array
   real(C_DOUBLE),         intent(in   ) :: nodeVels_c(numNodeVelElem_c)      ! 1D array containing all the nodal translational velocities data
   integer(C_INT),         intent(in   ) :: numNodeOmegaElem_c                ! total number of array elements in the nodal angular velocities array
   real(C_DOUBLE),         intent(in   ) :: nodeOmegas_c(numNodeOmegaElem_c) ! 1D array containing all the nodal angular velocities data
   integer(C_INT),         intent(in   ) :: numDCMElem_c                      ! total number of array elements in the nodal DCM data
   real(C_DOUBLE),         intent(in   ) :: nodeDCMs_c(numDCMElem_c)          ! 1D array containing all the nodal DCM data
   integer(C_INT),         intent(in   ) :: numRtrPtsElem_c                   ! total number of rotor point elements
   real(C_DOUBLE),         intent(in   ) :: rtrPts_c(numRtrPtsElem_c)         ! location of the rotor points
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   


   integer(IntKi)           :: n, c, i, j                     ! counters
   real(DbKi)               :: utimes(2)                      ! t and t+dt timestep values (s)
   real(ReKi)               :: FusO(3)                        ! fuselage (or kite) reference point location in global (inertial) coordinates (m)
   real(ReKi)               :: FusOv(3)                       ! fuselage reference point translational velocities in global (inertial) coordinates (m)
   real(ReKi)               :: FusODCM(3,3)                   ! fuselage reference point DCM to transform from global to kite coordinates
   real(ReKi)               :: FusOomegas(3)                  ! fuselage reference point rotational velocities in global coordinates (m/s)
   real(ReKi)               :: FusOacc(3)                     ! fuselage reference point accelerations in global coordinates (m/s^2)
   integer(IntKi)           :: errStat, errStat2              ! error status values
   character(ErrMsgLen)     :: errMsg, errMsg2                ! error messages
   character(*), parameter  :: routineName = 'KFAST_AssRes'
   
   errStat = ErrID_None
   errMsg  = ''
   
   n = t / p%DT
   
      ! Transfer fuselage (kite) principal point inputs
   FusO       = FusO_c
   FusOv      = FusOv_c
   FusOomegas = FusOomegas_c
   FusOacc    = FusOacc_c
   
      ! Transfer C-based nodal and rotor quantities into Fortran-based data structures (but not meshes, yet)
      !   The resulting data resides inside the MiscVars data structure (m)
   
   m%FusODCM = reshape(FusODCM_c,(/3,3/))  
   
   if ( p%useKAD ) then
      call TransferMBDynToKAD( FusO, numNodePtElem_c, nodePts_c, numNodeVelElem_c, nodeVels_c, numNodeOmegaElem_c, nodeOmegas_c, numDCMElem_c, nodeDCMs_c, rtrPts_c, p, m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors()
            return
         end if
   end if
   
   if ( p%useIfW ) then
      call TransferMBDynToIfW( WindPt_c, FusO, numNodePtElem_c, nodePts_c, rtrPts_c, p, m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors()
            return
         end if
   end if
   

! -------------------------------------------------------------------------
! InflowWind
! -------------------------------------------------------------------------      
      ! The inputs to InflowWind are the positions where wind velocities [the outputs] are to be computed.  These inputs are set above by
      !  the TransferMBDynInputs() call.
   if ( p%useIfW ) then
      call InflowWind_CalcOutput(t, m%IfW%u, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z, m%IfW%OtherSt, m%IfW%y, m%IfW%m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors()
            return
         end if
   end if
   
! -------------------------------------------------------------------------
! Controller
! -------------------------------------------------------------------------      
   
   if (m%NewTime) then
      if ( p%useKFC ) then
         m%KFC%u%dcm_g2b       = matmul(m%FusODCM, transpose(p%DCM_Fast2Ctrl))
         m%KFC%u%pqr           = matmul(m%FusODCM, FusOomegas)
         m%KFC%u%acc_norm      = TwoNorm(FusOacc)
         m%KFC%u%Xg            = FusO - p%anchorPt
         m%KFC%u%Xg            = matmul(p%DCM_Fast2Ctrl, m%KFC%u%Xg)
         m%KFC%u%Vg            = matmul(p%DCM_Fast2Ctrl, FusOv)
         m%KFC%u%Vb            = matmul(m%FusODCM, FusOv)
         m%KFC%u%Ag            = matmul(p%DCM_Fast2Ctrl, FusOacc)
         m%KFC%u%Ab            = matmul(m%FusODCM, FusOacc)
         m%KFC%u%rho           = p%AirDens
         m%KFC%u%apparent_wind = m%IfW%y%VelocityUVW(:,2) - FusOv
         m%KFC%u%apparent_wind = matmul(p%DCM_Fast2Ctrl, m%KFC%u%apparent_wind)
        ! m%KFC%u%tether_force  = 0 
         m%KFC%u%wind_g        = matmul(p%DCM_Fast2Ctrl, m%IfW%y%VelocityUVW(:,1))
      
         call KFC_CalcOutput(t, m%KFC%u, m%KFC%p, m%KFC%y, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors()
            return
         end if
      
      end if
      
      m%NewTime = .false. ! only call the controller once per timestep
      
   end if
  
! -------------------------------------------------------------------------
! MoorDyn
! ------------------------------------------------------------------------- 
   
   if ( p%useMD ) then
      utimes(1) = t - p%DT
      utimes(2) = t
         ! Pos and Vel of each Fairlead (Bridle connectin) at t.  
         !   We need to set the TranslationDisp, Orientation, TranslationVel, RotationVel properties on the mesh
      m%MD%u(1)%PtFairleadDisplacement%TranslationDisp = 0.0
      m%MD%u(1)%PtFairleadDisplacement%Orientation     = 0.0
      m%MD%u(1)%PtFairleadDisplacement%TranslationVel  = 0.0
      m%MD%u(1)%PtFairleadDisplacement%RotationVel     = 0.0
         ! Pos and Vel of each Fairlead (Bridle connectin) at t + dt
      m%MD%u(2)%PtFairleadDisplacement%TranslationDisp = 0.0
      m%MD%u(2)%PtFairleadDisplacement%Orientation     = 0.0
      m%MD%u(2)%PtFairleadDisplacement%TranslationVel  = 0.0
      m%MD%u(2)%PtFairleadDisplacement%RotationVel     = 0.0
   
      call MD_CopyContState( m%MD%x, m%MD%x_copy, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call MD_UpdateStates( t, n, m%MD%u, utimes, m%MD%p, m%MD%x_copy, m%MD%xd, m%MD%z, m%MD%OtherSt, m%MD%m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call MD_CalcOutput( t, m%MD%u(2), m%MD%p, m%MD%x_copy, m%MD%xd, m%MD%z, m%MD%OtherSt, m%MD%y, m%MD%m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      if (errStat >= AbortErrLev ) then
         call TransferErrors()
         return
      end if
   
   end if
   
      
! -------------------------------------------------------------------------
! KiteAeroDyn
! -------------------------------------------------------------------------      
   
   if ( p%useKAD ) then
         ! Outputs from Controller for KAD
      if ( p%useKFC ) then
         m%KAD%u%Ctrl_SFlp = m%KFC%y%SFlp
         m%KAD%u%Ctrl_PFlp = m%KFC%y%PFlp
         m%KAD%u%Ctrl_Rudr = m%KFC%y%Rudr
         m%KAD%u%Ctrl_SElv = m%KFC%y%SElv
         m%KAD%u%Ctrl_PElv = m%KFC%y%PElv
      else
         m%KAD%u%Ctrl_SFlp = 0.0_ReKi
         m%KAD%u%Ctrl_PFlp = 0.0_ReKi
         m%KAD%u%Ctrl_Rudr = 0.0_ReKi
         m%KAD%u%Ctrl_SElv = 0.0_ReKi
         m%KAD%u%Ctrl_PElv = 0.0_ReKi
      end if
      
      m%KAD%u%Pitch_SPyRtr = 0.0_ReKi   ! Controller does not set these, yet.
      m%KAD%u%Pitch_PPyRtr = 0.0_ReKi
   
         ! Rotor Speeds from MBDyn  [2 per pylon]
      c = 1
      do i = 1, p%numPylons ! [moving from inboard to outboard]
         m%KAD%u%RtSpd_SPyRtr(1,i) = RtSpd_PyRtr_c(c)    ! top
         m%KAD%u%RtSpd_SPyRtr(2,i) = RtSpd_PyRtr_c(c+1)  ! bottom
         c = c+2
      end do
      do i = 1, p%numPylons   ! [moving from inboard to outboard]
         m%KAD%u%RtSpd_PPyRtr(1,i) = RtSpd_PyRtr_c(c)    ! top
         m%KAD%u%RtSpd_PPyRtr(2,i) = RtSpd_PyRtr_c(c+1)  ! bottom
         c = c+2
      end do
   
         ! Update the MBDyn Motions meshes based on the transferred inputs
   
         ! Map the MBDyn motion mesh data to the corresponding KAD motion mesh
   
      if ( p%useIfW ) then
            ! Transfer Inflow Wind outputs to the various KAD inflow inputs
         c=3
         do i = 1,size(m%KAD%u%V_Fus,2)
            m%KAD%u%V_Fus(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u%V_SWn,2)
            m%KAD%u%V_SWn(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u%V_PWn,2)
            m%KAD%u%V_PWn(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u%V_VS,2)
            m%KAD%u%V_VS(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u%V_SHS,2)
            m%KAD%u%V_SHS(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u%V_PHS,2)
            m%KAD%u%V_PHS(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do j = 1,p%numPylons
            do i = 1,size(m%KAD%u%V_SPy,2)
               m%KAD%u%V_SPy(:,i,j)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
         end do
         do j = 1,p%numPylons
            do i = 1,size(m%KAD%u%V_PPy,2)
               m%KAD%u%V_PPy(:,i,j)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
         end do   
         do i = 1,p%numPylons
            m%KAD%u%V_SPyRtr(:,1,i) = m%IfW%y%VelocityUVW(:,c)
            c = c+1
            m%KAD%u%V_SPyRtr(:,2,i) = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,p%numPylons
            m%KAD%u%V_PPyRtr(:,1,i) = m%IfW%y%VelocityUVW(:,c)
            c = c+1
            m%KAD%u%V_PPyRtr(:,2,i) = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
      else
         m%KAD%u%V_Fus     = 0.0_ReKi
         m%KAD%u%V_SWn     = 0.0_ReKi
         m%KAD%u%V_PWn     = 0.0_ReKi
         m%KAD%u%V_VS      = 0.0_ReKi
         m%KAD%u%V_SHS     = 0.0_ReKi
         m%KAD%u%V_PHS     = 0.0_ReKi
         m%KAD%u%V_SPy     = 0.0_ReKi
         m%KAD%u%V_PPy     = 0.0_ReKi         
         m%KAD%u%V_SPyRtr  = 0.0_ReKi    
         m%KAD%u%V_PPyRtr  = 0.0_ReKi  
      end if
      
      call KAD_CopyConstrState( m%KAD%z, m%KAD%z_copy, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors()
            return
         end if
      
      
      call KAD_CalcOutput( t, m%KAD%u, m%KAD%p, m%KAD%x, m%KAD%xd, m%KAD%z_copy, m%KAD%OtherSt, m%KAD%y, m%KAD%m, errStat, errMsg )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   
   end if
   
   call TransferLoadsToMBDyn()
   
   call TransferErrors()
   return
   
contains
                          
   subroutine TransferLoadsToMBDyn()
      
   end subroutine TransferLoadsToMBDyn
   
   
   
   subroutine TransferErrors()
   
              ! transfer Fortran variables to C:  
      errStat_c = errStat
      errMsg    = trim(errMsg)//C_NULL_CHAR
      errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )
      
   end subroutine TransferErrors
   
end subroutine KFAST_AssRes

subroutine KFAST_AfterPredict(errStat_c, errMsg_c) BIND (C, NAME='KFAST_AfterPredict')
   IMPLICIT NONE

   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_AfterPredict'
   
   errStat = ErrID_None
   errMsg  = ''

   m%NewTime = .true.
   
      ! Copy the temporary states and place them into the actual versions
   call MD_CopyContState   ( m%MD%x_copy, m%MD%x, MESH_NEWCOPY, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KAD_CopyConstrState( m%KAD%z_copy, m%KAD%z, MESH_NEWCOPY, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   
              ! transfer Fortran variables to C:  
   errStat_c = errStat
   errMsg    = trim(errMsg)//C_NULL_CHAR
   errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )
   
end subroutine KFAST_AfterPredict

subroutine KFAST_Output(errStat_c, errMsg_c) BIND (C, NAME='KFAST_Output')
   IMPLICIT NONE
   
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_Output'

   errStat = ErrID_None
   errMsg  = ''

   ! Write any outputs to file
   ! call KFAST_WriteOutput()
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
                 ! transfer Fortran variables to C:  
   errStat_c = errStat
   errMsg    = trim(errMsg)//C_NULL_CHAR
   errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )

end subroutine KFAST_Output

   
subroutine KFAST_End(errStat_c, errMsg_c) BIND (C, NAME='KFAST_End')
   IMPLICIT NONE
   
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_End'

   errStat = ErrID_None
   errMsg  = ''

   
   ! Call the End subroutines for KiteAeroDyn, MoorDyn, InflowWind, and the Controller
   !call KAD_End()
   !call MD_End()
   !call KFC_End()
   
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
! -------------------------------------------------------------------------
! Close the Output file
! -------------------------------------------------------------------------   
      
      
      ! transfer Fortran variables to C:  
   errStat_c = errStat
   errMsg    = trim(errMsg)//C_NULL_CHAR
   errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )
  
end subroutine KFAST_End
   
end module KiteFAST
   