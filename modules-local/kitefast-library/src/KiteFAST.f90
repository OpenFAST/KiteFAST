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
   use KiteFAST_IO
   use KiteFAST_Subs
   use KiteFAST_GlobalData
   
   implicit none 

private

  
   public :: KFAST_Init
   public :: KFAST_AssRes
   public :: KFAST_Output
   public :: KFAST_AfterPredict
   public :: KFAST_End


   contains

















  

subroutine KFAST_Init(dt_c, numFlaps, numPylons, numComp, numCompNds, modFlags, KAD_FileName_c, IfW_FileName_c, MD_FileName_c, KFC_FileName_c, &
                       outFileRoot_c, printSum, gravity, KAD_InterpOrder, FusODCM_c, numRtrPts_c, rtrPts_c, rtrMass_c, rtrI_Rot_c, rtrI_trans_c, rtrXcm_c, refPts_c, &
                       numNodePts_c, nodePts_c, nodeDCMs_c, nFusOuts_c, FusOutNd_c, nSWnOuts_c, SWnOutNd_c, &
                       nPWnOuts_c, PWnOutNd_c, nVSOuts_c, VSOutNd_c, nSHSOuts_c, SHSOutNd_c, nPHSOuts_c, PHSOutNd_c, nPylOuts_c, PylOutNd_c, numOutChan_c, chanlist_c, chanlist_len_c, errStat_c, errMsg_c ) BIND (C, NAME='KFAST_Init')
   IMPLICIT NONE


   real(C_DOUBLE),         intent(in   ) :: dt_c                           ! Timestep size (s)
   integer(C_INT),         intent(in   ) :: numFlaps                       ! Number of flaps
   integer(C_INT),         intent(in   ) :: numPylons                      ! Number of pylons, per wing
   integer(C_INT),         intent(in   ) :: numComp                        ! Number of kite components
   integer(C_INT),         intent(in   ) :: numCompNds(numComp)            ! MBDyn nodes per component.  The array is populated in the following order: Fuselage, Starboard Wing, Port Wing, Vertical Stabilizer, 
                                                                           !    Starboard Horizontal Stabilizer, Port Horizontal Stabilizer, Starboard pylon, from inner to outer, and then Port pylon, from inner to outer.
   integer(C_INT),         intent(in   ) :: modFlags(4)                    ! Four element array of module flags.  0 = inactive, 1 = active. Module indices are: 0 = KiteAeroDyn, 1 = InflowWind, 2 = MoorDyn, 3 = KiteFAST Controller
   character(kind=C_CHAR), intent(in   ) :: KAD_FileName_c(IntfStrLen)     ! Full path and name of the KiteAeroDyn input file.
   character(kind=C_CHAR), intent(in   ) :: IfW_FileName_c(IntfStrLen)     ! Full path and name of the InflowWind input file.
   character(kind=C_CHAR), intent(in   ) :: MD_FileName_c(IntfStrLen)      ! Full path and name of the MoorDyn input file.
   character(kind=C_CHAR), intent(in   ) :: KFC_FileName_c(IntfStrLen)     ! Full path and name of the KiteFAST controller shared object file.
   character(kind=C_CHAR), intent(in   ) :: outFileRoot_c(IntfStrLen)      ! Full path and basename of the KiteFAST output file.
   integer(C_INT),         intent(in   ) :: printSum                       ! Print the Summary file?  1 = Yes, 0 = No.
   real(C_DOUBLE),         intent(in   ) :: gravity                        ! Scalar gravity constant.  (m/s^2)
   integer(C_INT),         intent(in   ) :: KAD_InterpOrder                ! KiteAeroDyn outputs interpolation order. 0 = hold outputs between calls, 1 = linear interpolation, 2 = 2nd order interpolation.
   real(C_DOUBLE),         intent(in   ) :: FusODCM_c(9)                   ! Initial DCM matrix to transform the location of the Kite Fuselage reference point from global to kite coordinates.
   integer(C_INT),         intent(in   ) :: numRtrPts_c                    ! Total number of rotor points (both wings).
   real(C_DOUBLE),         intent(in   ) :: rtrPts_c(numRtrPts_c*3)                    ! Initial location of each rotor's reference point [RRP] in global coordinates. (m)
   real(C_DOUBLE),         intent(in   ) :: rtrMass_c(numRtrPts_c)                   ! Mass of the rotor/drivetrain (kg)
   real(C_DOUBLE),         intent(in   ) :: rtrI_Rot_c(numRtrPts_c)                  ! Rotational inertia about the shaft axis of the top and bottom rotors/drivetrains on the pylons on the wing meshes (kg-m2)
   real(C_DOUBLE),         intent(in   ) :: rtrI_trans_c(numRtrPts_c)                ! Transverse inertia about the rotor reference point of the top and bottom rotors/drivetrains on the pylons on the wing meshes (kg-m2)
   real(C_DOUBLE),         intent(in   ) :: rtrXcm_c(numRtrPts_c)                    ! Distance along the shaft from the rotor reference point of the top and bottom rotors/drivetrains on the pylons on the wing meshes to the center of mass of the rotor/drivetrain (positive along positive x) (m)
   real(C_DOUBLE),         intent(in   ) :: refPts_c(numComp*3)                    ! Initial location of the MBDyn component reference points in the global coordinates. (m)  The length of this array comes from  numComp * 3.
   integer(C_INT),         intent(in   ) :: numNodePts_c                   ! The total number of MBDyn structural nodes.  We need this total number (which could be derived from the numCompNds array) to size the following arrays in the Fortran code. 
   real(C_DOUBLE),         intent(in   ) :: nodePts_c(numNodePts_c*3)                   ! Initial location of the MBDyn structural nodes in the global coordinates. (m)  The array is populated in the same order at the numCompNds array.
   real(C_DOUBLE),         intent(in   ) :: nodeDCMs_c(numNodePts_c*9)                  ! Initial DCMs matrices to transform each nodal point from global to kite coordinates.
   integer(C_INT),         intent(in   ) :: nFusOuts_c                     ! Number of user-requested output locations on the fuselage  ( 0-9 )
   integer(C_INT),         intent(in   ) :: FusOutNd_c(nFusOuts_c)         ! Node number(s) (within the component) of the requested output locations.  Structural node index for motions and KiteAeroDyn quantities,  and Gauss point index for MBDyn structural loads
   integer(C_INT),         intent(in   ) :: nSWnOuts_c                     ! Number of user-requested output locations on the starboard wing  ( 0-9 )
   integer(C_INT),         intent(in   ) :: SWnOutNd_c(nSWnOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nPWnOuts_c                     ! Number of user-requested output locations on the port wing  ( 0-9 )
   integer(C_INT),         intent(in   ) :: PWnOutNd_c(nPWnOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nVSOuts_c                      ! Number of user-requested output locations on the vertical stabilizer  ( 0-9 )
   integer(C_INT),         intent(in   ) :: VSOutNd_c(nVSOuts_c)           ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nSHSOuts_c                     ! Number of user-requested output locations on the starboard horizontal stabilizer  ( 0-9 )
   integer(C_INT),         intent(in   ) :: SHSOutNd_c(nSHSOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nPHSOuts_c                     ! Number of user-requested output locations on the port horizontal stabilizer  ( 0-9 )
   integer(C_INT),         intent(in   ) :: PHSOutNd_c(nPHSOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nPylOuts_c                     ! Number of user-requested output locations on each pylon  ( 0-9 )
   integer(C_INT),         intent(in   ) :: PylOutNd_c(nPylOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: numOutChan_c                   ! Number of user-requested output channel names
   type(c_ptr)   ,target,  intent(in   ) :: chanlist_c(numOutChan_c)       ! Array of output channel names (strings)
   integer(C_INT),         intent(in   ) :: chanlist_len_c(numOutChan_c )   ! The length of each string in the above array
   integer(C_INT),         intent(  out) :: errStat_c                      ! Error code coming from KiteFAST
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)           ! Error message

      ! Local variables
   real(DbKi)                      :: dt, test
   real(ReKi)                      :: FusO(3)
   type(KAD_InitInputType)         :: KAD_InitInp
   type(KAD_InitOutputType)        :: KAD_InitOut
   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   type(InflowWind_InitInputType)  :: IfW_InitInp 
   type(InflowWind_InitOutputType) :: IfW_InitOut
   type(MD_InitInputType)          :: MD_InitInp
   type(MD_InitOutputType)         :: MD_InitOut
   type(KFC_InitInputType)         :: KFC_InitInp
   character(*), parameter         :: routineName = 'KFAST_Init'
   integer(IntKi)                  :: i,j,c, count, maxSPyNds, maxPPyNds, SumFileUnit
   real(DbKi)                      :: interval
   CHARACTER(ChanLen)              :: OutList(MaxOutPts)              ! MaxOutPts is defined in KiteFAST_IO.f90, ChanLen defined in NWTC_Base.f90
   character, pointer              :: chanName_f(:)   
   character(255)                  :: enabledModules
   character(255)                  :: disabledModules
   integer(IntKi) :: lenstr, maxPyNds
   ! Initialize all the sub-modules : MoorDyn, KiteAeroDyn, Controller, and InflowWind
   ! Set KiteFAST-level parameters, misc vars
   ! Open an Output file
   errStat = ErrID_None
   errMsg  = ''
  
         ! Initialize the NWTC Subroutine Library
   call NWTC_Init( EchoLibVer=.FALSE. )

   dt = dt_c
 
   call SetupSim(dt, modFlags, gravity, outFileRoot_c, numOutChan_c, chanlist_c, chanlist_len_c, p, OutList, errStat, errMsg)
   if (errStat >= AbortErrLev ) then
      call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
      return
   end if 
   
   call Init_KiteSystem(dt_c, numFlaps, numPylons, numComp, numCompNds, modFlags, KAD_FileName_c, IfW_FileName_c, MD_FileName_c, KFC_FileName_c, &
                       outFileRoot_c, printSum, gravity, KAD_InterpOrder, FusODCM_c, numRtrPts_c, rtrPts_c, rtrMass_c, rtrI_Rot_c, rtrI_trans_c, rtrXcm_c, refPts_c, &
                       numNodePts_c, nodePts_c, nodeDCMs_c, nFusOuts_c, FusOutNd_c, nSWnOuts_c, SWnOutNd_c, &
                       nPWnOuts_c, PWnOutNd_c, nVSOuts_c, VSOutNd_c, nSHSOuts_c, SHSOutNd_c, nPHSOuts_c, PHSOutNd_c, nPylOuts_c, PylOutNd_c, KAD_InitOut, MD_InitOut, IfW_InitOut, errStat, errMsg )
   if (errStat >= AbortErrLev ) then
      call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
      return
   end if

      ! Set parameters for output channels:
   call KFAST_SetOutParam(OutList, p%numKFASTOuts, p, errStat2, errMsg2 ) ! requires:  sets: p%OutParam.
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if

   call AllocAry( m%WriteOutput, p%numKFASTOuts, 'KFAST outputs', errStat2, errMsg2 )
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if
! -------------------------------------------------------------------------
! Open the Output file and generate header data
! -------------------------------------------------------------------------      
   call KFAST_SetNumOutputs( p, KAD_InitOut, MD_InitOut, IfW_InitOut, errStat, errMsg )
   if ( p%numOuts > 0 ) then
      call KFAST_OpenOutput( KFAST_Ver, p%outFileRoot, p, errStat, errMsg )    
   
      call KFAST_WriteOutputTimeChanName( p%UnOutFile )
      call KFAST_WriteOutputChanNames( p, KAD_InitOut, MD_InitOut, IfW_InitOut )
      call KFAST_WriteOutputNL( p%UnOutFile )
   
      call KFAST_WriteOutputTimeChanUnits( p%UnOutFile )   
      call KFAST_WriteOutputUnitNames( p, KAD_InitOut, MD_InitOut, IfW_InitOut )    
      call KFAST_WriteOutputNL( p%UnOutFile )
   end if  
! -------------------------------------------------------------------------
! Create Summary file
! -------------------------------------------------------------------------      
      
   if (printSum == 1) then  
      call SumModuleStatus( p, enabledModules, disabledModules )
      call KFAST_OpenSummary( KFAST_Ver, p%outFileRoot, enabledModules, disabledModules, p%dt, SumFileUnit, errStat, errMsg )
      call KFAST_WriteSummary( SumFileUnit, p, m, KAD_InitOut, MD_InitOut, IfW_InitOut, ErrStat2, ErrMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      close(SumFileUnit)
         
   end if 
   
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
   return


                       
end subroutine KFAST_Init

subroutine KFAST_AssRes(t_c, isInitialTime_c, WindPt_c, FusO_c, FusODCM_c, FusOv_c, FusOomegas_c, FusOacc_c, FusOalphas_c, numNodePts_c, nodePts_c, &
                          nodeDCMs_c, nodeVels_c, nodeOmegas_c, nodeAccs_c,  numRtrPts_c, rtrPts_c, &
                          rtrDCMs_c, rtrVels_c, rtrOmegas_c, rtrAccs_c, rtrAlphas_c, nodeLoads_c, rtrLoads_c, errStat_c, errMsg_c ) BIND (C, NAME='KFAST_AssRes')
   IMPLICIT NONE

   real(C_DOUBLE),         intent(in   ) :: t_c                          !  simulation time for the current timestep (s)
   integer(C_INT),         intent(in   ) :: isInitialTime_c              !  1 = first time KFAST_AssRes has been called for this particular timestep, 0 = otherwise
   real(C_DOUBLE),         intent(in   ) :: WindPt_c(3)                  !  Position of the ground station where the fixed wind measurement is taken, expressed in global coordinates. (m)
   real(C_DOUBLE),         intent(in   ) :: FusO_c(3)                    !  Current  timestep position of the Fuselage reference point, expressed in global coordinates. (m) 
   real(C_DOUBLE),         intent(in   ) :: FusODCM_c(9)                 !  Current  timestep DCM matrix to transform the location of the Fuselage reference point from global to kite coordinates.
   real(C_DOUBLE),         intent(in   ) :: FusOv_c(3)                   !  Current timestep velocity of the Fuselage reference point, expressed in global coordinates. (m/s)
   real(C_DOUBLE),         intent(in   ) :: FusOomegas_c(3)              !  Current timestep rotational velocity of the Fuselage reference point, expressed in global coordinates. (rad/s) 
   real(C_DOUBLE),         intent(in   ) :: FusOacc_c(3)                 !  Current timestep translational acceleration of the Fuselage reference point, expressed in global coordinates. (m/s^2) 
   real(C_DOUBLE),         intent(in   ) :: FusOalphas_c(3)              !  Current timestep rotational acceleration of the Fuselage reference point, expressed in global coordinates. (rad/s^2) 
   integer(C_INT),         intent(in   ) :: numNodePts_c                 !  Total umber of MBDyn structural nodes. This must match what was sent during KFAST_Init, but is useful here for sizing Fortran arrays.
   real(C_DOUBLE),         intent(in   ) :: nodePts_c(numNodePts_c*3)    !  Location of the MBDyn structural nodes for the current timestep, expressed in the global coordinates. (m)  
                                                                         !     The array is populated in the following order: Fuselage, Starboard Wing, Port Wing, Vertical Stabilizer, 
                                                                         !       Starboard Horizontal Stabilizer, Port Horizontal Stabilizer, Starboard pylon, from inner to outer, and then
                                                                         !       Port pylon, from inner to outer.
   real(C_DOUBLE),         intent(in   ) :: nodeDCMs_c(numNodePts_c*9)   !  DCMs matrices to transform each nodal point from global to kite coordinates.
   real(C_DOUBLE),         intent(in   ) :: nodeVels_c(numNodePts_c*3)   !  Translational velocities of each nodal point in global coordinates. (m/s)
   real(C_DOUBLE),         intent(in   ) :: nodeOmegas_c(numNodePts_c*3) !  Rotational velocities of each nodal point in global coordinates. (rad/s)
   real(C_DOUBLE),         intent(in   ) :: nodeAccs_c(numNodePts_c*3)   !  Translational accelerations of each nodal point in global coordinates. (m/s^2)
   integer(C_INT),         intent(in   ) :: numRtrPts_c                  !  Total number of rotor points.  This must match what was sent during KFAST_Init, but is used here for straigh-forward declaration of array sizes on the Fortran side.
   real(C_DOUBLE),         intent(in   ) :: rtrPts_c(numRtrPts_c*3)      !  Location of each rotor's reference point [RRP] in global coordinates. (m)  The order of these points follows this sequence:
                                                                         !     Start on starboard side moving from the inner pylon outward to the most outboard pylon.  Within a plyon, start with the top rotor and then the bottom rotor
                                                                         !       then repeat this sequence for the port side.
   real(C_DOUBLE),         intent(in   ) :: rtrDCMs_c(numRtrPts_c*9)     !  DCMs matrices to transform each RRP point from global to kite coordinates.
   real(C_DOUBLE),         intent(in   ) :: rtrVels_c(numRtrPts_c*3)     !  Translational velocity of the nacelle (RRP) in global coordinates. (m/s)
   real(C_DOUBLE),         intent(in   ) :: rtrOmegas_c(numRtrPts_c*3)   !  Rotational velocity of the nacelle (RRP) in global coordinates. (rad/s)
   real(C_DOUBLE),         intent(in   ) :: rtrAccs_c(numRtrPts_c*3)     !  Translational accelerations of the nacelle (RRP) in global coordinates. (m/s^2)
   real(C_DOUBLE),         intent(in   ) :: rtrAlphas_c(numRtrPts_c*3)   !  Rotational accelerations of the nacelle (RRP) in global coordinates. (rad/s^2)
   real(C_DOUBLE),         intent(  out) :: nodeLoads_c(numNodePts_c*6)  !  KiteFAST loads (3 forces + 3 moments) in global coordinates ( N, N-m ) at the MBDyn structural nodes.  Sequence follows the pattern used for MBDyn structural node array.  Returned from KiteFAST to MBDyn.
   real(C_DOUBLE),         intent(  out) :: rtrLoads_c(numRtrPts_c*6)    !  Concentrated reaction loads at the nacelles on the pylons at the RRPs in global coordinates.  Length is 6 loads per rotor * number of RRPs. Returned from KiteFAST to MBDyn.
   integer(C_INT),         intent(  out) :: errStat_c                    !  Error code coming from KiteFAST
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)         !  Error message
   
   
   ! Local variables
   integer(IntKi)           :: errStat, errStat2              ! error status values
   character(ErrMsgLen)     :: errMsg, errMsg2                ! error messages
   character(*), parameter  :: routineName = 'KFAST_AssRes'
   
   errStat = ErrID_None
   errMsg  = ''
   
   call AssRes_OnShore( t_c, isInitialTime_c, WindPt_c, FusO_c, FusODCM_c, FusOv_c, FusOomegas_c, FusOacc_c, FusOalphas_c, numNodePts_c, nodePts_c, &
                          nodeDCMs_c, nodeVels_c, nodeOmegas_c, nodeAccs_c,  numRtrPts_c, rtrPts_c, &
                          rtrDCMs_c, rtrVels_c, rtrOmegas_c, rtrAccs_c, rtrAlphas_c, nodeLoads_c, rtrLoads_c, errStat, errMsg ) 

   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
   

end subroutine KFAST_AssRes

subroutine KFAST_AfterPredict(t_c, errStat_c, errMsg_c) BIND (C, NAME='KFAST_AfterPredict')
   IMPLICIT NONE
   real(C_DOUBLE),         intent(in   ) :: t_c
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat
   character(ErrMsgLen)            :: errMsg

   errStat = ErrID_None
   errMsg  = ''
   
   call AfterPredict_Onshore(t_c, errStat, errMsg)
  
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
      
end subroutine KFAST_AfterPredict

subroutine KFAST_Output(t_c, numGaussPts_c, gaussPtLoads_c, errStat_c, errMsg_c) BIND (C, NAME='KFAST_Output')
   IMPLICIT NONE
   real(C_DOUBLE),         intent(in   ) :: t_c                              ! simulation time for the current timestep (s)   
   integer(C_INT),         intent(in   ) :: numGaussPts_c                    ! Total number of gauss points in the MBDyn model
   real(C_DOUBLE),         intent(in   ) :: gaussPtLoads_c(numGaussPts_c*6)  ! Array of loads in the global coordinate system (3 forces + 3 moments) corresponding to each MBDyn gauss point. ( N, N-m )
   integer(C_INT),         intent(  out) :: errStat_c
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_Output'
   real(DbKi)                      :: t
   
   errStat = ErrID_None
   errMsg  = ''
   t = real(t_c,DbKi)

   
      

   ! Write any outputs to file
   if (p%numOuts > 0 ) then
      
      call TransferMBDynLoadInputs( numGaussPts_c, gaussPtLoads_c, p, m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
         ! Compute the MBDyn-related outputs using the MBDyn load data and the most recently recieved motion data
      call KFAST_ProcessOutputs()
      
      call KFAST_WriteOutputTime( t, p%UnOutFile )
      call KFAST_WriteOutput( p, m%KAD%y, m%MD_Tether%y, m%IfW%y, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call KFAST_WriteOutputNL( p%UnOutFile ) 
      
   end if
   
      ! transfer Fortran variables to C:  
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)

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
   call End_Onshore(errStat, errMsg)
      
! -------------------------------------------------------------------------
! Close the Output file
! -------------------------------------------------------------------------   
   close(p%UnOutFile)
      
      ! transfer Fortran variables to C:  
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
  
end subroutine KFAST_End
   
end module KiteFAST
   
