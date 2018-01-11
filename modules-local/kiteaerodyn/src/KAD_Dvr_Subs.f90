!**********************************************************************************************************************************
! LICENSING
! Copyright (C) 2018  National Renewable Energy Laboratory
!
!    This file is part of OpenFAST.
!
! Licensed under the Apache License, Version 2.0 (the "License");
! you may not use this file except in compliance with the License.
! You may obtain a copy of the License at
!
!     http://www.apache.org/licenses/LICENSE-2.0
!
! Unless required by applicable law or agreed to in writing, software
! distributed under the License is distributed on an "AS IS" BASIS,
! WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
! See the License for the specific language governing permissions and
! limitations under the License.
!
!**********************************************************************************************************************************
!> This module provides the routines necessary to run KiteAeroDyn in stand-alone mode.
module KAD_Dvr_Subs

   
   use NWTC_Library   
   use KiteAeroDyn
   use KiteAeroDyn_Types
   
   implicit none 

private

   type(ProgDesc), parameter  :: KAD_Dvr_Ver = ProgDesc( 'KiteAeroDyn_Driver', '', '' )
   
   type :: KAD_Dvr_MeshMapData
      type(MeshMapType)	:: FusO_P_2_Fus_L
      type(MeshMapType)	:: FusO_P_2_SWn_L
      type(MeshMapType)	:: FusO_P_2_PWn_L
      type(MeshMapType)	:: FusO_P_2_VSP_L
      type(MeshMapType)	:: FusO_P_2_SHS_L
      type(MeshMapType)	:: FusO_P_2_PHS_L
      type(MeshMapType)	:: FusO_P_2_SPy_L(4)
      type(MeshMapType)	:: FusO_P_2_PPy_L(4)
      type(MeshMapType)	:: FusO_P_2_SPyRtr_P(8)
      type(MeshMapType)	:: FusO_P_2_PPyRtr_P(8)   
   end type KAD_Dvr_MeshMapData

   type :: KAD_Dvr_Motions
      real(ReKi), allocatable      :: Times(:)      
      real(ReKi), allocatable      :: KitePXi(:) 
      real(ReKi), allocatable      :: KitePYi(:) 
      real(ReKi), allocatable      :: KitePZi(:) 
      real(ReKi), allocatable      :: Roll(:)    
      real(ReKi), allocatable      :: Pitch(:)    
      real(ReKi), allocatable      :: Yaw(:)       
      real(ReKi), allocatable      :: KiteTVXi(:)
      real(ReKi), allocatable      :: KiteTVYi(:)
      real(ReKi), allocatable      :: KiteTVZi(:)
      real(ReKi), allocatable      :: KiteRVXi(:)
      real(ReKi), allocatable      :: KiteRVYi(:)
      real(ReKi), allocatable      :: KiteRVZi(:)    
      real(ReKi), allocatable      :: SRtSpd(:,:,:)
      real(ReKi), allocatable      :: SRtPitch(:,:,:)
      real(ReKi), allocatable      :: PRtSpd(:,:,:)
      real(ReKi), allocatable      :: PRtPitch(:,:,:)
      real(ReKi), allocatable      :: FlpCtrl(:,:) 
      real(ReKi), allocatable      :: RudrCtrl(:,:) 
      real(ReKi), allocatable      :: ElvCtrl(:,:)   
   end type KAD_Dvr_Motions
      
   type, public :: KAD_Dvr_InitInputType
      real(DbKi)      :: DTAero                ! Time interval for aerodynamic calculations (s)
      character(1024) :: OutFileRoot           ! Root name for any output files (sring) [use "" for .dvr rootname]
      logical         :: TabDel                ! Make output tab-delimited? (fixed-width otherwise) (flag)
      character(200)  :: OutFmt                ! Format used for text tabular output, excluding the time channel.  Resulting field should be 10 characters. (quoted string)
      logical         :: Beep                  ! Beep on exit? (flag) 
      real(ReKi)      :: HWindSpd              ! Horizontal wind speed (m/s) [>=0.0]
      real(ReKi)      :: HWindDir              ! Horizontal wind propagation direction (meteoroligical rotation from aligned with X (positive rotates towards -Y)) (deg)
      real(ReKi)      :: RefHt                 ! Reference height for horizontal wind speed (m) [>0.0]
      real(ReKi)      :: PLexp                 ! Vertical wind shear power-law exponent (-) [>=0.0]
      integer(IntKi)  :: NumTimes              ! Number of time stamps (-) [>=2]
      type(KAD_InitInputType)  :: KAD_InitInp  ! KiteAeroDyn initialization input data
      type(KAD_Dvr_Motions) :: Motions         ! Motions (and Controls) time history for the simulation
   end type KAD_Dvr_InitInputType
   
   public :: KAD_Dvr_Read_InputFile
   public :: KAD_Dvr_ValidateInitData
   public :: KAD_Dvr_Simulate

   contains
   
!----------------------------------------------------------------------------------------------------------------------------------
!> Routine for establishing the mesh mapping data.  These mapping are created once during initialization,
!>  but then used to contstruct the mesh-based inputs for KiteAeroDyn at a given time step.   
subroutine KAD_Dvr_CreateMeshMappings( numPylons, u, MeshMapData, errStat, errMsg )
   integer(IntKi),                intent(in   )  :: numPylons      !< Number of pylons
   type(KAD_InputType),           intent(in   )  :: u              !< Inputs at Time
   type(KAD_Dvr_MeshMapData),     intent(inout)  :: MeshMapData    !< Mesh mapping data
   integer(IntKi),                intent(  out)  :: errStat        !< Status of error message
   character(1024),               intent(  out)  :: errMsg         !< Error message if errStat /= ErrID_None

   integer(IntKi)    :: i               ! counter
   integer(IntKi)    :: errStat2        ! Status of error message
   character(1024)   :: errMsg2         ! Error message if errStat /= ErrID_None

      ! Initialize error handling variables
   errStat  = ErrID_None
   errMsg   = ""

   call MeshMapCreate( u%FusOMotions, u%FusMotions, MeshMapData%FusO_P_2_Fus_L, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_Fus_L' )     
      
   call MeshMapCreate( u%FusOMotions, u%SWnMotions, MeshMapData%FusO_P_2_SWn_L, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_SWn_L' )     
      
   call MeshMapCreate( u%FusOMotions, u%PWnMotions, MeshMapData%FusO_P_2_PWn_L, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_PWn_L' )     
   
   call MeshMapCreate( u%FusOMotions, u%VSPMotions, MeshMapData%FusO_P_2_VSP_L, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_VSP_L' )     
      
   call MeshMapCreate( u%FusOMotions, u%SHSMotions, MeshMapData%FusO_P_2_SHS_L, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_SHS_L' )     
      
   call MeshMapCreate( u%FusOMotions, u%PHSMotions, MeshMapData%FusO_P_2_PHS_L, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_PHS_L' )     
      
   do i = 1 , numPylons      
      call MeshMapCreate( u%FusOMotions, u%SPyMotions(i), MeshMapData%FusO_P_2_SPy_L(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_SPy_L('//trim(num2lstr(i))//')' )     
      call MeshMapCreate( u%FusOMotions, u%PPyMotions(i), MeshMapData%FusO_P_2_PPy_L(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_PPy_L('//trim(num2lstr(i))//')' ) 
   end do
   
      ! Create Pylon Rotor Mappings: Point to Point
   do i = 1 , numPylons *2
      call MeshMapCreate( u%FusOMotions, u%SPyRtrMotions(i), MeshMapData%FusO_P_2_SPyRtr_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_SPyRtr_P('//trim(num2lstr(i))//')' ) 
      call MeshMapCreate( u%FusOMotions, u%PPyRtrMotions(i), MeshMapData%FusO_P_2_PPyRtr_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' KAD_Dvr_CreateMeshMappings: FusO_P_2_PPyRtr_P('//trim(num2lstr(i))//')' ) 
   end do
   
end subroutine KAD_Dvr_CreateMeshMappings

!----------------------------------------------------------------------------------------------------------------------------------
!> Routine for obtaining windspeed vector at a given height using an exponential shear model.  The model requires that Z be >= 0.0
!>   and RefHt be > 0.0
subroutine GetWindSpeed(Vref, Z, RefHt, VShr, Delta, WindSpeed, errStat, errMsg)

   real(ReKi),       intent(in   )  :: Vref            !< Reference windspeed 
   real(ReKi),       intent(in   )  :: Z               !< Height where windspeed will be computed
   real(ReKi),       intent(in   )  :: RefHt           !< Reference height where computed windspeed equals reference windspeed
   real(ReKi),       intent(in   )  :: VShr            !< Wind shear factor
   real(ReKi),       intent(in   )  :: Delta           !< Horizontal wind propagation direction (meteoroligical rotation from aligned with X (positive rotates towards -Y)) (rad)
   real(ReKi),       intent(  out)  :: WindSpeed(3,1)  !< The computed windspeed vector
   integer(IntKi),   intent(  out)  :: errStat         !< Status of error message
   character(1024),  intent(  out)  :: errMsg          !< Error message if errStat /= ErrID_None

   real(ReKi) :: CosDelta, SinDelta, V1
  
      ! Initialize error handling variables
   errStat  = ErrID_None
   errMsg   = ""

   CosDelta = COS( Delta )
   SinDelta = SIN( Delta )
   if ( Z < 0.0_ReKi ) then
      call SetErrStat( ErrID_Fatal, 'The windspeed height location must be >= 0.0', errStat, errMsg, 'GetWindSpeed' ) 
      return
   end if
   if ( RefHt <= 0.0_ReKi ) then
      call SetErrStat( ErrID_Fatal, 'The reference height for windspeed calculations must be > 0.0', errStat, errMsg, 'GetWindSpeed' ) 
      return
   end if
   V1 = Vref * ( ( Z / RefHt ) ** VShr )
         
   WindSpeed(1,1) =  V1 * CosDelta
   WindSpeed(2,1) = -V1 * SinDelta
   WindSpeed(3,1) =  0.0_ReKi
   
end subroutine GetWindSpeed

!----------------------------------------------------------------------------------------------------------------------------------
!> Routine for setting the inputs for KiteAeroDyn for a given time step, t.  The inputs are based on the Motions time history data
!>   provided by the Driver input file, as well as the wind speed model.  Since this is a rigid-body simulation, the input values at
!>   the various kite nodes is determined by mapping the Fuselage reference point data onto the other meshes/nodes
subroutine Set_u( t, numTimes, numPylons, numFlaps, lastInd, Motions, HWindSpd, HWindDir, RefHt, PLexp, MeshMapData, u, p, z, errStat, errMsg )
   real(DbKi),                    intent(in   )  :: t              !< Current simulation time in seconds
   integer(IntKi),                intent(in   )  :: numTimes       !< number of time steps in the simulation
   integer(IntKi),                intent(in   )  :: numPylons      !< Number of pylons per wing
   integer(IntKi),                intent(in   )  :: numFlaps       !< Number of flaps per wing
   integer(IntKi),                intent(inout)  :: lastInd        !< Most-recently used index for interpolation routines
   type(KAD_Dvr_Motions),         intent(in   )  :: Motions        !< Driver-specified motions and controls
   real(ReKi),                    intent(in   )  :: HWindSpd       !< Horizontal wind speed (m/s) [>=0.0]
   real(ReKi),                    intent(in   )  :: HWindDir       !< Horizontal wind propagation direction (meteoroligical rotation from aligned with X (positive rotates towards -Y)) (deg)
   real(ReKi),                    intent(in   )  :: RefHt          !< Reference height for horizontal wind speed (m) [>0.0]
   real(ReKi),                    intent(in   )  :: PLexp          !< Vertical wind shear power-law exponent (-) [>=0.0]
   type(KAD_Dvr_MeshMapData),     intent(inout)  :: MeshMapData    !< Mesh mapping data
   type(KAD_InputType),           intent(inout)  :: u              !< Inputs at Time
   type(KAD_ParameterType),       intent(in   )  :: p              !< Parameters
   type(KAD_ConstraintStateType), intent(in   )  :: z              !< Constraint states at Time
   integer(IntKi),                intent(  out)  :: errStat        !< Status of error message
   character(1024),               intent(  out)  :: errMsg         !< Error message if errStat /= ErrID_None
 
   CHARACTER(1024)   :: dvrFilename     ! Filename and path for the driver input file.  This is passed in as a command line argument when running the Driver exe. 
   character(1024)   :: routineName   
   integer(IntKi)    :: errStat2        ! Status of error message
   character(1024)   :: errMsg2         ! Error message if errStat /= ErrID_None
   integer(IntKi)    :: i,j             ! loop counter
   integer(IntKi)    :: index           ! index into rotor meshes
   real(ReKi)        :: theta(3)        ! Euler angles (radians)
   real(ReKi)        :: ptZ
   
   errStat  = ErrID_None
   errMsg   = ""
   
      ! Relative translational displacement of the fuselage reference point   
   u%FusOMotions%TranslationDisp(1,1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%KitePXi(:), lastInd, numTimes )
   u%FusOMotions%TranslationDisp(2,1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%KitePYi(:), lastInd, numTimes )
   u%FusOMotions%TranslationDisp(3,1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%KitePZi(:), lastInd, numTimes )
 
      ! Fuselage displaced orientation of fuselage reference point
   theta(1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%Roll(:) , lastInd, numTimes )
   theta(2) = InterpStp ( real(t, ReKi), Motions%Times, Motions%Pitch(:), lastInd, numTimes )
   theta(3) = InterpStp ( real(t, ReKi), Motions%Times, Motions%Yaw(:)  , lastInd, numTimes ) 
   u%FusOMotions%Orientation(:,:,1) = EulerConstruct(theta)
   
   u%FusOMotions%TranslationVel(1,1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%KiteTVXi(:) , lastInd, numTimes )
   u%FusOMotions%TranslationVel(2,1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%KiteTVYi(:) , lastInd, numTimes )
   u%FusOMotions%TranslationVel(3,1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%KiteTVZi(:) , lastInd, numTimes )
   
   u%FusOMotions%RotationVel(1,1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%KiteRVXi(:) , lastInd, numTimes )
   u%FusOMotions%RotationVel(2,1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%KiteRVYi(:) , lastInd, numTimes )
   u%FusOMotions%RotationVel(3,1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%KiteRVZi(:) , lastInd, numTimes )
   
   
      ! Fuselage node locations, orientations, and translational velocities based on fuselage reference point motions
   call Transfer_Point_to_Line2( u%FusOMotions, u%FusMotions, MeshMapData%FusO_P_2_Fus_L, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: Transfer_FusO_to_Fus' )   
         
      ! Undisturbed wind velocities at fuselage nodes
   do i=1,u%FusMotions%nNodes
      ptZ = u%FusMotions%Position(3,i)+u%FusMotions%TranslationDisp(3,i)
      call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_Fus(:,i), errStat2, errMsg2 )
   end do
   
! Starboard Wing
   call Transfer_Point_to_Line2( u%FusOMotions, u%SWnMotions, MeshMapData%FusO_P_2_SWn_L, errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: Transfer_FusO_to_SWn' )           
      ! Undisturbed wind velocities
   do i=1,u%SWnMotions%nNodes
      ptZ = u%SWnMotions%Position(3,i)+u%SWnMotions%TranslationDisp(3,i)
      call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_SWn(:,i), errStat2, errMsg2 )
   end do
   
! Port Wing
   call Transfer_Point_to_Line2( u%FusOMotions, u%PWnMotions, MeshMapData%FusO_P_2_PWn_L, errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: Transfer_FusO_to_PWn' )   
      ! Undisturbed wind velocities
   do i=1,u%PWnMotions%nNodes
      ptZ = u%PWnMotions%Position(3,i)+u%PWnMotions%TranslationDisp(3,i)
      call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_PWn(:,i), errStat2, errMsg2 )
   end do
   
! Vertical Stabilizer
   call Transfer_Point_to_Line2( u%FusOMotions, u%VSPMotions, MeshMapData%FusO_P_2_VSP_L, errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: Transfer_FusO_to_VSP' )   
      ! Undisturbed wind velocities
   do i=1,u%VSPMotions%nNodes
      ptZ = u%VSPMotions%Position(3,i)+u%VSPMotions%TranslationDisp(3,i)
      call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_VSP(:,i), errStat2, errMsg2 )
   end do   
! Starboard Horizontal Stabilizer
   call Transfer_Point_to_Line2( u%FusOMotions, u%SHSMotions, MeshMapData%FusO_P_2_SHS_L, errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: Transfer_FusO_to_SHS' )   
      ! Undisturbed wind velocities
   do i=1,u%SHSMotions%nNodes
      ptZ = u%SHSMotions%Position(3,i)+u%SHSMotions%TranslationDisp(3,i)
      call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_SHS(:,i), errStat2, errMsg2 )
   end do 
   
! Port Horizontal Stabilizer
   call Transfer_Point_to_Line2( u%FusOMotions, u%PHSMotions, MeshMapData%FusO_P_2_PHS_L, errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: Transfer_FusO_to_PHS' )   
      ! Undisturbed wind velocities
   do i=1,u%PHSMotions%nNodes
      ptZ = u%PHSMotions%Position(3,i)+u%PHSMotions%TranslationDisp(3,i)
      call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_PHS(:,i), errStat2, errMsg2 )
   end do   
   
   do j = 1, numPylons
      
      call Transfer_Point_to_Line2( u%FusOMotions, u%SPyMotions(j), MeshMapData%FusO_P_2_SPy_L(j), errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: FusO_P_2_SPy_L' )   
      ! Undisturbed wind velocities
      do i=1,u%SPyMotions(j)%nNodes
         ptZ = u%SPyMotions(j)%Position(3,i)+u%SPyMotions(j)%TranslationDisp(3,i)
         call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_SPy(:,i,j), errStat2, errMsg2 )
      end do
      
      call Transfer_Point_to_Line2( u%FusOMotions, u%PPyMotions(j), MeshMapData%FusO_P_2_PPy_L(j), errStat2, errMsg2 )     
         call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: FusO_P_2_PPy_L' )   
      ! Undisturbed wind velocities
      do i=1,u%PPyMotions(j)%nNodes
         ptZ = u%PPyMotions(j)%Position(3,i)+u%PPyMotions(j)%TranslationDisp(3,i)
         call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_PPy(:,i,j), errStat2, errMsg2 )
      end do  
      
   end do
   
   do j = 0, numPylons-1
      do i = 1,2 ! Two per pylon
         
         index = i+j*numPylons
         
         ! Starboard Rotors
         call Transfer_Point_to_Point( u%FusOMotions, u%SPyRtrMotions(index), MeshMapData%FusO_P_2_SPyRtr_P(index), errStat2, errMsg2 )
            call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: FusO_P_2_SPyRtr_P' )   
            ! Undisturbed wind velocity
         ptZ = u%SPyRtrMotions(index)%Position(3,1)+u%SPyRtrMotions(index)%TranslationDisp(3,1)
         call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_SPyRtr(:,i,j+1), errStat2, errMsg2 )
      
         ! Port Rotors
         call Transfer_Point_to_Point( u%FusOMotions, u%PPyRtrMotions(index), MeshMapData%FusO_P_2_PPyRtr_P(index), errStat2, errMsg2 )     
            call SetErrStat(errStat2, errMsg2, errStat, ErrMsg,' Set_u: FusO_P_2_PPyRtr_P' )   
            ! Undisturbed wind velocities 
         ptZ = u%PPyRtrMotions(index)%Position(3,1)+u%PPyRtrMotions(index)%TranslationDisp(3,1)
         call GetWindSpeed( HWindSpd, ptZ, RefHt, PLexp, HWindDir, u%V_PPyRtr(:,i,j+1), errStat2, errMsg2 )
         
         u%Omega_SPyRtr(i,j+1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%SRtSpd  (:,i,j+1), lastInd, numTimes )  
         u%Omega_PPyRtr(i,j+1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%PRtSpd  (:,i,j+1), lastInd, numTimes ) 
         u%Pitch_SPyRtr(i,j+1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%SRtPitch(:,i,j+1), lastInd, numTimes )  
         u%Pitch_PPyRtr(i,j+1) = InterpStp ( real(t, ReKi), Motions%Times, Motions%PRtPitch(:,i,j+1), lastInd, numTimes ) 
      end do
   end do

      ! Flaps control settings
   do i=1, numFlaps
      u%Ctrl_SFlp(i) = InterpStp ( real(t, ReKi), Motions%Times, Motions%FlpCtrl(:,i), lastInd, numTimes )      
   end do
   do i=1, numFlaps
      u%Ctrl_PFlp(i) = InterpStp ( real(t, ReKi), Motions%Times, Motions%FlpCtrl(:,i+numFlaps), lastInd, numTimes )
   end do
      ! Rudder control settings
   do i=1,2
      u%Ctrl_Rudr(i) = InterpStp ( real(t, ReKi), Motions%Times, Motions%RudrCtrl(:,i), lastInd, numTimes )  
   end do
      ! Elevator control settings, starboard wing, then port wing
   do i=1,2
      u%Ctrl_SElv(i) = InterpStp ( real(t, ReKi), Motions%Times, Motions%ElvCtrl(:,i), lastInd, numTimes )  
   end do
   do i=1,2
      u%Ctrl_PElv(i) = InterpStp ( real(t, ReKi), Motions%Times, Motions%ElvCtrl(:,i+2), lastInd, numTimes )  
   end do
   
end subroutine Set_u

!----------------------------------------------------------------------------------------------------------------------------------
!> Routine to parse the driver file time history data associated with the kite motions, rotors, and controls
subroutine ReadMotions( UnIn, fileName, NumTimes, NumFlaps, NumPylons, Motions, errStat, errMsg, UnEc )
   integer(IntKi),               intent(in   )  :: UnIn              !< File unit number
   character(*),                 intent(in   )  :: filename          !< Name of the driver input file
   integer(IntKi),               intent(in   )  :: NumTimes          !< Number of time steps of motion data in the driver file
   integer(IntKi),               intent(in   )  :: NumFlaps          !< Number of flaps
   integer(IntKi),               intent(in   )  :: NumPylons         !< Number of pylons
   type(KAD_Dvr_Motions),        intent(inout)  :: Motions           !< The motions time series data
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None
   integer(IntKi),               intent(in   )  :: UnEc              !< File unit for the echo file ( 0 if not echoing the input file )
   
   integer(intKi)                               :: i,k,c, NumVals    ! counters
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'ReadMotions'
   real(ReKi), allocatable                      :: vals(:)           ! A line of motion history data
   
      ! Initialize variables for this routine
   errStat = ErrID_None
   errMsg  = ""
   
      ! Allocate the various Motions arrays
   call AllocAry( Motions%Times, NumTimes, 'Motions%Times', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%KitePXi, NumTimes, 'Motions%KitePXi', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%KitePYi, NumTimes, 'Motions%KitePYi', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%KitePZi, NumTimes, 'Motions%KitePZi', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%Roll, NumTimes, 'Motions%Roll', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%Pitch, NumTimes, 'Motions%Pitch', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%Yaw, NumTimes, 'Motions%Yaw', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%KiteTVXi, NumTimes, 'Motions%KiteTVXi', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%KiteTVYi, NumTimes, 'Motions%KiteTVYi', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%KiteTVZi, NumTimes, 'Motions%KiteTVZi', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%KiteRVXi, NumTimes, 'Motions%KiteRVXi', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%KiteRVYi, NumTimes, 'Motions%KiteRVYi', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%KiteRVZi, NumTimes, 'Motions%KiteRVZi', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )          
   call AllocAry( Motions%SRtSpd, NumTimes, 2, NumPylons, 'Motions%SRtSpd', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%PRtSpd, NumTimes, 2, NumPylons, 'Motions%PRtSpd', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%SRtPitch, NumTimes, 2, NumPylons, 'Motions%SRtPitch', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%PRtPitch, NumTimes, 2, NumPylons, 'Motions%PRtPitch', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%FlpCtrl, NumTimes, 2*NumFlaps, 'Motions%FlpCtrl', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%RudrCtrl, NumTimes, 2, 'Motions%RudrCtrl', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Motions%ElvCtrl, NumTimes, 4, 'Motions%ElvCtrl', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
   if (errStat >= AbortErrLev) return   
      
      
      ! Allocate vals array
   numVals = 13 + NumPylons * 8 + 2*NumFlaps + 6
 
   call AllocAry( vals, numVals, 'vals', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   do i=1, NumTimes

      call ReadAry( UnIn, fileName, vals, NumVals, 'Values','Table row', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
      Motions%Times(i)      = vals(1)
      Motions%KitePXi(i)   = vals(2)
      Motions%KitePYi(i)   = vals(3)
      Motions%KitePZi(i)   = vals(4)
      Motions%Roll(i)      = vals(5)*D2R ! Roll
      Motions%Pitch(i)     = vals(6)*D2R ! Pitch
      Motions%Yaw(i)       = vals(7)*D2R ! Yaw
      Motions%KiteTVXi(i)  = vals(8)
      Motions%KiteTVYi(i)  = vals(9)
      Motions%KiteTVZi(i)  = vals(10)
      Motions%KiteRVXi(i)  = vals(11)*D2R
      Motions%KiteRVYi(i)  = vals(12)*D2R
      Motions%KiteRVZi(i)  = vals(13)*D2R
      
         ! All pylon rotor speeds, starboard wing, then port wing
      c = 1
   
      do k=1,NumPylons  ! All starboard rotors
         Motions%SRtSpd(i,1,k)   = vals(13+c  )*RPM2RPS   ! Top RtSpd 
         Motions%SRtSpd(i,2,k)   = vals(13+c+1)*RPM2RPS ! Bottom RtSpd
       
         c = c + 2 ! increment rotor count
      end do
      do k=1,NumPylons  ! All port rotors
         Motions%PRtSpd(i,1,k)   = vals(13+c  )*RPM2RPS   ! Top RtSpd 
         Motions%PRtSpd(i,2,k)   = vals(13+c+1)*RPM2RPS ! Bottom RtSpd
       
         c = c + 2 ! increment rotor count
      end do
               ! All pylon rotor pitch angles (deg), starboard wing, then port wing
      
      do k=1,NumPylons  ! All starboard rotors
         Motions%SRtPitch(i,1,k)   = vals(13+c  )*D2R   ! Top RtSpd 
         Motions%SRtPitch(i,2,k)   = vals(13+c+1)*D2R ! Bottom RtSpd
         c = c + 2 ! increment rotor count
      end do
      
      do k=1,NumPylons  ! All port rotors
         Motions%PRtPitch(i,1,k)   = vals(13+c  )*D2R   ! Top RtSpd 
         Motions%PRtPitch(i,2,k)   = vals(13+c+1)*D2R ! Bottom RtSpd
         c = c + 2 ! increment rotor count
      end do
      
         ! Flap control settings, starboard wing, then port wing
      do k=1,2*NumFlaps
         Motions%FlpCtrl(i,k)     = vals(13+c)
         c = c + 1
      end do
          ! Rudder control settings
      do k=1,2
         Motions%RudrCtrl(i,k)     = vals(13+c)
         c = c + 1
      end do
         ! Elevator control settings, starboard wing, then port wing
      do k=1,4
         Motions%ElvCtrl(i,k)     = vals(13+c)
         c = c + 1
      end do
     
   end do
      
end subroutine ReadMotions
   
!----------------------------------------------------------------------------------------------------------------------------------
!> Routine to parse the driver input file   
subroutine KAD_Dvr_Read_InputFile( inputFile, InitInp, errStat, errMsg )
   character(*),                intent(in   )  :: inputFile         !< Name of the KiteAeroDyn driver input file
   type(KAD_Dvr_InitInputType), intent(inout)  :: InitInp           !< Initialization input data for the KiteAeroDyn driver program
   integer(IntKi),              intent(  out)  :: errStat           !< Error status of the operation
   character(*),                intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None
     
      ! Local variables
   integer(IntKi)            :: i,j         ! loop counter

   character(ErrMsgLen)      :: errMsg2     ! temporary Error message if errStat /= ErrID_None
   integer(IntKi)            :: errStat2    ! temporary Error status of the operation
   character(*), parameter   :: routineName = 'Read_Dvr_InputFile'
   character(1024)           :: fileName    ! The trimmed input file name
   integer(IntKi)            :: UnIn, UnEc  ! File units
   character(1024)           :: FTitle      ! "File Title": the 2nd line of the input file, which contains a description of its contents
   integer(IntKi)            :: NumPylons   ! Number of pylons
   integer(IntKi)            :: NumFlaps    ! Number of flaps
   logical                   :: Echo        ! Whether or not to echo the input file
   
         ! Initialize variables for this routine
   errStat  = ErrID_None
   errMsg   = ""
   UnEc     = -1 
   Echo = .false.   
   fileName = trim(inputFile)
   
   call GetNewUnit( UnIn )   
  
   call OpenFInpfile(UnIn, trim(fileName), errStat, errMsg)
      if ( errStat /= ErrID_None ) then
         call SetErrStat( ErrID_Fatal, 'Could not open driver input file for reading', errStat, errMsg, RoutineName )
         call CleanUp()
         return
      end if
 
   ! Read the lines up/including to the "Echo" simulation control variable
   ! If echo is FALSE, don't write these lines to the echo file. 
   ! If Echo is TRUE, rewind and write on the second try.
   
   i = 1 !set the number of times we've read the file
   do 
   !----------- HEADER -------------------------------------------------------------
   
      call ReadCom( UnIn, fileName, 'KiteAeroDyn Driver input file header (line 1)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
      call ReadStr( UnIn, fileName, FTitle, 'FTitle', 'KiteAeroDyn Driver input file header: File Description (line 2)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )        

   !-------------------------- INPUT CONFIGURATION ------------------------

      ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' INPUT CONFIGURATION ', errStat2, errMsg2, UnEc  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      
      ! Echo - Echo input to "<RootName>.ech".
   
   call ReadVar( UnIn, fileName, Echo, 'Echo',   'Echo flag', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
   
   if (.NOT. Echo .OR. i > 1) EXIT !exit this loop
   
      ! Otherwise, open the echo file, then rewind the input file and echo everything we've read
      
   i = i + 1         ! make sure we do this only once (increment counter that says how many times we've read this file)
   
   call OpenEcho ( UnEc, trim(filename)//'.ech', errStat2, errMsg2, KAD_Dvr_Ver )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if ( errStat >= AbortErrLev ) then
         call Cleanup()
         return
      end if
   
   if ( UnEc > 0 )  WRITE (UnEc,'(/,A,/)')  'Data from '//trim(KAD_Dvr_Ver%Name)//' primary input file "'//trim( fileName )//'":'
         
   rewind( UnIn, IOSTAT=errStat2 )  
      if (errStat2 /= 0_IntKi ) then
         call SetErrStat( ErrID_Fatal, 'Error rewinding file "'//trim(fileName)//'".', errStat, errMsg, RoutineName )
         call Cleanup()
         return
      end if           
   end do    

   if (NWTC_VerboseLevel == NWTC_Verbose) then
      call WrScr( ' Heading of the '//trim(KAD_Dvr_Ver%Name)//' input file: ' )      
      call WrScr( '   '//trim( FTitle ) )
   end if
   
   
      ! DTAero - Time interval for aerodynamic calculations (s):
   call ReadVar( UnIn, fileName, InitInp%DTAero, "DTAero", "Time interval for aerodynamic calculations (s)", errStat2, errMsg2, UnEc)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Name of the primary KiteAeroDyn input file (string)
   call ReadVar ( UnIn, fileName, InitInp%KAD_InitInp%FileName, 'KAD_InFile', 'Name of the primary KiteAeroDyn input file (string)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

  
   !-------------------------- ENERGY KITE REFERENCE CONFIGURATION ------------------------

      ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' ENERGY KITE REFERENCE CONFIGURATION ', errStat2, errMsg2, UnEc  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Number of flaps  per wing
   call ReadVar ( UnIn, fileName, NumFlaps, 'NumFlaps', 'Number of flaps  per wing', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   InitInp%KAD_InitInp%NumFlaps = NumFlaps
   
      ! Number of pylons per wing
   call ReadVar ( UnIn, fileName, NumPylons, 'NumPylons', 'Number of pylons per wing', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   InitInp%KAD_InitInp%NumPylons = NumPylons
   
   ! Even though we will validate the bulk of the input 
      ! Table header
   call ReadCom( UnIn, fileName, ' KiteX   KiteY   KiteZ ', errStat2, errMsg2, UnEc  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call ReadCom( UnIn, fileName, '  (m)     (m)     (m)  ', errStat2, errMsg2, UnEc  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
      ! Allocate Kite component origin point arrays   
   call AllocAry( InitInp%KAD_InitInp%SPyOR, 3, NumPylons, 'InitInp%SPyOR', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( InitInp%KAD_InitInp%PPyOR, 3, NumPylons, 'InitInp%PPyOR', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( InitInp%KAD_InitInp%SPyRtrOR, 3, 2, NumPylons, 'InitInp%SPyRtrOR', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( InitInp%KAD_InitInp%PPyRtrOR, 3, 2, NumPylons, 'InitInp%PPyRtrOR', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
     
   call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%SWnOR, 3, 'SWnOR', 'Origin of Starboard Wing (most inboard node) in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%PWnOR, 3, 'PWnOR', 'Origin of Port Wing (most inboard node) in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%VSPOR, 3, 'VSPOR', 'Origin of Vertical Stabilizer in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%SHSOR, 3, 'SHSOR', 'Origin of Starboard Horizontal Stabilizer in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%PHSOR, 3, 'PHSOR', 'Origin of Port Horizontal Stabilizer in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   do i = 1, NumPylons
      call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%SPyOR(:,i), 3, 'SPyOR', 'Origin of Starboard Pylon '//trim(num2lstr(i))//' in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )  
   end do
  
   do i = 1, NumPylons
      call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%PPyOR(:,i), 3, 'PPyOR', 'Origin of Port Pylon '//trim(num2lstr(i))//' in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )  
   end do
  
   do j = 1, NumPylons
      call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%SPyRtrOR(:,1,j), 3, 'SPyRtrOR', 'Origin of Top rotor on starboard Pylon '//trim(num2lstr(j))//' in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )  
      call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%SPyRtrOR(:,2,j), 3, 'SPyRtrOR', 'Origin of Bottom rotor on starboard Pylon '//trim(num2lstr(j))//' in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )  
   end do
  
   do j = 1, NumPylons
      call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%PPyRtrOR(:,1,j), 3, 'PPyRtrOR', 'Origin of Top rotor on port Pylon '//trim(num2lstr(j))//' in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )  
      call ReadAry ( UnIn, fileName, InitInp%KAD_InitInp%PPyRtrOR(:,2,j), 3, 'PPyRtrOR', 'Origin of Bottom rotor on port Pylon '//trim(num2lstr(j))//' in body-fixed coordinate system (m)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )  
   end do
  
   
   !-------------------------- I/O SETTINGS ------------------------

      ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' I/O SETTINGS ', errStat2, errMsg2, UnEc  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      ! Root name for any output files (sring) [use "" for .dvr rootname]
   call ReadVar ( UnIn, fileName, InitInp%OutFileRoot, 'OutFileRoot', 'Root name for any output files (sring) [use "" for .dvr rootname]', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Make output tab-delimited? (fixed-width otherwise) (flag)
   call ReadVar ( UnIn, fileName, InitInp%TabDel, 'TabDel', 'Make output tab-delimited? (fixed-width otherwise) (flag)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Format used for text tabular output, excluding the time channel.  Resulting field should be 10 characters. (quoted string)
   call ReadVar ( UnIn, fileName, InitInp%OutFmt, 'OutFmt', 'Format used for text tabular output, excluding the time channel.  Resulting field should be 10 characters. (quoted string)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Beep on exit? (flag)
   call ReadVar ( UnIn, fileName, InitInp%Beep, 'Beep', 'Beep on exit? (flag)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      
   !-------------------------- WIND AND ENERGY KITE TIME-HISTORY MOTION ------------------------

      ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' WIND AND ENERGY KITE TIME-HISTORY MOTION ', errStat2, errMsg2, UnEc  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Horizontal wind speed (m/s) [>=0.0]
   call ReadVar ( UnIn, fileName, InitInp%HWindSpd, 'HWindSpd', 'Horizontal wind speed (m/s) [>=0.0]', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Horizontal wind propagation direction (meteoroligical rotation from aligned with X (positive rotates towards -Y)) (deg)
   call ReadVar ( UnIn, fileName, InitInp%HWindDir, 'HWindDir', 'Horizontal wind propagation direction (meteoroligical rotation from aligned with X (positive rotates towards -Y)) (deg)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   InitInp%HWindDir = InitInp%HWindDir*D2R  ! convert to radians
   
      ! Reference height for horizontal wind speed (m) [>0.0]
   call ReadVar ( UnIn, fileName, InitInp%RefHt, 'RefHt', 'Reference height for horizontal wind speed (m) [>0.0]', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Vertical wind shear power-law exponent (-) [>=0.0]
   call ReadVar ( UnIn, fileName, InitInp%PLexp, 'PLexp', 'Vertical wind shear power-law exponent (-) [>=0.0]', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Number of time stamps (-) [>=2]
   call ReadVar ( UnIn, fileName, InitInp%NumTimes, 'NumTimes', 'Number of time stamps (-) [>=2]', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
         ! Skip the table header line2.
   call ReadCom( UnIn, fileName, ' Motions table header line 1 ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call ReadCom( UnIn, fileName, ' Motions table header line 2 ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   call ReadMotions( UnIn, fileName, InitInp%NumTimes, NumFlaps, NumPylons, InitInp%Motions, errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call Cleanup()
   
   contains
   
   !====================================================================================================
   subroutine Cleanup()
   !     The routine cleans up the module driver input file and echo file 
   !----------------------------------------------------------------------------------------------------  
      
      close(UnIn)
      close(UnEc)
      
   end subroutine Cleanup
   
end subroutine KAD_Dvr_Read_InputFile
   
!----------------------------------------------------------------------------------------------------------------------------------
!> Routine to validate the driver input file data   
subroutine KAD_Dvr_ValidateInitData( dvrInitInp, errStat, errMsg )
   type(KAD_Dvr_InitInputType), intent(inout) :: dvrInitInp  !< Driver initialization input data 
   integer(IntKi),              intent(  out) :: errStat     !< Status of error message
   character(1024),             intent(  out) :: errMsg      !< Error message if errStat /= ErrID_None
 
      ! Local variables
   integer(IntKi)            :: i,j         ! loop counter
   real(DbKi)                :: maxTime     ! Maximum time value in the motions time series data
   character(*), parameter   :: routineName = 'KAD_Dvr_ValidateInitData'

         ! Initialize variables for this routine
   errStat  = ErrID_None
   errMsg   = ""

   if ( dvrInitInp%DTAero   <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'Simulation timestep must be greater than zero', errStat, errMsg, routineName )
   if ( dvrInitInp%HWindSpd <  0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'The horizontal reference wind speed must be greater than or equal to zero', errStat, errMsg, routineName )
   if ( dvrInitInp%RefHt    <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'The height for the reference wind speed must be greater than zero', errStat, errMsg, routineName )
   if ( dvrInitInp%PLexp    <  0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'The vertical wind shear power-law exponent must be greater than or equal to zero', errStat, errMsg, routineName )
   if ( dvrInitInp%NumTimes <  2        ) call SetErrStat( ErrID_Fatal, 'The number of kite time series time steps must be greater than 1', errStat, errMsg, routineName )
   if ( ( dvrInitInp%HWindDir  < -PI ) .or.  ( dvrInitInp%HWindDir  > PI ) ) call SetErrStat( ErrID_Fatal, 'The reference wind direction must be between [-180,180]', errStat, errMsg, routineName )
 
   ! Validate time series data
   if ( .not. EqualRealNos(dvrInitInp%Motions%Times(1), 0.0_ReKi) ) call SetErrStat( ErrID_Fatal, 'The first time stamp in the time-series data must be 0.0', errStat, errMsg, routineName )
   
   maxTime = -1.0_ReKi
   
   do i = 1, dvrInitInp%NumTimes
      
         ! Make sure time stamps are monotonic-increasing
      if ( dvrInitInp%Motions%Times(i) <= maxTime ) then
         call SetErrStat( ErrID_Fatal, 'The time stamps in the time-series data must be monotonic and increasing.  Row '//trim(num2lstr(i))//' does not meet this criteria', errStat, errMsg, routineName )
         return
      end if
      maxTime = dvrInitInp%Motions%Times(i)
      
         ! Verify Kite motion angles are in the range [-180,180]
      if ( ( dvrInitInp%Motions%Roll(i)  < -PI ) .or.  ( dvrInitInp%Motions%Roll(i)  > PI ) ) then
         call SetErrStat( ErrID_Fatal, 'The Roll angle of time series row '//trim(num2lstr(i))//' must be between [-180,180]', errStat, errMsg, routineName )
         return
      end if
      if ( ( dvrInitInp%Motions%Pitch(i)  < -PI ) .or.  ( dvrInitInp%Motions%Pitch(i)  > PI ) ) then
         call SetErrStat( ErrID_Fatal, 'The Pitch angle of time series row '//trim(num2lstr(i))//' must be between [-180,180]', errStat, errMsg, routineName )
         return
      end if
      if ( ( dvrInitInp%Motions%Yaw(i)  < -PI ) .or.  ( dvrInitInp%Motions%Yaw(i)  > PI ) ) then
         call SetErrStat( ErrID_Fatal, 'The Yaw angle of time series row '//trim(num2lstr(i))//' must be between [-180,180]', errStat, errMsg, routineName )
         return
      end if
      
         ! Verify Rotor pitch angles are in the range [-180,180] 
      do j = 1, dvrInitInp%KAD_InitInp%NumPylons

         if ( ( dvrInitInp%Motions%SRtPitch(i,1,j)  < -PI ) .or.  ( dvrInitInp%Motions%SRtPitch(i,1,j)  > PI ) ) then
            call SetErrStat( ErrID_Fatal, 'The starboard, pylon #'//trim(num2lstr(j))//', top rotor, has an invalid blade pitch angle on time series row '//trim(num2lstr(i))//' must be between [-180,180]', errStat, errMsg, routineName )
            return
         end if
         if ( ( dvrInitInp%Motions%SRtPitch(i,2,j)  < -PI ) .or.  ( dvrInitInp%Motions%SRtPitch(i,2,j)  > PI ) ) then
            call SetErrStat( ErrID_Fatal, 'The starboard, pylon #'//trim(num2lstr(j))//', bottom rotor, has an invalid blade pitch angle on time series row '//trim(num2lstr(i))//' must be between [-180,180]', errStat, errMsg, routineName )
            return
         end if
         if ( ( dvrInitInp%Motions%PRtPitch(i,1,j)  < -PI ) .or.  ( dvrInitInp%Motions%PRtPitch(i,1,j)  > PI ) ) then
            call SetErrStat( ErrID_Fatal, 'The port, pylon #'//trim(num2lstr(j))//', top rotor, has an invalid blade pitch angle on time series row '//trim(num2lstr(i))//' must be between [-180,180]', errStat, errMsg, routineName )
            return
         end if
         if ( ( dvrInitInp%Motions%PRtPitch(i,2,j)  < -PI ) .or.  ( dvrInitInp%Motions%PRtPitch(i,2,j)  > PI ) ) then
            call SetErrStat( ErrID_Fatal, 'The port, pylon #'//trim(num2lstr(j))//', bottom rotor, has an invalid blade pitch angle on time series row '//trim(num2lstr(i))//' must be between [-180,180]', errStat, errMsg, routineName )
            return
         end if
      end do
      
   end do
   
    
     

end subroutine KAD_Dvr_ValidateInitData

!----------------------------------------------------------------------------------------------------------------------------------
!> Routine to perform the stand-alone KiteAeroDyn simulation   
subroutine KAD_Dvr_Simulate( dvrInitInp, errStat, errMsg )

   type(KAD_Dvr_InitInputType), intent(inout) :: dvrInitInp  !< Driver initialization input data 
   integer(IntKi),              intent(  out) :: errStat     !< Status of error message
   character(1024),             intent(  out) :: errMsg      !< Error message if errStat /= ErrID_None
 
   CHARACTER(1024)                  :: dvrFilename     ! Filename and path for the driver input file.  This is passed in as a command line argument when running the Driver exe. 
   character(1024)                  :: routineName
   
   integer(IntKi)                   :: errStat2        ! Status of error message
   character(1024)                  :: errMsg2         ! Error message if errStat /= ErrID_None
   type(KAD_InitOutputType)         :: KAD_InitOut     ! Initialization output data
   type(KAD_InputType)              :: Inputs(2)       ! Inputs at InputTimes
   real(DbKi)                       :: InputTimes(2)   ! Input Times for updating the states 
   type(KAD_ParameterType)          :: KAD_p           ! Parameters
   type(KAD_ContinuousStateType)    :: KAD_x           ! Continuous states at Time
   type(KAD_DiscreteStateType)      :: KAD_xd          ! Discrete states at Time
   type(KAD_ConstraintStateType)    :: KAD_z           ! Constraint states at Time
   type(KAD_OtherStateType)         :: KAD_OtherState  ! Other states at Time
   type(KAD_OutputType)             :: KAD_y           ! Outputs computed at Time (Input only so that mesh connectivity information does not have to be recalculated)
   type(KAD_MiscVarType)            :: KAD_m           ! Initial misc/optimization variables           
   integer(IntKi)                   :: n               ! Time step counter
   integer(IntKi)                   :: i               ! loop counter
   type(KAD_Dvr_MeshMapData)        :: MeshMapData     ! Mesh mapping data
   integer(IntKi)                   :: lastInd         ! Most-recently used index for interpolation routines
   integer(IntKi)                   :: numPylons       ! Number of pylons
   integer(IntKi)                   :: numFlaps        ! Number of flaps
   real(DbKi)                       :: KAD_DT          ! KiteAeroDyn module DT
   integer(IntKi)                   :: lastTimeInc     ! Last time step increment for the time-marching loop
   
      ! Initialize variables for this routine
   errStat  = ErrID_None
   errMsg   = ""
   routineName = 'KAD_Dvr_Simulate'
   
   numPylons = dvrInitInp%KAD_InitInp%NumPylons
   numFlaps  = dvrInitInp%KAD_InitInp%NumFlaps
   
      ! Set the driver suggested time step, but KiteAeroDyn may change this in KAD_Init()
   KAD_DT    = dvrInitInp%DTAero
   
   ! Initialize KiteAeroDyn and all sub-modules   
   call KAD_Init( dvrInitInp%KAD_InitInp, Inputs(1), KAD_p, KAD_y, KAD_DT,  KAD_m, KAD_InitOut, errStat2, errMsg2 )
      if ( errStat2 >= AbortErrLev ) then
            write(*,*) trim(errMsg2)
            return
         end if
   
   ! Create mesh mappings
   call KAD_Dvr_CreateMeshMappings(numPylons, Inputs(1), MeshMapData, errStat2, errMsg2)
      if ( errStat2 >= AbortErrLev ) then
            write(*,*) trim(errMsg2)
            return
         end if
   
      ! Copy inputs to input array
   call KAD_CopyInput( Inputs(1), Inputs(2), MESH_NEWCOPY, errStat2, errMsg2 )   
      if ( errStat2 >= AbortErrLev ) then
            write(*,*) trim(errMsg2)
            return
         end if
   
   n = 0
   InputTimes(1) = dvrInitInp%Motions%Times(1)
   lastInd = 1
      
      ! Set inputs first time step
   call Set_u( InputTimes(1), dvrInitInp%NumTimes, numPylons, numFlaps, lastInd, dvrInitInp%Motions,  dvrInitInp%HWindSpd, &
                 dvrInitInp%HWindDir, dvrInitInp%RefHt, dvrInitInp%PLexp, MeshMapData, Inputs(1), KAD_p, KAD_z,  errStat2, errMsg2 ) 
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName)  
      if ( errStat >= AbortErrLev ) then
            write(*,*) trim(errMsg)
            return
         end if
      
      ! Calculate outputs at first time step
   call KAD_CalcOutput( InputTimes(1), Inputs(1), KAD_p, KAD_x, KAD_xd, KAD_z, KAD_OtherState, KAD_y, KAD_m, errStat2, errMsg2 )   
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName)
      if ( errStat >= AbortErrLev ) then
            call WrScr(trim(errMsg))
            return
         end if
      
   ! Write output to file
   write(*,*) ' Finished KAD_CalcOutput for time = '//trim(num2lstr(InputTimes(1)))
   
   lastTimeInc =  ( dvrInitInp%Motions%Times(dvrInitInp%NumTimes) - dvrInitInp%Motions%Times(1) ) / KAD_DT
   ! Time marching loop
   do i = 1, lastTimeInc
               
         ! Generate the inputs for t+dt
      InputTimes(2) = InputTimes(1) + KAD_DT
      call Set_u( InputTimes(2), dvrInitInp%NumTimes, numPylons, numFlaps, lastInd, dvrInitInp%Motions,  dvrInitInp%HWindSpd, &
                 dvrInitInp%HWindDir, dvrInitInp%RefHt, dvrInitInp%PLexp, MeshMapData, Inputs(2), KAD_p, KAD_z,  errStat2, errMsg2 ) 
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName)    
         if ( errStat >= AbortErrLev ) then
            write(*,*) trim(errMsg)
            return
         end if
         ! compute states for time, n+1, using extrapolated inputs at n+1, parameters, and the states at n
      call KAD_UpdateStates( InputTimes(1), n, Inputs, InputTimes, KAD_p, KAD_x, KAD_xd, KAD_z, KAD_OtherState, KAD_m, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName) 
         if ( errStat >= AbortErrLev ) then
            write(*,*) trim(errMsg)
            return
         end if
         
         ! Increment time      
      n = n + 1
      InputTimes(1) = InputTimes(2)
      call KAD_CopyInput( Inputs(2), Inputs(1), MESH_UPDATECOPY, ErrStat2, ErrMsg2 )   
         call SetErrStat(ErrStat2,ErrMsg2,ErrStat,ErrMsg,RoutineName)
         if ( errStat >= AbortErrLev ) then
            write(*,*) trim(errMsg)
            return
         end if
      
         ! Calculate outputs at time t
      call KAD_CalcOutput( InputTimes(1), Inputs(1), KAD_p, KAD_x, KAD_xd, KAD_z, KAD_OtherState, KAD_y, KAD_m, errStat2, errMsg2 )   
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName)
         if ( errStat >= AbortErrLev ) then
            write(*,*) trim(errMsg)
            return
         end if
         
         ! Write output to file
      write(*,*) ' Finished KAD_CalcOutput for time = '//trim(num2lstr(InputTimes(1)))

   end do

end subroutine KAD_Dvr_Simulate

end module KAD_Dvr_Subs