!**********************************************************************************************************************************
! VSM_Test: This code tests the VSM module
!..................................................................................................................................
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

module VSM_Test
   
   use VSM_Types
   use VSM
   use NWTC_Library
   
   
   implicit none

   
contains
   
   subroutine VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
      type(VSM_InitInputType), intent(inout) :: InitInData      ! Input data for initialization
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_SetBaseInitInpData'
         ! Initialize error handling variables
      errMsg  = ''
      errStat = ErrID_None
      
      InitInData%AirDens    = 1.225_ReKi
      InitInData%KinVisc    = 1.464E-05   
      InitInData%VSMToler   = 0.00001
      InitInData%VSMMaxIter = 250
      InitInData%VSMPerturb = 0.001
      InitInData%AFTabMod   = 3
      InitInData%InCol_Alfa = 1
      InitInData%InCol_Cl   = 2
      InitInData%InCol_Cd   = 3
      InitInData%InCol_Cm   = 4
    
   end subroutine VSMTest_SetBaseInitInpData
   
        
subroutine VSMTest_FlatPlate_8deg(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs   
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_FlatPlate_8deg'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n
      real(ReKi)                                :: alpha, inc, span, Uinf, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      
      
      nSteps   = 1
      interval = 1.0_DbKi 
      
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_FlatPlate_8deg'
      write(*,*) NewLine//"Running test VSMTest_FlatPlate_8deg"
      
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
       
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 20
      InitInData%NumVolElem = 0
      
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%elemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%elemLens', errStat, errMsg, routineName ) 
         return
      end if
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\FlatPlate.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 2.7_ReKi
      
      span = 20.0 ! meters
      inc = span / InitInData%NumElem
      PtA = 0.0_ReKi
      PtB = 0.0_ReKi
      do i = 1, InitInData%NumElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtB(2) = PtA(2) + inc
         InitInData%elemLens(i) = TwoNorm(PtB - PtA )
      end do
      
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )


         if ( errStat >= AbortErrLev ) then
            call Cleanup() 
            stop
         end if
      


         ! Set Inputs
      
      alpha = 8.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,:) = 0.0_ReKi
      u%U_Inf_v(1,:) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,:) = Uinf*sin(alpha)  ! m/s in the global Z direction   
      u%x_hat        = 0.0_ReKi
      u%y_hat        = 0.0_ReKi
      u%z_hat        = 0.0_ReKi
      u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
      u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
      u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
      
      u%deltaf = 0.0_ReKi
      z%Gammas = 0.6
     
      u%PtA = 0.0_ReKi
      u%PtB = 0.0_ReKi
      do i = 1, InitInData%NumElem
         u%PtA(2,i) = (i-1)*inc - span / 2.0
         u%PtB(2,i) = u%PtA(2,i) + inc
      end do
      
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
           
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*) "Ending test VSMTest_FlatPlate_8deg"
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) write(*,*)  ErrMsg       
      
         end subroutine Cleanup
   end subroutine VSMTest_FlatPlate_8deg
subroutine VSMTest_NACA_12deg(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs 
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_NACA_12deg'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n
      real(ReKi)                                :: alpha, inc, span, Uinf, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_NACA_12deg'
      write(*,*) NewLine//"Running test VSMTest_NACA_12deg"
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
        
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 20
      InitInData%NumVolElem = 0
      
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%elemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%elemLens', errStat, errMsg, routineName ) 
         return
      end if
      
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\NACA1410FW.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 2.7_ReKi
      
      span = 20.0 ! meters
      inc = span / InitInData%NumElem
      PtA = 0.0_ReKi
      PtB = 0.0_ReKi
      do i = 1, InitInData%NumElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtB(2) = PtA(2) + inc
         InitInData%elemLens(i) = TwoNorm(PtB - PtA )
      end do
      
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
      
      alpha = 12.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,:) = 0.0_ReKi
      u%U_Inf_v(1,:) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,:) = Uinf*sin(alpha)  ! m/s in the global Z direction   
      u%x_hat        = 0.0_ReKi
      u%y_hat        = 0.0_ReKi
      u%z_hat        = 0.0_ReKi
      u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
      u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
      u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
      u%PtA = 0.0_ReKi
      u%PtB = 0.0_ReKi
      u%deltaf = 0.0_ReKi
      z%Gammas = 0.6
     
      do i = 1, p%NumElem
         u%PtA(2,i) = (i-1)*inc - span / 2.0
         u%PtB(2,i) = u%PtA(2,i) + inc
      end do
      
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*) "Ending test VSMTest_NACA_12deg"
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
end subroutine VSMTest_NACA_12deg
      
subroutine VSMTest_NACA_0deg(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs   
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_NACA_0deg'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n
      real(ReKi)                                :: alpha, inc, span, Uinf, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_NACA_0deg'
      write(*,*) NewLine//"Running test VSMTest_NACA_0deg"
      
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
        
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 20
      InitInData%NumVolElem = 0
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%elemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%elemLens', errStat, errMsg, routineName ) 
         return
      end if
      
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\NACA1410FW.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 2.7_ReKi
      
      span = 20.0 ! meters
      inc = span / InitInData%NumElem
      PtA = 0.0_ReKi
      PtB = 0.0_ReKi
      do i = 1, InitInData%NumElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtB(2) = PtA(2) + inc
         InitInData%elemLens(i) = TwoNorm(PtB - PtA )
      end do
      
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
      
      alpha = 0.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,:) = 0.0_ReKi
      u%U_Inf_v(1,:) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,:) = Uinf*sin(alpha)  ! m/s in the global Z direction   
      u%x_hat        = 0.0_ReKi
      u%y_hat        = 0.0_ReKi
      u%z_hat        = 0.0_ReKi
      u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
      u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
      u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
      u%PtA = 0.0_ReKi
      u%PtB = 0.0_ReKi
      u%deltaf = 0.0_ReKi
      z%Gammas = 0.6
     
      do i = 1, p%NumElem
         u%PtA(2,i) = (i-1)*inc - span / 2.0
         u%PtB(2,i) = u%PtA(2,i) + inc
      end do
      
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*) "Ending test VSMTest_NACA_0deg"
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
   end subroutine VSMTest_NACA_0deg
      
   subroutine VSMTest_NACA_KiteWing(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs 
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_NACA_KiteWing'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n
      real(ReKi)                                :: alpha, inc, span, Uinf, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_NACA_KiteWing'
      write(*,*) NewLine//"Running test VSMTest_NACA_KiteWing"
      
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
        
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 14
      InitInData%NumVolElem = 0
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%elemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%elemLens', errStat, errMsg, routineName ) 
         return
      end if
      
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\NACA1410FW.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 1.0_ReKi
      
      span = 20.0 ! meters
      inc = span / InitInData%NumElem

      do i = 1, InitInData%NumElem
         InitInData%elemLens(i) = 1.0_ReKi
      end do
      
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
      
      alpha = 0.0*Pi/180.
      u%deltaf = 0.0_ReKi
      z%Gammas = 0.0
     
      u%PtA(1,1)= 0.0000000E+00
      u%PtA(2,1)= 0.0000000E+00
      u%PtA(3,1)= 100.0000
      u%PtA(1,2)= 0.0000000E+00
      u%PtA(2,2)= 1.000000
      u%PtA(3,2)= 100.0000
      u%PtA(1,3)= 0.0000000E+00
      u%PtA(2,3)= 2.000000
      u%PtA(3,3)= 100.0000
      u%PtA(1,4)= 0.0000000E+00
      u%PtA(2,4)= 3.000000
      u%PtA(3,4)= 100.0000
      u%PtA(1,5)= 0.0000000E+00
      u%PtA(2,5)= 4.000000
      u%PtA(3,5)= 100.0000
      u%PtA(1,6)= 0.0000000E+00
      u%PtA(2,6)= 5.000000
      u%PtA(3,6)= 100.0000
      u%PtA(1,7)= 0.0000000E+00
      u%PtA(2,7)= 6.000000
      u%PtA(3,7)= 100.0000
      
      u%PtA(1,8)= 0.0000000E+00
      u%PtA(2,8)= 0.0000000E+00
      u%PtA(3,8)= 100.0000
      u%PtA(1,9)= 0.0000000E+00
      u%PtA(2,9)= -1.000000
      u%PtA(3,9)= 100.0000
      u%PtA(1,10)= 0.0000000E+00
      u%PtA(2,10)= -2.000000
      u%PtA(3,10)= 100.0000
      u%PtA(1,11)= 0.0000000E+00
      u%PtA(2,11)= -3.000000
      u%PtA(3,11)= 100.0000
      u%PtA(1,12)= 0.0000000E+00
      u%PtA(2,12)= -4.000000
      u%PtA(3,12)= 100.0000
      u%PtA(1,13)= 0.0000000E+00
      u%PtA(2,13)= -5.000000
      u%PtA(3,13)= 100.0000
      u%PtA(1,14)= 0.0000000E+00
      u%PtA(2,14)= -6.000000
      u%PtA(3,14)= 100.0000
    
      u%PtB(1,1)= 0.0000000E+00
      u%PtB(2,1)= 1.000000
      u%PtB(3,1)= 100.0000
      u%PtB(1,2)= 0.0000000E+00
      u%PtB(2,2)= 2.000000
      u%PtB(3,2)= 100.0000
      u%PtB(1,3)= 0.0000000E+00
      u%PtB(2,3)= 3.000000
      u%PtB(3,3)= 100.0000
      u%PtB(1,4)= 0.0000000E+00
      u%PtB(2,4)= 4.000000
      u%PtB(3,4)= 100.0000
      u%PtB(1,5)= 0.0000000E+00
      u%PtB(2,5)= 5.000000
      u%PtB(3,5)= 100.0000
      u%PtB(1,6)= 0.0000000E+00
      u%PtB(2,6)= 6.000000
      u%PtB(3,6)= 100.0000
      u%PtB(1,7)= 0.0000000E+00
      u%PtB(2,7)= 7.000000
      u%PtB(3,7)= 100.0000
      
      u%PtB(1,8)= 0.0000000E+00
      u%PtB(2,8)= -1.000000
      u%PtB(3,8)= 100.0000
      u%PtB(1,9)= 0.0000000E+00
      u%PtB(2,9)= -2.000000
      u%PtB(3,9)= 100.0000
      u%PtB(1,10)= 0.0000000E+00
      u%PtB(2,10)= -3.000000
      u%PtB(3,10)= 100.0000
      u%PtB(1,11)= 0.0000000E+00
      u%PtB(2,11)= -4.000000
      u%PtB(3,11)= 100.0000
      u%PtB(1,12)= 0.0000000E+00
      u%PtB(2,12)= -5.000000
      u%PtB(3,12)= 100.0000
      u%PtB(1,13)= 0.0000000E+00
      u%PtB(2,13)= -6.000000
      u%PtB(3,13)= 100.0000
      u%PtB(1,14)= 0.0000000E+00
      u%PtB(2,14)= -7.000000
      u%PtB(3,14)= 100.0000    
    
      u%x_hat(1,1)= 8.7422769E-08
      u%x_hat(2,1)= 1.8023144E-15
      u%x_hat(3,1)= 1.000000
      u%x_hat(1,2)= 8.7422769E-08
      u%x_hat(2,2)= 1.8023144E-15
      u%x_hat(3,2)= 1.000000
      u%x_hat(1,3)= 8.7422769E-08
      u%x_hat(2,3)= 1.8023144E-15
      u%x_hat(3,3)= 1.000000
      u%x_hat(1,4)= 8.7422769E-08
      u%x_hat(2,4)= 1.8023144E-15
      u%x_hat(3,4)= 1.000000
      u%x_hat(1,5)= 8.7422769E-08
      u%x_hat(2,5)= 1.8023144E-15
      u%x_hat(3,5)= 1.000000
      u%x_hat(1,6)= 8.7422769E-08
      u%x_hat(2,6)= 1.8023144E-15
      u%x_hat(3,6)= 1.000000
      !u%x_hat(1,7)= -1.000000
      !u%x_hat(2,7)= 0.0000000E+00
      !u%x_hat(3,7)= 8.7422777E-08
      u%x_hat(1,7)= 0.000000
      u%x_hat(2,7)= 0.0000000E+00
      u%x_hat(3,7)= 1.0
      
      u%x_hat(1,8)= 8.7422769E-08
      u%x_hat(2,8)= 1.8023144E-15
      u%x_hat(3,8)= 1.000000
      u%x_hat(1,9)= 8.7422769E-08
      u%x_hat(2,9)= 1.8023144E-15
      u%x_hat(3,9)= 1.000000
      u%x_hat(1,10)= 8.7422769E-08
      u%x_hat(2,10)= 1.8023144E-15
      u%x_hat(3,10)= 1.000000
      u%x_hat(1,11)= 8.7422769E-08
      u%x_hat(2,11)= 1.8023144E-15
      u%x_hat(3,11)= 1.000000
      u%x_hat(1,12)= 8.7422769E-08
      u%x_hat(2,12)= 1.8023144E-15
      u%x_hat(3,12)= 1.000000
      u%x_hat(1,13)= 8.7422769E-08
      u%x_hat(2,13)= 1.8023144E-15
      u%x_hat(3,13)= 1.000000
      !u%x_hat(1,14)= -1.000000
      !u%x_hat(2,14)= 0.0000000E+00
      !u%x_hat(3,14)= 8.7422777E-08
      u%x_hat(1,14)= 0.000000
      u%x_hat(2,14)= 0.0000000E+00
      u%x_hat(3,14)= 1.0
      
    
      u%y_hat(1,1)= 1.000000
      u%y_hat(2,1)= -1.9984014E-15
      u%y_hat(3,1)= -8.7422769E-08
      u%y_hat(1,2)= 1.000000
      u%y_hat(2,2)= -1.9984014E-15
      u%y_hat(3,2)= -8.7422769E-08
      u%y_hat(1,3)= 1.000000
      u%y_hat(2,3)= -1.9984014E-15
      u%y_hat(3,3)= -8.7422769E-08
      u%y_hat(1,4)= 1.000000
      u%y_hat(2,4)= -1.9984014E-15
      u%y_hat(3,4)= -8.7422769E-08
      u%y_hat(1,5)= 1.000000
      u%y_hat(2,5)= -1.9984014E-15
      u%y_hat(3,5)= -8.7422769E-08
      u%y_hat(1,6)= 1.000000
      u%y_hat(2,6)= -1.9984014E-15
      u%y_hat(3,6)= -8.7422769E-08
      !u%y_hat(1,7)= 0.0000000E+00
      !u%y_hat(2,7)= 1.000000
      !u%y_hat(3,7)= 0.0000000E+00
      u%y_hat(1,7)= 1.0000000E+00
      u%y_hat(2,7)= 0.000000
      u%y_hat(3,7)= 0.0000000E+00
      
      u%y_hat(1,8)= 1.000000
      u%y_hat(2,8)= -1.9984014E-15
      u%y_hat(3,8)= -8.7422769E-08
      u%y_hat(1,9)= 1.000000
      u%y_hat(2,9)= -1.9984014E-15
      u%y_hat(3,9)= -8.7422769E-08
      u%y_hat(1,10)= 1.000000
      u%y_hat(2,10)= -1.9984014E-15
      u%y_hat(3,10)= -8.7422769E-08
      u%y_hat(1,11)= 1.000000
      u%y_hat(2,11)= -1.9984014E-15
      u%y_hat(3,11)= -8.7422769E-08
      u%y_hat(1,12)= 1.000000
      u%y_hat(2,12)= -1.9984014E-15
      u%y_hat(3,12)= -8.7422769E-08
      u%y_hat(1,13)= 1.000000
      u%y_hat(2,13)= -1.9984014E-15
      u%y_hat(3,13)= -8.7422769E-08
      !u%y_hat(1,14)= 0.0000000E+00
      !u%y_hat(2,14)= 1.000000
      !u%y_hat(3,14)= 0.0000000E+00
      u%y_hat(1,14)= 1.0000000E+00
      u%y_hat(2,14)= 0.000000
      u%y_hat(3,14)= 0.0000000E+00
      
    
      u%U_Inf_v(1,1)= 13.43938
      u%U_Inf_v(2,1)= 0.0000000E+00
      u%U_Inf_v(3,1)= 0.0000000E+00
      u%U_Inf_v(1,2)= 13.43938
      u%U_Inf_v(2,2)= 0.0000000E+00
      u%U_Inf_v(3,2)= 0.0000000E+00
      u%U_Inf_v(1,3)= 13.43938
      u%U_Inf_v(2,3)= 0.0000000E+00
      u%U_Inf_v(3,3)= 0.0000000E+00
      u%U_Inf_v(1,4)= 13.43938
      u%U_Inf_v(2,4)= 0.0000000E+00
      u%U_Inf_v(3,4)= 0.0000000E+00
      u%U_Inf_v(1,5)= 13.43938
      u%U_Inf_v(2,5)= 0.0000000E+00
      u%U_Inf_v(3,5)= 0.0000000E+00
      u%U_Inf_v(1,6)= 13.43938
      u%U_Inf_v(2,6)= 0.0000000E+00
      u%U_Inf_v(3,6)= 0.0000000E+00
      u%U_Inf_v(1,7)= 13.43938
      u%U_Inf_v(2,7)= 0.0000000E+00
      u%U_Inf_v(3,7)= 0.0000000E+00
      u%U_Inf_v(1,8)= 13.43938
      u%U_Inf_v(2,8)= 0.0000000E+00
      u%U_Inf_v(3,8)= 0.0000000E+00
      u%U_Inf_v(1,9)= 13.43938
      u%U_Inf_v(2,9)= 0.0000000E+00
      u%U_Inf_v(3,9)= 0.0000000E+00
      u%U_Inf_v(1,10)= 13.43938
      u%U_Inf_v(2,10)= 0.0000000E+00
      u%U_Inf_v(3,10)= 0.0000000E+00
      u%U_Inf_v(1,11)= 13.43938
      u%U_Inf_v(2,11)= 0.0000000E+00
      u%U_Inf_v(3,11)= 0.0000000E+00
      u%U_Inf_v(1,12)= 13.43938
      u%U_Inf_v(2,12)= 0.0000000E+00
      u%U_Inf_v(3,12)= 0.0000000E+00
      u%U_Inf_v(1,13)= 13.43938
      u%U_Inf_v(2,13)= 0.0000000E+00
      u%U_Inf_v(3,13)= 0.0000000E+00
      u%U_Inf_v(1,14)= 13.43938
      u%U_Inf_v(2,14)= 0.0000000E+00
      u%U_Inf_v(3,14)= 0.0000000E+00    
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*) "Ending test VSMTest_NACA_KiteWing"
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
end subroutine VSMTest_NACA_KiteWing
subroutine VSMTest_NACA_8deg_10deg_sweep(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs   
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_NACA_8deg_10deg_sweep'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n
      real(ReKi)                                :: alpha, inc, span, Uinf, sweep, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_NACA_8deg_10deg_sweep'
      write(*,*) NewLine//"Running test VSMTest_NACA_8deg_10deg_sweep"
      
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
        
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 20
      InitInData%NumVolElem = 0
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%elemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%elemLens', errStat, errMsg, routineName ) 
         return
      end if
      
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\NACA1410FW.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 2.7_ReKi
      
      span = 20.0 ! meters
      sweep = 10.0*D2R      !  sweep angle
      inc = span / InitInData%NumElem
      PtA = 0.0_ReKi
      PtB = 0.0_ReKi
      do i = 1, InitInData%NumElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtA(1) = abs(PtA(2)*tan(sweep))
         PtB(2) = PtA(2) + inc
         PtB(1) = abs(PtB(2)*tan(sweep))
         InitInData%elemLens(i) = TwoNorm(PtB - PtA )
      end do
     
      
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
      
      alpha = 8.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,:) = 0.0_ReKi
      u%U_Inf_v(1,:) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,:) = Uinf*sin(alpha)  ! m/s in the global Z direction   
      u%x_hat        = 0.0_ReKi
      u%y_hat        = 0.0_ReKi
      u%z_hat        = 0.0_ReKi
      u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
      u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
      u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
      u%PtA = 0.0_ReKi
      u%PtB = 0.0_ReKi
      u%deltaf = 0.0_ReKi
      z%Gammas = 0.6
     
      do i = 1, p%NumElem
         u%PtA(2,i) = (i-1)*inc - span / 2.0
         u%PtA(1,i) = abs(u%PtA(2,i)*tan(sweep))
         u%PtB(2,i) = u%PtA(2,i) + inc
         u%PtB(1,i) = abs(u%PtB(2,i)*tan(sweep))
      end do
      
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*) "Ending test VSMTest_NACA_8deg_10deg_sweep"
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
end subroutine VSMTest_NACA_8deg_10deg_sweep
subroutine VSMTest_NACA_10deg_w_flaps(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs 
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_NACA_10deg_w_flaps'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n
      real(ReKi)                                :: alpha, inc, span, Uinf, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_NACA_10deg_w_flaps'
      write(*,*) NewLine//"Running test VSMTest_NACA_10deg_w_flaps"
      
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
        
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 80
      InitInData%NumVolElem = 0
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%elemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%elemLens', errStat, errMsg, routineName ) 
         return
      end if
      
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\NACA1410FW.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 2.7_ReKi
      
      span = 20.0 ! meters
      inc = span / InitInData%NumElem
      do i = 1, p%NumElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtB(2) = PtA(2) + inc
         InitInData%elemLens(i) = TwoNorm(PtB - PtA )
      end do
      
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs

      alpha = 10.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,:) = 0.0_ReKi
      u%U_Inf_v(1,:) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,:) = Uinf*sin(alpha)  ! m/s in the global Z direction   
      u%x_hat        = 0.0_ReKi
      u%y_hat        = 0.0_ReKi
      u%z_hat        = 0.0_ReKi
      u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
      u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
      u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
      u%PtA = 0.0_ReKi
      u%PtB = 0.0_ReKi
      u%deltaf = 0.0_ReKi
      z%Gammas = 0.6
     
      do i = 1, p%NumElem
         u%PtA(2,i) = (i-1)*inc - span / 2.0
         u%PtB(2,i) = u%PtA(2,i) + inc
         if (u%PtB(2,i) <= -7.0) u%deltaf(i) = 3.0
         if (u%PtA(2,i) >=  7.0) u%deltaf(i) = -4.0
      end do
      
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*) "Ending test VSMTest_NACA_10deg_w_flaps"
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
   end subroutine VSMTest_NACA_10deg_w_flaps    
      
   subroutine VSMTest_FlatPlate_2wing_twists(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs  
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_FlatPlate_2wing_twists'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n, nWingElem
      real(ReKi)                                :: alpha, inc, span, Uinf, midspan, twist, twist1, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_FlatPlate_2wing_twists'
      write(*,*) NewLine//"Running test VSMTest_FlatPlate_2wing_twists"
      
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if

      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 120
      InitInData%NumVolElem = 0
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%elemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%elemLens', errStat, errMsg, routineName ) 
         return
      end if
      
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\FlatPlate.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 2.7_ReKi
      
      nWingElem = InitInData%NumElem / 3
      span = 10.0 ! meters
      inc = span / nWingElem
      
      do i = 1, nWingElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtB(2) = PtA(2) + inc
         InitInData%elemLens(i) = TwoNorm(PtB - PtA )
      end do
      
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
        
      u%x_hat        = 0.0_ReKi
      u%y_hat        = 0.0_ReKi
      u%z_hat        = 0.0_ReKi
      u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
      u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
      u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
      u%PtA = 0.0_ReKi
      u%PtB = 0.0_ReKi
      u%deltaf = 0.0_ReKi
      z%Gammas = 0.6
     
      ! Wing 1 - smaller span in front
      span = 10.0 ! meters
      inc = span / nWingElem
      alpha = 0.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,1:nWingElem) = 0.0_ReKi
      u%U_Inf_v(1,1:nWingElem) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,1:nWingElem) = Uinf*sin(alpha)  ! m/s in the global Z direction 
      do i = 1, nWingElem
         u%PtA(2,i) = (i-1)*inc - span / 2.0
         u%PtB(2,i) = u%PtA(2,i) + inc
         midspan = abs(( u%PtA(2,i) + u%PtB(2,i) ) / 2.0)
         twist1 = (9.0 - 12.0*midspan/5.0)
         twist = twist1*D2R
         u%x_hat(3,i) = cos(twist)
         u%x_hat(1,i) = sin(twist)
         u%y_hat(3,i) = -sin(twist)
         u%y_hat(1,i) = cos(twist)
      end do
      
      ! Wing 2
      span = 20.0 ! meters
      inc = span / (2*nWingElem)
      alpha = 0.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,nWingElem+1:p%NumElem) = 0.0_ReKi
      u%U_Inf_v(1,nWingElem+1:p%NumElem) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,nWingElem+1:p%NumElem) = Uinf*sin(alpha)  ! m/s in the global Z direction 
      do i = 1, 2*nWingElem
         u%PtA(2,i+nWingElem) = (i-1)*inc - span / 2.0
         u%PtA(1,i+nWingElem) = 4.0
         u%PtB(2,i+nWingElem) = u%PtA(2,i+nWingElem) + inc
         u%PtB(1,i+nWingElem) = 4.0
         midspan = abs(( u%PtA(2,i+nWingElem) + u%PtB(2,i+nWingElem) ) / 2.0)
         twist1 = (5.0 - 6.0*midspan/10.0)
         twist = twist1*D2R
         u%x_hat(3,i+nWingElem) = cos(twist)
         u%x_hat(1,i+nWingElem) = sin(twist)
         u%y_hat(3,i+nWingElem) = -sin(twist)
         u%y_hat(1,i+nWingElem) = cos(twist)
      end do
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*) "Ending test VSMTest_FlatPlate_2wing_twists"
      return

      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
   end subroutine VSMTest_FlatPlate_2wing_twists
subroutine VSMTest_FlatPlate_2wing_8and4deg(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs 
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_FlatPlate_2wing_8and4deg'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n, nWingElem
      real(ReKi)                                :: alpha, inc, span, Uinf, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_FlatPlate_2wing_8and4deg'
      write(*,*) NewLine//"Running test VSMTest_FlatPlate_2wing_8and4deg"
      
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
        
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 120
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%elemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%elemLens', errStat, errMsg, routineName ) 
         return
      end if
      
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\FlatPlate.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 2.7_ReKi
      ! Wing 1
      nWingElem = InitInData%NumElem / 3
      span = 10.0 ! meters
      inc = span / nWingElem
      PtA = 0.0
      PtB = 0.0
      do i = 1, nWingElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtA(1) = 4.0
         PtB(2) = PtA(2) + inc
         PtB(1) = 4.0
         InitInData%elemLens(i) = TwoNorm(PtB - PtA )
      end do
      ! Wing 2
      span = 20.0 ! meters
      inc = span / (2*nWingElem)
      do i = 1, 2*nWingElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtA(1) = 4.0
         PtB(2) = PtA(2) + inc
         PtB(1) = 4.0
         InitInData%elemLens(i+nWingElem) = TwoNorm(PtB - PtA )
      end do
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
      
      
      u%x_hat        = 0.0_ReKi
      u%y_hat        = 0.0_ReKi
      u%z_hat        = 0.0_ReKi
      u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
      u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
      u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
      u%PtA = 0.0_ReKi
      u%PtB = 0.0_ReKi
      u%deltaf = 0.0_ReKi
      z%Gammas = 0.6
     
      ! Wing 1 - smaller span in front
      span = 10.0 ! meters
      inc = span / nWingElem  
      alpha = 8.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,1:nWingElem) = 0.0_ReKi
      u%U_Inf_v(1,1:nWingElem) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,1:nWingElem) = Uinf*sin(alpha)  ! m/s in the global Z direction 
      do i = 1, nWingElem
         u%PtA(2,i) = (i-1)*inc - span / 2.0
         u%PtB(2,i) = u%PtA(2,i) + inc
      end do
      
      ! Wing 2
      span = 20.0 ! meters
      inc = span / (2*nWingElem)
      alpha = 4.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,nWingElem+1:p%NumElem) = 0.0_ReKi
      u%U_Inf_v(1,nWingElem+1:p%NumElem) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,nWingElem+1:p%NumElem) = Uinf*sin(alpha)  ! m/s in the global Z direction 
      do i = 1, 2*nWingElem
         u%PtA(2,i+nWingElem) = (i-1)*inc - span / 2.0
         u%PtA(1,i+nWingElem) = 4.0
         u%PtB(2,i+nWingElem) = u%PtA(2,i+nWingElem) + inc
         u%PtB(1,i+nWingElem) = 4.0
      end do
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*) "Ending test VSMTest_FlatPlate_2wing_8and4deg"
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
end subroutine VSMTest_FlatPlate_2wing_8and4deg
      
subroutine VSMTest_NACA_2wing_8and4deg(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs   
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_NACA_2wing_8and4deg'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n, nWingElem
      real(ReKi)                                :: alpha, inc, span, Uinf, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_NACA_2wing_8and4deg'
      write(*,*) NewLine//"Running test VSMTest_NACA_2wing_8and4deg"
      
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
        
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 80
      InitInData%NumVolElem = 0
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%elemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%elemLens', errStat, errMsg, routineName ) 
         return
      end if
      
      
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\NACA1410FW.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 2.7_ReKi
      
      ! Wing 1
      nWingElem = InitInData%NumElem / 2 
      span = 10.0 ! meters
      inc = span / nWingElem
      do i = 1, nWingElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtB(2) = PtA(2) + inc
         InitInData%elemLens(i) = TwoNorm(PtB - PtA )
      end do
      
      ! Wing 2
      span = 20.0 ! meters
      !inc = span / (2*nWingElem)
      inc = span / (nWingElem)
      do i = 1, nWingElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtA(1) = 4.0
         PtB(2) = PtA(2) + inc
         PtB(1) = 4.0
         InitInData%elemLens(i+nWingElem) = TwoNorm(PtB - PtA )
      end do
      
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
      !nWingElem = p%NumElem / 3
      nWingElem = p%NumElem / 2  
      u%x_hat        = 0.0_ReKi
      u%y_hat        = 0.0_ReKi
      u%z_hat        = 0.0_ReKi
      u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
      u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
      u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
      u%PtA = 0.0_ReKi
      u%PtB = 0.0_ReKi
      u%deltaf = 0.0_ReKi
      z%Gammas = 0.6
     
      ! Wing 1 - smaller span in front
      span = 10.0 ! meters
      inc = span / nWingElem
      alpha = 8.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,1:nWingElem) = 0.0_ReKi
      u%U_Inf_v(1,1:nWingElem) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,1:nWingElem) = Uinf*sin(alpha)  ! m/s in the global Z direction 
      do i = 1, nWingElem
         u%PtA(2,i) = (i-1)*inc - span / 2.0
         u%PtB(2,i) = u%PtA(2,i) + inc
      end do
      
      ! Wing 2
      span = 20.0 ! meters
      !inc = span / (2*nWingElem)
      inc = span / (nWingElem)
      alpha = 4.0*Pi/180.
      Uinf  = 1.0_ReKi
      u%U_Inf_v(2:3,nWingElem+1:p%NumElem) = 0.0_ReKi
      u%U_Inf_v(1,nWingElem+1:p%NumElem) = Uinf*cos(alpha)  ! m/s in the global X 
      u%U_Inf_v(3,nWingElem+1:p%NumElem) = Uinf*sin(alpha)  ! m/s in the global Z direction 
      !do i = 1, 2*nWingElem
      do i = 1, nWingElem
         u%PtA(2,i+nWingElem) = (i-1)*inc - span / 2.0
         u%PtA(1,i+nWingElem) = 4.0
         u%PtB(2,i+nWingElem) = u%PtA(2,i+nWingElem) + inc
         u%PtB(1,i+nWingElem) = 4.0
      end do
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*) "Ending test VSMTest_NACA_2wing_8and4deg"
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
end subroutine VSMTest_NACA_2wing_8and4deg
subroutine VSMTest_ReverseDir(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs 
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_ReverseDir'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n
      real(ReKi)                                :: alpha, inc, span, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_ReverseDir'
   
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
        
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 20
      InitInData%NumVolElem = 0
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%ElemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%ElemLens', errStat, errMsg, routineName ) 
         return
      end if
      
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\FlatPlate.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 1.0_ReKi
      
      span = 20.0
      inc = span / InitInData%NumElem
      do i = 1, InitInData%NumElem/2
         PtB(2) = (i-1)*inc - span / 2.0
         PtA(2) = PtB(2) + inc
         InitInData%ElemLens(i) = TwoNorm(PtB-PtA)
      end do
      do i = InitInData%NumElem/2 + 1, InitInData%NumElem
         PtA(2) = (i-1)*inc - span / 2.0
         PtB(2) = PtA(2) + inc
         InitInData%ElemLens(i) = TwoNorm(PtB-PtA)
      end do
         
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
         alpha = 2.0*Pi/180.
         u%U_Inf_v(2:3,:) = 0.0_ReKi
         u%U_Inf_v(1,:) = cos(alpha)  ! m/s in the global X 
         u%U_Inf_v(3,:) = sin(alpha)  ! m/s in the global Z direction   
         u%x_hat        = 0.0_ReKi
         u%y_hat        = 0.0_ReKi
         u%z_hat        = 0.0_ReKi
         u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
         u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
         u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
         u%PtA = 0.0_ReKi
         u%PtB = 0.0_ReKi
         u%deltaf = 0.0_ReKi
         z%Gammas = 0.6

         
         do i = 1, p%NumElem/2
            u%PtB(2,i) = (i-1)*inc - span / 2.0
            u%PtA(2,i) = u%PtB(2,i) + inc
         end do
         do i = p%NumElem/2 + 1, p%NumElem
            u%PtA(2,i) = (i-1)*inc - span / 2.0
            u%PtB(2,i) = u%PtA(2,i) + inc
         end do
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the

      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if

      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )
      
      call Cleanup()
      write(*,*)
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
   end subroutine VSMTest_ReverseDir
   
   subroutine VSMTest_Basic(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(VSM_InitInputType)                   :: InitInData      ! Input data for initialization
      type(VSM_InitOutputType)                  :: InitOutData     ! Output data from initialization
      type(VSM_ParameterType)                   :: p               ! Parameters
      type(VSM_InputType)                       :: u               ! System inputs
      type(VSM_MiscVarType)                     :: m               ! Misc Vars
      type(VSM_OutputType)                      :: y               ! System outputs 
      type(VSM_OtherStateType)                  :: OtherState      ! other states
      type(VSM_ConstraintStateType)             :: z               ! Constraint states at t;
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'VSMTest_Basic'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      real(DbKi)                                :: t
      integer(IntKi)                            :: nSteps
      integer(IntKi)                            :: i
      integer(IntKi)                            :: n
      real(ReKi)                                :: alpha, inc, span, PtA(3), PtB(3)
      
         ! Initialize error handling variables
      errMsg   = ''
      errStat  = ErrID_None
      nSteps   = 1
      interval = 1.0_DbKi 
      InitInData%OutRootName='..\..\build\testoutput\VSM\VSMTest_Basic'
   
      call VSMTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
        
      InitInData%CtrlPtMod  = 2
      InitInData%NumAFfiles = 1
      InitInData%NumElem    = 20
      InitInData%NumVolElem = 0
      allocate(InitInData%AFNames(1), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFNames', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%Chords(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%Chords', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%AFIDs(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%AFIDs', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%ElemLens(InitInData%NumElem), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%ElemLens', errStat, errMsg, routineName ) 
         return
      end if
      InitInData%AFNames(1)    = '..\..\modules-local\vsm\tests\Airfoils\FlatPlate.dat'
      InitInData%AFIDs = 1
      InitInData%Chords = 1.0_ReKi

      span = 20.0
      inc = span / p%NumElem
      do i = 1, p%NumElem
         u%PtA(2,i) = (i-1)*inc - span / 2.0
         u%PtB(2,i) = u%PtA(2,i) + inc
         InitInData%ElemLens(i) = TwoNorm(u%PtB(:,i)-u%PtA(:,i))
      end do
         
         ! Initialize the VSM module
      call VSM_Init( InitInData, u, p, z, OtherState, m, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
         alpha = 20.0*Pi/180.
         u%U_Inf_v(2:3,:) = 0.0_ReKi
         u%U_Inf_v(1,:) = cos(alpha)  ! m/s in the global X 
         u%U_Inf_v(3,:) = sin(alpha)  ! m/s in the global Z direction   
         u%x_hat        = 0.0_ReKi
         u%y_hat        = 0.0_ReKi
         u%z_hat        = 0.0_ReKi
         u%x_hat(3,:)   = 1.0_ReKi  ! in the global Z direction
         u%y_hat(1,:)   = 1.0_ReKi  ! in the global X direction
         u%z_hat(2,:)   = 1.0_ReKi  ! in the global Y direction
         u%PtA = 0.0_ReKi
         u%PtB = 0.0_ReKi
         u%deltaf = 0.0_ReKi
         z%Gammas = 0.6!2.0/pi
         !z%Gammas(1) = 0.01
         !z%Gammas(p%NumElem) = 0.01

         do i = 1, p%NumElem
            u%PtA(2,i) = (i-1)*inc - span / 2.0
            u%PtB(2,i) = u%PtA(2,i) + inc
         end do
         
      
         ! Obtain outputs from VSM module for t = 0.  Since we do not have states, yet, we will use an elliptical distribution of circulations as the
         !  initial guess of the states
      !z%Gammas = 0.1
      !z%Gammas(1)  = 0.07563
      !z%Gammas(2)  = 0.08677
      !z%Gammas(3)  = 0.09338
      !z%Gammas(4)  = 0.09724
      !z%Gammas(5)  = 0.09956
      !z%Gammas(6)  = 0.10102
      !z%Gammas(7)  = 0.10194
      !z%Gammas(8)  = 0.10252
      !z%Gammas(9)  = 0.10287
      !z%Gammas(10)  = 0.10303
      !z%Gammas(11)  = .10304 
      !z%Gammas(12)  = .10289 
      !z%Gammas(13)  = .10255 
      !z%Gammas(14)  = .10198 
      !z%Gammas(15)  = .10107 
      !z%Gammas(16)  = .09963 
      !z%Gammas(17)  = .09731 
      !z%Gammas(18)  = .09346 
      !z%Gammas(19)  = .08685 
      !z%Gammas(20)  = .07571 
      
      
      
      
      
      t = 0.0_DbKi
      call VSM_CalcOutput( t, n, u, p, z, OtherState, m, y, errStat, errMsg ) 
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
   
      !   ! Time Marching Loop
      !do i = 1, nSteps   
      !      ! Update States  for the same inputs a t and t + dt for this basic test: steady conditions
      !   call VSM_UpdateStates( t, n, u, u, p, z, m, errStat, errMsg )   
      !   z%Gammas = 0.1
      !      ! Calc Output
      !   call VSM_CalcOutput( t, n, u, p, z, m, y, errStat, errMsg ) 
      !      if ( errStat >= AbortErrLev ) then
      !         call Cleanup()
      !         stop
      !      end if
      !end do
      !
         ! Print the results to a log
      !call WrScr('      Fx      ,      Fy      ,      Fz      ,      Mx      ,      My      ,      Mz      ,      P      ')   
      !write(tmpValues,*) y%Fx, y%Fy, y%Fz, y%Mx, y%My, y%Mz, y%P  
      !call WrScr(TRIM(tmpValues))
      call VSM_End( u, p, z, y, OtherState, m, errStat, errMsg )

               
      call Cleanup()
      write(*,*)
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
   end subroutine VSMTest_Basic
   
end module VSM_Test