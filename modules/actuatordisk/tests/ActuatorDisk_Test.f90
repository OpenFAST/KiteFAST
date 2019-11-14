!**********************************************************************************************************************************
! ActuatorDisk_Test: This code tests the ActuatorDisk module
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

module ActuatorDisk_Test
   
   use ActuatorDisk_Types
   use ActuatorDisk
   use NWTC_Library
   
   
   implicit none

   
contains
   
   subroutine ActDskTest_SetBaseInitInpData(InitInData, errStat, errMsg)
      type(ActDsk_InitInputType), intent(inout) :: InitInData      ! Input data for initialization
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'ActDskTest_SetBaseInitInpData'
         ! Initialize error handling variables
      errMsg  = ''
      errStat = ErrID_None
      InitInData%RotorMod = 1
      InitInData%R        = 2.0_ReKi
      InitInData%AirDens  = 1024.0_ReKi
      InitInData%InitInpFile%numRtSpd = 2
      InitInData%InitInpFile%numVrel  = 3
      InitInData%InitInpFile%numPitch = 2
      InitInData%InitInpFile%numSkew  = 2

      InitInData%FileName = ''
      allocate(InitInData%InitInpFile%RtSpds(InitInData%InitInpFile%numRtSpd), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%InitInpFile%RtSpds', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%InitInpFile%Vrels(InitInData%InitInpFile%numVrel), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%InitInpFile%Vrels', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%InitInpFile%Skews(InitInData%InitInpFile%numSkew), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%InitInpFile%Skews', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%InitInpFile%Pitches(InitInData%InitInpFile%numPitch), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%InitInpFile%Pitches', errStat, errMsg, routineName ) 
         return
      end if
      allocate(InitInData%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table(7,InitInData%InitInpFile%numRtSpd, InitInData%InitInpFile%numVrel, InitInData%InitInpFile%numSkew, InitInData%InitInpFile%numPitch), stat = errStat2)
      if (errStat2 /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not allocate memory for InitInData%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table', errStat, errMsg, routineName ) 
         return
      end if
      
      InitInData%InitInpFile%RtSpds   = (/3.0_ReKi, 15.0_ReKi/) ! rad/s
      InitInData%InitInpFile%Vrels    = (/0.0_ReKi, 8.0_ReKi, 15.0_ReKi/) ! m/s
      InitInData%InitInpFile%Skews    = (/0.0_ReKi, 0.5_ReKi/)  ! these are in rad, even though the input file data would be deg
      InitInData%InitInpFile%Pitches  = (/0.0_ReKi, 0.3_ReKi/)  ! these are in rad, even though the input file data would be deg
      
      InitInData%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table = RESHAPE( (/ &  !  C_Fx       C_Fy       C_Fz       C_Mx   C_My     C_Mz     C_P
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
                                                 1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi &
                                            /), (/7, 2, 3, 2, 2/) )
   end subroutine ActDskTest_SetBaseInitInpData
   
   subroutine ActDskTest_ReadFile(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(ActDsk_InitInputType)                :: InitInData           ! Input data for initialization
      type(ActDsk_InitOutputType)               :: InitOutData          ! Output data from initialization
      type(ActDsk_ParameterType)                :: p                    ! Parameters
      type(ActDsk_InputType)                    :: u                    ! System inputs
      type(ActDsk_OutputType)                   :: y                    ! System outputs  
      type(ActDsk_MiscVarType)                  :: m                    ! Misc Vars
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'ActDskTest_Basic'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      integer(IntKi)                            :: i
         ! Initialize error handling variables
      errMsg  = ''
      errStat = ErrID_None

      interval            = 1.0_DbKi 
      InitInData%R        = 2.0_ReKi
      InitInData%AirDens  = 98.0_ReKi
      InitInData%Filename = '..\..\modules\actuatordisk\tests\ActuatorDisk.dat'
      
         ! Initialize the ActuatorDisk module
      call ActDsk_Init( InitInData, u, p, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Time marching loop
      do i = 1,3
            ! Set Inputs
         u%RtSpd   = 4.0_ReKi
         u%pitch   = i*0.1_ReKi
         u%skew    = i*pi/5.0_ReKi
         u%DiskAve_Vrel = 8.0_ReKi
         
            ! Obtain outputs from ActuatorDisk module
         call ActDsk_CalcOutput( u, p, m, y, errStat, errMsg )
            if ( errStat >= AbortErrLev ) then
               call Cleanup()
               stop
            end if
   
            ! Print the results to a log
         call WrScr('      Fx      ,      Fy      ,      Fz      ,      Mx      ,      My      ,      Mz      ,      P      ')   
         write(tmpValues,*) y%Fx, y%Fy, y%Fz, y%Mx, y%My, y%Mz, y%P  
         call WrScr(TRIM(tmpValues))
      end do
      
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( errStat /= ErrID_None ) print *, trim(errMsg)      
      
         end subroutine Cleanup
   end subroutine ActDskTest_ReadFile

   subroutine ActDskTest_Basic(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(ActDsk_InitInputType)                :: InitInData           ! Input data for initialization
      type(ActDsk_InitOutputType)               :: InitOutData          ! Output data from initialization
      type(ActDsk_ParameterType)                :: p                    ! Parameters
      type(ActDsk_InputType)                    :: u                    ! System inputs
      type(ActDsk_MiscVarType)                  :: m                    ! System inputs
      type(ActDsk_OutputType)                   :: y                    ! System outputs      
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'ActDskTest_Basic'
      character(1024)                           :: tmpValues
      real(DbKi)                                :: interval
      
         ! Initialize error handling variables
      errMsg  = ''
      errStat = ErrID_None

      interval            = 1.0_DbKi 
   
      call ActDskTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
      
         ! Initialize the ActuatorDisk module
      call ActDsk_Init( InitInData, u, p, y, interval, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
      
      u%RtSpd   = 10.0_ReKi
      u%skew = 0.0_ReKi
      u%DiskAve_Vrel = 10.0_ReKi
      
         ! Obtain outputs from ActuatorDisk module
      call ActDsk_CalcOutput( u, p, m, y, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
   
         ! Print the results to a log
      call WrScr('      Fx      ,      Fy      ,      Fz      ,      Mx      ,      My      ,      Mz      ,      P      ')   
      write(tmpValues,*) y%Fx, y%Fy, y%Fz, y%Mx, y%My, y%Mz, y%P  
      call WrScr(TRIM(tmpValues))
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
   end subroutine ActDskTest_Basic
   
end module ActuatorDisk_Test