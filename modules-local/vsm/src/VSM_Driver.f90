!**********************************************************************************************************************************
! VSM_Driver: This code tests a stand-alone version of the VSM module
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
   
program VSM_Driver

   use NWTC_Library
   use VSM_Types
   use VSM
   use VSM_Test
   
   implicit none

    ! Variables
   
   real(DbKi)  :: dt, t
   integer     :: i, j, k, n 
   !type(VSM_InitInputType)                    :: InitInData           ! Input data for initialization
   !type(VSM_InitOutputType)                   :: InitOutData          ! Output data from initialization
   !type(VSM_ParameterType)                    :: p                    ! Parameters
   !type(VSM_InputType)                        :: u                    ! System inputs
   !type(VSM_OutputType)                       :: y                    ! System outputs
   integer(IntKi)                                :: errStat, errStat2    ! Status of error message
   character(1024)                               :: errMsg, errMsg2      ! Error message if errStat /= ErrID_None
   
   
   
   CHARACTER(1024)                               :: dvrFilename          ! Filename and path for the driver input file.  This is passed in as a command line argument when running the Driver exe.
   integer                                       :: nSteps
   real(DbKi)                                    :: simTime  
   integer                                       :: nSimSteps
   character(1024)                               :: routineName
   real(DbKi)                                    :: interval
   
      ! Initialize the NWTC library
   call NWTC_Init()
   
      ! Initialize error handling variables
   errMsg  = ''
   errStat = ErrID_None
   
   routineName = 'VSM_Driver'
   write(*,*) 
   write(*,*) "Running "//trim(routineName)
   
   
   
      ! Parse the driver file if one was provided, if not, then set driver parameters using hardcoded values
   if ( command_argument_count() > 1 ) then
      call print_help()
      stop
   end if
  
   
      
      ! Parse the driver input file and run the simulation based on that file
      
   if ( command_argument_count() == 1 ) then
      
      call get_command_argument(1, dvrFilename)
      !call ReadDriverInputFile( dvrFilename, dvrInitInp, errStat2, errMsg2 )
      !   call SetErrStat(errStat2, errMsg2, ErrStat, ErrMsg, RoutineName )
      !   if (ErrStat >= AbortErrLev) then
      !      call Cleanup()
      !      stop
      !   end if
     
   
   else
     ! Testing mode
     
     !call VSMTest_Basic(errStat, errMsg)
     
     
     !call VSMTest_ReverseDir(errStat, errMsg)
     
     
     call VSMTest_FlatPlate_8deg(errStat, errMsg) 
     call VSMTest_NACA_12deg(errStat, errMsg) 
     call VSMTest_NACA_0deg(errStat, errMsg) 
     call VSMTest_NACA_KiteWing(errStat, errMsg)
     call VSMTest_NACA_8deg_10deg_sweep(errStat, errMsg) 
     call VSMTest_NACA_10deg_w_flaps(errStat, errMsg) 
     call VSMTest_FlatPlate_2wing_8and4deg(errStat, errMsg) 
     call VSMTest_NACA_2wing_8and4deg(errStat, errMsg) 
     call VSMTest_FlatPlate_2wing_twists(errStat, errMsg) 
     
    ! call VSMTest_ReadFile(errStat, errMsg)
         ! Set the inputs
   end if
   
  
   
   
   
   
  
   call Cleanup()
   stop
   
   contains
   
   !====================================================================================================
   subroutine Cleanup()
   !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
   !     any existing echo information
   !----------------------------------------------------------------------------------------------------  
      
      if ( ErrStat /= ErrID_None ) write(*,*) ErrMsg       
      
   end subroutine Cleanup

   
   
   
   subroutine print_help()
    print '(a)', 'usage: '
    print '(a)', ''
    print '(a)', 'VSM_Driver.exe [driverfilename]'
    print '(a)', ''
    print '(a)', 'Where the optional argument, driverfilename, is the name of the VSM driver input file.'
    print '(a)', ''

   end subroutine print_help
   
end program VSM_Driver

