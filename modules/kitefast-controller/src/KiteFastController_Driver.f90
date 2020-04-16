!**********************************************************************************************************************************
! LICENSING
! Copyright (C) 2020 Makani Wind Power and RRD Engineering, LLC
!
!    This file is part of KiteFAST.
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
program KiteFastController_Driver
   
   use NWTC_Library
   use KiteFastController_Types  
   use KiteFastController
   use KFC_Dvr_Subs

   implicit none
   
      ! Variables
   integer(IntKi)                   :: errStat    ! Status of error message
   character(1024)                  :: errMsg      ! Error message if errStat /= ErrID_None 
   character(1024)                  :: routineName          
   
   type(KFC_InputType)              :: u
   type(KFC_OutputType)             :: y
   type(KFC_ParameterType)          :: p
   type(KFC_InitInputType)          :: InitInp
   type(KFC_MiscVarType)            :: m           !< KFC MiscVars for the module
   type(KFC_OtherStateType)         :: o           !< KFC Other states, containing C-SIM output                            

   type(KFC_Dvr_InitInputType)      :: dvrprms  ! Initialization input data for the KFC Driver
   CHARACTER(1024)                  :: dvrFilename          ! Filename and path for the driver input file.  This is passed in as a command line argument when running the Driver exe. 

   real(DbKi)                       :: t
   real(DbKi)                       :: interval

   INTEGER                                        :: ErrStat2      
   CHARACTER(ErrMsgLen)                           :: errMsg2          ! temporary Error message if errStat /= ErrID_None        

   type(KFC_InitOutputType)         :: InitOut     !< Output data for initialization routine >>>RRD added
   type(ProgDesc), parameter        :: KFCdriver_Ver = ProgDesc( 'KiteFastCtrlDriver', '1', '3/27/2020' )

      ! Initialize the NWTC library
   call NWTC_Init()
   
      ! Initialize error handling variables
   errMsg  = ''
   errStat = ErrID_None
   t = 0.0_DbKi
   
   routineName = 'KiteFastController_Driver'
   print *, 'Running '//trim(routineName)//' Version '//KFCdriver_Ver%Name//' '//KFCdriver_Ver%Ver//' '//KFCdriver_Ver%Date
 
         ! Parse the driver file if one was provided, if not, then set driver parameters using hardcoded values
   if ( command_argument_count() > 1 ) then
      call print_help()
      stop
   end if
                 
   if ( command_argument_count() == 1 ) then
             
      call get_command_argument(1, dvrFilename)
      
         ! Parse the driver input file 
      call KFC_Dvr_Read_InputFile( dvrFilename, dvrprms,InitInp, KFCdriver_Ver, errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg, RoutineName )
         if (errStat >= AbortErrLev) then
            call Cleanup()
         end if  
         
      if (dvrprms%KFCdrivermode ==1) then
         ! Run the simulation based on that file 
         call KFC_Dvr_Simulate( dvrprms, InitInp, errStat2, errMsg2 )
      endif         
   
   endif   
   
   if (( command_argument_count() == 0 ) .or. (dvrprms%KFCdrivermode ==0))  then !This is the old driver portion
      
     ! Testing mode
     call KFCTest_Basic(errStat, errMsg)
       
   endif

   call Cleanup()
      
contains

   !====================================================================================================
subroutine Cleanup()
   !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
   !     any existing echo information
   !----------------------------------------------------------------------------------------------------  
      
      if ( errStat /= ErrID_None ) print *, errMsg       
      stop
   end subroutine Cleanup

   
   subroutine print_help()
    print '(a)', 'usage: '
    print '(a)', ''
    print '(a)', 'KiteFastController_Driver.exe [driverfilename]'
    print '(a)', ''
    print '(a)', 'Where the optional argument, driverfilename, is the name of the KiteFastController driver input file.'
    print '(a)', ''

   end subroutine print_help
   

Subroutine KFCTest_Basic(errStat, errMsg)
   integer(IntKi), INTENT(OUT)                   :: errStat    ! Status of error message
   character(1024), INTENT(OUT)                  :: errMsg      ! Error message if errStat /= ErrID_None 
 
   !LOCAL
   type(KFC_InitInputType)          :: InitInp
   type(KFC_InputType)              :: u
   type(KFC_ParameterType)          :: p
   type(KFC_OutputType)             :: y
   real(DbKi)                       :: interval
   type(KFC_MiscVarType)            :: m           !< KFC MiscVars for the module
   type(KFC_OtherStateType)         :: o           !< KFC Other states, containing C-SIM output                            
   type(KFC_InitOutputType)         :: InitOut     !< Output data for initialization routine >>>RRD added

   real(DbKi)                       :: t

   ! Initialize the KiteFastController module
InitInp%numFlaps = 3
InitInp%numPylons = 2
InitInp%DT  = 0.01_DbKi
InitInp%InpFileData%DLL_Filename = '//home/rdamiani/sandbox/build/modules/kitefast-controller/libkitefastcontroller_controller.so'
interval = InitInp%DT
InitInp%InputFileName = '/home/rdamiani/rdamiani/sandbox/glue-codes/kitefast/test_cases/RRD_m600_landbased/RRD_ctrl_input.dat' !RRD added  
InitInp%useDummy = .True. !RRD added 
InitInp%OutFileRoot='/mnt/d/Users/rdamiani/sandbox/glue-codes/kitefast/test_cases/RRD_m600_landbased/RRD_ctrl_output' !RRD added 

call KFC_Init(InitInp, u, p, y, interval,m,o, InitOut, errStat, errMsg)
print *, "KiteFastController_Driver calling KFC_Init received ErrStat=", errStat, " ErrMsg=" , trim(errMsg)
if ( errStat >= AbortErrLev ) call Cleanup()

  ! Step the controller
t = 1.0
u%dcm_g2b = reshape((/ 1, 2, 3, 4, 5, 6, 7, 8, 9 /), shape(u%dcm_g2b))
u%pqr = (/ 1, 1, 1 /)
u%acc_norm = 1.0
u%Xg = (/ 1, 2, 3 /)
u%Vg = (/ 1, 2, 3 /) !reshape((/ 1, 2, 3 /), shape(u%Vg))
u%Vb = (/ 1, 2, 3 /) !reshape((/ 1, 2, 3 /), shape(u%Vb))
u%Ag = (/ 1, 2, 3 /) !reshape((/ 1, 2, 3 /), shape(u%Ag))
u%Ab = (/ 1, 2, 3 /) !reshape((/ 1, 2, 3 /), shape(u%Ab))
u%rho = 1.0        
u%apparent_wind = (/ -30, 1, 1 /) !reshape((/ 1, 2, 3 /), shape(u%apparent_wind))
u%tether_forceb = (/ 1, 1, 1 /) !reshape((/ 1, 2, 3 /), shape(u%tether_forceb))
u%wind_g = 1.0 !reshape((/ 1, 2, 3 /), shape(u%wind_g))
!    y%GenSPyRtr = reshape((/ 1, 2, 3, 4 /), shape(y%GenSPyRtr))
!    y%GenPPyRtr = reshape((/ 1, 2, 3, 4 /), shape(y%GenPPyRtr))

!RRD added the following aerotorque stuff
u%SPyAeroTorque=100.
u%PPyAeroTorque=150.

call KFC_Step(t, u, p, y, m,o, errStat, errMsg )
print *, "KiteFastController_Driver calling KFC_Step received ErrStat=", errStat, " ErrMsg=" , trim(errMsg)
if ( errStat >= AbortErrLev ) call Cleanup()
! Print the controller outputs

call KFC_End(p, errStat, errMsg)
print *, "KiteFastController_Driver calling KFC_End received ErrStat=", errStat, " ErrMsg=" , trim(errMsg)
if ( errStat >= AbortErrLev ) call Cleanup()

End Subroutine KFCTest_Basic

end program KiteFastController_Driver