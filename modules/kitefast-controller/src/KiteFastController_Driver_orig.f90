program KiteFastController_Driver
   
   use NWTC_Library
   use KiteFastController_Types  
   use KiteFastController
   
   implicit none
   
      ! Variables
   integer(IntKi)                   :: errStat    ! Status of error message
   character(1024)                  :: errMsg      ! Error message if errStat /= ErrID_None 
   character(1024)                  :: routineName          
   ! type(KAD_Dvr_InitInputType)      :: dvrInitInp           ! Initialization input data for the KiteAeroDyn Driver
   type(KFC_InputType)              :: u
   type(KFC_OutputType)             :: y
   type(KFC_ParameterType)          :: p
   type(KFC_InitInputType)          :: InitInp
   type(KFC_MiscVarType)            :: m           !< KFC MiscVars for the module
   type(KFC_OtherStateType)         :: o           !< KFC Other states, containing C-SIM output                            

   real(DbKi)                       :: t
   real(DbKi)                       :: interval

   type(KFC_InitOutputType)         :: InitOut     !< Output data for initialization routine >>>RRD added
   

      ! Initialize the NWTC library
   call NWTC_Init()
   
      ! Initialize error handling variables
   errMsg  = ''
   errStat = ErrID_None
   t = 0.0_DbKi
   
   routineName = 'KiteFastController_Driver'
   print *, 'Running '//trim(routineName)
 
      ! Initialize the KiteFastController module
   InitInp%numFlaps = 3
   InitInp%numPylons = 2
   InitInp%DT  = 0.01_DbKi
   InitInp%InpFileData%DLL_Filename = '/mnt/d/Users/rdamiani/sandbox/build/modules/kitefast-controller/libkitefastcontroller_controller.so'
   interval = InitInp%DT
   InitInp%InputFileName = '/mnt/d/Users/rdamiani/sandbox/glue-codes/kitefast/test_cases/RRD_m600_landbased/RRD_ctrl_input.dat' !RRD added  
   InitInp%useDummy = .True. !RRD added 
   InitInp%OutFileRoot='/mnt/d/Users/rdamiani/sandbox/glue-codes/kitefast/test_cases/RRD_m600_landbased/RRD_ctrl_output' !RRD added 
   
   call KFC_Init(InitInp, u, p, y, interval,m,o, InitOut, errStat, errMsg)
      print *, "KiteFastController_Driver calling KFC_Init received ErrStat=", errStat, " ErrMsg=" , trim(errMsg)
      if ( errStat >= AbortErrLev ) call Cleanup()
      ! Establish the KiteFastController inputs
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
      
contains
   
   subroutine Cleanup()
      if ( errStat /= ErrID_None ) print *, errMsg       
      stop
   end subroutine Cleanup
   
   subroutine print_help()
      print '(a)', 'usage: '
      print '(a)', ''
      print '(a)', 'KiteAeroDyn_Driver.exe [driverfilename]'
      print '(a)', ''
      print '(a)', 'Where the optional argument, driverfilename, is the name of the KiteAeroDyn driver input file.'
      print '(a)', ''
   end subroutine print_help

end program KiteFastController_Driver
