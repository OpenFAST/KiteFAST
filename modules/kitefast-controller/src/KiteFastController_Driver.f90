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
   real(DbKi)                       :: t
   real(DbKi)                       :: interval
   
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
   InitInp%DLL_Filename = '/home/makani/Desktop/sandbox/build/modules/kitefast-controller/libkitefastcontroller_controller.so'
   interval = InitInp%DT
  
   call KFC_Init(InitInp, u, p, y, interval, errStat, errMsg)
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

   call KFC_Step(t, u, p, y, errStat, errMsg )
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
