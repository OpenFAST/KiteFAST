program KiteFastController_Driver
   
   use NWTC_Library
   use KiteFastController_Types
   use KiteFastController
   
   implicit none
   
   
      ! Variables
   integer(IntKi)                   :: errStat, errStat2    ! Status of error message
   character(1024)                  :: errMsg, errMsg2      ! Error message if errStat /= ErrID_None 
   CHARACTER(1024)                  :: dvrFilename          ! Filename and path for the driver input file.  This is passed in as a command line argument when running the Driver exe. 
   character(1024)                  :: routineName          
  ! type(KAD_Dvr_InitInputType)      :: dvrInitInp           ! Initialization input data for the KiteAeroDyn Driver
   type(KFC_InputType)              :: u
   type(KFC_OutputType)             :: y
   type(KFC_ParameterType)          :: p
   type(KFC_InitOutputType)         :: InitOut
   type(KFC_InitInputType)          :: InitInp
   real(DbKi)                       :: t
   
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
   InitInp%DLL_Filename = 'TestController.dll'
   
   call KFC_Init(InitInp, p, InitOut, errStat, errMsg)
      if ( errStat >= AbortErrLev ) call Cleanup()
      ! Establish the KiteFastController inputs

      ! Step the controller
   call KFC_CalcOutput(t, u, p, y, errStat, errMsg )
      if ( errStat >= AbortErrLev ) call Cleanup()
      ! Print the controller outputs
   
   call KFC_End(p, errStat, errMsg)
      
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
    print '(a)', 'KiteAeroDyn_Driver.exe [driverfilename]'
    print '(a)', ''
    print '(a)', 'Where the optional argument, driverfilename, is the name of the KiteAeroDyn driver input file.'
    print '(a)', ''

   end subroutine print_help
   
   
   
   
end program KiteFastController_Driver