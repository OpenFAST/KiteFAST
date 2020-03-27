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
!> This module provides the routines necessary to run KiteFastController in stand-alone mode.
module KFC_Dvr_Subs
   
   use NWTC_Library   
   use KiteFastController
   use KiteFastController_Types 
   
   implicit none 

private

   type(ProgDesc), parameter  :: KFC_Dvr_Ver = ProgDesc( 'KiteFastController_Driver', '', '' )
      
   type, public :: KFC_Dvr_InitInputType
      character(1024) :: DrvInputFile          ! Driver Input File
      character(1024) :: DrvCtrlInputs         ! Time series data file
      integer(IntKi)  :: UnCtrlFile            ! DRVCtrlInputs file id
      integer(IntKi)  :: CtrlLineLength        ! length of array containing: t_c,rho_c,Xg_c(3),Vg_c(3),Ag_c(3),acc_norm_c,Vb_c(3),Ab_c(3),pqr_c(3),wind_g_c(3),apparent_wind_c(3), tether_forceb_c(3), dcm_g2b_c(9), aeroTorq(8)
      real(DbKi), allocatable :: OneCtrlLine(:)   ! array containing control inputs
      character(1024) :: OutFileRoot           ! Root name for any output files (sring) [use "" for .dvr rootname]
      integer(IntKi)  ::  KFCdrivermode        ! flag for how to run teh driver, for now 0 (single step) or 1 (with time series) 
   end type KFC_Dvr_InitInputType
   
   public :: KFC_Dvr_Read_InputFile
   public :: KFC_Dvr_Simulate

   contains
   
!----------------------------------------------------------------------------------------------------------------------------------
!> Routine to parse the driver input file   
subroutine KFC_Dvr_Read_InputFile( inputFile, dvrprms, InitInp, KFCdriver_Ver, errStat, errMsg )
   character(*),                intent(in   )  :: inputFile         !< Name of the KFC_driver input file
   type(KFC_Dvr_InitInputType), intent(inout)  :: dvrprms  !< Driver initialization input data 
   type(KFC_InitInputType),     intent(inout)  :: InitInp           !< Initialization input data for the KFC driver program
   
   integer(IntKi),              intent(  out)  :: errStat           !< Error status of the operation
   character(*),                intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None
   type(ProgDesc)                              :: KFCdriver_Ver     !<version
      ! Local variables
   integer(IntKi)            :: i,j, count         ! loop and counter
   real(DbKi), allocatable :: SPyRtrIrot(:)   ! array containing rotor inertias to be read in (FAST would provide those usually) that will be passed to InitInput counterpart
   real(DbKi), allocatable :: PPyRtrIrot(:)   ! array containing rotor inertias to be read in (FAST would provide those usually) that will be passed to InitInput counterpart
   
   character(ErrMsgLen)      :: errMsg2     ! temporary Error message if errStat /= ErrID_None
   integer(IntKi)            :: errStat2    ! temporary Error status of the operation
   character(*), parameter   :: routineName = 'KFC_Read_Dvr_InputFile'
   character(1024)           :: fileName    ! The trimmed input file name
   integer(IntKi)            :: UnIn, UnEc  ! File units
   character(1024)           :: FTitle      ! "File Title": the 2nd line of the input file, which contains a description of its contents
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
   
      call ReadCom( UnIn, fileName, 'KiteFastController Driver input file header (line 1)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
      call ReadStr( UnIn, fileName, FTitle, 'FTitle', 'KiteFastController Driver input file header: File Description (line 2)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )        

   !-------------------------- SIMULATION CONTROL ------------------------

      ! Skip the comment line.
      call ReadCom( UnIn, fileName, ' SIMULATION CONTROL ', errStat2, errMsg2, UnEc  ) 
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
         ! Echo - Echo input to "<RootName>.ech".
      
      call ReadVar( UnIn, fileName, Echo, 'Echo',   'Echo flag', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      if (.NOT. Echo .OR. i > 1) EXIT !exit this loop
      
         ! Otherwise, open the echo file, then rewind the input file and echo everything we've read
         
      i = i + 1         ! make sure we do this only once (increment counter that says how many times we've read this file)
      
      call OpenEcho ( UnEc, trim(filename)//'.ech', errStat2, errMsg2, KFCdriver_Ver )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            return
         end if
      
      if ( UnEc > 0 )  WRITE (UnEc,'(/,A,/)')  'Data from '//trim(KFCdriver_Ver%Name)//' primary input file "'//trim( fileName )//'":'
            
      rewind( UnIn, IOSTAT=errStat2 )  
         if (errStat2 /= 0_IntKi ) then
            call SetErrStat( ErrID_Fatal, 'Error rewinding file "'//trim(fileName)//'".', errStat, errMsg, RoutineName )
            call Cleanup()
            return
         end if           
   end do    
   
   if (NWTC_VerboseLevel == NWTC_Verbose) then
      call WrScr( ' Heading of the '//trim(KFCdriver_Ver%Name)//' input file: ' )      
      call WrScr( '   '//trim( FTitle ) )
   end if
   
      ! numFlaps and numPylons (-):
   call ReadVar( UnIn, fileName, InitInp%numFlaps, "numFlaps", "Number of Flaps per Wing (-)", errStat2, errMsg2, UnEc)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call ReadVar( UnIn, fileName, InitInp%numPylons, "numPylons", "Number of Pylons per Wing (-)", errStat2, errMsg2, UnEc)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      ! Read numPylons*2 rotInertias but first allocate as these would be allocated by KFAST normally
   
   call AllocAry(InitInp%SPyRtrIrot, 2, InitInp%numPylons, 'InitInp%SPyRtrIrot', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(InitInp%PPyRtrIrot, 2, InitInp%numPylons, 'InitInp%PPyRtrIrot', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         CALL Cleanup()
         return
      end if
      !also allocate the local 1-D versions needed to read arrays
      j=2*InitInp%numPylons
   call AllocAry(SPyRtrIrot, j, 'SPyRtrIrot', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(PPyRtrIrot, j, 'PPyRtrIrot', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         CALL Cleanup()
         return
      end if
   
   CALL ReadAry( UnIn, fileName, SPyRtrIrot, j, "SPyRtrIrot", "Array of Starboard rotor Inertias [kg*m^2] ", ErrStat2, ErrMsg2)
   CALL ReadAry( UnIn, fileName, PPyRtrIrot, j, "PPyRtrIrot", "Array of Port rotor Inertias [kg*m^2] ", ErrStat2, ErrMsg2)
   
   CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      ENDIF   
   !Now transfer the local info to InitInp
      count = 1
      do j = 1, InitInp%numPylons
         do i = 1,2
            InitInp%SPyRtrIrot(i,j)   = SPyRtrIrot(count)  
               if ( InitInp%SPyRtrIrot(i,j) < 0.0_RekI )  call SetErrStat( ErrID_Fatal, 'Rotor rotational inertia must be greater or equal to zero', errStat, errMsg, routineName )       
            InitInp%PPyRtrIrot(i,j)   = PPyRtrIrot(count)  
               if ( InitInp%PPyRtrIrot(i,j) < 0.0_RekI )  call SetErrStat( ErrID_Fatal, 'Rotor rotational inertia must be greater or equal to zero', errStat, errMsg, routineName ) 
            count = count + 1
         end do
      end do

      ! KFCmode flag (-):
    call ReadVar( UnIn, fileName, InitInp%KFCmode, "KFCmode", "KFC mode (0/1/2) (-)", errStat2, errMsg2, UnEc)
        call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

         !-------------------------- SPECIFIC INPUTS TO PASS DOWNSTREAM------------------------

      ! Skip the comment line.
    call ReadCom( UnIn, fileName, ' INPUTS ', errStat2, errMsg2, UnEc  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Root name for any output files (sring) [use "" for .dvr rootname]
    call ReadVar ( UnIn, fileName, InitInp%OutFileRoot, 'OutFileRoot', 'Root name for any output files (sring) [use "" for .dvr rootname]', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Name of the primary KiteFastController input file (string)
    call ReadVar ( UnIn, fileName, InitInp%InputFileName, 'InputFileName', 'Name of the primary KiteFastController input file (string)', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! KFCmode flag (-):
    call ReadVar( UnIn, fileName, dvrprms%KFCdrivermode, "KFCdriverMode", "KFCDriver mode (0/1) (-)", errStat2, errMsg2, UnEc)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! DrvCtrlInputs filename:
    call ReadVar( UnIn, fileName, dvrprms%DrvCtrlInputs, "DrvCtrlInputs", "KFCDriver time series of inputs (-)", errStat2, errMsg2, UnEc)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call Cleanup()
   
   contains
   
   !====================================================================================================
   subroutine Cleanup()
   !     The routine cleans up the module driver input file and echo file 
   !----------------------------------------------------------------------------------------------------  
      deallocate(SPyRtrIrot)
      deallocate(PPyRtrIrot)
      close(UnIn)
      close(UnEc)
      
   end subroutine Cleanup
   
end subroutine KFC_Dvr_Read_InputFile
   
!----------------------------------------------------------------------------------------------------------------------------------


!----------------------------------------------------------------------------------------------------------------------------------
!> Routine to perform the stand-alone KiteFastController simulation   
subroutine KFC_Dvr_Simulate( dvrprms, InitInp, errStat, errMsg )

   type(KFC_Dvr_InitInputType), intent(inout) :: dvrprms  !< Driver initialization input data 
   type(KFC_InitInputType),     INTENT(INOUT)    :: InitInp     !< Input data for initialization routine >>>RRD: made this inout instead of in
   integer(IntKi),              intent(  out) :: errStat     !< Status of error message
   character(1024),             intent(  out) :: errMsg      !< Error message if errStat /= ErrID_None
   
   !local vars
   
   real(DbKi)                       :: t               ! simulation time
   character(1024)                  :: routineName

   integer(IntKi)                   :: errStat2        ! Status of error message
   character(1024)                  :: errMsg2         ! Error message if errStat /= ErrID_None

   type(KFC_InitOutputType)         :: InitOut ! Initialization output data
   type(KFC_InputType)              :: u       ! Inputs 
   type(KFC_ParameterType)          :: p           ! Parameters
   type(KFC_OtherStateType)         :: o    ! Other states at Time
   type(KFC_OutputType)             :: y    ! Outputs computed at Time (Input only so that mesh connectivity information does not have to be recalculated)
   type(KFC_MiscVarType)            :: m    ! misc/optimization variables           
   
   real(DbKi)                       :: interval        ! KFC time step
   
   
      ! Initialize variables for this routine
   errStat  = ErrID_None
   errMsg   = ""
   routineName = 'KFC_Dvr_Simulate'

   CALL KFC_OpenCtrlSeries( dvrprms, ErrStat2, ErrMsg2 )
   call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName)  
   if ( errStat2 >= AbortErrLev ) then
      call WrScr( trim(errMsg2) )
      return
   end if

   call KFC_Init( InitInp, u, p, y, interval, m, o, InitOut, errStat2, errMsg2 )
   call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName)     
   if ( errStat2 >= AbortErrLev ) then
            call WrScr( trim(errMsg2) )
            return
         end if
   
      ! Set inputs first time step
   call Set_u( dvrprms, InitInp, u, t  ) 
      
 
   ! Time marching loop
   DO 
      CALL KFC_RdCtrlDataLine( dvrprms, ErrStat2, ErrMsg2)         
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName)  
      IF ( errStat >= AbortErrLev ) then
            call WrScr( trim(errMsg) )
            return
      ELSEIF  ( errStat <0 ) then ! EOF reached need to close things
         RETURN
      END IF
         ! Generate the inputs for t+dt
      CALL Set_u( dvrprms, InitInp, u, t )
         
      ! Step the controller
      CALL KFC_Step(t, u, p, y, m, o, errStat2, errMsg2)
       call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName) 
         if ( errStat >= AbortErrLev ) then
            call WrScr( trim(errMsg) )
            return
         end if

   end do

   call KFC_End(p, errStat2, errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName)
    
   if ( errStat >= ErrID_None ) call WrScr( trim(errMsg) )
            
END subroutine KFC_Dvr_Simulate
!====================================================================================================

SUBROUTINE KFC_OpenCtrlSeries( dvrprms, ErrStat, ErrMsg )
   ! This subroutine initializes the controller time series input data file for reading,
   ! contains valid names, and opening the output file if there are any requested outputs
   !----------------------------------------------------------------------------------------------------
      TYPE(KFC_Dvr_InitInputType),        INTENT( INOUT ) :: dvrprms      ! data file path+name in, file unit  out
      INTEGER,                       INTENT(   OUT ) :: ErrStat              ! a non-zero value indicates an error occurred           
      CHARACTER(*),                  INTENT(   OUT ) :: ErrMsg               ! Error message if ErrStat /= ErrID_None
      
         ! Local variables
      INTEGER                                        :: ErrStat2      
      CHARACTER(ErrMsgLen)                           :: errMsg2          ! temporary Error message if errStat /= ErrID_None        
      character(*), parameter                       :: routineName = 'KFC_OpenCtrlSeries'
      !-------------------------------------------------------------------------------------------------      
      ! Initialize local variables
      !-------------------------------------------------------------------------------------------------      
      ErrStat = ErrID_None  
      ErrMsg  = ""
       
      !-------------------------------------------------------------------------------------------------      
      ! Open the data file
      !-------------------------------------------------------------------------------------------------      
      
      print *, ">>>>>>>> RRD_Debug: In ",routineName," CtrlFile=", trim(dvrprms%DrvCtrlInputs)," <<<<<<<<<<<<<<<<<<<<< \n"  
     
      CALL GetNewUnit( dvrprms%UnCtrlFile )
   
      CALL OpenFInpFile ( dvrprms%UnCtrlFile, dvrprms%DrvCtrlInputs, ErrStat2, ErrMsg2) 
         CALL SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if ( errStat >= AbortErrLev ) then
         ErrMsg = ' Error opening time series input data file: '//TRIM(ErrMsg)
         close(dvrprms%UnCtrlFile)
         return
      end if
      
      
      dvrprms%CtrlLineLength  =  47      !t_c,rho_c,Xg_c(3),Vg_c(3),Ag_c(3),acc_norm_c,Vb_c(3),Ab_c(3),pqr_c(3),wind_g_c(3),apparent_wind_c(3), tether_forceb_c(3), dcm_g2b_c(9), aeroTorq(8)
      Call AllocAry( dvrprms%OneCtrlLine, dvrprms%CtrlLineLength, 'dvrprms%OneCtrlLine', errStat2, errMsg2 )
         Call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
      !Then Call Readoneliner for t=0 ctrl info (skip first lines) and rewind file for the next kfc_step still at t=0
      Call ReadCom( dvrprms%UnCtrlFile, dvrprms%DrvCtrlInputs, 'KiteFastController Ctrl Input Time-Series data file title (line 1)', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      Call ReadCom( dvrprms%UnCtrlFile, dvrprms%DrvCtrlInputs, 'KiteFastController Ctrl Input Time-Series data file subtitle (line 2)', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      Call ReadCom( dvrprms%UnCtrlFile, dvrprms%DrvCtrlInputs, 'KiteFastController Ctrl Input Time-Series data file headers (line 3)', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      Call ReadCom( dvrprms%UnCtrlFile, dvrprms%DrvCtrlInputs, 'KiteFastController Ctrl Input Time-Series data file: Units header line (line 4)', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            return
         end if
   
      CALL KFC_RdCtrlDataLine( dvrprms,  ErrStat, ErrMsg)
      BackSpace( dvrprms%UnCtrlFile, IOSTAT=errStat2 )  
      if (errStat2 /= 0_IntKi ) then
         call SetErrStat( ErrID_Fatal, 'Error rewinding file "'//trim(dvrprms%DrvCtrlInputs)//'".', errStat, errMsg, RoutineName )
         call Cleanup()
         return
      end if           
         !print *, ">>>>>>>> RRD_Debug: End of ",routineName,"  <<<<<<<<<<<<<<<<<<<<<"
                 !====================================================================================================
      contains
      subroutine Cleanup()
         !     The routine closes the file in case of problems
            print *, ">>>>>RRD_debug: In ",RoutineName," ErrMsg=",errMsg
            close(dvrprms%UnCtrlFile)
      end subroutine Cleanup      

   END SUBROUTINE KFC_OpenCtrlSeries
!====================================================================================================

   SUBROUTINE KFC_RdCtrlDataLine( dvrprms, ErrStat, ErrMsg)
      ! This routine reads the data from control data file when called and assigns them to structure "o"
      !-----------------------------------------------------------------------------------------------
         IMPLICIT                        NONE
         
         TYPE(KFC_Dvr_InitInputType),        INTENT( INOUT ) :: dvrprms      ! data file path+name in,

         INTEGER(IntKi),           INTENT(OUT)   :: ErrStat                            !< Error status
         CHARACTER(*),             INTENT(OUT)   :: ErrMsg                             !< Error message
      
            ! Local variables.
         
         INTEGER                                        :: ErrStat2  
         CHARACTER(ErrMsgLen)                           :: errMsg2        ! The error message, if an error occurred
         character(*), parameter                       :: routineName = 'KFC_RdCtrlDataLine'
         ErrStat = ErrID_None
         ErrMsg  = ''
         
         
         CALL ReadAry( dvrprms%UnCtrlFile, dvrprms%DrvCtrlInputs, dvrprms%OneCtrlLine, dvrprms%CtrlLineLength, "OneCtrlLine", "Array of controls t_c,rho_c,Xg_c,...AeroTroques ", ErrStat2, ErrMsg2)
         
         CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
         IF ( ErrStat >= AbortErrLev ) THEN
            
            CALL Cleanup()
            RETURN
         ELSEIF ( ErrStat <0 ) THEN  
            print *, "END OF FILE REACHED In "//routineName//" reading "//dvrprms%DrvCtrlInputs  
            CALL Cleanup()
            RETURN
         END IF
         
         !print *, ">>>>>>RRD_Debug:  In ", routineName, " read-in t=", o%OneCtrlLine(1)

         contains
         subroutine Cleanup()
            !     The routine closes the file in case of problems
               close(dvrprms%UnCtrlFile)
         end subroutine Cleanup      
      
      END SUBROUTINE KFC_RdCtrlDataLine
!====================================================================================================

      SUBROUTINE Set_u( dvrprms, InitInp,  u, myt )
         
         TYPE(KFC_Dvr_InitInputType),   INTENT( IN ) :: dvrprms      ! data file path+name in,
         type(KFC_InputType),           intent(INOUT   )  :: u           !< Inputs at Time t
         TYPE(KFC_InitInputType),   INTENT( IN ) :: InitInp      ! Initialization INput structure
         real(DbKi),                    intent(OUT   )  :: myt           !< Current simulation time in seconds
         !Local
         character(*), parameter                       :: routineName = 'Set_u'
         integer(IntKi)            :: nrotors, cp,cs,i,j  ! counters

         !t_c,rho_c,Xg_c(3),Vg_c(3),Ag_c(3),acc_norm_c,Vb_c(3),Ab_c(3),pqr_c(3),wind_g_c(3),apparent_wind_c(3), tether_forceb_c(3), dcm_g2b_c(9), aeroTorq(8)
         myt = dvrprms%OneCtrlLine(1)
         u%rho = dvrprms%OneCtrlLine(2) 
         u%Xg = dvrprms%OneCtrlLine(3:5)
         u%Vg = dvrprms%OneCtrlLine(6:8)
         u%Ag = dvrprms%OneCtrlLine(9:11)
         u%acc_norm = dvrprms%OneCtrlLine(12)
         u%Vb = dvrprms%OneCtrlLine(13:15)
         u%Ab = dvrprms%OneCtrlLine(16:18)
         u%pqr= dvrprms%OneCtrlLine(19:21)
         u%wind_g= dvrprms%OneCtrlLine(22:24)
         u%apparent_wind= dvrprms%OneCtrlLine(25:27)
         u%tether_forceb= dvrprms%OneCtrlLine(28:30)
         u%dcm_g2b= reshape(dvrprms%OneCtrlLine(31:39),(/3,3/))
         !Aerotorq= dvrprms%OneCtrlLine(40:47)
         
         nrotors=4*InitInp%numPylons
         !because of the way KFAST has the ordering I need to provide SPy and PPy
         do j=1,InitInp%numPylons
            do i=1,2
               cp=(i-1)*(InitInp%numPylons+j)   + (2-i)*(nRotors+1-j-InitInp%numPylons)
               cs=(i-1)*(InitInp%numPylons-j+1) + (2-i)*(nRotors-InitInp%numPylons+j)
               
               u%PPyAeroTorque(i,j) = dvrprms%OneCtrlLine(cp+39)
               u%SPyAeroTorque(i,j) = dvrprms%OneCtrlLine(cs+39)
   
            end do   
         end do
   
      end SUBROUTINE Set_u
      
end module KFC_Dvr_Subs