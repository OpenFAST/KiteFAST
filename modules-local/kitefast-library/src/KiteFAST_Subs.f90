module KiteFAST_Subs
   
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
   use KiteFAST_GlobalData
   
   implicit none 

public

   contains
   
subroutine TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
   integer(IntKi),         intent(in   ) :: errStat    ! Error status of the operation
   character(*),           intent(inout) :: errMsg     ! Error message if ErrStat /= ErrID_None
   integer(C_INT),         intent(inout) :: errStat_c      
   character(kind=C_CHAR), intent(inout) :: errMsg_c(IntfStrLen)   

              ! transfer Fortran variables to C:  
      errStat_c = errStat
      errMsg    = trim(errMsg)//C_NULL_CHAR
      errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )
      
end subroutine TransferErrors 

subroutine DumpMotionsMeshData(mesh, meshLabel)
type(MeshType),  intent(in)  :: mesh              !< The motions mesh 
character(*)  ,  intent(in)  :: meshLabel

integer(IntKi) :: i

do i = 1, mesh%NNodes
   call WrScr(trim(meshLabel)//": Node "//trim(num2lstr(i))//" Position = "//trim(num2lstr(mesh%Position(1,i)))//", "//trim(num2lstr(mesh%Position(2,i)))//", "//trim(num2lstr(mesh%Position(3,i))) )
end do

end subroutine

subroutine CreateMBDynPtfmMeshes(PtfmO, PtfmODCM, mbdPtfmMotions, mbdPtfmLoads, errStat, errMsg )
   real(ReKi),                   intent(in   )  :: PtfmO(3)          !< Reference position for the platform reference point in global coordinates
   real(R8Ki),                   intent(in   )  :: PtfmODCM(3,3)     !< DCM needed to transform into the platform reference axes
   type(MeshType),               intent(  out)  :: mbdPtfmMotions    !< The resulting mesh 
   type(MeshType),               intent(  out)  :: mbdPtfmLoads      !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   integer(intKi)                               :: j                 ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateMBDynPtLoadsMesh'

      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   if (errStat >= AbortErrLev) return
   
   !=====================================
   ! MBDyn Platform Motions Mesh
   !=====================================
   call MeshCreate ( BlankMesh = mbdPtfmMotions     &
                     ,IOS       = COMPONENT_INPUT &
                     ,Nnodes    = 1               &
                     ,errStat   = errStat2        &
                     ,ErrMess   = errMsg2         &
                     ,Orientation     = .true.    &
                     ,TranslationDisp = .true.    &
                     ,TranslationVel  = .true.    &
                     ,RotationVel     = .true.    &
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation      
   position =0.0_ReKi 

   call MeshPositionNode(mbdPtfmMotions, 1, position, errStat2, errMsg2)  
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         

         
   
   call MeshConstructElement( mbdPtfmMotions, ELEMENT_POINT, errStat2, errMsg2, p1=1 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
  
            
   call MeshCommit(mbdPtfmMotions, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return

   mbdPtfmMotions%Orientation(:,:,1)     = PtfmODCM
   mbdPtfmMotions%TranslationDisp(:,1)   = PtfmO
     
   mbdPtfmMotions%TranslationVel  = 0.0_ReKi
   mbdPtfmMotions%RotationVel     = 0.0_ReKi
   
   
   !=====================================
   ! MBDyn Platform Loads Mesh
   !=====================================

   call MeshCreate ( BlankMesh = mbdPtfmLoads     &
                     ,IOS         = COMPONENT_OUTPUT &
                     ,Nnodes      = 1               &
                     ,errStat     = errStat2        &
                     ,ErrMess     = errMsg2         &
                     ,force       = .true.          &
                     ,moment      = .true.          &
                     ,translationdisp = .true.      &
                     )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
         ! set node initial position/orientation
   
   call MeshPositionNode(mbdPtfmLoads, 1, position, errStat2, errMsg2, orientation)  
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call MeshConstructElement( mbdPtfmLoads, ELEMENT_POINT, errStat2, errMsg2, p1=1 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )   
       
   call MeshCommit(mbdPtfmLoads, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return
      
   mbdPtfmLoads%Force           = 0.0_ReKi
   mbdPtfmLoads%Moment          = 0.0_ReKi
   mbdPtfmLoads%TranslationDisp(:,1)   = PtfmO  - mbdPtfmLoads%Position(:,1)


end subroutine CreateMBDynPtfmMeshes

subroutine SetupSim(dt, modFlags, gravity, outFileRoot_c, numOutChan_c, chanlist_c, chanlist_len_c, p, OutList, errStat, errMsg)

   real(DbKi),                intent(in   ) :: dt                             ! Timestep size (s)
   integer(C_INT),            intent(in   ) :: modFlags(:)                    ! array of module flags.  0 = inactive, 1 = active. Module indices are: 0 = KiteAeroDyn, 1 = InflowWind, 2 = MoorDyn-Tether, 3 = KiteFAST Controller
   real(C_DOUBLE),            intent(in   ) :: gravity                        ! Scalar gravity constant.  (m/s^2)
   character(kind=C_CHAR),    intent(in   ) :: outFileRoot_c(IntfStrLen)      ! Full path and basename of the KiteFAST output file.
   integer(C_INT),            intent(in   ) :: numOutChan_c                   ! Number of user-requested output channel names
   type(c_ptr)   ,target,     intent(in   ) :: chanlist_c(numOutChan_c)       ! Array of output channel names (strings)
   integer(C_INT),            intent(in   ) :: chanlist_len_c(numOutChan_c )  ! The length of each string in the above array

   type(KFAST_ParameterType), intent(inout) :: p
   CHARACTER(ChanLen),        intent(inout) :: OutList(:)             ! List of requested output channels: MaxOutPts is defined in KiteFAST_IO.f90, ChanLen defined in NWTC_Base.f90
   integer(IntKi),            intent(  out) :: errStat                        ! Error status of the operation
   character(*),              intent(  out) :: errMsg                         ! Error message if errStat /= ErrID_None

   character, pointer                       :: chanName_f(:)                  ! Fortran version of channel name
   integer(IntKi)                           :: i, lenstr
   character(ChanLen)                       :: tmpStr

   errStat = ErrID_None
   errMsg  = ''
   
   
   
         ! Initialize the NWTC Subroutine Library
   call NWTC_Init( EchoLibVer=.FALSE. )

   OtherSt%doStartupInterp = .true.
   
   ! If a module's input file/dll file is empty, then turn off that module
   
   if ( modFlags(1) > 0 ) then
      p%useKAD = .true.
   else
      p%useKAD = .false.
   end if
   
   
   if ( modFlags(2) > 0 ) then
      p%useIfW = .true.
   else
      p%useIfW = .false.
   end if
   
   if ( modFlags(3) > 0 ) then
      p%useMD_Tether = .true.
   else
      p%useMD_Tether = .false.
   end if
   
   if ( modFlags(4) > 0 ) then
      p%useKFC = .true.
   else
      p%useKFC = .false.
   end if
 
      ! Set KiteFAST parameters which apply to all configurations
   p%dt = dt
   p%Gravity = gravity
   if ( gravity < 0.0_ReKi ) call SetErrStat(ErrID_Fatal,'Scalar gravity value must be greater than or equal to zero',errStat,errMsg,'SimSetup')
      
      
   p%outFileRoot = transfer(outFileRoot_c(1:IntfStrLen-1),p%outFileRoot)
   call RemoveNullChar(p%outFileRoot)

   ! Initialize some kite data to zeros in case we end up not using a kite
   p%numPylons = 0
   p%numFlaps  = 0
   p%numFusNds = 0
   p%numSwnNds = 0
   p%numPWnNds = 0
   p%numVSNds  = 0
   p%numSHSNds = 0
   p%numPHSNds = 0
   
   !  TRANSFER OF OUTPUT CHANNEL NAMES
   p%numKFASTOuts = numOutChan_c
   do i = 1,numOutChan_c
      
      call c_f_pointer(chanlist_c(i),chanName_f, [chanlist_len_c(i)])
     ! write(*,*) chanName_f
      lenstr = cstrlen(chanName_f)
      tmpStr = transfer(chanName_f(1:lenstr-1),tmpStr)
      OutList(i) = tmpStr(1:lenstr-1)
     ! write(*,*) trim(OutList(i)),'T'
   end do
   
end subroutine SetupSim

!====================================================================================================
subroutine KFAST_SetNumOutputs( p, KAD_InitOut, MD_InitOut, IfW_InitOut, errStat, errMsg )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   type(KFAST_ParameterType),     intent( inout ) :: p   
   type(KAD_InitOutPutType ),     intent( in    ) :: KAD_InitOut              !
   type(MD_InitOutPutType ),      intent( in    ) :: MD_InitOut              !
   type(InflowWind_InitOutPutType ),     intent( in    ) :: IfW_InitOut              !
   integer,                       intent(   out ) :: ErrStat              ! a non-zero value indicates an error occurred           
   character(*),                  intent(   out ) :: ErrMsg               ! Error message if ErrStat /= ErrID_None
   
 
   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   ErrStat = ErrID_None  
   ErrMsg  = ""
   
   if ( allocated( KAD_InitOut%WriteOutputHdr ) ) then
      p%numKADOuts = size(KAD_InitOut%WriteOutputHdr)
   else
      p%numKADOuts = 0
   end if
   if ( allocated( MD_InitOut%WriteOutputHdr ) ) then
      p%numMDOuts = size(MD_InitOut%WriteOutputHdr)
   else
      p%numMDOuts = 0
   end if
   if ( allocated( IfW_InitOut%WriteOutputHdr ) ) then
      p%numIfWOuts = size(IfW_InitOut%WriteOutputHdr)
   else
      p%numIfWOuts = 0
   end if
   p%numOuts = p%numKFASTOuts + p%numKADOuts + p%numMDOuts + p%numIfWOuts
   
end subroutine KFAST_SetNumOutputs

!====================================================================================================
subroutine KFAST_OpenOutput( ProgVer, OutRootName, p, errStat, errMsg )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   type(ProgDesc),                intent( in    ) :: ProgVer
   character(*),                  intent( in    ) :: OutRootName          ! Root name for the output file
   type(KFAST_ParameterType),     intent( inout ) :: p   
   integer,                       intent( inout ) :: errStat              ! a non-zero value indicates an error occurred           
   character(*),                  intent( inout ) :: errMsg               ! Error message if ErrStat /= ErrID_None
   
      ! Local variables  
   character(1024)                                :: OutFileName          ! The name of the output file  including the full path.
   integer                                        :: errStat2       
   character(ErrMsgLen)                           :: errMsg2
   character(1024)                                :: gitVersionStr
   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   errStat = ErrID_None  
   errMsg  = ""
   
   !-------------------------------------------------------------------------------------------------      
   ! Open the output file, if necessary, and write the header
   !-------------------------------------------------------------------------------------------------              
      
   p%OutSFmt = "A10"  
   p%OutFmt  = "ES10.3E2"
   p%Delim = ' '
   gitVersionStr = QueryGitVersion()

   
      ! Open the file for output
   OutFileName = trim(OutRootName)//'.out'
   call GetNewUnit( p%UnOutFile )
   
   call OpenFOutFile ( p%UnOutFile, OutFileName, errStat2, errMsg2 ) 
   if ( errStat2 >= AbortErrLev ) then
      call SetErrStat(errStat2,' Error opening KiteFAST output file: '//trim(errMsg2), errStat, errMsg, 'KFAST_OpenOutput')
      return
   end if
      
       
      ! Write the output file header
      
   write (p%UnOutFile,'(/,A/)', IOSTAT=errStat2)  'These predictions were generated by '//trim(GETNVD(ProgVer))//&
                     ' on '//CurDate()//' at '//CurTime()//' using commit: '//trim(gitVersionStr)//'.'
      
   write(p%UnOutFile, '(//)') ! add 3 lines to make file format consistant with FAST v8 (headers on line 7; units on line 8) [this allows easier post-processing]


end subroutine KFAST_OpenOutput
!====================================================================================================
subroutine KFAST_WriteOutputTimeChanName( UnOutFile )
! This subroutine writes the Time channel name without new line character(s)
!----------------------------------------------------------------------------------------------------

   integer(IntKi),  intent(in) :: UnOutFile
  
      ! Write the names of the output channels:   
   call WrFileNR ( p%UnOutFile, '   Time        ' )
   
end subroutine KFAST_WriteOutputTimeChanName
!====================================================================================================
subroutine KFAST_WriteOutputTimeChanUnits( UnOutFile )
! This subroutine writes the Time channel name without new line character(s)
!----------------------------------------------------------------------------------------------------

   integer(IntKi),  intent(in) :: UnOutFile
   
      ! Write the names of the output channels:   
   call WrFileNR ( p%UnOutFile, '    s          ' )
   
end subroutine KFAST_WriteOutputTimeChanUnits

!====================================================================================================
subroutine KFAST_WriteOutputChanNames( p, KAD_InitOut, MD_InitOut, IfW_InitOut )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   type(KFAST_ParameterType),        intent( inout ) :: p   
   type(KAD_InitOutPutType ),        intent( in    ) :: KAD_InitOut              !
   type(MD_InitOutPutType ),         intent( in    ) :: MD_InitOut              !
   type(InflowWind_InitOutPutType ), intent( in    ) :: IfW_InitOut              !

   
      ! Local variables
   integer                                        :: i                    ! Generic loop counter      
            

   if ( p%numOuts > 0 ) then   ! Output has been requested 
      
         ! Write the names of the output channels:   
      
      if ( p%numKFASTOuts > 0 ) then
         do i=1,p%numKFASTOuts
            call WrFileNR ( p%UnOutFile, p%Delim//p%OutParam(i)%Name )
         end do ! I
      end if
      if ( p%numKADOuts > 0 ) then
         do i=1,p%numKADOuts
            call WrFileNR ( p%UnOutFile, p%Delim//KAD_InitOut%WriteOutputHdr(i) )
         end do ! I
      end if
      if ( p%numIfWOuts > 0 ) then
         do i=1,p%numIfWOuts
            call WrFileNR ( p%UnOutFile, p%Delim//IfW_InitOut%WriteOutputHdr(i) )
         end do ! I
      end if
      if ( p%numMDOuts > 0 ) then
         do i=1,p%numMDOuts
            call WrFileNR ( p%UnOutFile, p%Delim//MD_InitOut%WriteOutputHdr(i) )
         end do ! I
      end if

      ! Intentionally did not write end of line character(s)
      
   end if   ! there are any requested outputs   

end subroutine KFAST_WriteOutputChanNames

!====================================================================================================
subroutine KFAST_WriteOutputUnitNames( p, KAD_InitOut, MD_InitOut, IfW_InitOut )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   type(KFAST_ParameterType),        intent( inout ) :: p   
   type(KAD_InitOutPutType ),        intent( in    ) :: KAD_InitOut             !
   type(MD_InitOutPutType ),         intent( in    ) :: MD_InitOut              !
   type(InflowWind_InitOutPutType ), intent( in    ) :: IfW_InitOut             !
   
      ! Local variables
   integer                                        :: i                    ! Generic loop counter      

   
            
   if ( p%numOuts > 0 ) then   ! Output has been requested 
      
   
         ! Write the names of the output channel units:
      
      
      
      if ( p%numKFASTOuts > 0 ) then
         do i=1,p%numKFASTOuts
            call WrFileNR ( p%UnOutFile, p%Delim//p%OutParam(i)%Units )
         end do ! I
      end if
      if ( p%numKADOuts > 0 ) then
         do i=1,p%numKADOuts
            call WrFileNR ( p%UnOutFile, p%Delim//KAD_InitOut%WriteOutputUnt(i) )
         end do ! I
      end if
      if ( p%numIfWOuts > 0 ) then
         do i=1,p%numIfWOuts
            call WrFileNR ( p%UnOutFile, p%Delim//IfW_InitOut%WriteOutputUnt(i) )
         end do ! I
      end if
      if ( p%numMDOuts > 0 ) then
         do i=1,p%numMDOuts
            call WrFileNR ( p%UnOutFile, p%Delim//MD_InitOut%WriteOutputUnt(i) )
         end do ! I
      end if
      
      ! Intentionally did not write end of line character(s)

   end if   ! there are any requested outputs   

end subroutine KFAST_WriteOutputUnitNames

subroutine KFAST_WriteOutputNL( UnFile )

      ! Passed variables

   integer(IntKi),        intent( in  ) :: UnFile   
   
   write (UnFile,'()')
   
end subroutine KFAST_WriteOutputNL

!====================================================================================================
subroutine KFAST_WriteOutputTime( t, UnOutFile )
! This subroutine 
!----------------------------------------------------------------------------------------------------

      ! Passed variables
   real(DbKi),                    intent( in    ) :: t
   integer(IntKi),                intent( in    ) :: UnOutFile   

   character(200)                                 :: Frmt          ! A string to hold a format specifier
   character(14)                                  :: TmpStr        ! temporary string to print the time output as text


            ! Write one line of tabular output:
   Frmt = '"'//p%Delim//'"'//p%OutFmt      ! format for array elements from individual modules

         ! time
   write( tmpStr, '(F10.6)' ) t
   call WrFileNR( p%UnOutFile, tmpStr )
      
end subroutine KFAST_WriteOutputTime


!====================================================================================================
subroutine KFAST_WriteOutput( p, y_KAD, y_MD, y_IfW, errStat, errMsg )
! This subroutine 
!----------------------------------------------------------------------------------------------------

      ! Passed variables
   type(KFAST_ParameterType),     intent( inout ) :: p   
   type(KAD_OutPutType ),         intent( in    ) :: y_KAD         !
   type(MD_OutPutType ),          intent( in    ) :: y_MD          !
   type(InflowWind_OutPutType ),  intent( in    ) :: y_IfW         !
   integer,                       intent(   out ) :: errStat       ! a non-zero value indicates an error occurred           
   character(*),                  intent(   out ) :: errMsg        ! Error message if ErrStat /= ErrID_None
   
      ! Local variables
   integer                                        :: errStat2              
   character(ErrMsgLen)                           :: errMsg2       ! error messages
   character(200)                                 :: Frmt          ! A string to hold a format specifier
   character(14)                                  :: TmpStr        ! temporary string to print the time output as text

   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   errStat = ErrID_None  
   errMsg  = ""

   if ( p%numOuts > 0 ) then   ! Output has been requested so let's write to the output file            
 
              ! Write one line of tabular output:
      Frmt = '"'//p%Delim//'"'//p%OutFmt      ! format for array elements from individual modules

         ! write the individual module output (convert to SiKi if necessary, so that we don't need to print so many digits in the exponent)
      
      if (p%numKFASTOuts > 0 ) then  
         call WrNumAryFileNR ( p%UnOutFile, real(m%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      endif
      
      if ( p%numKADOuts > 0 ) then
        call WrNumAryFileNR ( p%UnOutFile, real(y_KAD%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      end if
      
      if ( p%numIfWOuts > 0 ) then
            call WrNumAryFileNR ( p%UnOutFile, real(y_IfW%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      end if
      
      if ( p%numMDOuts > 0 ) then
            call WrNumAryFileNR ( p%UnOutFile, real(y_MD%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      end if  

   end if   ! are there any requested outputs   

   return
   
end subroutine KFAST_WriteOutput


subroutine SumModuleStatus( p, enabledModules, disabledModules )
   type(KFAST_ParameterType),     intent( in    ) :: p   
   character(255),                intent(   out ) :: enabledModules
   character(255),                intent(   out ) :: disabledModules

   enabledModules  = ' '
   disabledModules = ' '
   
   if (p%useKAD) then
      enabledModules  = ' KiteAeroDyn'
   else
      disabledModules = ' KiteAeroDyn'
   end if
   
   if (p%useIfW) then
      enabledModules  = trim(enabledModules)//', InflowWind'
   else
      disabledModules = trim(disabledModules)//', InflowWind'
   end if
   
   if (p%useMD_Tether) then
      enabledModules  = trim(enabledModules)//', MoorDyn (Tether)'
   else
      disabledModules = trim(disabledModules)//', MoorDyn (Tether)'
   end if
   if (p%useKFC) then
      enabledModules  = trim(enabledModules)//', KiteFastController'
   else
      enabledModules  = trim(enabledModules)//', Dummy KiteFastController'   
   end if
   
   if ( scan(enabledModules(1:1),',') > 0 ) enabledModules = enabledModules(3:255)
   if ( scan(disabledModules(1:1),',') > 0 ) disabledModules = disabledModules(3:255)
   
end subroutine SumModuleStatus

subroutine WriteNodeInfo(SumFileUnit, CompIndx, nNds, Pts, NdDCMs, FusO, RefPt, NOuts, OutNds, errStat, errMsg )
   integer(IntKi),                  intent(in   ) :: SumFileUnit          ! the unit number for the summary file
   integer(IntKi),                  intent(in   ) :: CompIndx
   integer(IntKi),                  intent(in   ) :: nNds
   real(ReKi),                      intent(in   ) :: Pts(:,:)
   real(R8Ki),                      intent(in   ) :: NdDCMs(:,:,:)
   real(ReKi),                      intent(in   ) :: FusO(3)              ! Fuselage reference point in global coordinates
   real(ReKi),                      intent(in   ) :: RefPt(3)             ! Component reference point in kite coordinates
   integer(IntKi),                  intent(in   ) :: NOuts
   integer(IntKi),                  intent(in   ) :: OutNds(:)
   integer(IntKi),                  intent(  out) :: errStat              ! Error status of the operation
   character(*),                    intent(  out) :: errMsg               ! Error message if ErrStat /= ErrID_None
   
      ! Local variables
   integer                                        :: k,l                  ! Generic loop counter      

   integer(IntKi)                                 :: TmpErrStat           ! Temporary error status for checking how the WRITE worked
   character(32)                                  :: Components(12)
   character(1)                                   :: NoValStr = '-'
   character(4)                                   :: OutNumStr
   character(19)                                  :: NodeType(3)
   character(3)                                   :: CompAbrv(6)
   real(ReKi)                                     :: globalPt(3), kitePt(3), kitePt2(3)

   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   ErrStat = ErrID_None  
   ErrMsg  = ""
   Components = (/'Fuselage                        ', &
                  'Starboard wing                  ', &
                  'Port wing                       ', &
                  'Vertical stabilizer             ', &
                  'Starboard horizontal stabilizer ', &
                  'Port horizontal stabilizer      ', &
                  'Starboard pylon                 ', &
                  'Port pylon                      ', &
                  'Top rotor on starboard pylon    ', &
                  'Bottom rotor on starboard pylon ', &
                  'Top rotor on port pylon         ', &
                  'Bottom rotor on port pylon      '/)
   NodeType = (/'Reference point    ', &
                'Finite-element node', & 
                'Gauss point        '/)

   CompAbrv = (/'Fus', 'SWn', 'PWn', ' VS', 'SHS', 'PHS'/)
   
      ! reference point
   write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) Components(CompIndx), NodeType(1), NoValStr, NoValStr, RefPt(1), RefPt(2), RefPt(3)
      
      ! finite-element points
   do k = 1,nNds
      globalPt = Pts(:,k) - FusO
      kitePt = matmul(NdDCMs(:,:,k), globalPt)
      OutNumStr = ' -  '
      do l= 1,NOuts
         if ( k == OutNds(l) ) then
            OutNumStr = trim(CompAbrv(CompIndx))//Num2LStr(l)
            continue
         end if
      end do
         
      write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,9X,A4,6X,3(F7.3,1X))',IOSTAT=TmpErrStat) Components(CompIndx), NodeType(2), Num2LStr(k), OutNumStr, kitePt(1), kitePt(2), kitePt(3)
   end do
      
      ! gauss points
   do k = 1,nNds-1
      OutNumStr = ' -  '
      do l= 1,NOuts
         if ( k == OutNds(l) ) then
            OutNumStr = trim(CompAbrv(CompIndx))//Num2LStr(l)
            continue
         end if
      end do
      globalPt = Pts(:,k) - FusO
      kitePt = matmul(NdDCMs(:,:,k), globalPt)
      globalPt = Pts(:,k+1) - FusO
      kitePt2 = matmul(NdDCMs(:,:,k+1), globalPt)
      if ( mod(k,2) == 1 ) then
         kitePt = (1.0-sqrt(3.0)/3.0)*kitePt2 + (sqrt(3.0)/3.0)*kitePt
      else
         kitePt = (1.0-sqrt(3.0)/3.0)*kitePt + (sqrt(3.0)/3.0)*kitePt2
      end if
         
      write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,9X,A4,6X,3(F7.3,1X))',IOSTAT=TmpErrStat) Components(CompIndx), NodeType(3), Num2LStr(k), OutNumStr, kitePt(1), kitePt(2), kitePt(3)
   end do   
      


end subroutine WriteNodeInfo

!====================================================================================================
subroutine KFAST_OpenSummary( Prog, OutRootName, enabledModules, disabledModules, dt, SumFileUnit, errStat, errMsg )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   type(ProgDesc),                intent( in    ) :: Prog
   character(*),                  intent( in    ) :: OutRootName          ! Root name for the output file
   character(255),                intent( in    ) :: enabledModules
   character(255),                intent( in    ) :: disabledModules
   real(DbKi),                    intent( in    ) :: dt
   integer(IntKi),                intent(   out ) :: SumFileUnit          ! the unit number for the summary file
   integer,                       intent(   out ) :: errStat              ! a non-zero value indicates an error occurred           
   character(*),                  intent(   out ) :: errMsg               ! Error message if ErrStat /= ErrID_None
   
      ! Local variables   
   character(1024)                                :: OutFileName          ! The name of the output file  including the full path.
   integer                                        :: errStat2              
   character(ErrMsgLen)                           :: errMsg2              ! error messages
   integer(IntKi)                                 :: TmpErrStat           ! Temporary error status for checking how the WRITE worked

   character(1024)                                :: gitVersionStr
   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   ErrStat = ErrID_None  
   ErrMsg  = ""
   
   gitVersionStr = QueryGitVersion()
   SumFileUnit = -1
   call GetNewUnit( SumFileUnit )
   OutFileName = trim(OutRootName)//'.sum'
   call OpenFOutFile ( SumFileUnit, OutFileName, errStat2, errMsg2 )
      call SetErrStat(errStat2,' Error opening KiteFAST summary file: '//trim(errMsg2), errStat, errMsg, 'KFAST_OpenSummary')

   if (ErrStat >=AbortErrLev) return


         ! Write the summary file header
   write(SumFileUnit,'(/,A/)',IOSTAT=TmpErrStat)   'This summary file was generated by '//trim( Prog%Name )//&
                     ' '//trim( Prog%Ver )//' on '//CurDate()//' at '//CurTime()//' using commit: '//trim(gitVersionStr)//'.'
   
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) 'Enabled  modules: '//trim(enabledModules)
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) 'Disabled modules: '//trim(disabledModules)
   write(SumFileUnit,'()')
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) 'Time step (s): '//num2lstr(dt)
   write(SumFileUnit,'()')

end subroutine KFAST_OpenSummary


!====================================================================================================
subroutine KFAST_WriteSummary( SumFileUnit, p, m, KAD_InitOut, MD_InitOut, IfW_InitOut, errStat, errMsg )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   integer(IntKi),                intent( in    ) :: SumFileUnit          ! the unit number for the summary file

   type(KFAST_ParameterType),     intent( inout ) :: p   
   type(KFAST_MiscVarType),       intent( in    ) :: m  
   type(KAD_InitOutPutType ),     intent( in    ) :: KAD_InitOut              !
   type(MD_InitOutPutType ),      intent( in    ) :: MD_InitOut              !
   type(InflowWind_InitOutPutType ),     intent( in    ) :: IfW_InitOut              !
   integer,                       intent(   out ) :: ErrStat              ! a non-zero value indicates an error occurred           
   character(*),                  intent(   out ) :: ErrMsg               ! Error message if ErrStat /= ErrID_None
   
      ! Local variables
   integer                                        :: i,k,l                  ! Generic loop counter      
   character(1024)                                :: OutFileName          ! The name of the output file  including the full path.
   integer                                        :: errStat2              
   character(ErrMsgLen)                           :: errMsg2              ! error messages
   integer(IntKi)                                 :: TmpErrStat           ! Temporary error status for checking how the WRITE worked
   character(32)                                  :: Components(12)
   character(1)                                   :: NoValStr = '-'
   character(4)                                   :: OutNumStr
   character(19)                                  :: NodeType(3)
   real(ReKi)                                     :: xloc, yloc, zloc
   real(ReKi)                                     :: globalPt(3), kitePt(3), kitePt2(3)


   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   ErrStat = ErrID_None  
   ErrMsg  = ""
   Components = (/'Fuselage                        ', &
                  'Starboard wing                  ', &
                  'Port wing                       ', &
                  'Vertical stabilizer             ', &
                  'Starboard horizontal stabilizer ', &
                  'Port horizontal stabilizer      ', &
                  'Starboard pylon                 ', &
                  'Port pylon                      ', &
                  'Top rotor on starboard pylon    ', &
                  'Bottom rotor on starboard pylon ', &
                  'Top rotor on port pylon         ', &
                  'Bottom rotor on port pylon      '/)
   NodeType = (/'Reference point    ', &
                'Finite-element node', & 
                'Gauss point        '/)
   xloc = 0.0
   yloc = 0.0
   zloc = 0.0
   
   
   !write(SumFileUnit,'(A20)',IOSTAT=TmpErrStat) '   compiled with:   '
   !write(SumFileUnit,'(A1)',IOSTAT=TmpErrStat) ''
   !write(SumFileUnit,'(A39)',IOSTAT=TmpErrStat) 'Description from the MDyn input file:  '
   !write(SumFileUnit,'(A1)',IOSTAT=TmpErrStat) ''
   

   
   write(SumFileUnit,'(A90)' ,IOSTAT=TmpErrStat) 'Reference Points, MBDyn Finite-Element Nodes, and MBDyn Gauss Points (in Kite Coordinates)'
   write(SumFileUnit,'()')
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '   Component                         Type                 Number Output Number     x       y       z '
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '     (-)                             (-)                   (-)        (-)         (m)     (m)     (m)'
   
   
   ! Cycle through components 
   call WriteNodeInfo(SumFileUnit, 1, p%numFusNds, m%FusPts, m%FusNdDCMs, m%FusO, (/0.0,0.0,0.0/), p%NFusOuts, p%FusOutNds, errStat2, errMsg2 )
   call WriteNodeInfo(SumFileUnit, 2, p%numSWnNds, m%SWnPts, m%SWnNdDCMs, m%FusO, m%SWnO, p%NSWnOuts, p%SWnOutNds, errStat2, errMsg2 )
   call WriteNodeInfo(SumFileUnit, 3, p%numPWnNds, m%PWnPts, m%PWnNdDCMs, m%FusO, m%PWnO, p%NPWnOuts, p%PWnOutNds, errStat2, errMsg2 )
   call WriteNodeInfo(SumFileUnit, 4, p%numVSNds,  m%VSPts,  m%VSNdDCMs,  m%FusO, m%VSO,  p%NVSOuts,  p%VSOutNds,  errStat2, errMsg2 )
   call WriteNodeInfo(SumFileUnit, 5, p%numSHSNds, m%SHSPts, m%SHSNdDCMs, m%FusO, m%SHSO, p%NSHSOuts, p%SHSOutNds, errStat2, errMsg2 )
   call WriteNodeInfo(SumFileUnit, 6, p%numPHSNds, m%PHSPts, m%PHSNdDCMs, m%FusO, m%PHSO, p%NPHSOuts, p%PHSOutNds, errStat2, errMsg2 )
  
   do i = 1, p%numPylons
         
         ! reference point
      write(SumFileUnit,'(3X,A,8X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(7))//num2lstr(i), NodeType(1), NoValStr, NoValStr, m%SPyO(1,i),  m%SPyO(2,i),  m%SPyO(3,i)

         ! finite-element points
      do k = 1,p%numSPyNds(i)
         globalPt = m%SPyPts(:,k,i) - m%FusO
         kitePt = matmul(m%SPyNdDCMs(:,:,k,i), globalPt)
         OutNumStr = ' -  '
         do l= 1,p%NPylOuts
            if ( k == p%PylOutNds(l) ) then
               OutNumStr = 'SP'//Num2LStr(i)//Num2LStr(l)
               continue
            end if
         end do
         
         write(SumFileUnit,'(3X,A,8X,A19,4X,A1,9X,A4,6X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(7))//num2lstr(i), NodeType(2), Num2LStr(k), OutNumStr, kitePt(1), kitePt(2), kitePt(3)
      end do
      
         ! gauss points
      do k = 1,p%numSPyNds(i)-1
         OutNumStr = ' -  '
         do l= 1,p%NPylOuts
            if ( k == p%PylOutNds(l) ) then
               OutNumStr = 'SP'//Num2LStr(i)//Num2LStr(l)
               continue
            end if
         end do
         globalPt = m%SPyPts(:,k,i) - m%FusO
         kitePt = matmul(m%SPyNdDCMs(:,:,k,i), globalPt)
         globalPt = m%SPyPts(:,k+1,i) - m%FusO
         kitePt2 = matmul(m%SPyNdDCMs(:,:,k+1,i), globalPt)
         if ( mod(k,2) == 1 ) then
            kitePt = (1.0-sqrt(3.0)/3.0)*kitePt2 + (sqrt(3.0)/3.0)*kitePt
         else
            kitePt = (1.0-sqrt(3.0)/3.0)*kitePt + (sqrt(3.0)/3.0)*kitePt2
         end if

         write(SumFileUnit,'(3X,A,8X,A19,4X,A1,9X,A4,6X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(7))//num2lstr(i), NodeType(3), Num2LStr(k), OutNumStr, kitePt(1), kitePt(2), kitePt(3)
      end do   
         
         ! reference point
      write(SumFileUnit,'(3X,A,13X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(8))//num2lstr(i), NodeType(1), NoValStr, NoValStr, m%PPyO(1,i),  m%PPyO(2,i),  m%PPyO(3,i)

         ! finite-element points
      do k = 1,p%numPPyNds(i)
         globalPt = m%PPyPts(:,k,i) - m%FusO
         kitePt = matmul(m%PPyNdDCMs(:,:,k,i), globalPt)
         OutNumStr = ' -  '
         do l= 1,p%NPylOuts
            if ( k == p%PylOutNds(l) ) then
               OutNumStr = 'PP'//Num2LStr(i)//Num2LStr(l)
               continue
            end if
         end do
         
         write(SumFileUnit,'(3X,A,13X,A19,4X,A1,9X,A4,6X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(8))//num2lstr(i), NodeType(2), Num2LStr(k), OutNumStr, kitePt(1), kitePt(2), kitePt(3)
      end do
      
         ! gauss points
      do k = 1,p%numPPyNds(i)-1
         OutNumStr = ' -  '
         do l= 1,p%NPylOuts
            if ( k == p%PylOutNds(l) ) then
               OutNumStr = 'PP'//Num2LStr(i)//Num2LStr(l)
               continue
            end if
         end do
         globalPt = m%PPyPts(:,k,i) - m%FusO
         kitePt = matmul(m%PPyNdDCMs(:,:,k,i), globalPt)
         globalPt = m%PPyPts(:,k+1,i) - m%FusO
         kitePt2 = matmul(m%PPyNdDCMs(:,:,k+1,i), globalPt)
         if ( mod(k,2) == 1 ) then
            kitePt = (1.0-sqrt(3.0)/3.0)*kitePt2 + (sqrt(3.0)/3.0)*kitePt
         else
            kitePt = (1.0-sqrt(3.0)/3.0)*kitePt + (sqrt(3.0)/3.0)*kitePt2
         end if

         write(SumFileUnit,'(3X,A,13X,A19,4X,A1,9X,A4,6X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(8))//num2lstr(i), NodeType(3), Num2LStr(k), OutNumStr, kitePt(1), kitePt(2), kitePt(3)
      end do   
         
   end do
      
   do i = 1, p%numPylons
      
         ! reference point
      globalPt = m%SPyRtrO(:,1,i) - m%FusO
      kitePt = matmul(m%FusODCM, globalPt)
      write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(9))//num2lstr(i), NodeType(1), NoValStr, NoValStr, kitePt(1), kitePt(2), kitePt(3)
      globalPt = m%SPyRtrO(:,2,i) - m%FusO
      kitePt = matmul(m%FusODCM, globalPt)
      write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(10))//num2lstr(i), NodeType(1), NoValStr, NoValStr, kitePt(1), kitePt(2), kitePt(3)
      globalPt = m%PPyRtrO(:,1,i) - m%FusO
      kitePt = matmul(m%FusODCM, globalPt)
      write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(11))//num2lstr(i), NodeType(1), NoValStr, NoValStr, kitePt(1), kitePt(2), kitePt(3)
      globalPt = m%PPyRtrO(:,2,i) - m%FusO
      kitePt = matmul(m%FusODCM, globalPt)
      write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) trim(Components(12))//num2lstr(i), NodeType(1), NoValStr, NoValStr, kitePt(1), kitePt(2), kitePt(3)
   
      
   end do
   
   write(SumFileUnit,'()')
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) 'Requested Channels in KiteFASTMBD Output Files: '//num2lstr(p%numKFASTOuts+p%numKADOuts+p%numMDOuts+p%numIfWOuts)
   
   write(SumFileUnit,'()')
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '   Number  Name       Units      Generated by'
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '    (-)    (-)         (-)       (KiteFASTMBD, KiteAeroDyn, InflowWind, MoorDyn)'
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '      0   Time        (s)        KiteFASTMBD'
   k = 1
   do i = 1,p%numKFASTOuts
         write(SumFileUnit,'(3X,I4,2X,A11,2X,A9,2X,A)',IOSTAT=TmpErrStat) k, p%OutParam(i)%Name, p%OutParam(i)%Units, 'KiteFASTMBD'
         k = k + 1
   end do
      
      do i = 1,p%numKADOuts
         write(SumFileUnit,'(3X,I4,2X,A11,2X,A9,2X,A)',IOSTAT=TmpErrStat) k, KAD_InitOut%WriteOutputHdr(i), KAD_InitOut%WriteOutputUnt(i), 'KiteAeroDyn'
         k = k + 1
      end do
      
      do i = 1,p%numIfWOuts
         write(SumFileUnit,'(3X,I4,2X,A11,2X,A9,2X,A)',IOSTAT=TmpErrStat) k, IfW_InitOut%WriteOutputHdr(i), IfW_InitOut%WriteOutputUnt(i), 'InflowWind'
         k = k + 1
      end do
      
      do i = 1,p%numMDOuts
         write(SumFileUnit,'(3X,I4,2X,A11,2X,A9,2X,A)',IOSTAT=TmpErrStat) k, MD_InitOut%WriteOutputHdr(i), MD_InitOut%WriteOutputUnt(i), 'MoorDyn'
         k = k + 1
      end do
      
      write (SumFileUnit,'()')
   
   if ( TmpErrStat /= 0 ) then
      call SetErrStat(ErrID_Fatal,'Error writing to summary file.',ErrStat,ErrMsg,'')
      return
   end if
   
   
   
end subroutine KFAST_WriteSummary


!> Calculates the length of a C string.
function cstrlen(carray) result(res)
   character(kind=c_char), intent(in) :: carray(:)
   integer :: res

   integer :: ii

   do ii = 1, size(carray)
   if (carray(ii) == c_null_char) then
      res = ii - 1
      return
   end if
   end do
   res = ii

end function cstrlen

subroutine CreateMBDynL2MotionsMesh(origin, numNodes, positions, alignDCM, nodeDCMs, mesh, errStat, errMsg)
   real(ReKi),                   intent(inout)  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numNodes          !< Number of nodes in the mesh
   real(ReKi),                   intent(in   )  :: positions(:,:)    !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the Kite axes
   real(R8Ki),                   intent(in   )  :: nodeDCMs(:,:,:) !< DCM needed to transform into a node's axes
   type(MeshType),               intent(inout)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   integer(intKi)                               :: j                 ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateL2MotionsMesh'

      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   call MeshCreate ( BlankMesh = mesh     &
                     ,IOS       = COMPONENT_INPUT &
                     ,Nnodes    = numNodes        &
                     ,errStat   = errStat2        &
                     ,ErrMess   = errMsg2         &
                     ,Orientation     = .true.    &
                     ,TranslationDisp = .true.    &
                     ,TranslationVel  = .true.    &
                     ,RotationVel     = .true.    &
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation
   do j=1,numNodes       
      position = positions(:,j) - origin(:) 
      position = matmul(alignDCM, position) 
      orientation = nodeDCMs(:,:,j)
      orientation = matmul(orientation, transpose(alignDCM))
      
      call MeshPositionNode(mesh, j, position, errStat2, errMsg2, orientation)  
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
   end do !j
         
      ! create line2 elements
   do j=1,numNodes-1
      call MeshConstructElement( mesh, ELEMENT_LINE2, errStat2, errMsg2, p1=j, p2=j+1 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   end do 
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return
   
      
   
   
   do j=1,numNodes 
      mesh%Orientation(:,:,j)     = nodeDCMs(:,:,j)
      mesh%TranslationDisp(:,j)   = positions(:,j)  - mesh%Position(:,j)
   end do
   
   mesh%TranslationVel  = 0.0_ReKi
   mesh%RotationVel     = 0.0_ReKi
   
   
end subroutine CreateMBDynL2MotionsMesh  

subroutine CreateMBDynL2MotionsWingMesh(origin, numSWnNodes, SWnPos, numPWnNodes, PWnPos, alignDCM, SWnDCMs, PWnDCMs, mesh, errStat, errMsg)
   real(ReKi),                   intent(inout)  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numSWnNodes       !< Number of nodes in the starboard wing
   real(ReKi),                   intent(in   )  :: SWnPos(:,:)       !< Coordinates of the undisplaced starboard wing nodes
   integer(IntKi),               intent(in   )  :: numPWnNodes       !< Number of nodes in the port wing
   real(ReKi),                   intent(in   )  :: PWnPos(:,:)       !< Coordinates of the undisplaced port wing nodes
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the Kite axes
   real(R8Ki),                   intent(in   )  :: SWnDCMs(:,:,:)    !< Starboard wing nodal DCMs needed to transform into kite axes
   real(R8Ki),                   intent(in   )  :: PWnDCMs(:,:,:)    !< Port wing nodal DCMs needed to transform into kite axes
   type(MeshType),               intent(inout)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   integer(intKi)                               :: c,j,jstart        ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateMBDynL2MotionsWingMesh'

      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   call MeshCreate ( BlankMesh = mesh     &
                     ,IOS       = COMPONENT_INPUT &
                     ,Nnodes    = numSWnNodes + numPWnNodes        &
                     ,errStat   = errStat2        &
                     ,ErrMess   = errMsg2         &
                     ,Orientation     = .true.    &
                     ,TranslationDisp = .true.    &
                     ,TranslationVel  = .true.    &
                     ,RotationVel     = .true.    &
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation
   position = 0.0_ReKi
   c = 1
   do j=numPWnNodes,1,-1       
      position = PWnPos(:,j) - origin(:) 
      position = matmul(alignDCM, position) 
      orientation = PWnDCMs(:,:,j)
      orientation = matmul(orientation, transpose(alignDCM))
      
      call MeshPositionNode(mesh, c, position, errStat2, errMsg2, orientation)  
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      c = c + 1   
   end do !j
   
   jstart = 1
   if ( EqualRealNos( TwoNorm( PWnPos(:,1) - SWnPos(:,1) ), 0.0_ReKi ) ) then
      jstart = 2
      ! TODO: For now we will throw an error if the inboard nodes are co-located!
      call SetErrStat( ErrID_Fatal, 'The current version of KiteFAST does not allow the most inboard nodes of the port and starboard wings to be colocated', errStat, errMsg, RoutineName )
      return
   end if
   
   do j=jstart,numSWnNodes       
      position = SWnPos(:,j) - origin(:) 
      position = matmul(alignDCM, position) 
      orientation = SWnDCMs(:,:,j)
      orientation = matmul(orientation, transpose(alignDCM))
      
      call MeshPositionNode(mesh, c, position, errStat2, errMsg2, orientation)  
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      c = c + 1    
   end do !j
         
      ! create line2 elements
   do j=1,numSWnNodes + numPWnNodes - jstart
      call MeshConstructElement( mesh, ELEMENT_LINE2, errStat2, errMsg2, p1=j, p2=j+1 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   end do 
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return
   c = 1
   do j=numPWnNodes,1,-1       
      mesh%Orientation(:,:,c)   = PWnDCMs(:,:,j)
      mesh%TranslationDisp(:,c) = PWnPos(:,j)  - mesh%Position(:,c)
      c = c+1
   end do
   do j=jstart,numSWnNodes     
      mesh%Orientation(:,:,c)   = SWnDCMs(:,:,j)
      mesh%TranslationDisp(:,c) = SWnPos(:,j)  - mesh%Position(:,c)
      c = c+1
   end do
   mesh%TranslationVel  = 0.0_ReKi
   mesh%RotationVel     = 0.0_ReKi
   
   
end subroutine CreateMBDynL2MotionsWingMesh  


subroutine CreateMBDynPtLoadsMesh(origin, numNodes, positions, alignDCM, nodeDCMs, mesh, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numNodes          !< Number of nodes in the mesh
   real(ReKi),                   intent(in   )  :: positions(:,:)    !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the Kite axes
   real(R8Ki),                   intent(in   )  :: nodeDCMs(3,3, numNodes) !< DCM needed to transform into a node's axes
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   integer(intKi)                               :: j                 ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateMBDynPtLoadsMesh'

      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   call MeshCreate ( BlankMesh = mesh     &
                     ,IOS         = COMPONENT_OUTPUT &
                     ,Nnodes      = numNodes        &
                     ,errStat     = errStat2        &
                     ,ErrMess     = errMsg2         &
                     ,force       = .true.          &
                     ,moment      = .true.          &
                     ,translationdisp = .true.      &
                     )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
   
         ! set node initial position/orientation
   position = 0.0_ReKi
   do j=1,numNodes       
      position = positions(:,j) - origin(:) 
      position = matmul(alignDCM, position) 
      orientation = nodeDCMs(:,:,j)
      orientation = matmul(orientation, transpose(alignDCM))
      
      call MeshPositionNode(mesh, j, position, errStat2, errMsg2, orientation)  
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call MeshConstructElement( mesh, ELEMENT_POINT, errStat2, errMsg2, p1=j )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )   
         
   end do !j
         
   
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return
      
   mesh%Force           = 0.0_ReKi
   mesh%Moment          = 0.0_ReKi
   do j=1,numNodes  
      mesh%TranslationDisp(:,j)   = positions(:,j)  - mesh%Position(:,j)
   end do
   
end subroutine CreateMBDynPtLoadsMesh  

subroutine CreateMBDynPtLoadsWingMesh(origin, numSWnNodes, SWnPos, numPWnNodes, PWnPos, alignDCM, SWnDCMs, PWnDCMs, mesh, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numSWnNodes       !< Number of nodes in the starboard wing
   real(ReKi),                   intent(in   )  :: SWnPos(:,:)       !< Coordinates of the undisplaced starboard wing nodes
   integer(IntKi),               intent(in   )  :: numPWnNodes       !< Number of nodes in the port wing
   real(ReKi),                   intent(in   )  :: PWnPos(:,:)       !< Coordinates of the undisplaced port wing nodes
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the Kite axes
   real(R8Ki),                   intent(in   )  :: SWnDCMs(3,3, numSWnNodes) !< DCM needed to transform into a startboard wing's nodal axes
   real(R8Ki),                   intent(in   )  :: PWnDCMs(3,3, numPWnNodes) !< DCM needed to transform into a port wing's nodal axes
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   integer(intKi)                               :: c,j,jstart        ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateMBDynPtLoadsWingMesh'

      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   call MeshCreate ( BlankMesh = mesh     &
                     ,IOS         = COMPONENT_OUTPUT &
                     ,Nnodes      = numSWnNodes+numPWnNodes        &
                     ,errStat     = errStat2        &
                     ,ErrMess     = errMsg2         &
                     ,force       = .true.          &
                     ,moment      = .true.          &
                     ,translationdisp = .true.      &
                     )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
   
         ! set node initial position/orientation
   position = 0.0_ReKi
   c = 1
   do j = numPWnNodes, 1, -1       
      position = PWnPos(:,j) - origin(:) 
      position = matmul(alignDCM, position) 
      orientation = PWnDCMs(:,:,j)
      orientation = matmul(orientation, transpose(alignDCM))
      call MeshPositionNode(mesh, c, position, errStat2, errMsg2, orientation)  
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )   
      c = c + 1
   end do !j
   jstart = 1
   if ( EqualRealNos( TwoNorm( PWnPos(:,numPWnNodes) - SWnPos(:,1) ), 0.0_ReKi ) ) then
      jstart = 2
      ! TODO: For now we will throw an error if the inboard nodes are co-located!
      call SetErrStat( ErrID_Fatal, 'The current version of KiteFAST does not allow the most inboard nodes of the port and starboard wings to be colocated', errStat, errMsg, RoutineName )
      return
   end if
   
   do j = jstart, numSWnNodes       
      position = SWnPos(:,j) - origin(:) 
      position = matmul(alignDCM, position) 
      orientation = SWnDCMs(:,:,j)
      orientation = matmul(orientation, transpose(alignDCM))
      call MeshPositionNode(mesh, c, position, errStat2, errMsg2, orientation)  
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )   
      c = c + 1
   end do !j
   
   do j = 1, numSWnNodes + numPWnNodes + 1 - jstart   
      call MeshConstructElement( mesh, ELEMENT_POINT, errStat2, errMsg2, p1=j )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )  
   end do
   
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return
      

   mesh%Force           = 0.0_ReKi
   mesh%Moment          = 0.0_ReKi
   mesh%TranslationDisp = 0.0_ReKi
   c = 1
   do j = numPWnNodes, 1, -1   
      mesh%TranslationDisp(:,c) = PWnPos(:,j)  - mesh%Position(:,c)
      c = c + 1
   end do
   do j = jstart, numSWnNodes  
      mesh%TranslationDisp(:,c)   = SWnPos(:,j)  - mesh%Position(:,c)
      c = c + 1
   end do
   
   
end subroutine CreateMBDynPtLoadsWingMesh  

subroutine CreateMeshMappings(m, p, KAD, MD, errStat, errMsg)

   type(KFAST_MiscVarType),     intent(inout) :: m
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KAD_States),           intent(in   ) :: KAD
   type(MD_Data),            intent(in   ) :: MD
   integer(IntKi),           intent(  out)  :: errStat           !< Error status of the operation
   character(*),             intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None
   
   character(*), parameter  :: RoutineName = 'CreateMeshMappings'
   integer(IntKi)           :: i
   integer(intKi)           :: errStat2          ! temporary Error status
   character(ErrMsgLen)     :: errMsg2           ! temporary Error message
   
   errStat = ErrID_None
   errMsg  = ''
   
   if ( p%useKAD ) then
         ! Mappings between MBDyn input motions meshes and the KiteAeroDyn motions meshes
      call MeshMapCreate( m%mbdFusMotions, KAD%u(1)%FusMotions, m%Fus_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%Fus_L2_L2' )     
            if (ErrStat>=AbortErrLev) then 
               call DumpMotionsMeshData(m%mbdFusMotions, "m%mbdFusMotions")
               call DumpMotionsMeshData(KAD%u(1)%FusMotions, "KAD%u(1)%FusMotions")
               return
            end if
      m%mbdFusMotions%RemapFlag = .false.
      KAD%u(1)%FusMotions%RemapFlag = .false.
      
      call MeshMapCreate( m%mbdSWnMotions, KAD%u(1)%SWnMotions, m%SWn_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SWn_L2_L2' )     
            if (ErrStat>=AbortErrLev) then 
               call DumpMotionsMeshData(m%mbdSWnMotions, "m%mbdSWnMotions")
               call DumpMotionsMeshData(KAD%u(1)%SWnMotions, "KAD%u(1)%SWnMotions")
               return
            end if
      m%mbdSWnMotions%RemapFlag = .false.
      KAD%u(1)%SWnMotions%RemapFlag = .false.
         
      call MeshMapCreate( m%mbdPWnMotions, KAD%u(1)%PWnMotions, m%PWn_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PWn_L2_L2' )     
            if (ErrStat>=AbortErrLev) then 
               call DumpMotionsMeshData(m%mbdPWnMotions, "m%mbdPWnMotions")
               call DumpMotionsMeshData(KAD%u(1)%PWnMotions, "KAD%u(1)%PWnMotions")
               return
            end if
      m%mbdPWnMotions%RemapFlag = .false.
      KAD%u(1)%PWnMotions%RemapFlag = .false.
      
      call MeshMapCreate( m%mbdVSMotions, KAD%u(1)%VSMotions, m%VS_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%VS_L2_L2' )     
            if (ErrStat>=AbortErrLev) then 
               call DumpMotionsMeshData(m%mbdVSMotions, "m%mbdVSMotions")
               call DumpMotionsMeshData(KAD%u(1)%VSMotions, "KAD%u(1)%VSMotions")
               return
            end if
      m%mbdVSMotions%RemapFlag = .false.
      KAD%u(1)%VSMotions%RemapFlag = .false.
      
      call MeshMapCreate( m%mbdSHSMotions, KAD%u(1)%SHSMotions, m%SHS_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SHS_L2_L2' )     
            if (ErrStat>=AbortErrLev) then 
               call DumpMotionsMeshData(m%mbdSHSMotions, "m%mbdSHSMotions")
               call DumpMotionsMeshData(KAD%u(1)%SHSMotions, "KAD%u(1)%SHSMotions")
               return
            end if
      m%mbdSHSMotions%RemapFlag = .false.
      KAD%u(1)%SHSMotions%RemapFlag = .false.
      
      call MeshMapCreate( m%mbdPHSMotions, KAD%u(1)%PHSMotions, m%PHS_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PHS_L2_L2' )     
            if (ErrStat>=AbortErrLev) then 
               call DumpMotionsMeshData(m%mbdPHSMotions, "m%mbdPHSMotions")
               call DumpMotionsMeshData(KAD%u(1)%PHSMotions, "KAD%u(1)%PHSMotions")
               return
            end if
      m%mbdPHSMotions%RemapFlag = .false.
      KAD%u(1)%PHSMotions%RemapFlag = .false.

      allocate(m%SPy_L2_L2(p%NumPylons), STAT=errStat2)
         if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPy_L2_L2', errStat, errMsg, RoutineName )     
      allocate(m%PPy_L2_L2(p%NumPylons), STAT=errStat2)
         if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPy_L2_L2', errStat, errMsg, RoutineName )     
   
      do i = 1 , p%NumPylons           
         call MeshMapCreate( m%mbdSPyMotions(i), KAD%u(1)%SPyMotions(i), m%SPy_L2_L2(i), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SPy_L2_L2('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) then 
                  call DumpMotionsMeshData(m%mbdSPyMotions(i), "m%mbdSPyMotions("//trim(num2lstr(i))//")")
                  call DumpMotionsMeshData(KAD%u(1)%SPyMotions(i), "KAD%u(1)%SPyMotions("//trim(num2lstr(i))//")")
                  return
               end if
         m%mbdSPyMotions(i)%RemapFlag = .false.
         KAD%u(1)%SPyMotions(i)%RemapFlag = .false.  
         
         call MeshMapCreate( m%mbdPPyMotions(i), KAD%u(1)%PPyMotions(i), m%PPy_L2_L2(i), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PPy_L2_L2('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) then 
                  call DumpMotionsMeshData(m%mbdPPyMotions(i), "m%mbdPPyMotions("//trim(num2lstr(i))//")")
                  call DumpMotionsMeshData(KAD%u(1)%PPyMotions(i), "KAD%u(1)%PPyMotions("//trim(num2lstr(i))//")")
                  return
               end if
         m%mbdPPyMotions(i)%RemapFlag = .false.
         KAD%u(1)%PPyMotions(i)%RemapFlag = .false.
      end do
   
   
         ! Mappings between KiteAeroDyn loads point meshes and MBDyn loads point meshes 

      call MeshMapCreate( m%KAD%y%FusLoads, m%mbdFusLoads, m%Fus_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%Fus_P_P' )     
            if (ErrStat>=AbortErrLev) return
      m%KAD%y%FusLoads%RemapFlag = .false.
      m%mbdFusLoads%RemapFlag = .false.
      
      call MeshMapCreate( m%KAD%y%SWnLoads, m%mbdSWnLoads, m%SWn_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SWn_P_P' )     
            if (ErrStat>=AbortErrLev) return
      m%KAD%y%SWnLoads%RemapFlag = .false.
      m%mbdSWnLoads%RemapFlag = .false.
      
      call MeshMapCreate( m%KAD%y%PWnLoads, m%mbdPWnLoads, m%PWn_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PWn_P_P' )     
            if (ErrStat>=AbortErrLev) return
      m%KAD%y%PWnLoads%RemapFlag = .false.
      m%mbdPWnLoads%RemapFlag = .false.
      
      call MeshMapCreate( m%KAD%y%VSLoads, m%mbdVSLoads, m%VS_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%VS_P_P' )     
            if (ErrStat>=AbortErrLev) return
      m%KAD%y%VSLoads%RemapFlag = .false.
      m%mbdVSLoads%RemapFlag = .false.
      
      call MeshMapCreate( m%KAD%y%SHSLoads, m%mbdSHSLoads, m%SHS_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SHS_P_P' )     
            if (ErrStat>=AbortErrLev) return
      m%KAD%y%SHSLoads%RemapFlag = .false.
      m%mbdSHSLoads%RemapFlag = .false.
      
      call MeshMapCreate( m%KAD%y%PHSLoads, m%mbdPHSLoads, m%PHS_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PHS_P_P' )     
            if (ErrStat>=AbortErrLev) return
      m%KAD%y%PHSLoads%RemapFlag = .false.
      m%mbdPHSLoads%RemapFlag = .false.
      

      allocate(m%SPy_P_P(p%NumPylons), STAT=errStat2)
         if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPy_P_P', errStat, errMsg, RoutineName )     
      allocate(m%PPy_P_P(p%NumPylons), STAT=errStat2)
         if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPy_P_P', errStat, errMsg, RoutineName )     
   
      do i = 1 , p%NumPylons           
         call MeshMapCreate( m%KAD%y%SPyLoads(i), m%mbdSPyLoads(i), m%SPy_P_P(i), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SPy_P_P('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) return
          m%KAD%y%SPyLoads(i)%RemapFlag = .false.
          m%mbdSPyLoads(i)%RemapFlag = .false.
            
         call MeshMapCreate( m%KAD%y%PPyLoads(i), m%mbdPPyLoads(i), m%PPy_P_P(i), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PPy_P_P('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) return
         m%KAD%y%PPyLoads(i)%RemapFlag = .false.
         m%mbdPPyLoads(i)%RemapFlag = .false.
         
      end do
   end if ! if ( p%useKAD )
   
   if ( p%useMD_Tether ) then
         ! Need to transfer the MBDyn bridle point motions to MoorDyn
      call MeshMapCreate( m%mbdWngMotions, MD%u(1)%PtFairleadDisplacement(1), m%MD_L2_2_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%MD_L2_2_P' )     
               if (ErrStat>=AbortErrLev) return
      m%mbdWngMotions%RemapFlag = .false.
      MD%u(1)%PtFairleadDisplacement(1)%RemapFlag = .false.
      
         ! Need to transfer the MoorDyn bridle point loads back the to MBDyn wing mesh for loads
      call MeshMapCreate( MD%y%PtFairleadLoad(1), m%mbdWngLoads,  m%MD_P_2_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%MD_P_2_P' )     
               if (ErrStat>=AbortErrLev) return
      MD%y%PtFairleadLoad(1)%RemapFlag = .false.
      m%mbdWngLoads%RemapFlag = .false.
      
   end if 
   
end subroutine CreateMeshMappings

!====================================================================================================
subroutine KFAST_RotorCalcs(NacDCM, NacOmega, NacAcc, NacAlpha, RtrSpd, GenTorq, F_Aero, M_Aero, g, mass, Irot, Itran, Xcm, Freact, Mreact, errStat, errMsg )
! This subroutine computes the reaction loads/moments on the a single rotor nacelle point 
!----------------------------------------------------------------------------------------------------
   real(R8Ki),             intent(in   ) :: NacDCM(3,3) ! Displaced rotation (absolute orientation of the nacelle) (-)
   real(ReKi),             intent(in   ) :: NacOmega(3)
   real(ReKi),             intent(in   ) :: NacAcc(3)
   real(ReKi),             intent(in   ) :: NacAlpha(3)
   real(ReKi),             intent(in   ) :: RtrSpd
   real(ReKi),             intent(in   ) :: GenTorq
   real(ReKi),             intent(in   ) :: F_Aero(3)
   real(ReKi),             intent(in   ) :: M_Aero(3)
   real(ReKi),             intent(in   ) :: g
   real(ReKi),             intent(in   ) :: mass
   real(ReKi),             intent(in   ) :: Irot
   real(ReKi),             intent(in   ) :: Itran
   real(ReKi),             intent(in   ) :: Xcm
   real(ReKi),             intent(  out) :: Freact(3)
   real(ReKi),             intent(  out) :: Mreact(3)
   integer(IntKi),         intent(  out) :: errStat    ! Error status of the operation
   character(*),           intent(  out) :: errMsg     ! Error message if ErrStat /= ErrID_None
 
   real(R8Ki)      :: xhat(3)           ! rotor system x-axis expressed in the global inertial system
   real(R8Ki)      :: cm_r(3)           ! vector from rotor reference point to the rotor center of mass (cm) expressed in the global system
   real(R8Ki)      :: Icm_Tran
   real(R8Ki)      :: Fcm_Aero(3)
   real(R8Ki)      :: tmp(3), gvec(3), tmp2(3), FcmReact(3), McmReact(3), RtrAccCM_kite(3)
   real(R8Ki)      :: Mcm_Aero(3), RtrOmega(3), RtrOmega_kite(3), RtrAlpha_kite(3)

      ! Initialize local variables      
   ErrStat = ErrID_None  
   ErrMsg  = ""
   xhat(1) = NacDCM(1,1)
   xhat(2) = NacDCM(1,2)
   xhat(3) = NacDCM(1,3)
   gvec(1)  = 0.0_ReKi
   gvec(2)  = 0.0_ReKi
   gvec(3)  =-g
   !write(*,*) '-------------------------------------------'
   !write(*,*) 'NacDCM(1,:) = ', real(NacDCM(1,:), SiKi)
   !write(*,*) 'NacDCM(2,:) = ', real(NacDCM(2,:), SiKi)
   !write(*,*) 'NacDCM(3,:) = ', real(NacDCM(3,:), SiKi)
   !write(*,*) 'NacOmega = ', NacOmega
   !write(*,*) 'NacAcc   = ', NacAcc
   !write(*,*) 'NacAlpha = ', NacAlpha
   !write(*,*) 'mass   = ', mass
   !write(*,*) 'xhat   = ', real(xhat,SiKi)
   !write(*,*) 'Xcm    = ', Xcm
   !write(*,*) 'Irot   = ', Irot
   !write(*,*) 'Itran  = ', Itran
   !write(*,*) 'RtrSpd = ', RtrSpd
   !write(*,*) 'GenTorq= ', GenTorq
   !write(*,*) 'F_Aero = ', F_Aero
   !write(*,*) 'M_Aero = ', M_Aero
      ! Compute the inputs relative to the rotor/drivetrain CM and expressed in the local nacelle coordinate system
   cm_r     = Xcm*xhat
   Icm_Tran = Itran - mass*Xcm*Xcm
   Fcm_Aero = matmul(NacDCM, F_Aero)
   tmp      = M_Aero - cross_product(cm_r, F_Aero)
   Mcm_Aero = matmul(NacDCM, tmp)
   gvec     = matmul(NacDCM,gvec)
   RtrOmega = NacOmega + RtrSpd*xhat
   RtrOmega_kite = matmul(NacDCM,RtrOmega)
   tmp = cross_product(RtrOmega, cm_r)
   tmp = cross_product(RtrOmega, tmp)
   tmp2 = cross_product(NacAlpha, cm_r)
   tmp = NacAcc + tmp2 + tmp
   RtrAccCM_kite = matmul(NacDCM,tmp)
   RtrAlpha_kite = matmul(NacDCM,NacAlpha)

   FcmReact(1) = -Fcm_Aero(1) - mass*gvec(1) + mass*RtrAccCM_kite(1)
   FcmReact(2) = -Fcm_Aero(2) - mass*gvec(2) + mass*RtrAccCM_kite(2)
   FcmReact(3) = -Fcm_Aero(3) - mass*gvec(3) + mass*RtrAccCM_kite(3)
   McmReact(1) = GenTorq
   McmReact(2) = -Mcm_Aero(2) + Irot*RtrAlpha_kite(2) + (Irot - Icm_Tran)*RtrOmega_kite(3)*RtrOmega_kite(1)
   McmReact(3) = -Mcm_Aero(3) + Irot*RtrAlpha_kite(3) - (Irot - Icm_Tran)*RtrOmega_kite(2)*RtrOmega_kite(1)
   Freact      = -matmul(transpose(NacDCM),FcmReact)
   Mreact      = -matmul(transpose(NacDCM),McmReact) + cross_product(cm_r,Freact)
   !write(*,*) 'Freact = ', Freact
   !write(*,*) 'Mreact = ', Mreact
end subroutine KFAST_RotorCalcs

subroutine TransferMBDynInputs( numNodePts_c, nodePts_c,  nodeDCMs_c, nodeVels_c, nodeOmegas_c, nodeAccs_c, rtrPts_c, rtrDCMs_c, rtrVels_c, rtrOmegas_c, rtrAccs_c, rtrAlphas_c, p, m, errStat, errMsg )
   integer(C_INT),            intent(in   ) :: numNodePts_c      ! total number of array elements in the nodal points array
   real(C_DOUBLE),            intent(in   ) :: nodePts_c(:)      ! 1D array containing all the nodal point coordinate data
   real(C_DOUBLE),            intent(in   ) :: nodeDCMs_c(:)     ! 1D array containing all the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: nodeVels_c(:)     ! 1D array containing all the nodal translational velocities data
   real(C_DOUBLE),            intent(in   ) :: nodeOmegas_c(:)   ! 1D array containing all the nodal angular velocities data
   real(C_DOUBLE),            intent(in   ) :: nodeAccs_c(:)     ! 1D array containing all the nodal translational accelerations data
   
   real(C_DOUBLE),            intent(in   ) :: rtrPts_c(:)       ! 1D array of the rotor positions in global coordinates (m)
   real(C_DOUBLE),            intent(in   ) :: rtrDCMs_c(:)      ! 1D array of the rotor point DCMs
   real(C_DOUBLE),            intent(in   ) :: rtrVels_c(:)      ! 1D array of the rotor point velocities in global coordinates (m/s)
   real(C_DOUBLE),            intent(in   ) :: rtrOmegas_c(:)    ! 1D array of the rotor point rotational velocities in global coordinates (m/s)
   real(C_DOUBLE),            intent(in   ) :: rtrAccs_c(:)      ! 1D array of the rotor point accelerations in global coordinates (m/s)
   real(C_DOUBLE),            intent(in   ) :: rtrAlphas_c(:)    ! 1D array of the rotor point rotational accelerations in global coordinates (m/s)
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat           ! Error status of the operation
   character(*),              intent(  out) :: errMsg            ! Error message if errStat /= ErrID_None


   integer(IntKi)   :: i, j, c, n

   errStat = ErrID_None
   errMsg  = ''
   
   c=1
   n=1
   
   do i = 1,p%numFusNds
      m%FusNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%FusPts(:,i)            = nodePts_c(n:n+2)
      m%FusVels(:,i)           = nodeVels_c(n:n+2)
      m%FusOmegas(:,i)         = nodeOmegas_c(n:n+2)
      m%FusAccs(:,i)           = nodeAccs_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSwnNds
      m%SWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SWnPts(:,i)            = nodePts_c(n:n+2)
      m%SWnVels(:,i)           = nodeVels_c(n:n+2)
      m%SWnOmegas(:,i)         = nodeOmegas_c(n:n+2)
      m%SWnAccs(:,i)           = nodeAccs_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPwnNds
      m%PWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PWnPts(:,i)            = nodePts_c(n:n+2)
      m%PWnVels(:,i)           = nodeVels_c(n:n+2)
      m%PWnOmegas(:,i)         = nodeOmegas_c(n:n+2)
      m%PWnAccs(:,i)           = nodeAccs_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numVSNds
      m%VSNdDCMs(:,:,i)        = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%VSPts(:,i)             = nodePts_c(n:n+2)
      m%VSVels(:,i)            = nodeVels_c(n:n+2)
      m%VSOmegas(:,i)          = nodeOmegas_c(n:n+2)
      m%VSAccs(:,i)            = nodeAccs_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSHSNds
      m%SHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SHSPts(:,i)            = nodePts_c(n:n+2)
      m%SHSVels(:,i)           = nodeVels_c(n:n+2)
      m%SHSOmegas(:,i)         = nodeOmegas_c(n:n+2)
      m%SHSAccs(:,i)           = nodeAccs_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPHSNds
      m%PHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PHSPts(:,i)            = nodePts_c(n:n+2)
      m%PHSVels(:,i)           = nodeVels_c(n:n+2)
      m%PHSOmegas(:,i)         = nodeOmegas_c(n:n+2)
      m%PHSAccs(:,i)           = nodeAccs_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)
         m%SPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%SPyPts(:,i,j)            = nodePts_c(n:n+2)
         m%SPyVels(:,i,j)           = nodeVels_c(n:n+2)
         m%SPyOmegas(:,i,j)         = nodeOmegas_c(n:n+2)
         m%SPyAccs(:,i,j)           = nodeAccs_c(n:n+2)
         c = c + 9
         n = n + 3
      end do
   end do
   do j = 1, p%numPylons
      do i = 1, p%numPPyNds(j)
         m%PPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%PPyPts(:,i,j)            = nodePts_c(n:n+2)
         m%PPyVels(:,i,j)           = nodeVels_c(n:n+2)
         m%PPyOmegas(:,i,j)         = nodeOmegas_c(n:n+2)
         m%PPyAccs(:,i,j)           = nodeAccs_c(n:n+2)
         c = c + 9
         n = n + 3
      end do
   end do
   
   if ( (c-1) /= numNodePts_c*9 ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of DCM elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numNodePts_c*9))//'.'
   end if
   
      ! Decode rotor positions and motions
   c=1
   n=1
   do i = 1, p%numPylons
      m%SPyRtrO(:,1,i)          = rtrPts_c(c:c+2)
      m%SPyRtrDCMs(:,:,1,i)     = reshape(rtrDCMs_c(n:n+8),(/3,3/))
      m%SPyRtrVels(:,1,i)       = rtrVels_c(c:c+2)
      m%SPyRtrOmegas(:,1,i)     = rtrOmegas_c(c:c+2)
      m%SPyRtrAccs(:,1,i)       = rtrAccs_c(c:c+2)
      m%SPyRtrAlphas(:,1,i)     = rtrAlphas_c(c:c+2)
      c = c + 3
      n = n + 9
      m%SPyRtrO(:,2,i)          = rtrPts_c(c:c+2)
      m%SPyRtrDCMs(:,:,2,i)     = reshape(rtrDCMs_c(n:n+8),(/3,3/))
      m%SPyRtrVels(:,2,i)       = rtrVels_c(c:c+2)
      m%SPyRtrOmegas(:,2,i)     = rtrOmegas_c(c:c+2)
      m%SPyRtrAccs(:,2,i)       = rtrAccs_c(c:c+2)
      m%SPyRtrAlphas(:,2,i)     = rtrAlphas_c(c:c+2)
      c = c + 3
      n = n + 9
   end do
   do i = 1, p%numPylons
      m%PPyRtrO(:,1,i)          = rtrPts_c(c:c+2)
      m%PPyRtrDCMs(:,:,1,i)     = reshape(rtrDCMs_c(n:n+8),(/3,3/))
      m%PPyRtrVels(:,1,i)       = rtrVels_c(c:c+2)
      m%PPyRtrOmegas(:,1,i)     = rtrOmegas_c(c:c+2)
      m%PPyRtrAccs(:,1,i)       = rtrAccs_c(c:c+2)
      m%PPyRtrAlphas(:,1,i)     = rtrAlphas_c(c:c+2)
      c = c + 3
      n = n + 9
      m%PPyRtrO(:,2,i)          = rtrPts_c(c:c+2)
      m%PPyRtrDCMs(:,:,2,i)     = reshape(rtrDCMs_c(n:n+8),(/3,3/))
      m%PPyRtrVels(:,2,i)       = rtrVels_c(c:c+2)
      m%PPyRtrOmegas(:,2,i)     = rtrOmegas_c(c:c+2)
      m%PPyRtrAccs(:,2,i)       = rtrAccs_c(c:c+2)
      m%PPyRtrAlphas(:,2,i)     = rtrAlphas_c(c:c+2)
      c = c + 3
      n = n + 9
   end do
   
      ! Now attach this motion data to the MBDyn motion meshes and transfer motions to the corresponding KAD input meshes
   call TransferMBDynInputs2MBDMeshes( p, m, errStat, errMsg )
   
end subroutine TransferMBDynInputs

subroutine TransferMBDynInitInputs( numNodePts_c, nodePts_c, nodeDCMs_c, rtrPts_c, p, m, errStat, errMsg )
   integer(C_INT),            intent(in   ) :: numNodePts_c                  ! total number of array elements in the nodal points array
   real(C_DOUBLE),            intent(in   ) :: nodePts_c(:)       ! 1D array containing all the nodal point coordinate data
   real(C_DOUBLE),            intent(in   ) :: nodeDCMs_c(:)         ! 1D array containing all the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: rtrPts_c(:)                      ! 1D array of the rotor positions in global coordinates (m)
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None


   integer(IntKi)   :: i, j, c, n, tplynodes

   errStat = ErrID_None
   errMsg  = ''
   
   
   tplynodes = 0
   do j = 1,p%numPylons
      tplynodes = tplynodes + p%numPPyNds(j)
      tplynodes = tplynodes + p%numSPyNds(j)
   end do
   
   c = 9*( p%numFusNds + p%numSwnNds + p%numPwnNds + p%numVSNds +  p%numSHSNds + p%numPHSNds + tplynodes )
   if ( c /= numNodePts_c*9 ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of DCM elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numNodePts_c*9))//'.'
   end if
   
   c=1
   n=1
   
   do i = 1,p%numFusNds
      m%FusNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%FusPts(:,i)            = nodePts_c(n:n+2)
      m%FusVels(:,i)           = 0.0_ReKi
      m%FusOmegas(:,i)         = 0.0_ReKi
      m%FusAccs(:,i)            = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSwnNds
      m%SWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SWnPts(:,i)            = nodePts_c(n:n+2)
      m%SWnVels(:,i)           = 0.0_ReKi
      m%SWnOmegas(:,i)         = 0.0_ReKi
      m%SWnAccs(:,i)            = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPwnNds
      m%PWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PWnPts(:,i)            = nodePts_c(n:n+2)
      m%PWnVels(:,i)           = 0.0_ReKi
      m%PWnOmegas(:,i)         = 0.0_ReKi
      m%PWnAccs(:,i)            = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numVSNds
      m%VSNdDCMs(:,:,i)        = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%VSPts(:,i)             = nodePts_c(n:n+2)
      m%VSVels(:,i)            = 0.0_ReKi
      m%VSOmegas(:,i)          = 0.0_ReKi
      m%VSAccs(:,i)            = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSHSNds
      m%SHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SHSPts(:,i)            = nodePts_c(n:n+2)
      m%SHSVels(:,i)           = 0.0_ReKi
      m%SHSOmegas(:,i)         = 0.0_ReKi
      m%SHSAccs(:,i)            = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPHSNds
      m%PHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PHSPts(:,i)            = nodePts_c(n:n+2)
      m%PHSVels(:,i)           = 0.0_ReKi
      m%PHSOmegas(:,i)         = 0.0_ReKi
      m%PHSAccs(:,i)            = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)
         m%SPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%SPyPts(:,i,j)            = nodePts_c(n:n+2)
         m%SPyVels(:,i,j)           = 0.0_ReKi
         m%SPyOmegas(:,i,j)         = 0.0_ReKi
         m%SPyAccs(:,i,j)            = 0.0_ReKi
         c = c + 9
         n = n + 3
      end do
   end do
   do j = 1, p%numPylons
      do i = 1, p%numPPyNds(j)
         m%PPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%PPyPts(:,i,j)            = nodePts_c(n:n+2)
         m%PPyVels(:,i,j)           = 0.0_ReKi
         m%PPyOmegas(:,i,j)         = 0.0_ReKi
         m%PPyAccs(:,i,j)            = 0.0_ReKi
         c = c + 9
         n = n + 3
      end do
   end do
  
      ! Decode rotor positions
   c=1
   do i = 1, p%numPylons
      m%SPyRtrO(:,1,i)         = rtrPts_c(c:c+2)
      c = c + 3
      m%SPyRtrO(:,2,i)         = rtrPts_c(c:c+2)
      c = c + 3
   end do
   do i = 1, p%numPylons
      m%PPyRtrO(:,1,i)         = rtrPts_c(c:c+2)
      c = c + 3
      m%PPyRtrO(:,2,i)         = rtrPts_c(c:c+2)
      c = c + 3
   end do
   
end subroutine TransferMBDynInitInputs

subroutine TransferMBDynToIfW( WindPt_c, FusO, p, m, errStat, errMsg )
   real(C_DOUBLE),            intent(in   ) :: WindPt_c(3)                      ! The location of the ground station point where the freestream wind is measured [global coordinates] (m)
   real(ReKi),                intent(in   ) :: FusO(3)                          ! Location of principal kite reference point in global coordinates
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None


   integer(IntKi)   :: i, j, c, v

   errStat = ErrID_None
   errMsg  = ''
   
   m%IfW%u%PositionXYZ(:,1) = WindPt_c
   m%IfW%u%PositionXYZ(:,2) = FusO
   
   v=3
   
   do i = 1,OtherSt%KAD%u(1)%FusMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = OtherSt%KAD%u(1)%FusMotions%Position(:,i) + OtherSt%KAD%u(1)%FusMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,OtherSt%KAD%u(1)%SWnMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = OtherSt%KAD%u(1)%SWnMotions%Position(:,i) + OtherSt%KAD%u(1)%SWnMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,OtherSt%KAD%u(1)%PWnMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = OtherSt%KAD%u(1)%PWnMotions%Position(:,i) + OtherSt%KAD%u(1)%PWnMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,OtherSt%KAD%u(1)%VSMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = OtherSt%KAD%u(1)%VSMotions%Position(:,i) + OtherSt%KAD%u(1)%VSMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,OtherSt%KAD%u(1)%SHSMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = OtherSt%KAD%u(1)%SHSMotions%Position(:,i) + OtherSt%KAD%u(1)%SHSMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,OtherSt%KAD%u(1)%PHSMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = OtherSt%KAD%u(1)%PHSMotions%Position(:,i) + OtherSt%KAD%u(1)%PHSMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do j = 1, p%numPylons
      do i = 1, OtherSt%KAD%u(1)%SPyMotions(j)%NNodes
         m%IfW%u%PositionXYZ(:,v)   = OtherSt%KAD%u(1)%SPyMotions(j)%Position(:,i) + OtherSt%KAD%u(1)%SPyMotions(j)%TranslationDisp(:,i)
         v = v + 1
      end do
   end do
   do j = 1, p%numPylons
      do i = 1, OtherSt%KAD%u(1)%PPyMotions(j)%NNodes
         m%IfW%u%PositionXYZ(:,v)   = OtherSt%KAD%u(1)%PPyMotions(j)%Position(:,i) + OtherSt%KAD%u(1)%PPyMotions(j)%TranslationDisp(:,i)
         v = v + 1
      end do
   end do
   
      ! Decode rotor positions
   do j = 1, p%numPylons
      do i=1,2 ! two per pylon
         c = i+(j-1)*p%numPylons
         m%IfW%u%PositionXYZ(:,v) = OtherSt%KAD%u(1)%SPyRtrMotions(c)%Position(:,1) + OtherSt%KAD%u(1)%SPyRtrMotions(c)%TranslationDisp(:,1) 
         v = v + 1
      end do
   end do
   
   do j = 1, p%numPylons
      do i=1,2 ! two per pylon
         c = i+(j-1)*p%numPylons
         m%IfW%u%PositionXYZ(:,v) = OtherSt%KAD%u(1)%PPyRtrMotions(c)%Position(:,1) + OtherSt%KAD%u(1)%PPyRtrMotions(c)%TranslationDisp(:,1) 
         v = v + 1
      end do
   end do

end subroutine TransferMBDynToIfW

subroutine TransferLoadsToMBDyn( p, m, nodeLoads_c, rtrLoads_c, errStat, errMsg )
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   real(C_DOUBLE),            intent(inout) :: nodeLoads_c(:)  ! 1D array of the nodal point loads
   real(C_DOUBLE),            intent(inout) :: rtrLoads_c(:)   ! 1D array of the rotor point loads
   integer(IntKi),            intent(  out) :: errStat         !< Error status of the operation
   character(*),              intent(  out) :: errMsg          !< Error message if errStat /= ErrID_None

   integer(IntKi)  :: i,j,k,c, compOffset
   integer(IntKi)           :: errStat2              ! error status values
   character(ErrMsgLen)     :: errMsg2                ! error messages
   character(*), parameter  :: routineName = 'TransferLoadsToMBDyn'
   
   errStat = ErrID_None
   errMsg  = ''

   nodeLoads_c = 0.0
   rtrLoads_c  = 0.0
   
   if ( p%useMD_Tether ) then
         ! First map MD loads back to the wing mesh
         
      !TODO: add translationdisp field to m%mbdWngLoads and manually apply the m%mbdWngMotions TranslationDisp to m%mbdWngLoads mesh
      call Transfer_Point_to_Point( m%MD_Tether%y%PtFairleadLoad(1), m%mbdWngLoads,  m%MD_P_2_P, errStat2, errMsg2, m%MD_Tether%u(1)%PtFairleadDisplacement(1), m%mbdWngLoads )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' TransferLoadsToMBDyn: Transfer_MD_P_2_P' )   
            
         ! Now attach these loads to the corresponding array elements which are sent back to MBDyn
      compOffset = p%numFusNds
      !TODO Fix bug if wing mesh does not contain one of the two common nodes at the fuselage
      !istart = 1
      !if (m%mbdWngLoads%NNodes /= (p%numSwnNds+p%numPwnNds) istart = 2
      do i = 1, p%numSwnNds
         j = 6*compOffset + 6*(i-1) + 1
         nodeLoads_c(j  ) = m%mbdWngLoads%Force(1,i + p%numPwnNds)
         nodeLoads_c(j+1) = m%mbdWngLoads%Force(2,i + p%numPwnNds)
         nodeLoads_c(j+2) = m%mbdWngLoads%Force(3,i + p%numPwnNds)
         nodeLoads_c(j+3) = m%mbdWngLoads%Moment(1,i + p%numPwnNds)
         nodeLoads_c(j+4) = m%mbdWngLoads%Moment(2,i + p%numPwnNds)
         nodeLoads_c(j+5) = m%mbdWngLoads%Moment(3,i + p%numPwnNds)
      end do
      compOffset = compOffset + p%numSwnNds
      
      do i = 1, p%numPwnNds
         j = 6*compOffset + 6*(i-1) + 1
         nodeLoads_c(j  ) = m%mbdWngLoads%Force(1,p%numPwnNds-i+1)
         nodeLoads_c(j+1) = m%mbdWngLoads%Force(2,p%numPwnNds-i+1)
         nodeLoads_c(j+2) = m%mbdWngLoads%Force(3,p%numPwnNds-i+1)
         nodeLoads_c(j+3) = m%mbdWngLoads%Moment(1,p%numPwnNds-i+1)
         nodeLoads_c(j+4) = m%mbdWngLoads%Moment(2,p%numPwnNds-i+1)
         nodeLoads_c(j+5) = m%mbdWngLoads%Moment(3,p%numPwnNds-i+1)
      end do
         
   end if
      
      
      
   if ( p%useKAD ) then
      
      ! Map KAD component nodal loads to the MBDyn load meshes
      !TODO: add translationdisp field to m%mbdFusLoads and manually apply the m%mbdFusMotions TranslationDisp to m%mbdFusLoads mesh   
      call Transfer_Point_to_Point( m%KAD%y%FusLoads, m%mbdFusLoads, m%Fus_P_P, errStat2, errMsg2, m%KAD%y%FusLoads, m%mbdFusLoads )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' TransferLoadsToMBDyn: m%Fus_P_P' )     
            if (ErrStat>=AbortErrLev) return
            
            
      
      call Transfer_Point_to_Point( m%KAD%y%SWnLoads, m%mbdSWnLoads, m%SWn_P_P, errStat2, errMsg2, m%KAD%y%SWnLoads, m%mbdSWnLoads )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' TransferLoadsToMBDyn: m%SWn_P_P' )     
            if (ErrStat>=AbortErrLev) return
      call Transfer_Point_to_Point( m%KAD%y%PWnLoads, m%mbdPWnLoads, m%PWn_P_P, errStat2, errMsg2, m%KAD%y%PWnLoads, m%mbdPWnLoads )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' TransferLoadsToMBDyn: m%PWn_P_P' )     
            if (ErrStat>=AbortErrLev) return
      call Transfer_Point_to_Point( m%KAD%y%VSLoads, m%mbdVSLoads, m%VS_P_P, errStat2, errMsg2, m%KAD%y%VSLoads, m%mbdVSLoads )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' TransferLoadsToMBDyn: m%VS_P_P' )     
            if (ErrStat>=AbortErrLev) return
      call Transfer_Point_to_Point( m%KAD%y%SHSLoads, m%mbdSHSLoads, m%SHS_P_P, errStat2, errMsg2, m%KAD%y%SHSLoads, m%mbdSHSLoads )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' TransferLoadsToMBDyn: m%SHS_P_P' )     
            if (ErrStat>=AbortErrLev) return
      call Transfer_Point_to_Point( m%KAD%y%PHSLoads, m%mbdPHSLoads, m%PHS_P_P, errStat2, errMsg2, m%KAD%y%PHSLoads, m%mbdPHSLoads )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' TransferLoadsToMBDyn: m%PHS_P_P' )     
            if (ErrStat>=AbortErrLev) return
     
      do i = 1 , p%NumPylons           
         call Transfer_Point_to_Point( m%KAD%y%SPyLoads(i), m%mbdSPyLoads(i), m%SPy_P_P(i), errStat2, errMsg2, m%KAD%y%SPyLoads(i), m%mbdSPyLoads(i) )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' TransferLoadsToMBDyn: m%SPy_P_P('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) return
            
         call Transfer_Point_to_Point( m%KAD%y%PPyLoads(i), m%mbdPPyLoads(i), m%PPy_P_P(i), errStat2, errMsg2, m%KAD%y%PPyLoads(i), m%mbdPPyLoads(i) )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' TransferLoadsToMBDyn: m%PPy_P_P('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) return
      end do
      
      ! Now transfer the MBDyn mesh loads over to the corresponding elements of the data going back to MBDyn
         
         ! Fuselage
      compOffset = 0
      do i = 1, p%numFusNds
         j = compOffset*6 + 6*(i-1) + 1
         nodeLoads_c(j  ) = m%mbdFusLoads%Force(1,i)
         nodeLoads_c(j+1) = m%mbdFusLoads%Force(2,i)
         nodeLoads_c(j+2) = m%mbdFusLoads%Force(3,i)
         nodeLoads_c(j+3) = m%mbdFusLoads%Moment(1,i)
         nodeLoads_c(j+4) = m%mbdFusLoads%Moment(2,i)
         nodeLoads_c(j+5) = m%mbdFusLoads%Moment(3,i)
      end do       
      compOffset = p%numFusNds
         
         ! Starboard Wing  (add these to any loads coming from MoorDyn)
      do i = 1, p%numSwnNds
         j = compOffset*6 + 6*(i-1) + 1
         nodeLoads_c(j  ) = nodeLoads_c(j  ) + m%mbdSWnLoads%Force(1,i)
         nodeLoads_c(j+1) = nodeLoads_c(j+1) + m%mbdSWnLoads%Force(2,i)
         nodeLoads_c(j+2) = nodeLoads_c(j+2) + m%mbdSWnLoads%Force(3,i)
         nodeLoads_c(j+3) = nodeLoads_c(j+3) + m%mbdSWnLoads%Moment(1,i)
         nodeLoads_c(j+4) = nodeLoads_c(j+4) + m%mbdSWnLoads%Moment(2,i)
         nodeLoads_c(j+5) = nodeLoads_c(j+5) + m%mbdSWnLoads%Moment(3,i)
      end do
      compOffset = compOffset + p%numSwnNds
         
         ! Port Wing  (add these to any loads coming from MoorDyn)
         do i = 1, p%numPwnNds
         j = compOffset*6 + 6*(i-1) + 1
         nodeLoads_c(j  ) = nodeLoads_c(j  ) + m%mbdPWnLoads%Force(1,i)
         nodeLoads_c(j+1) = nodeLoads_c(j+1) + m%mbdPWnLoads%Force(2,i)
         nodeLoads_c(j+2) = nodeLoads_c(j+2) + m%mbdPWnLoads%Force(3,i)
         nodeLoads_c(j+3) = nodeLoads_c(j+3) + m%mbdPWnLoads%Moment(1,i)
         nodeLoads_c(j+4) = nodeLoads_c(j+4) + m%mbdPWnLoads%Moment(2,i)
         nodeLoads_c(j+5) = nodeLoads_c(j+5) + m%mbdPWnLoads%Moment(3,i)
      end do
      compOffset = compOffset + p%numPwnNds
        
         ! Vertical stabilizer
      do i = 1, p%numVSNds
         j = compOffset*6 + 6*(i-1) + 1
         nodeLoads_c(j  ) = m%mbdVSLoads%Force(1,i)
         nodeLoads_c(j+1) = m%mbdVSLoads%Force(2,i)
         nodeLoads_c(j+2) = m%mbdVSLoads%Force(3,i)
         nodeLoads_c(j+3) = m%mbdVSLoads%Moment(1,i)
         nodeLoads_c(j+4) = m%mbdVSLoads%Moment(2,i)
         nodeLoads_c(j+5) = m%mbdVSLoads%Moment(3,i)
      end do       
      compOffset = compOffset + p%numVSNds
 
         ! Starboard horizontal stabilizer
      do i = 1, p%numSHSNds
         j = compOffset*6 + 6*(i-1) + 1
         nodeLoads_c(j  ) = m%mbdSHSLoads%Force(1,i)
         nodeLoads_c(j+1) = m%mbdSHSLoads%Force(2,i)
         nodeLoads_c(j+2) = m%mbdSHSLoads%Force(3,i)
         nodeLoads_c(j+3) = m%mbdSHSLoads%Moment(1,i)
         nodeLoads_c(j+4) = m%mbdSHSLoads%Moment(2,i)
         nodeLoads_c(j+5) = m%mbdSHSLoads%Moment(3,i)
      end do       
      compOffset = compOffset + p%numSHSNds

         ! Port horizontal stabilizer
      do i = 1, p%numPHSNds
         j = compOffset*6 + 6*(i-1) + 1
         nodeLoads_c(j  ) = m%mbdPHSLoads%Force(1,i)
         nodeLoads_c(j+1) = m%mbdPHSLoads%Force(2,i)
         nodeLoads_c(j+2) = m%mbdPHSLoads%Force(3,i)
         nodeLoads_c(j+3) = m%mbdPHSLoads%Moment(1,i)
         nodeLoads_c(j+4) = m%mbdPHSLoads%Moment(2,i)
         nodeLoads_c(j+5) = m%mbdPHSLoads%Moment(3,i)
      end do       
      compOffset = compOffset + p%numPHSNds

      do j = 1 , p%NumPylons  
         do i = 1, p%numSPyNds(j)
            k = compOffset*6 + 6*(i-1) + 1   
            nodeLoads_c(k  ) = m%mbdSPyLoads(j)%Force(1,i)
            nodeLoads_c(k+1) = m%mbdSPyLoads(j)%Force(2,i)
            nodeLoads_c(k+2) = m%mbdSPyLoads(j)%Force(3,i)
            nodeLoads_c(k+3) = m%mbdSPyLoads(j)%Moment(1,i)
            nodeLoads_c(k+4) = m%mbdSPyLoads(j)%Moment(2,i)
            nodeLoads_c(k+5) = m%mbdSPyLoads(j)%Moment(3,i)
         end do
         compOffset = compOffset + p%numSPyNds(j)
      end do
      do j = 1 , p%NumPylons  
         do i = 1, p%numPPyNds(j)
            k = compOffset*6 + 6*(i-1) + 1   
            nodeLoads_c(k  ) = m%mbdPPyLoads(j)%Force(1,i)
            nodeLoads_c(k+1) = m%mbdPPyLoads(j)%Force(2,i)
            nodeLoads_c(k+2) = m%mbdPPyLoads(j)%Force(3,i)
            nodeLoads_c(k+3) = m%mbdPPyLoads(j)%Moment(1,i)
            nodeLoads_c(k+4) = m%mbdPPyLoads(j)%Moment(2,i)
            nodeLoads_c(k+5) = m%mbdPPyLoads(j)%Moment(3,i)
         end do
         compOffset = compOffset + p%numPPyNds(j)
      end do
         
      ! Map  rotor/drivetrain reaction loads to the corresponding MBDyn loads (no mesh mapping required)
      c = 1
      do j = 1, p%NumPylons
         do i = 1,2  
            rtrLoads_c(c  ) = m%SPyRtrFReact(1,i,j)  
            rtrLoads_c(c+1) = m%SPyRtrFReact(2,i,j) 
            rtrLoads_c(c+2) = m%SPyRtrFReact(3,i,j) 
            rtrLoads_c(c+3) = m%SPyRtrMReact(1,i,j) 
            rtrLoads_c(c+4) = m%SPyRtrMReact(2,i,j) 
            rtrLoads_c(c+5) = m%SPyRtrMReact(3,i,j) 
            c = c + 6
         end do
      end do
      do j = 1, p%NumPylons
         do i = 1,2  
            rtrLoads_c(c  ) = m%PPyRtrFReact(1,i,j)  
            rtrLoads_c(c+1) = m%PPyRtrFReact(2,i,j) 
            rtrLoads_c(c+2) = m%PPyRtrFReact(3,i,j) 
            rtrLoads_c(c+3) = m%PPyRtrMReact(1,i,j) 
            rtrLoads_c(c+4) = m%PPyRtrMReact(2,i,j) 
            rtrLoads_c(c+5) = m%PPyRtrMReact(3,i,j) 
            c = c + 6
         end do
      end do
         
   end if
   
end subroutine TransferLoadsToMBDyn

subroutine TransferMBDynInputs2KADMeshes( isInitialTime, FusO, p, m, errStat, errMsg )
   integer(IntKi),            intent(in   ) :: isInitialTime
   real(ReKi),                intent(in   ) :: FusO(3)
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None

   integer(IntKi)   :: i,j, c
   character(*), parameter    :: routineName = 'TransferMBDynInputs2KADMeshes'
   integer(intKi)             :: errStat2          ! temporary Error status
   character(ErrMsgLen)       :: errMsg2           ! temporary Error message

   
   errStat = ErrID_None
   errMsg  = ''
   
   ! NOTE: KAD%u(1) index is for the current timestep!
   
   m%KAD%u%FusOMotions%TranslationDisp(:,1) =  FusO 
   
      ! Map the motions over to the corresponding KAD input meshes.  
   call Transfer_Line2_to_Line2(m%mbdFusMotions, m%KAD%u%FusMotions, m%Fus_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdSWnMotions, m%KAD%u%SWnMotions, m%SWn_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdPWnMotions, m%KAD%u%PWnMotions, m%PWn_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdVSMotions , m%KAD%u%VSMotions , m%VS_L2_L2,  errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdSHSMotions, m%KAD%u%SHSMotions, m%SHS_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdPHSMotions, m%KAD%u%PHSMotions, m%PHS_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   do i = 1, p%numPylons
      call Transfer_Line2_to_Line2(m%mbdSPyMotions(i), m%KAD%u%SPyMotions(i), m%SPy_L2_L2(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )    
      call Transfer_Line2_to_Line2(m%mbdPPyMotions(i), m%KAD%u%PPyMotions(i), m%PPy_L2_L2(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )                
   end do
   
      ! Rotors : Directly transfer these MBDyn motions to the cooresponding KAD input meshes
   do j = 1, p%numPylons
      do i=1,2 ! two per pylon
         c = i+(j-1)*p%numPylons
         m%KAD%u%SPyRtrMotions(c)%TranslationDisp(:,1) =  m%SPyRtrO(:,i,j) - m%KAD%u%SPyRtrMotions(c)%Position(:,1)
         m%KAD%u%SPyRtrMotions(c)%Orientation(:,:,1)   =  m%SPyRtrDCMs(:,:,i,j)
         m%KAD%u%SPyRtrMotions(c)%TranslationVel(:,1)  =  m%SPyRtrVels(:,i,j)
      end do
   end do
   
   do j = 1, p%numPylons
      do i=1,2 ! two per pylon
         c = i+(j-1)*p%numPylons
         m%KAD%u%PPyRtrMotions(c)%TranslationDisp(:,1) = m%PPyRtrO(:,i,j) - m%KAD%u%PPyRtrMotions(c)%Position(:,1)
         m%KAD%u%PPyRtrMotions(c)%Orientation(:,:,1)   = m%PPyRtrDCMs(:,:,i,j)
         m%KAD%u%PPyRtrMotions(c)%TranslationVel(:,1)  = m%PPyRtrVels(:,i,j)
      end do
   end do
   
   !if ( isInitialTime > 0 ) then
      call KAD_CopyInput(m%KAD%u, OtherSt%KAD%u(1), MESH_NEWCOPY, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )  
    ! else
      ! ! Now filter the inputs:   
      ! ! call MeshExtrapInterp1(u1, u2, tin, u_out, tin_out, ErrStat, ErrMsg )
       ! call MeshExtrapInterp1( m%KAD%u%FusOMotions, OtherSt%KAD%u(2)%FusOMotions, (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%FusOMotions, p%KAD_filtConst, errStat2, errMsg2 )
          ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
       ! call MeshExtrapInterp1( m%KAD%u%FusMotions, OtherSt%KAD%u(2)%FusMotions, (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%FusMotions, p%KAD_filtConst, errStat2, errMsg2 )
          ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
       ! call MeshExtrapInterp1( m%KAD%u%SWnMotions, OtherSt%KAD%u(2)%SWnMotions, (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%SWnMotions, p%KAD_filtConst, errStat2, errMsg2 )
          ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
       ! call MeshExtrapInterp1( m%KAD%u%PWnMotions, OtherSt%KAD%u(2)%PWnMotions, (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%PWnMotions, p%KAD_filtConst, errStat2, errMsg2 )
          ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
       ! call MeshExtrapInterp1( m%KAD%u%VSMotions , OtherSt%KAD%u(2)%VSMotions , (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%VSMotions , p%KAD_filtConst, errStat2, errMsg2 )
          ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
       ! call MeshExtrapInterp1( m%KAD%u%SHSMotions, OtherSt%KAD%u(2)%SHSMotions, (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%SHSMotions, p%KAD_filtConst, errStat2, errMsg2 )
          ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
       ! call MeshExtrapInterp1( m%KAD%u%PHSMotions, OtherSt%KAD%u(2)%PHSMotions, (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%PHSMotions, p%KAD_filtConst, errStat2, errMsg2 )
          ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
       ! do j = 1, p%numPylons
          ! call MeshExtrapInterp1( m%KAD%u%SPyMotions(j), OtherSt%KAD%u(2)%SPyMotions(j), (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%SPyMotions(j), p%KAD_filtConst, errStat2, errMsg2 )
             ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
          ! call MeshExtrapInterp1( m%KAD%u%PPyMotions(j), OtherSt%KAD%u(2)%PPyMotions(j), (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%PPyMotions(j), p%KAD_filtConst, errStat2, errMsg2 )
             ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
          ! do i=1,2 ! two per pylon
             ! c = i+(j-1)*p%numPylons
             ! call MeshExtrapInterp1( m%KAD%u%SPyRtrMotions(c), OtherSt%KAD%u(2)%SPyRtrMotions(c), (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%SPyRtrMotions(c), p%KAD_filtConst, errStat2, errMsg2 )
                ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
             ! call MeshExtrapInterp1( m%KAD%u%PPyRtrMotions(c), OtherSt%KAD%u(2)%PPyRtrMotions(c), (/0.0_DbKi, 1.0_DbKi/), OtherSt%KAD%u(1)%PPyRtrMotions(c), p%KAD_filtConst, errStat2, errMsg2 )
                ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
          ! end do
       ! end do
    ! end if
   
end subroutine TransferMBDynInputs2KADMeshes


subroutine TransferMBDynInputs2MBDMeshes( p, m, errStat, errMsg )
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None

   integer(IntKi)   :: i,j
   character(*), parameter    :: routineName = 'TransferMBDynInputs2MBDMeshes'
   
   errStat = ErrID_None
   errMsg  = ''
   
      ! Use the current point locations and the mesh node reference positions to compute the current displacements
      ! Then set the displacements, velocities, and DCMs to the MBDyn-based meshes

   
   do i = 1,p%numFusNds
      m%mbdFusMotions%Orientation  (:,:,i) = m%FusNdDCMs(:,:,i)
      m%mbdFusMotions%TranslationDisp(:,i) = m%FusPts(:,i) - m%mbdFusMotions%Position(:,i)
      m%mbdFusLoads%TranslationDisp(:,i)   = m%mbdFusMotions%TranslationDisp(:,i)
      m%mbdFusMotions%TranslationVel (:,i) = m%FusVels(:,i)
      m%mbdFusMotions%RotationVel    (:,i) = m%FusOmegas(:,i)
   end do
   do i = 1,p%numSwnNds
      m%mbdSWnMotions%Orientation  (:,:,i) = m%SWnNdDCMs(:,:,i)
      m%mbdSWnMotions%TranslationDisp(:,i) = m%SWnPts(:,i) - m%mbdSWnMotions%Position(:,i)
      m%mbdSWnLoads%TranslationDisp(:,i)   = m%mbdSWnMotions%TranslationDisp(:,i)
      m%mbdSWnMotions%TranslationVel (:,i) = m%SWnVels(:,i)
      m%mbdSWnMotions%RotationVel    (:,i) = m%SWnOmegas(:,i)
   end do
   do i = 1,p%numPwnNds
      m%mbdPWnMotions%Orientation  (:,:,i) = m%PWnNdDCMs(:,:,i)
      m%mbdPWnMotions%TranslationDisp(:,i) = m%PWnPts(:,i) - m%mbdPWnMotions%Position(:,i)
      m%mbdPWnLoads%TranslationDisp(:,i)   = m%mbdPWnMotions%TranslationDisp(:,i)
      m%mbdPWnMotions%TranslationVel (:,i) = m%PWnVels(:,i)
      m%mbdPWnMotions%RotationVel    (:,i) = m%PWnOmegas(:,i)
   end do
   do i = 1,p%numVSNds
      m%mbdVSMotions%Orientation  (:,:,i) = m%VSNdDCMs(:,:,i)
      m%mbdVSMotions%TranslationDisp(:,i) = m%VSPts(:,i) - m%mbdVSMotions%Position(:,i)
      m%mbdVSLoads%TranslationDisp(:,i)   = m%mbdVSMotions%TranslationDisp(:,i)
      m%mbdVSMotions%TranslationVel (:,i) = m%VSVels(:,i)
      m%mbdVSMotions%RotationVel    (:,i) = m%VSOmegas(:,i)
   end do
   do i = 1,p%numSHSNds
      m%mbdSHSMotions%Orientation  (:,:,i) = m%SHSNdDCMs(:,:,i)
      m%mbdSHSMotions%TranslationDisp(:,i) = m%SHSPts(:,i) - m%mbdSHSMotions%Position(:,i)
      m%mbdSHSLoads%TranslationDisp(:,i)   = m%mbdSHSMotions%TranslationDisp(:,i)
      m%mbdSHSMotions%TranslationVel (:,i) = m%SHSVels(:,i)
      m%mbdSHSMotions%RotationVel    (:,i) = m%SHSOmegas(:,i)
   end do
   do i = 1,p%numPHSNds
      m%mbdPHSMotions%Orientation  (:,:,i) = m%PHSNdDCMs(:,:,i)
      m%mbdPHSMotions%TranslationDisp(:,i) = m%PHSPts(:,i) - m%mbdPHSMotions%Position(:,i)
      m%mbdPHSLoads%TranslationDisp(:,i)   = m%mbdPHSMotions%TranslationDisp(:,i)
      m%mbdPHSMotions%TranslationVel (:,i) = m%PHSVels(:,i)
      m%mbdPHSMotions%RotationVel    (:,i) = m%PHSOmegas(:,i)
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)
         m%mbdSPyMotions(j)%Orientation  (:,:,i) = m%SPyNdDCMs(:,:,i,j)
         m%mbdSPyMotions(j)%TranslationDisp(:,i) = m%SPyPts(:,i,j) - m%mbdSPyMotions(j)%Position(:,i)
         m%mbdSPyLoads(j)%TranslationDisp(:,i)   = m%mbdSPyMotions(j)%TranslationDisp(:,i) 
         m%mbdSPyMotions(j)%TranslationVel (:,i) = m%SPyVels(:,i,j)
         m%mbdSPyMotions(j)%RotationVel    (:,i) = m%SPyOmegas(:,i,j)
      end do
   end do   
   do j = 1, p%numPylons
      do i = 1, p%numPPyNds(j)
         m%mbdPPyMotions(j)%Orientation  (:,:,i) = m%PPyNdDCMs(:,:,i,j)
         m%mbdPPyMotions(j)%TranslationDisp(:,i) = m%PPyPts(:,i,j) - m%mbdPPyMotions(j)%Position(:,i)
         m%mbdPPyLoads(j)%TranslationDisp(:,i)   = m%mbdPPyMotions(j)%TranslationDisp(:,i)
         m%mbdPPyMotions(j)%TranslationVel (:,i) = m%PPyVels(:,i,j)
         m%mbdPPyMotions(j)%RotationVel    (:,i) = m%PPyOmegas(:,i,j)
      end do
   end do
   
end subroutine TransferMBDynInputs2MBDMeshes

subroutine TransferMBDynLoadInputs( numNodeLoadsPts_c, nodeLoads_c, p, m, errStat, errMsg )
   integer(C_INT),            intent(in   ) :: numNodeLoadsPts_c                 ! total number of array elements in the nodal translational accelerations array
   real(C_DOUBLE),            intent(in   ) :: nodeLoads_c(numNodeLoadsPts_c*6)                        ! 1D array containing all the nodal translational accelerations data
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None


   integer(IntKi)   :: i, j, n

   errStat = ErrID_None
   errMsg  = ''
   
   n=1
   
   do i = 1,p%numFusNds-1
      m%FusLoadInpts(:,i)   = nodeLoads_c(n:n+5)
      n = n + 6
   end do
   do i = 1,p%numSwnNds-1
      m%SwnLoadInpts(:,i)   = nodeLoads_c(n:n+5)
      n = n + 6
   end do
   do i = 1,p%numPwnNds-1
      m%PWnLoadInpts(:,i)   = nodeLoads_c(n:n+5)
      n = n + 6
   end do
   do i = 1,p%numVSNds-1
      m%VSLoadInpts(:,i)    = nodeLoads_c(n:n+5)
      n = n + 6
   end do
   do i = 1,p%numSHSNds-1
      m%SHSLoadInpts(:,i)   = nodeLoads_c(n:n+5)
      n = n + 6
   end do
   do i = 1,p%numPHSNds-1
      m%PHSLoadInpts(:,i)   = nodeLoads_c(n:n+5)
      n = n + 6
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)-1
         m%SPyLoadInpts(:,i,j)   = nodeLoads_c(n:n+5)
         n = n + 6
      end do
   end do
   do j = 1, p%numPylons
      do i = 1, p%numPPyNds(j)-1
         m%PPyLoadInpts(:,i,j)   = nodeLoads_c(n:n+5)
         n = n + 6
      end do
   end do
  
end subroutine TransferMBDynLoadInputs

!====================================================================================================
subroutine KFAST_ProcessOutputs()
! This subroutine 
!----------------------------------------------------------------------------------------------------

   
   integer(IntKi)  :: i, j, iNd
   real(ReKi)      :: val3(3)
   real(R8Ki)      :: dcm(3,3)

   ! Process the outputs component by component
   
   do i = 1, p%NFusOuts
      iNd = p%FusOutNds(i)
      if ( (iNd > 0 ) .and. (iNd <= p%numFusNds) ) then
         val3 = matmul( m%FusODCM, (m%mbdFusMotions%TranslationDisp(:,iNd)  +  m%mbdFusMotions%Position(:,iNd)  - m%FusO) ) - m%mbdFusMotions%Position(:,iNd)
      
         m%AllOuts(FusTDx(i)) = val3(1)
         m%AllOuts(FusTDy(i)) = val3(2)
         m%AllOuts(FusTDz(i)) = val3(3)
      
         dcm = matmul( transpose(m%mbdFusMotions%RefOrientation(:,:,iNd)), m%mbdFusMotions%Orientation(:,:,iNd)  )
         dcm = matmul( transpose(m%FusODCM), dcm )
         val3 = R2D_D*EulerExtract(dcm)
         m%AllOuts(FusRDx(i)) = val3(1)
         m%AllOuts(FusRDy(i)) = val3(2)
         m%AllOuts(FusRDz(i)) = val3(3)
      
      
         val3 = R2D_D*matmul( m%mbdFusMotions%Orientation(:,:,iNd), m%mbdFusMotions%RotationVel(:,iNd) )
         m%AllOuts(FusRVn(i)) = val3(1)
         m%AllOuts(FusRVc(i)) = val3(2)
         m%AllOuts(FusRVs(i)) = val3(3)
      
         val3 = matmul( m%mbdFusMotions%Orientation(:,:,iNd), m%FusAccs(:,iNd) )
         m%AllOuts(FusTAn(i)) = val3(1)
         m%AllOuts(FusTAc(i)) = val3(2)
         m%AllOuts(FusTAs(i)) = val3(3)
      
         if ( iNd < p%numFusNds ) then  ! there is no gauss pt associated with the last FE motion node from MBDyn
            m%AllOuts(FusFRn(i)) = m%FusLoadInpts(1,iNd)
            m%AllOuts(FusFRc(i)) = m%FusLoadInpts(2,iNd)
            m%AllOuts(FusFRs(i)) = m%FusLoadInpts(3,iNd)
            m%AllOuts(FusMRn(i)) = m%FusLoadInpts(4,iNd)
            m%AllOuts(FusMRc(i)) = m%FusLoadInpts(5,iNd)
            m%AllOuts(FusMRs(i)) = m%FusLoadInpts(6,iNd)
         end if
      end if
   end do


   do i = 1, p%NSWnOuts
      iNd = p%SWnOutNds(i)
      if ( (iNd > 0 ) .and. (iNd <= p%numSWnNds) ) then
         val3 = matmul( m%FusODCM, (m%mbdSWnMotions%TranslationDisp(:,iNd)  +  m%mbdSWnMotions%Position(:,iNd)  - m%FusO) ) - m%mbdSWnMotions%Position(:,iNd)
      
         m%AllOuts(SWnTDx(i)) = val3(1)
         m%AllOuts(SWnTDy(i)) = val3(2)
         m%AllOuts(SWnTDz(i)) = val3(3)
      
         dcm = matmul( transpose(m%mbdSWnMotions%RefOrientation(:,:,iNd)), m%mbdSWnMotions%Orientation(:,:,iNd)  )
         dcm = matmul( transpose(m%FusODCM), dcm )
         val3 = R2D_D*EulerExtract(dcm)
         m%AllOuts(SWnRDx(i)) = val3(1)
         m%AllOuts(SWnRDy(i)) = val3(2)
         m%AllOuts(SWnRDz(i)) = val3(3)
      
         val3 = R2D_D*matmul( m%mbdSWnMotions%Orientation(:,:,iNd), m%mbdSWnMotions%RotationVel(:,iNd) )
         m%AllOuts(SWnRVn(i)) = val3(1)
         m%AllOuts(SWnRVc(i)) = val3(2)
         m%AllOuts(SWnRVs(i)) = val3(3)

         val3 = matmul( m%mbdSWnMotions%Orientation(:,:,iNd), m%SWnAccs(:,iNd) )
         m%AllOuts(SWnTAn(i)) = val3(1)
         m%AllOuts(SWnTAc(i)) = val3(2)
         m%AllOuts(SWnTAs(i)) = val3(3)
      
         if ( iNd < p%numSWnNds ) then  ! there is no gauss pt associated with the last FE motion node from MBDyn
            m%AllOuts(SWnFRn(i)) = m%SWnLoadInpts(1,iNd)
            m%AllOuts(SWnFRc(i)) = m%SWnLoadInpts(2,iNd)
            m%AllOuts(SWnFRs(i)) = m%SWnLoadInpts(3,iNd)
            m%AllOuts(SWnMRn(i)) = m%SWnLoadInpts(4,iNd)
            m%AllOuts(SWnMRc(i)) = m%SWnLoadInpts(5,iNd)
            m%AllOuts(SWnMRs(i)) = m%SWnLoadInpts(6,iNd)
         end if
      end if
   end do

   do i = 1, p%NPWnOuts
      iNd = p%PWnOutNds(i)
      if ( (iNd > 0 ) .and. (iNd <= p%numPWnNds) ) then
         val3 = matmul( m%FusODCM, (m%mbdPWnMotions%TranslationDisp(:,iNd)  +  m%mbdPWnMotions%Position(:,iNd)  - m%FusO) ) - m%mbdPWnMotions%Position(:,iNd)
      
         m%AllOuts(PWnTDx(i)) = val3(1)
         m%AllOuts(PWnTDy(i)) = val3(2)
         m%AllOuts(PWnTDz(i)) = val3(3)
      
         dcm = matmul( transpose(m%mbdPWnMotions%RefOrientation(:,:,iNd)), m%mbdPWnMotions%Orientation(:,:,iNd)  )
         dcm = matmul( transpose(m%FusODCM), dcm )
         val3 = R2D_D*EulerExtract(dcm)
         m%AllOuts(PWnRDx(i)) = val3(1)
         m%AllOuts(PWnRDy(i)) = val3(2)
         m%AllOuts(PWnRDz(i)) = val3(3)
      
         val3 = R2D_D*matmul( m%mbdPWnMotions%Orientation(:,:,iNd), m%mbdPWnMotions%RotationVel(:,iNd) )
         m%AllOuts(PWnRVn(i)) = val3(1)
         m%AllOuts(PWnRVc(i)) = val3(2)
         m%AllOuts(PWnRVs(i)) = val3(3)

         val3 = matmul( m%mbdPWnMotions%Orientation(:,:,iNd), m%PWnAccs(:,iNd) )
         m%AllOuts(PWnTAn(i)) = val3(1)
         m%AllOuts(PWnTAc(i)) = val3(2)
         m%AllOuts(PWnTAs(i)) = val3(3)
      
         if ( iNd < p%numPWnNds ) then  ! there is no gauss pt associated with the last FE motion node from MBDyn
            m%AllOuts(PWnFRn(i)) = m%PWnLoadInpts(1,iNd)
            m%AllOuts(PWnFRc(i)) = m%PWnLoadInpts(2,iNd)
            m%AllOuts(PWnFRs(i)) = m%PWnLoadInpts(3,iNd)
            m%AllOuts(PWnMRn(i)) = m%PWnLoadInpts(4,iNd)
            m%AllOuts(PWnMRc(i)) = m%PWnLoadInpts(5,iNd)
            m%AllOuts(PWnMRs(i)) = m%PWnLoadInpts(6,iNd)
         end if
      end if
   end do
   
   do i = 1, p%NVSOuts
      iNd = p%VSOutNds(i)
      if ( (iNd > 0 ) .and. (iNd <= p%numVSNds) ) then
         val3 = matmul( m%FusODCM, (m%mbdVSMotions%TranslationDisp(:,iNd)  +  m%mbdVSMotions%Position(:,iNd)  - m%FusO) ) - m%mbdVSMotions%Position(:,iNd)
      
         m%AllOuts(VSTDx(i)) = val3(1)
         m%AllOuts(VSTDy(i)) = val3(2)
         m%AllOuts(VSTDz(i)) = val3(3)
      
         dcm = matmul( transpose(m%mbdVSMotions%RefOrientation(:,:,iNd)), m%mbdVSMotions%Orientation(:,:,iNd)  )
         dcm = matmul( transpose(m%FusODCM), dcm )
         val3 = R2D_D*EulerExtract(dcm)
         m%AllOuts(VSRDx(i)) = val3(1)
         m%AllOuts(VSRDy(i)) = val3(2)
         m%AllOuts(VSRDz(i)) = val3(3)
      
         val3 = R2D_D*matmul( m%mbdVSMotions%Orientation(:,:,iNd), m%mbdVSMotions%RotationVel(:,iNd) )
         m%AllOuts(VSRVn(i)) = val3(1)
         m%AllOuts(VSRVc(i)) = val3(2)
         m%AllOuts(VSRVs(i)) = val3(3)

         val3 = matmul( m%mbdVSMotions%Orientation(:,:,iNd), m%VSAccs(:,iNd) )
         m%AllOuts(VSTAn(i)) = val3(1)
         m%AllOuts(VSTAc(i)) = val3(2)
         m%AllOuts(VSTAs(i)) = val3(3)
      
         if ( iNd < p%numVSNds ) then  ! there is no gauss pt associated with the last FE motion node from MBDyn
            m%AllOuts(VSFRn(i)) = m%VSLoadInpts(1,iNd)
            m%AllOuts(VSFRc(i)) = m%VSLoadInpts(2,iNd)
            m%AllOuts(VSFRs(i)) = m%VSLoadInpts(3,iNd)
            m%AllOuts(VSMRn(i)) = m%VSLoadInpts(4,iNd)
            m%AllOuts(VSMRc(i)) = m%VSLoadInpts(5,iNd)
            m%AllOuts(VSMRs(i)) = m%VSLoadInpts(6,iNd)
         end if
      end if
   end do
   
   
   do i = 1, p%NSHSOuts
      iNd = p%SHSOutNds(i)
      if ( (iNd > 0 ) .and. (iNd <= p%numSHSNds) ) then
         val3 = matmul( m%FusODCM, (m%mbdSHSMotions%TranslationDisp(:,iNd)  +  m%mbdSHSMotions%Position(:,iNd)  - m%FusO) ) - m%mbdSHSMotions%Position(:,iNd)
      
         m%AllOuts(SHSTDx(i)) = val3(1)
         m%AllOuts(SHSTDy(i)) = val3(2)
         m%AllOuts(SHSTDz(i)) = val3(3)
      
         dcm = matmul( transpose(m%mbdSHSMotions%RefOrientation(:,:,iNd)), m%mbdSHSMotions%Orientation(:,:,iNd)  )
         dcm = matmul( transpose(m%FusODCM), dcm )
         val3 = R2D_D*EulerExtract(dcm)
         m%AllOuts(SHSRDx(i)) = val3(1)
         m%AllOuts(SHSRDy(i)) = val3(2)
         m%AllOuts(SHSRDz(i)) = val3(3)
      
         val3 = R2D_D*matmul( m%mbdSHSMotions%Orientation(:,:,iNd), m%mbdSHSMotions%RotationVel(:,iNd) )
         m%AllOuts(SHSRVn(i)) = val3(1)
         m%AllOuts(SHSRVc(i)) = val3(2)
         m%AllOuts(SHSRVs(i)) = val3(3)

         val3 = matmul( m%mbdSHSMotions%Orientation(:,:,iNd), m%SHSAccs(:,iNd) )
         m%AllOuts(SHSTAn(i)) = val3(1)
         m%AllOuts(SHSTAc(i)) = val3(2)
         m%AllOuts(SHSTAs(i)) = val3(3)
      
         if ( iNd < p%numSHSNds ) then  ! there is no gauss pt associated with the last FE motion node from MBDyn
            m%AllOuts(SHSFRn(i)) = m%SHSLoadInpts(1,iNd)
            m%AllOuts(SHSFRc(i)) = m%SHSLoadInpts(2,iNd)
            m%AllOuts(SHSFRs(i)) = m%SHSLoadInpts(3,iNd)
            m%AllOuts(SHSMRn(i)) = m%SHSLoadInpts(4,iNd)
            m%AllOuts(SHSMRc(i)) = m%SHSLoadInpts(5,iNd)
            m%AllOuts(SHSMRs(i)) = m%SHSLoadInpts(6,iNd)
         end if
      end if
   end do
   
   do i = 1, p%NPHSOuts
      iNd = p%PHSOutNds(i)
      if ( (iNd > 0 ) .and. (iNd <= p%numPHSNds) ) then
         val3 = matmul( m%FusODCM, (m%mbdPHSMotions%TranslationDisp(:,iNd)  +  m%mbdPHSMotions%Position(:,iNd)  - m%FusO) ) - m%mbdPHSMotions%Position(:,iNd)
      
         m%AllOuts(PHSTDx(i)) = val3(1)
         m%AllOuts(PHSTDy(i)) = val3(2)
         m%AllOuts(PHSTDz(i)) = val3(3)
      
         dcm = matmul( transpose(m%mbdPHSMotions%RefOrientation(:,:,iNd)), m%mbdPHSMotions%Orientation(:,:,iNd)  )
         dcm = matmul( transpose(m%FusODCM), dcm )
         val3 = R2D_D*EulerExtract(dcm)
         m%AllOuts(PHSRDx(i)) = val3(1)
         m%AllOuts(PHSRDy(i)) = val3(2)
         m%AllOuts(PHSRDz(i)) = val3(3)
      
         val3 = R2D_D*matmul( m%mbdPHSMotions%Orientation(:,:,iNd), m%mbdPHSMotions%RotationVel(:,iNd) )
         m%AllOuts(PHSRVn(i)) = val3(1)
         m%AllOuts(PHSRVc(i)) = val3(2)
         m%AllOuts(PHSRVs(i)) = val3(3)
      
         val3 = matmul( m%mbdPHSMotions%Orientation(:,:,iNd), m%PHSAccs(:,iNd) )
         m%AllOuts(PHSTAn(i)) = val3(1)
         m%AllOuts(PHSTAc(i)) = val3(2)
         m%AllOuts(PHSTAs(i)) = val3(3)
      
         if ( iNd < p%numPHSNds ) then  ! there is no gauss pt associated with the last FE motion node from MBDyn
            m%AllOuts(PHSFRn(i)) = m%PHSLoadInpts(1,iNd)
            m%AllOuts(PHSFRc(i)) = m%PHSLoadInpts(2,iNd)
            m%AllOuts(PHSFRs(i)) = m%PHSLoadInpts(3,iNd)
            m%AllOuts(PHSMRn(i)) = m%PHSLoadInpts(4,iNd)
            m%AllOuts(PHSMRc(i)) = m%PHSLoadInpts(5,iNd)
            m%AllOuts(PHSMRs(i)) = m%PHSLoadInpts(6,iNd)
         end if
      end if
   end do
   do j = 1, p%NumPylons
      do i = 1, p%NPylOuts
         iNd = p%PylOutNds(i)
         if ( (iNd > 0 ) .and. (iNd <= p%numSPyNds(j)) ) then
            val3 = matmul( m%FusODCM, (m%mbdSPyMotions(j)%TranslationDisp(:,iNd)  +  m%mbdSPyMotions(j)%Position(:,iNd)  - m%FusO) ) - m%mbdSPyMotions(j)%Position(:,iNd)
      
            m%AllOuts(SPTDx(i,j)) = val3(1)
            m%AllOuts(SPTDy(i,j)) = val3(2)
            m%AllOuts(SPTDz(i,j)) = val3(3)
      
            dcm = matmul( transpose(m%mbdSPyMotions(j)%RefOrientation(:,:,iNd)), m%mbdSPyMotions(j)%Orientation(:,:,iNd)  )
            dcm = matmul( transpose(m%FusODCM), dcm )
            val3 = R2D_D*EulerExtract(dcm)
            m%AllOuts(SPRDx(i,j)) = val3(1)
            m%AllOuts(SPRDy(i,j)) = val3(2)
            m%AllOuts(SPRDz(i,j)) = val3(3)
      
            val3 = R2D_D*matmul( m%mbdSPyMotions(j)%Orientation(:,:,iNd), m%mbdSPyMotions(j)%RotationVel(:,iNd) )
            m%AllOuts(SPRVn(i,j)) = val3(1)
            m%AllOuts(SPRVc(i,j)) = val3(2)
            m%AllOuts(SPRVs(i,j)) = val3(3)
      
            val3 = matmul( m%mbdSPyMotions(j)%Orientation(:,:,iNd), m%SPyAccs(:,iNd,j) )
            m%AllOuts(SPTAn(i,j)) = val3(1)
            m%AllOuts(SPTAc(i,j)) = val3(2)
            m%AllOuts(SPTAs(i,j)) = val3(3)
      
            if ( iNd < p%numSPyNds(j) ) then  ! there is no gauss pt associated with the last FE motion node from MBDyn
               m%AllOuts(SPFRn(i,j)) = m%SPyLoadInpts(1,iNd,j)
               m%AllOuts(SPFRc(i,j)) = m%SPyLoadInpts(2,iNd,j)
               m%AllOuts(SPFRs(i,j)) = m%SPyLoadInpts(3,iNd,j)
               m%AllOuts(SPMRn(i,j)) = m%SPyLoadInpts(4,iNd,j)
               m%AllOuts(SPMRc(i,j)) = m%SPyLoadInpts(5,iNd,j)
               m%AllOuts(SPMRs(i,j)) = m%SPyLoadInpts(6,iNd,j)
            end if
         end if
         
         if ( (iNd > 0 ) .and. (iNd <= p%numPPyNds(j)) ) then 
            val3 = matmul( m%FusODCM, (m%mbdPPyMotions(j)%TranslationDisp(:,iNd)  +  m%mbdPPyMotions(j)%Position(:,iNd)  - m%FusO) ) - m%mbdPPyMotions(j)%Position(:,iNd)
      
            m%AllOuts(PPTDx(i,j)) = val3(1)
            m%AllOuts(PPTDy(i,j)) = val3(2)
            m%AllOuts(PPTDz(i,j)) = val3(3)
      
            dcm = matmul( transpose(m%mbdPPyMotions(j)%RefOrientation(:,:,iNd)), m%mbdPPyMotions(j)%Orientation(:,:,iNd)  )
            dcm = matmul( transpose(m%FusODCM), dcm )
            val3 = R2D_D*EulerExtract(dcm)
            m%AllOuts(PPRDx(i,j)) = val3(1)
            m%AllOuts(PPRDy(i,j)) = val3(2)
            m%AllOuts(PPRDz(i,j)) = val3(3)
      
            val3 = R2D_D*matmul( m%mbdPPyMotions(j)%Orientation(:,:,iNd), m%mbdPPyMotions(j)%RotationVel(:,iNd) )
            m%AllOuts(PPRVn(i,j)) = val3(1)
            m%AllOuts(PPRVc(i,j)) = val3(2)
            m%AllOuts(PPRVs(i,j)) = val3(3)
      
            val3 = matmul( m%mbdPPyMotions(j)%Orientation(:,:,iNd), m%PPyAccs(:,iNd,j) )
            m%AllOuts(PPTAn(i,j)) = val3(1)
            m%AllOuts(PPTAc(i,j)) = val3(2)
            m%AllOuts(PPTAs(i,j)) = val3(3)
      
            if ( iNd < p%numPPyNds(j) ) then  ! there is no gauss pt associated with the last FE motion node from MBDyn
               m%AllOuts(PPFRn(i,j)) = m%PPyLoadInpts(1,iNd,j)
               m%AllOuts(PPFRc(i,j)) = m%PPyLoadInpts(2,iNd,j)
               m%AllOuts(PPFRs(i,j)) = m%PPyLoadInpts(3,iNd,j)
               m%AllOuts(PPMRn(i,j)) = m%PPyLoadInpts(4,iNd,j)
               m%AllOuts(PPMRc(i,j)) = m%PPyLoadInpts(5,iNd,j)
               m%AllOuts(PPMRs(i,j)) = m%PPyLoadInpts(6,iNd,j)
            end if
         end if
      end do
   end do
   
   ! Rotor-related outputs
   do i = 1, p%numPylons  
      m%AllOuts(SPTRtSpd(i)) = m%KFC%y%SPyRtrSpd(1,i)
      m%AllOuts(SPBRtSpd(i)) = m%KFC%y%SPyRtrSpd(2,i)
      m%AllOuts(PPTRtSpd(i)) = m%KFC%y%PPyRtrSpd(1,i)
      m%AllOuts(PPBRtSpd(i)) = m%KFC%y%PPyRtrSpd(2,i)
      m%AllOuts(SPTRtAcc(i)) = m%KFC%y%SPyRtrAcc(1,i)
      m%AllOuts(SPBRtAcc(i)) = m%KFC%y%SPyRtrAcc(2,i)
      m%AllOuts(PPTRtAcc(i)) = m%KFC%y%PPyRtrAcc(1,i)
      m%AllOuts(PPBRtAcc(i)) = m%KFC%y%PPyRtrAcc(2,i)
   end do
   
   ! Kite Motions

   m%AllOuts(KitePxi  ) = m%FusO(1)
   m%AllOuts(KitePyi  ) = m%FusO(2)
   m%AllOuts(KitePzi  ) = m%FusO(3)
   val3 = R2D_D*EulerExtract(m%FusODCM)
   m%AllOuts(KiteRoll ) = val3(1)
   m%AllOuts(KitePitch) = val3(2)
   m%AllOuts(KiteYaw  ) = val3(3)
   val3 = matmul(m%FusODCM,m%FusOv)
   m%AllOuts(KiteTVx  ) = val3(1)
   m%AllOuts(KiteTVy  ) = val3(2)
   m%AllOuts(KiteTVz  ) = val3(3)
   val3 = R2D_D*matmul(m%FusODCM,m%FusOomegas)
   m%AllOuts(KiteRVx  ) = val3(1)
   m%AllOuts(KiteRVy  ) = val3(2)
   m%AllOuts(KiteRVz  ) = val3(3)
   val3 = matmul(m%FusODCM,m%FusOaccs)
   m%AllOuts(KiteTAx  ) = val3(1)
   m%AllOuts(KiteTAy  ) = val3(2)
   m%AllOuts(KiteTAz  ) = val3(3)
   val3 = R2D_D*matmul(m%FusODCM,m%FusOalphas)
   m%AllOuts(KiteRAx  ) = val3(1)
   m%AllOuts(KiteRAy  ) = val3(2)
   m%AllOuts(KiteRAz  ) = val3(3)
   
   !...............................................................................................................................
   ! Place the selected output channels into the WriteOutput(:) array with the proper sign:
   !...............................................................................................................................

   dO i = 1,p%numKFASTOuts  ! Loop through all selected output channels
      if (p%OutParam(i)%Indx == 0 ) then
         m%WriteOutput(i) = 0.0
      else
         m%WriteOutput(i) = p%OutParam(i)%SignM * m%AllOuts( p%OutParam(i)%Indx )
      end if
   end do             ! i - All selected output channels

end subroutine KFAST_ProcessOutputs

subroutine Init_KiteSystem(dt_c, numFlaps, numPylons, numComp, numCompNds, modFlags, KAD_FileName_c, IfW_FileName_c, MD_FileName_c, KFC_FileName_c, &
                       outFileRoot_c, printSum, gravity, KAD_InterpOrder, MD_InitInp, FusODCM_c, numRtrPts_c, rtrPts_c, rtrMass_c, rtrI_Rot_c, rtrI_trans_c, rtrXcm_c, refPts_c, &
                       numNodePts_c, nodePts_c, nodeDCMs_c, nFusOuts_c, FusOutNd_c, nSWnOuts_c, SWnOutNd_c, &
                       nPWnOuts_c, PWnOutNd_c, nVSOuts_c, VSOutNd_c, nSHSOuts_c, SHSOutNd_c, nPHSOuts_c, PHSOutNd_c, nPylOuts_c, PylOutNd_c, &
                       KAD_InitOut, MD_InitOut, IfW_InitOut, errStat, errMsg )

   real(C_DOUBLE),           intent(in   ) :: dt_c                           ! Timestep size (s)
   integer(C_INT),           intent(in   ) :: numFlaps                       ! Number of flaps
   integer(C_INT),           intent(in   ) :: numPylons                      ! Number of pylons, per wing
   integer(C_INT),           intent(in   ) :: numComp                        ! Number of kite components
   integer(C_INT),           intent(in   ) :: numCompNds(numComp)            ! MBDyn nodes per component.  The array is populated in the following order: Fuselage, Starboard Wing, Port Wing, Vertical Stabilizer, 
                                                                             !    Starboard Horizontal Stabilizer, Port Horizontal Stabilizer, Starboard pylon, from inner to outer, and then Port pylon, from inner to outer.
   integer(C_INT),           intent(in   ) :: modFlags(4)                    ! Four element array of module flags.  0 = inactive, 1 = active. Module indices are: 0 = KiteAeroDyn, 1 = InflowWind, 2 = MoorDyn, 3 = KiteFAST Controller
   character(kind=C_CHAR),   intent(in   ) :: KAD_FileName_c(IntfStrLen)     ! Full path and name of the KiteAeroDyn input file.
   character(kind=C_CHAR),   intent(in   ) :: IfW_FileName_c(IntfStrLen)     ! Full path and name of the InflowWind input file.
   character(kind=C_CHAR),   intent(in   ) :: MD_FileName_c(IntfStrLen)      ! Full path and name of the MoorDyn input file.
   character(kind=C_CHAR),   intent(in   ) :: KFC_FileName_c(IntfStrLen)     ! Full path and name of the KiteFAST controller shared object file.
   character(kind=C_CHAR),   intent(in   ) :: outFileRoot_c(IntfStrLen)      ! Full path and basename of the KiteFAST output file.
   integer(C_INT),           intent(in   ) :: printSum                       ! Print the Summary file?  1 = Yes, 0 = No.
   real(C_DOUBLE),           intent(in   ) :: gravity                        ! Scalar gravity constant.  (m/s^2)
   integer(C_INT),           intent(in   ) :: KAD_InterpOrder                ! KiteAeroDyn outputs interpolation order. 0 = hold outputs between calls, 1 = linear interpolation, 2 = 2nd order interpolation.
   type(MD_InitInputType),   intent(inout) :: MD_InitInp                     ! MoorDyn Tether initialization inputs
   real(C_DOUBLE),           intent(in   ) :: FusODCM_c(9)                   ! Initial DCM matrix to transform the location of the Kite Fuselage reference point from global to kite coordinates.
   integer(C_INT),           intent(in   ) :: numRtrPts_c                    ! Total number of rotor points (both wings).
   real(C_DOUBLE),           intent(in   ) :: rtrPts_c(numRtrPts_c*3)                    ! Initial location of each rotor's reference point [RRP] in global coordinates. (m)
   real(C_DOUBLE),           intent(in   ) :: rtrMass_c(numRtrPts_c)                   ! Mass of the rotor/drivetrain (kg)
   real(C_DOUBLE),           intent(in   ) :: rtrI_Rot_c(numRtrPts_c)                  ! Rotational inertia about the shaft axis of the top and bottom rotors/drivetrains on the pylons on the wing meshes (kg-m2)
   real(C_DOUBLE),           intent(in   ) :: rtrI_trans_c(numRtrPts_c)                ! Transverse inertia about the rotor reference point of the top and bottom rotors/drivetrains on the pylons on the wing meshes (kg-m2)
   real(C_DOUBLE),           intent(in   ) :: rtrXcm_c(numRtrPts_c)                    ! Distance along the shaft from the rotor reference point of the top and bottom rotors/drivetrains on the pylons on the wing meshes to the center of mass of the rotor/drivetrain (positive along positive x) (m)
   real(C_DOUBLE),           intent(in   ) :: refPts_c(numComp*3)                    ! Initial location of the MBDyn component reference points in the global coordinates. (m)  The length of this array comes from  numComp * 3.
   integer(C_INT),           intent(in   ) :: numNodePts_c                   ! The total number of MBDyn structural nodes.  We need this total number (which could be derived from the numCompNds array) to size the following arrays in the Fortran code. 
   real(C_DOUBLE),           intent(in   ) :: nodePts_c(numNodePts_c*3)                   ! Initial location of the MBDyn structural nodes in the global coordinates. (m)  The array is populated in the same order at the numCompNds array.
   real(C_DOUBLE),           intent(in   ) :: nodeDCMs_c(numNodePts_c*9)                  ! Initial DCMs matrices to transform each nodal point from global to kite coordinates.
   integer(C_INT),           intent(in   ) :: nFusOuts_c                     ! Number of user-requested output locations on the fuselage  ( 0-9 )
   integer(C_INT),           intent(in   ) :: FusOutNd_c(nFusOuts_c)         ! Node number(s) (within the component) of the requested output locations.  Structural node index for motions and KiteAeroDyn quantities,  and Gauss point index for MBDyn structural loads
   integer(C_INT),           intent(in   ) :: nSWnOuts_c                     ! Number of user-requested output locations on the starboard wing  ( 0-9 )
   integer(C_INT),           intent(in   ) :: SWnOutNd_c(nSWnOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),           intent(in   ) :: nPWnOuts_c                     ! Number of user-requested output locations on the port wing  ( 0-9 )
   integer(C_INT),           intent(in   ) :: PWnOutNd_c(nPWnOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),           intent(in   ) :: nVSOuts_c                      ! Number of user-requested output locations on the vertical stabilizer  ( 0-9 )
   integer(C_INT),           intent(in   ) :: VSOutNd_c(nVSOuts_c)           ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),           intent(in   ) :: nSHSOuts_c                     ! Number of user-requested output locations on the starboard horizontal stabilizer  ( 0-9 )
   integer(C_INT),           intent(in   ) :: SHSOutNd_c(nSHSOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),           intent(in   ) :: nPHSOuts_c                     ! Number of user-requested output locations on the port horizontal stabilizer  ( 0-9 )
   integer(C_INT),           intent(in   ) :: PHSOutNd_c(nPHSOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),           intent(in   ) :: nPylOuts_c                     ! Number of user-requested output locations on each pylon  ( 0-9 )
   integer(C_INT),           intent(in   ) :: PylOutNd_c(nPylOuts_c)         ! Node number(s) (within the component) of the requested output locations.
   type(KAD_InitOutputType), intent(  out) :: KAD_InitOut
   type(MD_InitOutputType) , intent(  out) :: MD_InitOut
   type(InflowWind_InitOutputType),intent(  out) :: IfW_InitOut
   integer(IntKi),         intent(inout) :: errStat                        ! Error code coming from KiteFAST
   character(ErrMsgLen),   intent(inout) :: errMsg                         ! Error message

      ! Local variables
   real(DbKi)                      :: dt, test
   real(ReKi)                      :: FusO(3)
   type(KAD_InitInputType)         :: KAD_InitInp
   integer(IntKi)                  :: errStat2
   character(ErrMsgLen)            :: errMsg2
   type(InflowWind_InitInputType)  :: IfW_InitInp 
   type(KFC_InitInputType)         :: KFC_InitInp
   character(*), parameter         :: routineName = 'KFAST_Init'
   integer(IntKi)                  :: i,j,c, count, maxSPyNds, maxPPyNds, SumFileUnit
   real(DbKi)                      :: interval
   character, pointer              :: chanName_f(:)   
   character(255)                  :: enabledModules
   character(255)                  :: disabledModules
   integer(IntKi) :: lenstr, maxPyNds
   CHARACTER(ChanLen)              :: tmpStr

   ! Initialize all the sub-modules : MoorDyn, KiteAeroDyn, Controller, and InflowWind
   ! Set KiteFAST-level parameters, misc vars
   ! Open an Output file
   errStat = ErrID_None
   errMsg  = ''

   dt          = dt_c
   p%numFlaps  = numFlaps
   p%numPylons = numPylons
   
   ! Initialize miscVars and OtherStates
   
   call AllocAry( OtherSt%SPyRtrLoads, 3, 2, p%numPylons, 'OtherSt%SPyRtrLoads', errStat2,errMsg2 )
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry( OtherSt%PPyRtrLoads, 3, 2, p%numPylons, 'OtherSt%PPyRtrLoads', errStat2,errMsg2 )
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry( OtherSt%SPyRtrDCMs, 3, 3, 2, numPylons, 'OtherSt%SPyRtrDCMs', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( OtherSt%PPyRtrDCMs, 3, 3, 2, numPylons, 'OtherSt%PPyRtrDCMs', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )   
         
      ! Set the Rotor-related parameters
   call AllocAry(p%SPyRtrMass, 2, p%numPylons, 'p%SPyRtrMass', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(p%PPyRtrMass, 2, p%numPylons, 'p%PPyRtrMass', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(p%SPyRtrIrot, 2, p%numPylons, 'p%SPyRtrIrot', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(p%PPyRtrIrot, 2, p%numPylons, 'p%PPyRtrIrot', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(p%SPyRtrItrans, 2, p%numPylons, 'p%SPyRtrItrans', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(p%PPyRtrItrans, 2, p%numPylons, 'p%PPyRtrItrans', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(p%SPyRtrXcm, 2, p%numPylons, 'p%SPyRtrXcm', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(p%PPyRtrXcm, 2, p%numPylons, 'p%PPyRtrXcm', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)  
   call AllocAry(m%SPyRtrFReact, 3, 2, p%numPylons, 'm%SPyRtrFReact', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(m%PPyRtrFReact, 3, 2, p%numPylons, 'm%PPyRtrFReact', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(m%SPyRtrMReact, 3, 2, p%numPylons, 'm%SPyRtrMReact', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(m%PPyRtrMReact, 3, 2, p%numPylons, 'm%PPyRtrMReact', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   if (errStat >= AbortErrLev ) then
      return
   end if
   
      ! Initialize these to zero in case we are not using KAD and hence will never compute them
   OtherSt%SPyRtrLoads = 0.0_ReKi
   OtherSt%PPyRtrLoads = 0.0_ReKi
   
   count = 1
   do j = 1, p%numPylons
      do i = 1,2
         p%SPyRtrXcm(i,j)    = rtrXcm_c(count)    
         p%PPyRtrXcm(i,j)    = rtrXcm_c(numRtrPts_c/2+count)  
         
         p%SPyRtrMass(i,j)   = rtrMass_c(count)  
            if ( p%SPyRtrMass(i,j) < 0.0_RekI )  call SetErrStat( ErrID_Fatal, 'Rotor mass must be greater or equal to zero', errStat, errMsg, routineName )           
         p%PPyRtrMass(i,j)   = rtrMass_c(numRtrPts_c/2+count)  
            if ( p%PPyRtrMass(i,j) < 0.0_RekI )  call SetErrStat( ErrID_Fatal, 'Rotor mass must be greater or equal to zero', errStat, errMsg, routineName )
         p%SPyRtrIrot(i,j)   = rtrI_Rot_c(count)  
            if ( p%SPyRtrIrot(i,j) < 0.0_RekI )  call SetErrStat( ErrID_Fatal, 'Rotor rotational inertia must be greater or equal to zero', errStat, errMsg, routineName )       
         p%PPyRtrIrot(i,j)   = rtrI_Rot_c(numRtrPts_c/2+count)  
            if ( p%PPyRtrIrot(i,j) < 0.0_RekI )  call SetErrStat( ErrID_Fatal, 'Rotor rotational inertia must be greater or equal to zero', errStat, errMsg, routineName ) 
         p%SPyRtrItrans(i,j) = rtrI_Trans_c(count)    
            if ( (p%SPyRtrItrans(i,j) - p%SPyRtrMass(i,j)*p%SPyRtrXcm(i,j)*p%SPyRtrXcm(i,j)  ) < 0.0_RekI )  call SetErrStat( ErrID_Fatal, 'Rotor translational inertia relative to rotor CM must be greater or equal to zero', errStat, errMsg, routineName )    
         p%PPyRtrItrans(i,j) = rtrI_Trans_c(numRtrPts_c/2+count)  
             if ( (p%PPyRtrItrans(i,j) - p%PPyRtrMass(i,j)*p%PPyRtrXcm(i,j)*p%PPyRtrXcm(i,j)  ) < 0.0_RekI )  call SetErrStat( ErrID_Fatal, 'Rotor translational inertia relative to rotor CM must be greater or equal to zero', errStat, errMsg, routineName )    
        
         count = count + 1
      end do
   end do
   
   
   call SetupMBDynMotionData()
      if (errStat >= AbortErrLev ) then
         return
      end if
      
!----------------------------------------------------------------
! Initialize the KiteAeroDyn Module
!----------------------------------------------------------------

      ! Set KiteAeroDyn initialization input data

   if (p%useKAD) then
      p%KAD_InterpOrder = KAD_InterpOrder
     print *, "KiteAeroDyn outputs using InterpOrder = ", p%KAD_InterpOrder
      KAD_InitInp%FileName  = transfer(KAD_FileName_c(1:IntfStrLen-1),KAD_InitInp%FileName)
      call RemoveNullChar(KAD_InitInp%FileName)
      KAD_InitInp%NumFlaps  = numFlaps
      KAD_InitInp%NumPylons = numPylons
      KAD_InitInp%outFileRoot = p%outFileRoot      
      interval              = dt
   
         ! Determine Kite component reference point locations in the kite coordinate system
              
      call AllocAry( KAD_InitInp%SPyOR, 3, numPylons, 'InitInp%SPyOR', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call AllocAry( KAD_InitInp%PPyOR, 3, numPylons, 'InitInp%PPyOR', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call AllocAry( KAD_InitInp%SPyRtrOR, 3, 2, numPylons, 'InitInp%SPyRtrOR', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call AllocAry( KAD_InitInp%PPyRtrOR, 3, 2, numPylons, 'InitInp%PPyRtrOR', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

      if (errStat >= AbortErrLev ) then
         return
      end if


      ! Set the component reference point locations as expressed in the kite-local coordinate system
      ! NOTE: The data coming from the Preprocessor is already in the kite-local system, so there are no transformation needed.
      KAD_InitInp%SWnOR =  m%SwnO 
      KAD_InitInp%PWnOR =  m%PwnO 
      KAD_InitInp%VSOR  =  m%VSO 
      KAD_InitInp%SHSOR =  m%SHSO 
      KAD_InitInp%PHSOR =  m%PHSO 
   
      do i = 1, numPylons
         KAD_InitInp%SPyOR(:,i) = m%SPyO(:,i)
      end do
      do i = 1, numPylons
         KAD_InitInp%PPyOR(:,i) = m%PPyO(:,i)
      end do
  
      do i = 1, numPylons
         KAD_InitInp%SPyRtrOR(:,1,i) = m%SPyRtrO(:,1,i)
         KAD_InitInp%SPyRtrOR(:,2,i) = m%SPyRtrO(:,2,i)
      end do
      do i = 1, numPylons
         KAD_InitInp%PPyRtrOR(:,1,i) = m%PPyRtrO(:,1,i)
         KAD_InitInp%PPyRtrOR(:,2,i) = m%PPyRtrO(:,2,i)
      end do
      
      if (p%KAD_InterpOrder == 0) then
         allocate( OtherSt%KAD%u( 2 ), OtherSt%KAD%t_in( 2 ), OtherSt%KAD%y( 1 ),stat = errStat2 )
      else
         allocate( OtherSt%KAD%u( p%KAD_InterpOrder+1 ), OtherSt%KAD%t_in( p%KAD_InterpOrder+1 ), OtherSt%KAD%y( p%KAD_InterpOrder+1 ),stat = errStat2 )
      end if
      if (ErrStat2 /= 0) then
         call SetErrStat(ErrID_Fatal,"Error allocating OtherSt%KAD%u.",errStat,errMsg,RoutineName)
         return
      end if
      
      call KAD_Init(KAD_InitInp, OtherSt%KAD%u(1), m%KAD%p, OtherSt%KAD%y(1), interval, m%KAD%x, m%KAD%xd, OtherSt%KAD%z, OtherSt%KAD%OtherSt, m%KAD%m, KAD_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
        
         ! NOTE: For the KAD input/output/time arrays the 1 index is for the most recent time, and then increasing indices move backwards in time!
      
      call KAD_CopyInput(OtherSt%KAD%u(1),m%KAD%u, MESH_NEWCOPY, errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call KAD_CopyOutput(OtherSt%KAD%y(1),m%KAD%y, MESH_NEWCOPY, errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call KAD_CopyInput(OtherSt%KAD%u(1),OtherSt%KAD%u(2), MESH_NEWCOPY, errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName ) 
         
      if (p%KAD_InterpOrder > 0) then! 1st and 2nd order    
         call KAD_CopyOutput(OtherSt%KAD%y(1),OtherSt%KAD%y(2), MESH_NEWCOPY, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )       
      end if
      if (p%KAD_InterpOrder == 2) then! 2nd order
         call KAD_CopyInput(OtherSt%KAD%u(1),OtherSt%KAD%u(3), MESH_NEWCOPY, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )  
         call KAD_CopyOutput(OtherSt%KAD%y(1),OtherSt%KAD%y(3), MESH_NEWCOPY, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )         
      end if

      
      p%KAD_dt = interval
      p%KAD_nCycles = nint(interval / dt) 
      print *, "Number of MBDyn Cycles per KAD steps: ", p%KAD_nCycles
      if ( .not. EqualRealNos(real(p%KAD_nCycles*dt - p%KAD_dt, ReKi), 0.0_ReKi) ) then
         call SetErrStat(ErrID_Fatal,'KiteAeroDyn DT must be an integer multiple of MBDyn DT',errStat,errMsg,routineName) 
      end if
      OtherSt%KAD_NewTime = .true.  ! This flag is needed to tell us when we have advanced time, and hence need to call the KAD's Step routines

         ! Set the cut off frequency for the KAD inputs low-pass filter
         ! Currently set to 10 Hz
      p%KAD_filtConst = exp(-2000.0_DbKi*PI_D*p%KAD_dt)
      
      if (errStat >= AbortErrLev ) then
         return
      end if
      
      p%AirDens = KAD_InitOut%AirDens
   else
      p%AirDens = 1.225_ReKi   ! Default air density
      KAD_InitOut%nIfWPts = 0  
      p%KAD_nCycles = 1
   end if
                       
!----------------------------------------------------------------
! Initialize the InflowWind Module
!----------------------------------------------------------------
   
   
   if (p%useIfW) then
      IfW_InitInp%InputFileName  = transfer(IfW_FileName_c(1:IntfStrLen-1),IfW_InitInp%InputFileName)
      call RemoveNullChar(IfW_InitInp%InputFileName)
      IfW_InitInp%Linearize         = .false.     
      IfW_InitInp%RootName          = TRIM(p%outFileRoot)//'.IfW'
      IfW_InitInp%UseInputFile      = .TRUE.
      IfW_InitInp%NumWindPoints     = KAD_InitOut%nIfWPts  + 2 
      IfW_InitInp%lidar%Tmax        = 0.0_ReKi
      IfW_InitInp%lidar%HubPosition = 0.0_ReKi
      IfW_InitInp%lidar%SensorType  = SensorType_None
      IfW_InitInp%Use4Dext          = .false.
      interval                      = dt
      call InflowWind_Init( IfW_InitInp, m%IfW%u, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z,  &
                            OtherSt%IfW%OtherSt, m%IfW%y, m%IfW%m, interval, IfW_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         
       if ( .not. EqualRealNos(real(interval, ReKi), real(dt,ReKi)) ) then
        call SetErrStat(ErrID_Fatal,'InflowWind DT must be equal to MBDyn DT',errStat,errMsg,routineName) 
       end if
       
      if (errStat >= AbortErrLev ) then
         return
      end if
      
      call InflowWind_CopyOutput(m%IfW%y, m%IfW%y_KFC, MESH_NEWCOPY, errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         
      deallocate(m%IfW%y_KFC%VelocityUVW)
      
      call AllocAry(m%IfW%u_KFC%PositionXYZ, 3, 2, 'm%IfW%u_KFC%PositionXYZ', errStat2,errMsg2)
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      call AllocAry(m%IfW%y_KFC%VelocityUVW, 3, 2, 'm%IfW%y_KFC%VelocityUVW', errStat2,errMsg2)
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         return
      end if   
   end if
   
!----------------------------------------------------------------
! Initialize the MoorDyn Module
!----------------------------------------------------------------

   if (p%useMD_Tether) then
      
      interval             = dt
   
      call MD_Init( MD_InitInp, m%MD_Tether%u(1), m%MD_Tether%p, OtherSt%MD_Tether%x, m%MD_Tether%xd, m%MD_Tether%z, &
                     OtherSt%MD_Tether%OtherSt, m%MD_Tether%y, m%MD_Tether%m, interval, MD_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         
      if ( .not. EqualRealNos(interval,dt) ) then
         call SetErrStat(ErrID_Fatal,'MoorDyn DT must be equal to MBDyn DT',errStat,errMsg,routineName) 
      end if
       
      if (errStat >= AbortErrLev ) then
         return
      end if
         
            ! Copy t+dt inputs to t for the next timestep
         call MD_CopyInput( m%MD_Tether%u(1), m%MD_Tether%u(2), MESH_NEWCOPY, errStat2, errMsg2 )
   
      if ( MD_InitOut%NAnchs > 1 ) then
         call SetErrStat(ErrID_Fatal,'KiteFAST can only support a single anchor point in the MoorDyn model',errStat,errMsg,routineName)
         if (errStat >= AbortErrLev ) then
            return
         end if
      end if
      if (MD_InitOut%NAnchs == 1 ) then
         p%anchorPt = MD_InitOut%Anchs(:,1)  
      else
         p%anchorPt = 0.0_ReKi
         if (size(m%MD_Tether%u(1)%PtFairleadDisplacement,1 ) == 2 ) then
            
                     ! Create the necessary Meshes to transfer motions and loads between the Platform reference point and the Mooring Platform Fairleads
            call CreateMBDynPtfmMeshes(MD_InitInp%PtfmPos(:,2), real(MD_InitInp%PtfmDCM(:,:,2),R8Ki), m%mbdPtfmMotions, m%mbdPtfmLoads, errStat, errMsg )   
               if (errStat>=AbortErrLev) return
         
        
               ! Need to transfer the MBDyn platform reference point motions to MoorDyn_Tether module
            call MeshMapCreate( m%mbdPtfmMotions, m%MD_Tether%u(1)%PtFairleadDisplacement(2), m%mbdPtfm_P_2_MD_P, errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%mbdPtfm_P_2_MD_P' )     
                     if (errStat>=AbortErrLev) return
            m%mbdPtfmMotions%RemapFlag = .false.
            m%MD_Tether%u(1)%PtFairleadDisplacement(2)%RemapFlag = .false.
            
               ! Need to transfer the MoorDyn Platform loads to the MBDyn platform reference point loads mesh        
            call MeshMapCreate( m%MD_Tether%y%PtFairleadLoad(2), m%mbdPtfmLoads,  m%MD_P_2_mbdPtfm_P, errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%MD_P_2_mbdPtfm_P' )     
                     if (ErrStat>=AbortErrLev) return
            m%MD_Tether%y%PtFairleadLoad(2)%RemapFlag = .false.
            m%mbdWngLoads%RemapFlag = .false.
         end if
      end if
   else
      p%anchorPt = 0.0_ReKi
   end if
   
!----------------------------------------------------------------
! Initialize the Controller Module
!----------------------------------------------------------------
  
   KFC_InitInp%DLL_FileName = transfer(KFC_FileName_c(1:IntfStrLen-1),KFC_InitInp%DLL_FileName)
   call RemoveNullChar(KFC_InitInp%DLL_FileName)

      ! Set the DCM between FAST inertial frame and the Controller ground frame
   p%DCM_Fast2Ctrl = reshape((/-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0/),(/3,3/))
      
   KFC_InitInp%numPylons    = numPylons
   KFC_InitInp%numFlaps     = numFlaps
   
   call AllocAry(KFC_InitInp%SPyRtrIrot, 2, p%numPylons, 'KFC_InitInp%SPyRtrIrot', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
   call AllocAry(KFC_InitInp%PPyRtrIrot, 2, p%numPylons, 'KFC_InitInp%PPyRtrIrot', errStat2,errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      
   interval = dt
   KFC_InitInp%SPyRtrIrot = p%SPyRtrIrot 
   KFC_InitInp%PPyRtrIrot = p%PPyRtrIrot
   
   ! Are we actually calling into a controller DLL / SO, or are we simply using a dummy controller to set the rotor speed (constant and hardcoded in the Fortran KFC_Init() routine)
   !   and set the generator torque to be equal and opposite to the aerodynamic torque.
      
   if (p%useKFC) then
      KFC_InitInp%UseDummy = .false.
   else
      KFC_InitInp%UseDummy = .true.
   end if
      
   call KFC_Init(KFC_InitInp, m%KFC%u, m%KFC%p, m%KFC%y, interval, errStat2, errMsg2 )
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         return
      end if
   p%KFC_dt = interval
   p%KFC_nCycles = nint(interval / dt) 
   print *, "Number of MBDyn Cycles per KFC steps: ", p%KFC_nCycles
   
   if ( .not. EqualRealNos(real(p%KFC_nCycles*dt - p%KFC_dt,ReKi), 0.0_ReKi) ) then
      call SetErrStat(ErrID_Fatal,'KiteFastController DT must be an integer multiple of MBDyn DT',errStat,errMsg,routineName) 
   end if

   OtherSt%KFC_NewTime = .false.  ! This flag is needed to tell us when we have advanced time, and hence need to call the Controller's Step routines
                                  ! Note: We do not step the controller during the initial call to AssRes()
                                  
! -------------------------------------------------------------------------
! Initialize mesh-mapping data
! -------------------------------------------------------------------------

   call CreateMeshMappings(m, p, OtherSt%KAD, m%MD_Tether, errStat2, errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         return
      end if

! -------------------------------------------------------------------------
! Process the KiteFAST requested output channels
! -------------------------------------------------------------------------      

   p%NFusOuts     = nFusOuts_c
   p%NSWnOuts     = nSWnOuts_c
   p%NPWnOuts     = nPWnOuts_c
   p%NVSOuts      = nVSOuts_c
   p%NSHSOuts     = nSHSOuts_c
   p%NPHSOuts     = nPHSOuts_c
   p%NPWnOuts     = nPWnOuts_c
   p%NPylOuts     = nPylOuts_c
   
   call AllocAry(m%FusLoadInpts,6, p%numFusNds-1, 'm%FusLoadInpts', errStat2, errMsg2)
   call AllocAry(m%SWnLoadInpts,6, p%numSWnNds-1, 'm%SWnLoadInpts', errStat2, errMsg2)
   call AllocAry(m%PWnLoadInpts,6, p%numPWnNds-1, 'm%PWnLoadInpts', errStat2, errMsg2)
   call AllocAry(m%VSLoadInpts ,6, p%numVSNds -1, 'm%VSLoadInpts', errStat2, errMsg2)
   call AllocAry(m%SHSLoadInpts,6, p%numSHSNds-1, 'm%SHSLoadInpts', errStat2, errMsg2)
   call AllocAry(m%PHSLoadInpts,6, p%numPHSNds-1, 'm%PHSLoadInpts', errStat2, errMsg2)
   
   maxPyNds = 0
   do i = 1, p%numPylons
      maxPyNds = max(maxPyNds, p%numSPyNds(i))
   end do
   
   call AllocAry(m%SPyLoadInpts,6, maxPyNds-1, p%numPylons, 'm%SPyLoadInpts', errStat2, errMsg2)
   maxPyNds = 0
   do i = 1, p%numPylons
      maxPyNds = max(maxPyNds, p%numPPyNds(i))
   end do
   call AllocAry(m%PPyLoadInpts,6, maxPyNds-1, p%numPylons, 'm%PPyLoadInpts', errStat2, errMsg2)
   
   do i = 1, p%NFusOuts
      p%FusOutNds(i)  = FusOutNd_c(i)
   end do
   do i = 1, p%NSWnOuts
      p%SWnOutNds(i)  = SWnOutNd_c(i)
   end do
   do i = 1, p%NPWnOuts
      p%PWnOutNds(i)  = PWnOutNd_c(i)
   end do
   do i = 1, p%NVSOuts
      p%VSOutNds(i)  = VSOutNd_c(i)
   end do
   do i = 1, p%NSHSOuts
      p%SHSOutNds(i)  = SHSOutNd_c(i)
   end do
   do i = 1, p%NPHSOuts
      p%PHSOutNds(i)  = PHSOutNd_c(i)
   end do
   do i = 1, p%NPylOuts
      p%PylOutNds(i)  = PylOutNd_c(i)
   end do
   
   contains
                       
      subroutine SetupMBDynMotionData( )

            ! Break out the number of structural nodes for a given component
         p%numFusNds = numCompNds(1)
         p%numSwnNds = numCompNds(2)
         p%numPWnNds = numCompNds(3)
         p%numVSNds  = numCompNds(4)
         p%numSHSNds = numCompNds(5)
         p%numPHSNds = numCompNds(6)
         c=7
   
         call AllocAry( p%numSPyNds, p%numPylons, 'p%numSPyNds', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( p%numPPyNds, p%numPylons, 'p%numPPyNds', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

         call AllocAry( m%SPyO, 3, p%numPylons, 'm%SPyO', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PPyO, 3, p%numPylons, 'm%PPyO', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
         maxSPyNds = 0
         maxPPyNds = 0
         do i = 1, p%numPylons
            p%numSPyNds(i) = numCompNds(c)
            if ( p%numSPyNds(i) >  maxSPyNds ) maxSPyNds = p%numSPyNds(i)
            c = c + 1
         end do
         do i = 1, p%numPylons    
            p%numPPyNds(i) = numCompNds(c)
            if ( p%numPPyNds(i) >  maxPPyNds ) maxPPyNds = p%numPPyNds(i)
            c = c + 1
         end do
   
            ! Allocate rotor data
 
         call AllocAry( m%SPyRtrO, 3,2,p%numPylons, 'm%SPyRtrO', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PPyRtrO, 3,2,p%numPylons, 'm%PPyRtrO', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SPyRtrVels, 3, 2, numPylons, 'm%SPyRtrVels', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PPyRtrVels, 3, 2, numPylons, 'm%PPyRtrVels', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )   
         call AllocAry( m%SPyRtrDCMs, 3, 3, 2, numPylons, 'm%SPyRtrDCMs', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PPyRtrDCMs, 3, 3, 2, numPylons, 'm%PPyRtrDCMs', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )   
         call AllocAry( m%SPyRtrOmegas, 3, 2, numPylons, 'm%SPyRtrOmegas', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )   
         call AllocAry( m%PPyRtrOmegas, 3, 2, numPylons, 'm%PPyRtrOmegas', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )   
         call AllocAry( m%SPyRtrAccs, 3, 2, numPylons, 'm%SPyRtrAccs', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )   
         call AllocAry( m%PPyRtrAccs, 3, 2, numPylons, 'm%PPyRtrAccs', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )   
         call AllocAry( m%SPyRtrAlphas, 3, 2, numPylons, 'm%SPyRtrAlphas', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )   
         call AllocAry( m%PPyRtrAlphas, 3, 2, numPylons, 'm%PPyRtrAlphas', errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )         
         
            ! Convert 1D float array data into specific quantities
      
         FusO = refPts_c(1:3)   ! This is in global coordinates
         m%FusO = FusO
         ! The remaining reference points are already in the Kite coordinate system!
         m%SWnO = refPts_c(4:6)
         m%PWnO = refPts_c(7:9)
         m%VSO = refPts_c(10:12)
         m%SHSO = refPts_c(13:15)
         m%PHSO = refPts_c(16:18)
         c = 19
         do i = 1, p%numPylons
            m%SPyO(:,i) = refPts_c(c:c+2)
            c = c + 3
         end do
         do i = 1, p%numPylons
            m%PPyO(:,i) = refPts_c(c:c+2)
            c = c + 3
         end do
 
         if ( (c-1) /= numComp*3 ) then
            errStat = ErrID_FATAL
            errMsg  = 'The transferred number of reference point elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numComp*3))//'.'
         end if
   
            ! Decode the nodal DCMs
   
         call AllocAry( m%FusNdDCMs, 3, 3, p%numFusNds, 'FusNdDCMs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SWnNdDCMs, 3, 3, p%numSWnNds, 'SWnNdDCMs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PWnNdDCMs, 3, 3, p%numPWnNds, 'PWnNdDCMs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%VSNdDCMs, 3, 3, p%numVSNds, 'VSNdDCMs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SHSNdDCMs, 3, 3, p%numSHSNds, 'SHSNdDCMs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PHSNdDCMs, 3, 3, p%numPHSNds, 'PHSNdDCMs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SPyNdDCMs, 3, 3, maxSPyNds, p%numPylons, 'SPyNdDCMs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PPyNdDCMs, 3, 3, maxPPyNds, p%numPylons, 'PPyNdDCMs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
         call AllocAry( m%FusPts, 3, p%numFusNds, 'FusPts', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SWnPts, 3, p%numSWnNds, 'SWnPts', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PWnPts, 3, p%numPWnNds, 'PWnPts', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%VSPts, 3, p%numVSNds, 'VSPts', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SHSPts, 3, p%numSHSNds, 'SHSPts', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PHSPts, 3, p%numPHSNds, 'PHSPts', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SPyPts, 3, maxSPyNds, p%numPylons, 'SPyPts', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PPyPts, 3, maxPPyNds, p%numPylons, 'PPyPts', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   
         call AllocAry( m%FusVels, 3, p%numFusNds, 'FusVels', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SWnVels, 3, p%numSWnNds, 'SWnVels', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PWnVels, 3, p%numPWnNds, 'PWnVels', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%VSVels, 3, p%numVSNds, 'VSVels', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SHSVels, 3, p%numSHSNds, 'SHSVels', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PHSVels, 3, p%numPHSNds, 'PHSVels', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SPyVels, 3, maxSPyNds, p%numPylons, 'SPyVels', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PPyVels, 3, maxPPyNds, p%numPylons, 'PPyVels', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

         call AllocAry( m%FusOmegas, 3, p%numFusNds, 'FusOmegas', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SWnOmegas, 3, p%numSWnNds, 'SWnOmegas', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PWnOmegas, 3, p%numPWnNds, 'PWnOmegas', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%VSOmegas, 3, p%numVSNds, 'VSOmegas', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SHSOmegas, 3, p%numSHSNds, 'SHSOmegas', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PHSOmegas, 3, p%numPHSNds, 'PHSOmegas', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SPyOmegas, 3, maxSPyNds, p%numPylons, 'SPyOmegas', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PPyOmegas, 3, maxPPyNds, p%numPylons, 'PPyOmegas', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

         call AllocAry( m%FusAccs, 3, p%numFusNds, 'FusAccs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SWnAccs, 3, p%numSWnNds, 'SWnAccs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PWnAccs, 3, p%numPWnNds, 'PWnAccs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%VSAccs, 3, p%numVSNds, 'VSAccs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SHSAccs, 3, p%numSHSNds, 'SHSAccs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PHSAccs, 3, p%numPHSNds, 'PHSAccs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%SPyAccs, 3, maxSPyNds, p%numPylons, 'SPyAccs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call AllocAry( m%PPyAccs, 3, maxPPyNds, p%numPylons, 'PPyAccs', errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
         ! TODO : Check ording of c data to make sure we get the expected global to local DCM
         m%FusODCM = reshape(FusODCM_c,(/3,3/))
   
         call TransferMBDynInitInputs( numNodePts_c, nodePts_c, nodeDCMs_c, rtrPts_c, p, m, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
  
      !----------------------------------------------------------------
      ! Initialize the Motions meshes which correspond to the motions 
      !   coming from MBDyn.  The points coming from MBDyn are assumed
      !   to be in the global, inertial coordinate system and the DCMs
      !   transform from global to local.
      !----------------------------------------------------------------

            ! Fuselage
         call CreateMBDynL2MotionsMesh(FusO, p%numFusNds, m%FusPts, m%FusODCM, m%FusNdDCMs, m%mbdFusMotions, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Starboard Wing
         call CreateMBDynL2MotionsMesh(FusO, p%numSWnNds, m%SWnPts, m%FusODCM, m%SWnNdDCMs, m%mbdSWnMotions, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Port Wing
         call CreateMBDynL2MotionsMesh(FusO, p%numPWnNds, m%PWnPts, m%FusODCM, m%PWnNdDCMs, m%mbdPWnMotions, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Total wing mesh which is needed for MoorDyn mapping
         call CreateMBDynL2MotionsWingMesh(FusO, p%numSWnNds, m%SWnPts, p%numPWnNds, m%PWnPts, m%FusODCM, m%SWnNdDCMs, m%PWnNdDCMs, m%mbdWngMotions, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Vertical Stabilizer
         call CreateMBDynL2MotionsMesh(FusO, p%numVSNds, m%VSPts, m%FusODCM, m%VSNdDCMs, m%mbdVSMotions, errStat2, errMsg2)
            ! Starboard Horizontal Stabilizer
         call CreateMBDynL2MotionsMesh(FusO, p%numSHSNds, m%SHSPts, m%FusODCM, m%SHSNdDCMs, m%mbdSHSMotions, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Port Horizontal Stabilizer
         call CreateMBDynL2MotionsMesh(FusO, p%numPHSNds, m%PHSPts, m%FusODCM, m%PHSNdDCMs, m%mbdPHSMotions, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

         allocate(m%mbdSPyMotions(p%numPylons), STAT=errStat2)
            if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdSPyMotions', errStat, errMsg, RoutineName )     
         allocate(m%mbdPPyMotions(p%numPylons), STAT=errStat2)
            if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdPPyMotions', errStat, errMsg, RoutineName )  

            ! Starboard Pylons
         do i = 1, p%numPylons
            call CreateMBDynL2MotionsMesh(FusO, p%numSPyNds(i), m%SPyPts(:,:,i), m%FusODCM, m%SPyNdDCMs(:,:,:,i), m%mbdSPyMotions(i), errStat2, errMsg2)
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         end do
            ! Port Pylons
         do i = 1, p%numPylons
            call CreateMBDynL2MotionsMesh(FusO, p%numPPyNds(i), m%PPyPts(:,:,i), m%FusODCM, m%PPyNdDCMs(:,:,:,i), m%mbdPPyMotions(i), errStat2, errMsg2)
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         end do   
 
         ! NOTE: We don't need a set of MBDyn rotor meshes because the data is transferred directly to KiteAeroDyn and then back to MBDyn 

            ! Fuselage
         call CreateMBDynPtLoadsMesh(FusO, p%numFusNds, m%FusPts, m%FusODCM, m%FusNdDCMs, m%mbdFusLoads, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Starboard Wing
         call CreateMBDynPtLoadsMesh(FusO, p%numSWnNds, m%SWnPts, m%FusODCM, m%SWnNdDCMs, m%mbdSWnLoads, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Port Wing
         call CreateMBDynPtLoadsMesh(FusO, p%numPWnNds, m%PWnPts, m%FusODCM, m%PWnNdDCMs, m%mbdPWnLoads, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Total Wing
         call CreateMBDynPtLoadsWingMesh(FusO, p%numSWnNds, m%SWnPts, p%numPWnNds, m%PWnPts, m%FusODCM, m%SWnNdDCMs, m%PWnNdDCMs, m%mbdWngLoads, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Vertical Stabilizer
         call CreateMBDynPtLoadsMesh(FusO, p%numVSNds, m%VSPts, m%FusODCM, m%VSNdDCMs, m%mbdVSLoads, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Starboard Horizontal Stabilizer
         call CreateMBDynPtLoadsMesh(FusO, p%numSHSNds, m%SHSPts, m%FusODCM, m%SHSNdDCMs, m%mbdSHSLoads, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Port Horizontal Stabilizer
         call CreateMBDynPtLoadsMesh(FusO, p%numPHSNds, m%PHSPts, m%FusODCM, m%PHSNdDCMs, m%mbdPHSLoads, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

         allocate(m%mbdSPyLoads(p%numPylons), STAT=errStat2)
            if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdSPyLoads', errStat, errMsg, RoutineName )     
         allocate(m%mbdPPyLoads(p%numPylons), STAT=errStat2)
            if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdPPyLoads', errStat, errMsg, RoutineName )  
            ! Starboard Pylons
         do i = 1, p%numPylons
            call CreateMBDynPtLoadsMesh(FusO, p%numSPyNds(i), m%SPyPts(:,:,i), m%FusODCM, m%SPyNdDCMs(:,:,:,i), m%mbdSPyLoads(i), errStat2, errMsg2)
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         end do
            ! Port Pylons
         do i = 1, p%numPylons
            call CreateMBDynPtLoadsMesh(FusO, p%numPPyNds(i), m%PPyPts(:,:,i), m%FusODCM, m%PPyNdDCMs(:,:,:,i), m%mbdPPyLoads(i), errStat2, errMsg2)
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         end do  
 
         ! NOTE: We don't need a set of MBDyn rotor meshes because the data is transferred directly to KiteAeroDyn and then back to MBDyn 
   
      end subroutine SetupMBDynMotionData
      
end subroutine Init_KiteSystem
    
subroutine AssRes_OnShore( t_c, isInitialTime_c, WindPt_c, WindPtVel_c, AnchorPt, FusO_c, FusODCM_c, FusOv_c, FusOomegas_c, FusOacc_c, FusOalphas_c, numNodePts_c, nodePts_c, &
                          nodeDCMs_c, nodeVels_c, nodeOmegas_c, nodeAccs_c,  numRtrPts_c, rtrPts_c, &
                          rtrDCMs_c, rtrVels_c, rtrOmegas_c, rtrAccs_c, rtrAlphas_c, PtfmO, PtfmOv, nodeLoads_c, rtrLoads_c, ptfmLoads_c, errStat, errMsg ) 

   real(C_DOUBLE),         intent(in   ) :: t_c                          !  simulation time for the current timestep (s)
   integer(C_INT),         intent(in   ) :: isInitialTime_c              !  1 = first time KFAST_AssRes has been called for this particular timestep, 0 = otherwise
   real(C_DOUBLE),         intent(in   ) :: WindPt_c(3)                  !  Position of the ground station where the wind measurement is taken, expressed in global coordinates. (m)
   real(C_DOUBLE),         intent(in   ) :: WindPtVel_c(3)               !  Velocity of the ground station where the wind measurement is taken, expressed in global coordinates. (m/s)
   real(ReKi),             intent(in   ) :: AnchorPt(3)                  !  Location of the tether anchor point. (m)
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
   real(ReKi),             intent(in   ) :: PtfmO(3)                     !  Current timestep position of the Platform reference point, expressed in global coordinates. (m) 
   real(ReKi),             intent(in   ) :: PtfmOv(3)                    !  Current timestep velocity of the Platform reference point, expressed in global coordinates. (m/s)

   real(C_DOUBLE),         intent(  out) :: nodeLoads_c(numNodePts_c*6)  !  KiteFAST loads (3 forces + 3 moments) in global coordinates ( N, N-m ) at the MBDyn structural nodes.  Sequence follows the pattern used for MBDyn structural node array.  Returned from KiteFAST to MBDyn.
   real(C_DOUBLE),         intent(  out) :: rtrLoads_c(numRtrPts_c*6)    !  Concentrated reaction loads at the nacelles on the pylons at the RRPs in global coordinates.  Length is 6 loads per rotor * number of RRPs. Returned from KiteFAST to MBDyn.
   real(C_DOUBLE),         intent(inout) :: ptfmLoads_c(6)               !  Concentrated loads at the plaform reference pont in global coordinates.  Returned from KiteFAST to MBDyn.
   integer(IntKi),         intent(inout) :: errStat                      ! Error code coming from KiteFAST
   character(ErrMsgLen),   intent(inout) :: errMsg                       ! Error message
   
   
   ! Local variables
   
   integer(IntKi)           :: n, n2, c, i, j                 ! counters
   integer(IntKi)           :: isInitialTime                  ! Is this the initial time of the simulation 1=Yes, should we update the states? 0=yes, 1=no
   real(DbKi)               :: t                              ! simulations time (s)
   real(DbKi)               :: utimes(2)                      ! t-p%dt and t timestep values (s)
   real(ReKi)               :: AeroMoment(3)                  ! AeroDynamic moment on a rotor as computed by KAD (N-m)
   real(ReKi)               :: xhat(3)                        ! 
   real(ReKi)               :: AeroForce(3)                   ! AeroDynamic force on a rotor as computed by KAD (N)              ! 
   integer(IntKi)           :: errStat2              ! error status values
   character(ErrMsgLen)     :: errMsg2                ! error messages
   character(*), parameter  :: routineName = 'AssRes_Onshore'
   logical                  :: test, doTransfersforKFC
   real(SiKi)               :: fracStep, t_s, dt_s
   real(ReKi)               :: IfW_ground(3)
   real(ReKi)               :: IfW_FusO(3)
   integer(IntKi)           :: numFairLeads
   
   errStat = ErrID_None
   errMsg  = ''
   doTransfersforKFC   = .false.
   
   isInitialTime = isInitialTime_c
   t = t_c
   t_s = real(t,SiKi)
   
     ! NOTE: For the KAD input/output/time arrays the 1 index is for the most recent time, and then increasing indices move backwards in time!
     !       The utimes arrays below are for MD, and they are structure in the opposite way. 2 index is the most recent, 1 index is the previous time!
   if (isInitialTime > 0 ) then
      utimes(2) = t
   else
      utimes(2) = t - p%DT
   end if
      
   utimes(1) = t
   n = nint(utimes(1) / p%DT)
   n2= nint(utimes(2) / p%DT)
   
      ! Transfer fuselage (kite) principal point inputs
   m%FusO          = FusO_c
   m%FusODCM       = reshape(FusODCM_c,(/3,3/))          
   m%FusOv         = FusOv_c        
   m%FusOomegas    = FusOomegas_c    
   m%FusOaccs      = FusOacc_c      
   m%FusOalphas    = FusOalphas_c
   
      ! Transfer C-based nodal and rotor quantities into Fortran-based data structures (and the MBD motion meshes)
      !   The resulting data resides inside the MiscVars data structure (m)

   call TransferMBDynInputs( numNodePts_c, nodePts_c, nodeDCMs_c, nodeVels_c, nodeOmegas_c, nodeAccs_c, rtrPts_c, rtrDCMs_c, rtrVels_c, rtrOmegas_c, rtrAccs_c, rtrAlphas_c, p, m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) return

         
   ! -------------------------------------------------------------------------
   ! Controller
   ! -------------------------------------------------------------------------      
   !fracStep = modulo( real(t,SiKi), real(p%KFC_dt,SiKi) )
   !dt_s = real(p%KFC_dt,SiKi)
   !fracStep = t_s - floor(t_s/dt_s)* dt_s
   !test = EqualRealNos( fracStep , 0.0_SiKi )
   
   if ( OtherSt%KFC_NewTime ) then
      OtherSt%KFC_NewTime = .false.
      !print *, "Stepping KFC at t = ", t
         ! NOTE: The controller is stepping from t - p%KFC_dt to t (GetXCur in MBDyn)
         !       therefore, all inputs to KFC needs to be at time, t - p%KFC_dt.
      if ( p%useIfW ) then
         m%IfW%u_KFC%PositionXYZ(:,1) = OtherSt%WindPt
         m%IfW%u_KFC%PositionXYZ(:,2) = OtherSt%FusO
         call InflowWind_CalcOutput(t-p%KFC_dt, m%IfW%u_KFC, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z, OtherSt%IfW%OtherSt, m%IfW%y_KFC, m%IfW%m, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) return

            ! Need previous timestep's inflow at ground station
         IfW_ground = m%IfW%y_KFC%VelocityUVW(:,1)     
            ! Need previous timestep's inflow at FusO
         IfW_FusO   = m%IfW%y_KFC%VelocityUVW(:,2)
      else
         IfW_ground = 0.0_ReKi
         IfW_FusO   = 0.0_ReKi
      end if
      
      if ( p%useKAD ) then
         do j = 1,p%numPylons
            do i = 1,2
               
               xhat   = OtherSt%SPyRtrDCMs(1,:,i,j)
               AeroMoment = OtherSt%SPyRtrLoads(:,i,j)
               m%KFC%u%SPyAeroTorque(i,j) = dot_product(xhat, AeroMoment )            

               xhat   = OtherSt%PPyRtrDCMs(1,:,i,j) 
               AeroMoment = OtherSt%PPyRtrLoads(:,i,j)
               m%KFC%u%PPyAeroTorque(i,j) = dot_product(xhat, AeroMoment )

            end do
         end do
      else
         m%KFC%u%SPyAeroTorque = 0.0_ReKi
         m%KFC%u%PPyAeroTorque = 0.0_ReKi
      end if
      
      m%KFC%u%tether_forceb = matmul(OtherSt%FusODCM, OtherSt%totalFairLeadLoads)
      m%KFC%u%dcm_g2b       = matmul(OtherSt%FusODCM, transpose(p%DCM_Fast2Ctrl))
      m%KFC%u%pqr           = matmul(OtherSt%FusODCM, OtherSt%FusOomegas)
      m%KFC%u%acc_norm      = TwoNorm(OtherSt%FusOacc)
      m%KFC%u%Xg            = OtherSt%FusO - AnchorPt     
      m%KFC%u%Xg            = matmul(p%DCM_Fast2Ctrl, m%KFC%u%Xg)
      m%KFC%u%Vg            = matmul(p%DCM_Fast2Ctrl, OtherSt%FusOv)
      m%KFC%u%Vb            = matmul(OtherSt%FusODCM, OtherSt%FusOv)
      m%KFC%u%Ag            = matmul(p%DCM_Fast2Ctrl, OtherSt%FusOacc)
      m%KFC%u%Ab            = matmul(OtherSt%FusODCM, OtherSt%FusOacc)
      m%KFC%u%rho           = p%AirDens
      
      m%KFC%u%apparent_wind = IfW_FusO - OtherSt%FusOv
      m%KFC%u%wind_g        = matmul(p%DCM_Fast2Ctrl, IfW_ground)
         
      m%KFC%u%apparent_wind = matmul(p%DCM_Fast2Ctrl, m%KFC%u%apparent_wind)
     
         
      call KFC_Step(t-p%KFC_dt, m%KFC%u, m%KFC%p, m%KFC%y, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      if (errStat >= AbortErrLev )  return

      
         ! Store the current timestep's data into otherstates for use as inputs to KFC_Step for the next KFC_DT interval (and call to KFC_Step)
      OtherSt%SPyRtrDCMs = m%SPyRtrDCMs
      OtherSt%PPyRtrDCMs = m%PPyRtrDCMs
      OtherSt%WindPt     = WindPt_c   
      OtherSt%FusODCM    = m%FusODCM
      OtherSt%FusO       = m%FusO
      OtherSt%FusOv      = m%FusOv
      OtherSt%FusOomegas = m%FusOomegas
      OtherSt%FusOacc    = m%FusOaccs
      doTransfersforKFC   = .true.  ! need to transfer the MD bridle forces and KAD rotor loads once they are computed (below) for the next call to KFC_Step()
   else if (isInitialTime > 0) then
         ! Store the current timestep's data into otherstates for use as inputs to KFC_Step for the next KFC_DT interval (and call to KFC_Step)
      OtherSt%SPyRtrDCMs = m%SPyRtrDCMs
      OtherSt%PPyRtrDCMs = m%PPyRtrDCMs
      OtherSt%WindPt     = WindPt_c   
      OtherSt%FusODCM    = m%FusODCM
      OtherSt%FusO       = m%FusO
      OtherSt%FusOv      = m%FusOv
      OtherSt%FusOomegas = m%FusOomegas
      OtherSt%FusOacc    = m%FusOaccs
      doTransfersforKFC   = .true.  ! need to transfer the MD bridle forces and KAD rotor loads once they are computed (below) for the next call to KFC_Step()   
   end if
             
   ! -------------------------------------------------------------------------
   ! MoorDyn
   ! ------------------------------------------------------------------------- 
   
   if ( p%useMD_Tether ) then
     
         ! Update the MBDyn motions mesh associated with the total wing
         ! Nodes for this wing mesh are specified left to right (port tip across to starboard tip)
      
      ! TODO: Handle possible colocated nodes at fuselage
!print *, "Beginning setting of mbdWngMotions mesh data"
      c = 1   
      do i = p%numPwnNds, 1, -1
         m%mbdWngMotions%Orientation  (:,:,c) = m%PWnNdDCMs(:,:,i)
         m%mbdWngMotions%TranslationDisp(:,c) = m%PWnPts(:,i) - m%mbdWngMotions%Position(:,c)
         m%mbdWngLoads%TranslationDisp(:,c)   = m%mbdWngMotions%TranslationDisp(:,c)
         m%mbdWngMotions%TranslationVel (:,c) = m%PWnVels(:,i)
         m%mbdWngMotions%RotationVel    (:,c) = m%PWnOmegas(:,i)
         c = c + 1
      end do
      do i = 1,p%numSwnNds
         m%mbdWngMotions%Orientation  (:,:,c) = m%SWnNdDCMs(:,:,i)
         m%mbdWngMotions%TranslationDisp(:,c) = m%SWnPts(:,i) - m%mbdWngMotions%Position(:,c)
         m%mbdWngLoads%TranslationDisp(:,c)   = m%mbdWngMotions%TranslationDisp(:,c)
         m%mbdWngMotions%TranslationVel (:,c) = m%SWnVels(:,i)
         m%mbdWngMotions%RotationVel    (:,c) = m%SWnOmegas(:,i)
         c = c + 1
      end do
!print *, "Finish setting mbdWngMotions data"   
         ! Pos and Vel of each Fairlead (Bridle connection) at t.  
         !   We need to set the TranslationDisp, Orientation, TranslationVel, RotationVel properties on the mesh
         ! To do this we need to map the motions from a wing mesh to the specific fairlead mesh ( one node per fairlead connection point)
         ! This means we need the t and t+dt motions of this wing mesh
      call Transfer_Line2_to_Point( m%mbdWngMotions, m%MD_Tether%u(1)%PtFairleadDisplacement(1), m%MD_L2_2_P, errStat2, errMsg )
         if (errStat >= AbortErrLev ) return

      if ( size(m%MD_Tether%u(1)%PtFairleadDisplacement,1) == 2 ) then
         
         ! Transfer Platform body motions to the tether fairlead attached to that platform
         m%mbdPtfmMotions%TranslationDisp(:,1) = PtfmO - m%mbdPtfmMotions%Position(:,1)
         m%mbdPtfmMotions%TranslationVel (:,1) = PtfmOv       
         m%mbdPtfmLoads%TranslationDisp  (:,1) = m%mbdPtfmMotions%TranslationDisp(:,1)

         call Transfer_Point_to_Point( m%mbdPtfmMotions, m%MD_Tether%u(1)%PtFairleadDisplacement(2), m%mbdPtfm_P_2_MD_P, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) return
         
      end if
         
      call MD_CopyContState( OtherSt%MD_Tether%x, OtherSt%MD_Tether%x_copy, MESH_NEWCOPY, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

      if ( isInitialTime < 1 ) then
         call MD_CopyInput( OtherSt%MD_Tether_u, m%MD_Tether%u(2), MESH_NEWCOPY, errStat2, errMsg2 )
         call MD_UpdateStates( utimes(2), n2, m%MD_Tether%u, utimes, m%MD_Tether%p, OtherSt%MD_Tether%x_copy, m%MD_Tether%xd, m%MD_Tether%z, OtherSt%MD_Tether%OtherSt, m%MD_Tether%m, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               if (errStat >= AbortErrLev ) return

      end if

!print *, "Beginning MD_CalcOutput"  
!print *, " at time="//trim(num2lstr(t))
!print *, "m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(1,1): "//trim(num2l
 !     str(m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(1,1)))//", m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(2,1): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(2,1)))//", m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(3,1): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(3,1)))
!print *, "m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(1,2): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(1,2)))//", m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(2,2): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(2,2)))//", m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(3,2): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%Position(3,2)))
!print *, "m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(1,1): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(1,1)))//", m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(2,1): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(2,1)))//", m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(3,1): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(3,1)))
!print *, "m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(1,2): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(1,2)))//", m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(2,2): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(2,2)))//", m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(3,2): "//trim(num2lstr(m%MD_Tether%u(1)%PtFairLeadDisplacement%TranslationDisp(3,2)))


      call MD_CalcOutput( t, m%MD_Tether%u(1), m%MD_Tether%p, OtherSt%MD_Tether%x_copy, m%MD_Tether%xd, m%MD_Tether%z, OtherSt%MD_Tether%OtherSt, m%MD_Tether%y, m%MD_Tether%m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      if (errStat >= AbortErrLev ) return

      
      if ( doTransfersforKFC ) then
            ! need to transfer the bridle forces once they are computed (below) for the next call to KFC_Step() because we just stepped the controller.
         numFairLeads = size(m%MD_Tether%y%PtFairLeadLoad(1)%Force,2)
         OtherSt%totalFairLeadLoads = 0.0_ReKi
         do i = 1, numFairLeads
            OtherSt%totalFairLeadLoads = OtherSt%totalFairLeadLoads + m%MD_Tether%y%PtFairLeadLoad(1)%Force(:,i) 
         end do
      end if
      if ( size(m%MD_Tether%u(1)%PtFairleadDisplacement,1) == 2 ) then
         ! Platform body loads from tether system
         call Transfer_Point_to_Point( m%MD_Tether%y%PtFairLeadLoad(2), m%mbdPtfmLoads, m%MD_P_2_mbdPtfm_P, errStat2, errMsg2, m%MD_Tether%u(1)%PtFairleadDisplacement(2), m%mbdPtfmLoads )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) return
         do i = 1,3
             ptfmLoads_c(i)   = ptfmLoads_c(i)   + m%mbdPtfmLoads%Force(i,1) 
             ptfmLoads_c(i+3) = ptfmLoads_c(i+3) + m%mbdPtfmLoads%Moment(i,1) 
         end do
      end if
      
!print *, "Finished MD_CalcOutput" 
!print *, "m%MD_Tether%y%PtFairLeadLoad%Force(1,1): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Force(1,1)))//", m%MD_Tether%y%PtFairLeadLoad%Force(2,1): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Force(2,1)))//", m%MD_Tether%y%PtFairLeadLoad%Force(3,1): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Force(3,1)))
!print *, "m%MD_Tether%y%PtFairLeadLoad%Force(1,2): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Force(1,2)))//", m%MD_Tether%y%PtFairLeadLoad%Force(2,2): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Force(2,2)))//", m%MD_Tether%y%PtFairLeadLoad%Force(3,2): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Force(3,2)))
!print *, "m%MD_Tether%y%PtFairLeadLoad%Moment(1,1): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Moment(1,1)))//", m%MD_Tether%y%PtFairLeadLoad%Moment(2,1): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Moment(2,1)))//", m%MD_Tether%y%PtFairLeadLoad%Moment(3,1): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Moment(3,1)))
!print *, "m%MD_Tether%y%PtFairLeadLoad%Moment(1,2): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Moment(1,2)))//", m%MD_Tether%y%PtFairLeadLoad%Moment(2,2): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Moment(2,2)))//", m%MD_Tether%y%PtFairLeadLoad%Moment(3,2): "//trim(num2lstr(m%MD_Tether%y%PtFairLeadLoad%Moment(3,2)))  
!m%MD_Tether%y%PtFairLeadLoad%Force(:,1)=0.0_ReKi
!m%MD_Tether%y%PtFairLeadLoad%Force(:,2)=0.0_ReKi
      
   end if
         
   ! if ( p%useKAD ) then
            ! ! Now transfer MBD motions mesh data to the corresponding KAD input meshes
            ! ! NOTE: I need the KAD input meshes populated here because the TransferMBDynToIFW will reference those input meshes.
      ! call TransferMBDynInputs2KADMeshes( m%FusO, p, m, errStat, errMsg )
   
   ! end if
   

! -------------------------------------------------------------------------
! InflowWind
! -------------------------------------------------------------------------      
      ! The inputs to InflowWind are the positions where wind velocities [the outputs] are to be computed.  These inputs are set above by
      !  the TransferMBDynInputs() call.  The inputs are for t+dt timestep.
   ! if ( p%useIfW ) then
      ! call TransferMBDynToIfW( WindPt_c, m%FusO, p, m, errStat2, errMsg2 )
         ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         ! if (errStat >= AbortErrLev ) then
            ! call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            ! return
         ! end if
        
      ! call InflowWind_CalcOutput(t, m%IfW%u, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z, OtherSt%IfW%OtherSt, m%IfW%y, m%IfW%m, errStat2, errMsg2 )
         ! call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         ! if (errStat >= AbortErrLev ) then
            ! call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            ! return
         ! end if
   ! end if





! -------------------------------------------------------------------------
! KiteAeroDyn
! -------------------------------------------------------------------------      
   
   if ( p%useKAD ) then
  
      if ( OtherSt%KAD_NewTime ) then
               
         if (isInitialTime == 0) then
            if (p%KAD_InterpOrder == 2) then ! 2nd Order
               call KAD_CopyInput(OtherSt%KAD%u(2),OtherSt%KAD%u(3), MESH_NEWCOPY, errStat2, errMsg2)
                call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )       
            end if 
            
            if (p%KAD_InterpOrder > 0) then ! 1st and 2nd Order
               call KAD_CopyInput(OtherSt%KAD%u(1),OtherSt%KAD%u(2), MESH_NEWCOPY, errStat2, errMsg2)
                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            end if
            if (errStat >= AbortErrLev ) return

         end if
         
         !print *, "Stepping KAD at t = ", t
         
         call TransferMBDynInputs2KADMeshes( isInitialTime, m%FusO, p, m, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            if (errStat >= AbortErrLev ) return

               ! The inputs to InflowWind are the positions where wind velocities [the outputs] are to be computed.  These inputs are set above by
               !  the TransferMBDynInputs() call.  The inputs are for t+dt timestep.
         if ( p%useIfW ) then
            call TransferMBDynToIfW( WindPt_c, m%FusO, p, m, errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               if (errStat >= AbortErrLev ) return

              
            call InflowWind_CalcOutput(t, m%IfW%u, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z, OtherSt%IfW%OtherSt, m%IfW%y, m%IfW%m, errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               if (errStat >= AbortErrLev )  return

         end if

   
         ! call WrScr("Updating KAD Outputs")
            ! Outputs from Controller for KAD (note: we always at least have a dummy controller to generate the necessary data)
         OtherSt%KAD%u(1)%Ctrl_SFlp = m%KFC%y%SFlp
         OtherSt%KAD%u(1)%Ctrl_PFlp = m%KFC%y%PFlp
         OtherSt%KAD%u(1)%Ctrl_Rudr = m%KFC%y%Rudr
         OtherSt%KAD%u(1)%Ctrl_SElv = m%KFC%y%SElv
         OtherSt%KAD%u(1)%Ctrl_PElv = m%KFC%y%PElv
         OtherSt%KAD%u(1)%Pitch_SPyRtr = m%KFC%y%SPyBldPitch   ! Controller only sets these to zero at this point.
         OtherSt%KAD%u(1)%Pitch_PPyRtr = m%KFC%y%PPyBldPitch
         OtherSt%KAD%u(1)%RtSpd_SPyRtr = m%KFC%y%SPyRtrSpd
         OtherSt%KAD%u(1)%RtSpd_PPyRtr = m%KFC%y%PPyRtrSpd
         
         if ( p%useIfW ) then
            
               ! Transfer Inflow Wind outputs to the various KAD inflow inputs
            c=3
            do i = 1,size(OtherSt%KAD%u(1)%V_Fus,2)
               OtherSt%KAD%u(1)%V_Fus(:,i)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
            do i = 1,size(OtherSt%KAD%u(1)%V_SWn,2)
               OtherSt%KAD%u(1)%V_SWn(:,i)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
            do i = 1,size(OtherSt%KAD%u(1)%V_PWn,2)
               OtherSt%KAD%u(1)%V_PWn(:,i)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
            do i = 1,size(OtherSt%KAD%u(1)%V_VS,2)
               OtherSt%KAD%u(1)%V_VS(:,i)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
            do i = 1,size(OtherSt%KAD%u(1)%V_SHS,2)
               OtherSt%KAD%u(1)%V_SHS(:,i)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
            do i = 1,size(OtherSt%KAD%u(1)%V_PHS,2)
               OtherSt%KAD%u(1)%V_PHS(:,i)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
            do j = 1,p%numPylons
               do i = 1,size(OtherSt%KAD%u(1)%V_SPy,2)
                  OtherSt%KAD%u(1)%V_SPy(:,i,j)   = m%IfW%y%VelocityUVW(:,c)
                  c = c+1
               end do
            end do
            do j = 1,p%numPylons
               do i = 1,size(OtherSt%KAD%u(1)%V_PPy,2)
                  OtherSt%KAD%u(1)%V_PPy(:,i,j)   = m%IfW%y%VelocityUVW(:,c)
                  c = c+1
               end do
            end do   
            do i = 1,p%numPylons
               OtherSt%KAD%u(1)%V_SPyRtr(:,1,i) = m%IfW%y%VelocityUVW(:,c)
               c = c+1
               OtherSt%KAD%u(1)%V_SPyRtr(:,2,i) = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
            do i = 1,p%numPylons
               OtherSt%KAD%u(1)%V_PPyRtr(:,1,i) = m%IfW%y%VelocityUVW(:,c)
               c = c+1
               OtherSt%KAD%u(1)%V_PPyRtr(:,2,i) = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
         else
            OtherSt%KAD%u(1)%V_Fus     = 0.0_ReKi
            OtherSt%KAD%u(1)%V_SWn     = 0.0_ReKi
            OtherSt%KAD%u(1)%V_PWn     = 0.0_ReKi
            OtherSt%KAD%u(1)%V_VS      = 0.0_ReKi
            OtherSt%KAD%u(1)%V_SHS     = 0.0_ReKi
            OtherSt%KAD%u(1)%V_PHS     = 0.0_ReKi
            OtherSt%KAD%u(1)%V_SPy     = 0.0_ReKi
            OtherSt%KAD%u(1)%V_PPy     = 0.0_ReKi         
            OtherSt%KAD%u(1)%V_SPyRtr  = 0.0_ReKi    
            OtherSt%KAD%u(1)%V_PPyRtr  = 0.0_ReKi  
         end if

         if ( (isInitialTime > 0) .or. (OtherSt%doStartupInterp) ) then 
            if (p%KAD_InterpOrder == 2) then ! 2nd order
               call KAD_CopyInput(OtherSt%KAD%u(1),OtherSt%KAD%u(3), MESH_NEWCOPY, errStat2, errMsg2)
                   call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               OtherSt%KAD%t_in(3) = t-2.0_DbKi*p%KAD_dt
            end if
            if (p%KAD_InterpOrder > 0) then ! 1st and 2nd order
               call KAD_CopyInput(OtherSt%KAD%u(1),OtherSt%KAD%u(2), MESH_NEWCOPY, errStat2, errMsg2)
                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               OtherSt%KAD%t_in(2) = t-p%KAD_dt
               OtherSt%KAD%t_in(1) = t              
            end if
            if (errStat >= AbortErrLev ) return

         end if
         if ( (p%KAD_InterpOrder > 0) .and. (.not. OtherSt%doStartupInterp) ) then ! 1st or 2nd order
               ! Extrapolate the inputs to t + KAD_dt
            call KAD_Input_ExtrapInterp(OtherSt%KAD%u, OtherSt%KAD%t_in, m%KAD%u, t+p%KAD_dt, errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
          
              ! Update the KAD states, n corresponds to time, t
            call KAD_UpdateStates( t, n, OtherSt%KAD%u, OtherSt%KAD%t_in, m%KAD%p, m%KAD%x, m%KAD%xd, OtherSt%KAD%z, OtherSt%KAD%OtherSt, m%KAD%m, errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            call KAD_CalcOutput( t+p%KAD_dt, m%KAD%u, m%KAD%p, m%KAD%x, m%KAD%xd, OtherSt%KAD%z, OtherSt%KAD%OtherSt, m%KAD%y, m%KAD%m, errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            if (errStat >= AbortErrLev ) return

         else  ! 0th order hold, or startup sequence         
            if (isInitialTime == 0 ) then
! TODO: Adjust this logic for startup sequence and InterpOrder > 0
! Switch InterpOrder=0 to use t_in array so that we can use the standard UpdateStates syntax instead of a special version using utimes for InterpOrder=0
               !call KAD_CopyInput(OtherSt%KAD%u(2), m%KAD%u, MESH_NEWCOPY, errStat2, errMsg2)
               
               if (p%KAD_InterpOrder==0) then
                  call KAD_CopyInput(OtherSt%KAD%u(1),OtherSt%KAD%u(2), MESH_NEWCOPY, errStat2, errMsg2)
                     call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
                  OtherSt%KAD%t_in(2) = t-p%KAD_dt
                  OtherSt%KAD%t_in(1) = t              
               end if
               
               call KAD_UpdateStates( t, n, OtherSt%KAD%u, OtherSt%KAD%t_in, m%KAD%p, m%KAD%x, m%KAD%xd, OtherSt%KAD%z, OtherSt%KAD%OtherSt, m%KAD%m, errStat2, errMsg2 )
                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
!               call KAD_UpdateStates( utimes(1), n, OtherSt%KAD%u, utimes, m%KAD%p, m%KAD%x, m%KAD%xd, OtherSt%KAD%z, OtherSt%KAD%OtherSt, m%KAD%m, errStat2, errMsg2 )
!                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               if (errStat >= AbortErrLev ) return

               !call KAD_CopyInput(m%KAD%u,OtherSt%KAD%u(2), MESH_NEWCOPY, errStat2, errMsg2)
            end if

            call KAD_CalcOutput( t, OtherSt%KAD%u(1), m%KAD%p, m%KAD%x, m%KAD%xd, OtherSt%KAD%z, OtherSt%KAD%OtherSt, m%KAD%y, m%KAD%m, errStat2, errMsg2 )
               call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            if (errStat >= AbortErrLev ) return

         end if
        
         if ( isInitialTime ==  0 ) then
            if ( p%KAD_InterpOrder == 2) then
               call KAD_CopyOutput(OtherSt%KAD%y(2),OtherSt%KAD%y(3), MESH_NEWCOPY, errStat2, errMsg2)
                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               OtherSt%KAD%t_in(3) = OtherSt%KAD%t_in(2)
            end if
            if (p%KAD_InterpOrder > 0) then
               call KAD_CopyOutput(OtherSt%KAD%y(1),OtherSt%KAD%y(2), MESH_NEWCOPY, errStat2, errMsg2)
                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               call KAD_CopyOutput(m%KAD%y, OtherSt%KAD%y(1), MESH_NEWCOPY, errStat2, errMsg2)
                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               OtherSt%KAD%t_in(2) = OtherSt%KAD%t_in(1)
               OtherSt%KAD%t_in(1) = t+p%KAD_dt
            end if
         else
            if ( p%KAD_InterpOrder == 2) then
               call KAD_CopyOutput(m%KAD%y, OtherSt%KAD%y(3), MESH_NEWCOPY, errStat2, errMsg2)
                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            end if
            if ( p%KAD_InterpOrder >0) then
               call KAD_CopyOutput(m%KAD%y, OtherSt%KAD%y(2), MESH_NEWCOPY, errStat2, errMsg2)
                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
               call KAD_CopyOutput(m%KAD%y, OtherSt%KAD%y(1), MESH_NEWCOPY, errStat2, errMsg2)
                  call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            end if
         end if
         
         if (errStat >= AbortErrLev ) return

         
         OtherSt%KAD_NewTime = .false.
         
      end if ! if ( OtherSt%KAD_NewTime
      
      if ( (p%KAD_InterpOrder > 0) .and. (.not. OtherSt%doStartupInterp) ) then ! 1st or 2nd order
         call KAD_Output_ExtrapInterp(OtherSt%KAD%y, OtherSt%KAD%t_in, m%KAD%y, t, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            if (errStat >= AbortErrLev ) return

      end if
      
   end if          ! if ( p%useKAD )
   
      ! Compute the Rotor Reaction Loads
   do j = 1, p%numPylons
      do i = 1,2
         c = (j-1)*2 + i
         
            ! Starboard Rotors         
         if ( p%useKAD ) then
            AeroMoment = m%KAD%y%SPyRtrLoads(c)%Moment(:,1)
            AeroForce  = m%KAD%y%SPyRtrLoads(c)%Force(:,1)
            if ( doTransfersforKFC ) then
                  ! These will be used for the next call to KFC_Step()
               OtherSt%SPyRtrLoads(:,i,j) = m%KAD%y%SPyRtrLoads(c)%Moment(:,1)          
               OtherSt%PPyRtrLoads(:,i,j) = m%KAD%y%PPyRtrLoads(c)%Moment(:,1)  
            end if
         else
            AeroMoment = 0.0_ReKi
            AeroForce  = 0.0_ReKi
         end if        

         call KFAST_RotorCalcs(m%SPyRtrDCMs(:,:,i,j), m%SPyRtrOmegas(:,i,j), m%SPyRtrAccs(:,i,j), m%SPyRtrAlphas(:,i,j), &
                               m%KFC%y%SPyRtrSpd(i,j), m%KFC%y%SPyGenTorque(i,j), AeroForce, AeroMoment , &
                               p%Gravity, p%SPyRtrMass(i,j),   p%SPyRtrIrot(i,j),     p%SPyRtrItrans(i,j), p%SPyRtrXcm(i,j), &
                               m%SPyRtrFReact(:,i,j), m%SPyRtrMReact(:,i,j), errStat2, errMsg2 ) 
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            ! Port Rotors
         if ( p%useKAD ) then
            AeroMoment = m%KAD%y%PPyRtrLoads(c)%Moment(:,1)
            AeroForce  = m%KAD%y%PPyRtrLoads(c)%Force(:,1)
         else
            AeroMoment = 0.0_ReKi
            AeroForce  = 0.0_ReKi
         end if         

         call KFAST_RotorCalcs(m%PPyRtrDCMs(:,:,i,j), m%PPyRtrOmegas(:,i,j), m%PPyRtrAccs(:,i,j), m%PPyRtrAlphas(:,i,j), &
                               m%KFC%y%PPyRtrSpd(i,j), m%KFC%y%PPyGenTorque(i,j), AeroForce, AeroMoment, &
                               p%Gravity, p%PPyRtrMass(i,j),   p%PPyRtrIrot(i,j),     p%PPyRtrItrans(i,j), p%PPyRtrXcm(i,j), &
                               m%PPyRtrFReact(:,i,j), m%PPyRtrMReact(:,i,j), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )                             
      end do
   end do
   
   call TransferLoadsToMBDyn( p, m, nodeLoads_c, rtrLoads_c, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

end subroutine AssRes_OnShore

subroutine AfterPredict_Onshore(t_c, errStat, errMsg)
   IMPLICIT NONE
   real(C_DOUBLE),         intent(in   ) :: t_c
   integer(IntKi),         intent(inout) :: errStat                      ! Error code coming from KiteFAST
   character(ErrMsgLen),   intent(inout) :: errMsg                       ! Error message

   integer(IntKi)                  :: errStat2
   character(ErrMsgLen)            :: errMsg2
   character(*), parameter         :: routineName = 'AfterPredict_Onshore'
   integer(IntKi)                  :: numFairLeads, i, count, j
   real(DbKi)                      :: t
   integer(IntKi)                  :: n
   
   errStat = ErrID_None
   errMsg  = ''
   
   t = t_c  
         
   if ( p%useMD_Tether ) then
         ! Copy the temporary states and place them into the actual versions
      call MD_CopyContState( OtherSt%MD_Tether%x_copy, OtherSt%MD_Tether%x, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         ! Copy t+dt inputs to t for the next timestep
      call MD_CopyInput( m%MD_Tether%u(1), OtherSt%MD_Tether_u, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   
   ! Note: t has already advanced in time from last AssRes() call
   n = nint((t-p%dt) / p%dt) + 1
   
   ! Check if it is time to toggle startup flag for KAD interporder
   if ( OtherSt%doStartupInterp .and. t > 2.0_DbKi ) then
      OtherSt%doStartupInterp = .false.
      print *, "Resetting startup flag to FALSE"
   end if

   ! Check if next timestep is going to be an integer multiple of the module DT values

   if ( (mod(n,p%KFC_nCycles) == 0) ) then
      OtherSt%KFC_NewTime = .true.
   end if
   if ( (mod(n,p%KAD_nCycles) == 0) ) then
      OtherSt%KAD_NewTime = .true.
   end if
      
end subroutine AfterPredict_Onshore

subroutine End_Onshore(errStat, errMsg)

   integer(IntKi),         intent(inout) :: errStat                      ! Error code coming from KiteFAST
   character(ErrMsgLen),   intent(inout) :: errMsg                       ! Error message
 

   integer(IntKi)                  :: errStat2
   character(ErrMsgLen)            :: errMsg2
   character(*), parameter         :: routineName = 'End_Onshore'

   errStat = ErrID_None
   errMsg  = ''

   
   !call KFAST_DestroyOtherState( OtherSt, ErrStat, ErrMsg )
   
   ! Call the End subroutines for KiteAeroDyn, MoorDyn, InflowWind, and the Controller
   if ( p%useKAD ) then
      if ( p%KAD_InterpOrder == 2 ) then
         call KAD_DestroyInput(OtherSt%KAD%u(3), errStat2 , errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call KAD_DestroyOutput(OtherSt%KAD%y(3), errStat2 , errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      else if ( p%KAD_InterpOrder == 1 ) then
         call KAD_DestroyOutput(OtherSt%KAD%y(2), errStat2 , errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      end if
      
      call KAD_DestroyInput(OtherSt%KAD%u(2), errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

      call KAD_End(OtherSt%KAD%u(1), m%KAD%p, m%KAD%x, m%KAD%xd, OtherSt%KAD%z, OtherSt%KAD%OtherSt, OtherSt%KAD%y(1), m%KAD%m, errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   
   if ( p%useMD_Tether ) then
      call MD_DestroyInput(m%MD_Tether%u(2), errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      !call MD_DestroyInput(OtherSt%MD_Tether_u, errStat2 , errMsg2)
      !   call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

      call MD_End(m%MD_Tether%u(1), m%MD_Tether%p, OtherSt%MD_Tether%x, m%MD_Tether%xd, m%MD_Tether%z, OtherSt%MD_Tether%OtherSt, m%MD_Tether%y, m%MD_Tether%m, errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   
   if ( p%useIfW ) then

      call InflowWind_End(m%IfW%u, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z, OtherSt%IfW%OtherSt, m%IfW%y, m%IfW%m, errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   
   
   call KFC_End(m%KFC%p, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
end subroutine End_Onshore

end module KiteFAST_Subs