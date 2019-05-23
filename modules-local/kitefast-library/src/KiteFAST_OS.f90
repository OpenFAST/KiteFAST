module KiteFAST_OS
   use, intrinsic :: ISO_C_Binding 
   use NWTC_Library 
   use MoorDyn_Types
   use MoorDyn
   use HydroDyn
   use HydroDyn_Types
   use InflowWind_Types
   use InflowWind
   use KiteAeroDyn_Types
   use KiteAeroDyn
   use KiteFastController
   use KiteFAST_Types
   use KiteFAST_OS_Types
   use KiteFAST_IO
   use KiteFAST_OS_IO
   use KiteFAST_Subs
   use KiteFAST_GlobalData
   
   implicit none 

private

  
   public :: KFAST_OS_Init
   public :: KFAST_OS_AssRes
   public :: KFAST_OS_Output
   public :: KFAST_OS_AfterPredict
   public :: KFAST_OS_End

   type(KFAST_OS_ParameterType), save  :: p_OS
   type(KFAST_OS_MiscVarType), save    :: m_OS
   type(KFAST_OS_OtherStateType), save :: OtherSt_OS


   contains

subroutine SetupSim_OS(modFlags, p, errStat, errMsg)

   integer(C_INT),            intent(in   ) :: modFlags(:)                    ! Four element array of module flags.  0 = inactive, 1 = active. Module indices are: 0 = KiteAeroDyn, 1 = InflowWind, 2 = MoorDyn, 3 = KiteFAST Controller
   type(KFAST_ParameterType), intent(inout) :: p
   integer(IntKi),            intent(  out) :: errStat                        ! Error status of the operation
   character(*),              intent(  out) :: errMsg                         ! Error message if errStat /= ErrID_None

   errStat = ErrID_None
   errMsg  = ''

   if ( modFlags(5) > 0 ) then  
      p_OS%useHD = .true.
   else
      p_OS%useHD = .false.
   end if
   
   if ( modFlags(6) > 0 ) then 
      if ( .not. p_OS%useHD ) then
         ! If we are using a mooring system we must be also using HydroDyn!
         call SetErrStat(ErrID_Fatal,'Enabling the Mooring module requires the HydroDyn module to be enabled.',errStat,errMsg,'SetupSim_OS')
      end if      
      p_OS%useMD_Mooring = .true.
   else
      p_OS%useMD_Mooring = .false.
   end if

end subroutine SetupSim_OS  

!====================================================================================================
subroutine KFAST_OS_WriteSummary( SumFileUnit, GSRefPtR_c, p, m, HD_InitOut, MD_InitOut, errStat, errMsg )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   integer(IntKi),                 intent( in    ) :: SumFileUnit          ! the unit number for the summary file
   real(C_DOUBLE),                 intent( in    ) :: GSRefPtR_c(3)        ! Initial location of the Ground Station (GS) reference point in global coordinates.
   type(KFAST_OS_ParameterType),   intent( inout ) :: p   
   type(KFAST_OS_MiscVarType),     intent( in    ) :: m  
   type(HydroDyn_InitOutPutType ), intent( in    ) :: HD_InitOut              !
   type(MD_InitOutPutType ),       intent( in    ) :: MD_InitOut              !
   integer,                        intent(   out ) :: ErrStat              ! a non-zero value indicates an error occurred           
   character(*),                   intent(   out ) :: ErrMsg               ! Error message if ErrStat /= ErrID_None
   
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
   
   write(SumFileUnit,'(A90)' ,IOSTAT=TmpErrStat) '--------------'   
   write(SumFileUnit,'(A90)' ,IOSTAT=TmpErrStat) 'Offshore Model'
   write(SumFileUnit,'(A90)' ,IOSTAT=TmpErrStat) '--------------'
   write(SumFileUnit,'()')
   write(SumFileUnit,'(A90)' ,IOSTAT=TmpErrStat) 'Reference Points in Global Coordinates'
   write(SumFileUnit,'()')
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '   Component                         Type                 Number Output Number     x       y       z '
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '     (-)                             (-)                   (-)        (-)         (m)     (m)     (m)'
   
   
         ! reference point for platform
   write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) 'Platform                        ', 'Reference point    ', '-', '-', 0, 0, 0
   write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) 'GS Refererence                  ', 'Reference point    ', '-', '-', GSRefPtR_c(1),GSRefPtR_c(2),GSRefPtR_c(3)
  ! write(SumFileUnit,'(3X,A32,2X,A19,4X,A1,10X,A1,8X,3(F7.3,1X))',IOSTAT=TmpErrStat) 'IMU Reference                   ', 'Reference point    ', '-', '-', 0, 0, 0

   write(SumFileUnit,'()')
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) 'Requested Channels in KiteFASTMBD Output Files for Offshore model: '//num2lstr(p%numKFASTOuts+p%numHDOuts+p%numMDOuts)
   
   write(SumFileUnit,'()')
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '   Number  Name       Units      Generated by'
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '    (-)    (-)         (-)       (KiteFASTMBD, HydroDyn, MoorDyn)'
   write(SumFileUnit,'(A)'   ,IOSTAT=TmpErrStat) '      0   Time        (s)        KiteFASTMBD'
   k = 1
   do i = 1,p%numKFASTOuts
         write(SumFileUnit,'(3X,I4,2X,A11,2X,A9,2X,A)',IOSTAT=TmpErrStat) k, p%OutParam(i)%Name, p%OutParam(i)%Units, 'KiteFASTMBD'
         k = k + 1
   end do
      
      do i = 1,p%numHDOuts
         write(SumFileUnit,'(3X,I4,2X,A11,2X,A9,2X,A)',IOSTAT=TmpErrStat) k, HD_InitOut%WriteOutputHdr(i), HD_InitOut%WriteOutputUnt(i), 'HydroDyn'
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
   end subroutine KFAST_OS_WriteSummary
!====================================================================================================
subroutine KFAST_OS_SetNumOutputs( p, HD_InitOut, MD_InitOut, errStat, errMsg )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   type(KFAST_OS_ParameterType),  intent( inout ) :: p   
   type(HydroDyn_InitOutPutType ),intent( in    ) :: HD_InitOut              !
   type(MD_InitOutPutType ),      intent( in    ) :: MD_InitOut              !
   integer,                       intent(   out ) :: ErrStat              ! a non-zero value indicates an error occurred           
   character(*),                  intent(   out ) :: ErrMsg               ! Error message if ErrStat /= ErrID_None
   
 
   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   ErrStat = ErrID_None  
   ErrMsg  = ""
   
   if ( allocated( HD_InitOut%WriteOutputHdr ) ) then
      p%numHDOuts = size(HD_InitOut%WriteOutputHdr)
   else
      p%numHDOuts = 0
   end if
   if ( allocated( MD_InitOut%WriteOutputHdr ) ) then
      p%numMDOuts = size(MD_InitOut%WriteOutputHdr)
   else
      p%numMDOuts = 0
   end if
   
   p%numOuts = p%numKFASTOuts + p%numHDOuts + p%numMDOuts
   
end subroutine KFAST_OS_SetNumOutputs

!====================================================================================================
subroutine KFAST_OS_WriteOutputChanNames( p, HD_InitOut, MD_InitOut )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   type(KFAST_OS_ParameterType),     intent( inout ) :: p   
   type(HydroDyn_InitOutPutType ),   intent( in    ) :: HD_InitOut              !
   type(MD_InitOutPutType ),         intent( in    ) :: MD_InitOut              !
   
      ! Local variables
   integer                                        :: i                    ! Generic loop counter      
            

   if ( p%numOuts > 0 ) then   ! Output has been requested 

      if ( p%numKFASTOuts > 0 ) then
         do i=1,p%numKFASTOuts
            call WrFileNR ( p%UnOutFile, p%Delim//p%OutParam(i)%Name )
         end do ! I
      end if

      if ( p%numHDOuts > 0 ) then
         do i=1,p%numHDOuts
            call WrFileNR ( p%UnOutFile, p%Delim//HD_InitOut%WriteOutputHdr(i) )
         end do ! I
      end if

      if ( p%numMDOuts > 0 ) then
         do i=1,p%numMDOuts
            call WrFileNR ( p%UnOutFile, p%Delim//MD_InitOut%WriteOutputHdr(i) )
         end do ! I
      end if
      
      ! Intentionally did not write end of line character(s)
      
   end if   ! there are any requested outputs   

end subroutine KFAST_OS_WriteOutputChanNames

!====================================================================================================
subroutine KFAST_OS_WriteOutputUnitNames( p, HD_InitOut, MD_InitOut )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   type(KFAST_OS_ParameterType),     intent( inout ) :: p   
   type(HydroDyn_InitOutPutType ),   intent( in    ) :: HD_InitOut             !
   type(MD_InitOutPutType ),         intent( in    ) :: MD_InitOut              !
   
      ! Local variables
   integer                                        :: i                    ! Generic loop counter       
            
   if ( p%numOuts > 0 ) then   ! Output has been requested 
      
      
      write (p%UnOutFile,'()')
   
         ! Write the names of the output channel units:
      
     
      if ( p%numKFASTOuts > 0 ) then
         do i=1,p%numKFASTOuts
            call WrFileNR ( p%UnOutFile, p%Delim//p%OutParam(i)%Name )
         end do ! I
      end if
      
      if ( p%numHDOuts > 0 ) then
         do i=1,p%numHDOuts
            call WrFileNR ( p%UnOutFile, p%Delim//HD_InitOut%WriteOutputUnt(i) )
         end do ! I
      end if

      if ( p%numMDOuts > 0 ) then
         do i=1,p%numMDOuts
            call WrFileNR ( p%UnOutFile, p%Delim//MD_InitOut%WriteOutputUnt(i) )
         end do ! I
      end if
      
      ! Intentionally did not write end of line character(s)

   end if   ! there are any requested outputs   

end subroutine KFAST_OS_WriteOutputUnitNames

subroutine KFAST_OS_ProcessOutputs()

   real(ReKi) :: val3(3)
   
   ! Platform Motions
   val3 = m_OS%PtfmO
   m%AllOuts(BuoySurge) = val3(1)
   m%AllOuts(BuoySway ) = val3(2)
   m%AllOuts(BuoyHeave) = val3(3)
   val3 = R2D_D*EulerExtract( m_OS%PtfmODCM )
   m%AllOuts(BuoyRoll ) = val3(1)
   m%AllOuts(BuoyPitch) = val3(2)
   m%AllOuts(BuoyYaw  ) = val3(3)
   val3 = matmul( m_OS%PtfmODCM, m_OS%PtfmOv )
   m%AllOuts(BuoyTVx  ) = val3(1)
   m%AllOuts(BuoyTVy  ) = val3(2)
   m%AllOuts(BuoyTVz  ) = val3(3)
   val3 = R2D_D*matmul( m_OS%PtfmODCM, m_OS%PtfmOomegas )
   m%AllOuts(BuoyRVx  ) = val3(1)
   m%AllOuts(BuoyRVy  ) = val3(2)
   m%AllOuts(BuoyRVz  ) = val3(3)
   val3 = matmul( m_OS%PtfmODCM, m_OS%PtfmOacc )
   m%AllOuts(BuoyTAx  ) = val3(1)
   m%AllOuts(BuoyTAy  ) = val3(2)
   m%AllOuts(BuoyTAz  ) = val3(3)

   val3 = m_OS%PtfmIMUPt
   m%AllOuts(BIMUPxi) = val3(1)
   m%AllOuts(BIMUPyi ) = val3(2)
   m%AllOuts(BIMUPzi) = val3(3)
   val3 = R2D_D*EulerExtract( m_OS%PtfmIMUDCM )
   m%AllOuts(BIMURoll ) = val3(1)
   m%AllOuts(BIMUPitch) = val3(2)
   m%AllOuts(BIMUYaw  ) = val3(3)
   val3 = matmul( m_OS%PtfmIMUDCM, m_OS%PtfmIMUv )
   m%AllOuts(BIMUTVx  ) = val3(1)
   m%AllOuts(BIMUTVy  ) = val3(2)
   m%AllOuts(BIMUTVz  ) = val3(3)
   val3 = R2D_D*matmul( m_OS%PtfmIMUDCM, m_OS%PtfmIMUomegas )
   m%AllOuts(BIMURVx  ) = val3(1)
   m%AllOuts(BIMURVy  ) = val3(2)
   m%AllOuts(BIMURVz  ) = val3(3)
   val3 = matmul( m_OS%PtfmIMUDCM, m_OS%PtfmIMUacc )
   m%AllOuts(BIMUTAx  ) = val3(1)
   m%AllOuts(BIMUTAy  ) = val3(2)
   m%AllOuts(BIMUTAz  ) = val3(3)
   
   val3 = m_OS%GSRefPt
   m%AllOuts(BGSRPxi) = val3(1)
   m%AllOuts(BGSRPyi) = val3(2)
   m%AllOuts(BGSRPzi) = val3(3)
   val3 = R2D_D*EulerExtract( m_OS%GSRefDCM )
   m%AllOuts(BGSRRoll ) = val3(1)
   m%AllOuts(BGSRPitch) = val3(2)
   m%AllOuts(BGSRYaw  ) = val3(3)
   val3 = matmul( m_OS%GSRefDCM, m_OS%GSRefv )
   m%AllOuts(BGSRTVx  ) = val3(1)
   m%AllOuts(BGSRTVy  ) = val3(2)
   m%AllOuts(BGSRTVz  ) = val3(3)
   val3 = R2D_D*matmul( m_OS%GSRefDCM, m_OS%GSRefomegas )
   m%AllOuts(BGSRRVx  ) = val3(1)
   m%AllOuts(BGSRRVy  ) = val3(2)
   m%AllOuts(BGSRRVz  ) = val3(3)
   val3 = matmul( m_OS%GSRefDCM, m_OS%GSRefacc )
   m%AllOuts(BGSRTAx  ) = val3(1)
   m%AllOuts(BGSRTAy  ) = val3(2)
   m%AllOuts(BGSRTAz  ) = val3(3)

end subroutine KFAST_OS_ProcessOutputs


!====================================================================================================
subroutine KFAST_OS_WriteOutput( t, p, y_HD, y_MD, errStat, errMsg )
! This subroutine 
!----------------------------------------------------------------------------------------------------

      ! Passed variables
   real(DbKi),                    intent( in    ) :: t
   type(KFAST_OS_ParameterType),  intent( inout ) :: p   
   type(HydroDyn_OutPutType ),    intent( in    ) :: y_HD         !
   type(MD_OutPutType ),          intent( in    ) :: y_MD          !
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

          ! time
      write( tmpStr, '(F10.6)' ) t
      call WrFileNR( p%UnOutFile, tmpStr )

         ! write the individual module output (convert to SiKi if necessary, so that we don't need to print so many digits in the exponent)
      
      if (p%numKFASTOuts > 0 ) then  
         call WrNumAryFileNR ( p%UnOutFile, real(m%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      endif
      
      if ( p%numHDOuts > 0 ) then
        call WrNumAryFileNR ( p%UnOutFile, real(y_HD%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      end if
      
      if ( p%numMDOuts > 0 ) then
            call WrNumAryFileNR ( p%UnOutFile, real(y_MD%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      end if  

   end if   ! are there any requested outputs   

   return
   
end subroutine KFAST_OS_WriteOutput

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


subroutine Init_Offshore(dt, TMax, PtfmO_c, PtfmODCM_c, GSRefPtR_c, HD_FileName_c, MD_FileName_c, p, m, OtherSt, HD_InitOut, MD_Mooring_InitOut, errStat, errMsg)
   real(DbKi),                intent(in   ) :: dt                             ! Timestep size (s)
   real(DbKi),                intent(in   ) :: TMax                           ! Last timestep of simulation (s)
   real(C_DOUBLE),            intent(in   ) :: PtfmO_c(3)                     ! Initial location of the Platform reference point in global coordinates.
   real(C_DOUBLE),            intent(in   ) :: PtfmODCM_c(9)                  ! Initial DCM matrix to transform the location of the Platform reference point from global to platform coordinates.
   real(C_DOUBLE),            intent(in   ) :: GSRefPtR_c(3)                  ! Initial location of the Ground Station (GS) reference point in global coordinates.
   character(kind=C_CHAR),    intent(in   ) :: HD_FileName_c(IntfStrLen)      ! Full path and name of the HydroDyn input file.
   character(kind=C_CHAR),    intent(in   ) :: MD_FileName_c(IntfStrLen)      ! Full path and name of the MoorDyn mooring input file.
   type(KFAST_ParameterType), intent(inout) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m  
   type(KFAST_OtherStateType),intent(inout) :: OtherSt  
   type(HydroDyn_InitOutputType),   intent(  out) :: HD_InitOut
   type(MD_InitOutputType),   intent(  out) :: MD_Mooring_InitOut
   integer(IntKi),            intent(  out) :: errStat                        ! Error status of the operation
   character(*),              intent(  out) :: errMsg                         ! Error message if errStat /= ErrID_None

      ! Local variables
   type(HydroDyn_InitInputType)                   :: HD_InitInp 
   type(MD_InitInputType)                   :: MD_InitInp
   type(KFC_InitInputType)                  :: KFC_InitInp
   character(*), parameter                  :: routineName = 'InitOffShore'
   real(DbKi)                               :: interval
   real(R8Ki)                               :: PtfmODCM(3,3)
   real(ReKi)                               :: PtfmO(3)
   integer(IntKi)                           :: errStat2
   character(ErrMsgLen)                     :: errMsg2


   errStat    = ErrID_None
   errMsg     = ''
   PtfmODCM   = reshape(PtfmODCM_c,(/3,3/))
   PtfmO      = PtfmO_c
   p_OS%GSRefPtR = GSRefPtR_c
      ! Must be using HD if Offshore application (check was done previously for useHD = .true.)
   HD_InitInp%Gravity       = p%Gravity
   HD_InitInp%UseInputFile  = .TRUE.
   HD_InitInp%InputFile     = transfer(HD_FileName_c(1:IntfStrLen-1),HD_InitInp%InputFile)
   HD_InitInp%OutRootName   = p%outFileRoot
   HD_InitInp%TMax          = TMax
   HD_InitInp%hasIce        = .false.
   HD_InitInp%PtfmLocationX = 0.0_ReKi
   HD_InitInp%PtfmLocationY = 0.0_ReKi
   interval = dt
   CALL HydroDyn_Init( HD_InitInp, m_OS%HD%u(1), m_OS%HD%p,  OtherSt_OS%HD%x, OtherSt_OS%HD%xd, m_OS%HD%z, &
                       OtherSt_OS%HD%OtherSt, m_OS%HD%y, m_OS%HD%m, interval, HD_InitOut, errStat2, errMsg2 )
      CALL SetErrStat(errStat2,errMsg2,errStat,errMsg,RoutineName)
      
   if (errStat >= AbortErrLev ) then
      return
   end if
   
   if ( .not. EqualRealNos(interval,dt) ) then
      call SetErrStat(ErrID_Fatal,'HydroDyn DT must be equal to MBDyn DT',errStat,errMsg,routineName)
      return
   end if 

      ! Set up necessary HD mesh mappings
   if ( m_OS%HD%u(1)%Morison%LumpedMesh%Committed  ) then            
      call MeshMapCreate( m_OS%HD%u(1)%Mesh,  m_OS%HD%u(1)%Morison%LumpedMesh, m_OS%HD_P_2_HD_M_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName//':HD_P_2_HD_M_P' )                  
   end if
      
   if ( m_OS%HD%u(1)%Morison%DistribMesh%Committed ) then
      call MeshMapCreate( m_OS%HD%u(1)%Mesh,  m_OS%HD%u(1)%Morison%DistribMesh, m_OS%HD_P_2_HD_M_L, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName//':HD_P_2_HD_M_L' )                  
   end if
   
   if (errStat >= AbortErrLev ) then
      return
   end if

   if (p_OS%useMD_Mooring) then
      MD_InitInp%FileName  = transfer(MD_FileName_c(1:IntfStrLen-1),MD_InitInp%FileName)
      call RemoveNullChar(MD_InitInp%FileName)
   
      MD_InitInp%RootName  = TRIM(p%outFileRoot)//'_Mooring.MD'
         ! The platform in this application is the location of the Kite's fuselage reference point in the inertial coordinate system
      MD_InitInp%PtfmPos   = PtfmO
      MD_InitInp%PtfmDCM   = PtfmODCM
      MD_InitInp%g         = p%Gravity                    ! 
      MD_InitInp%rhoW      = HD_InitOut%WtrDens                   ! This needs to be set according to HD water density value      
      MD_InitInp%WtrDepth  = HD_InitOut%WtrDpth                   ! HD sets the water depth
      interval             = dt
   
      call MD_Init( MD_InitInp, m_OS%MD_Mooring%u(1), m_OS%MD_Mooring%p, OtherSt_OS%MD_Mooring%x, m_OS%MD_Mooring%xd, m_OS%MD_Mooring%z, &
                     OtherSt_OS%MD_Mooring%OtherSt, m_OS%MD_Mooring%y, m_OS%MD_Mooring%m, interval, MD_Mooring_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         
      if (errStat >= AbortErrLev ) then
         return
      end if
        
      if ( .not. EqualRealNos(interval,dt) ) then
         call SetErrStat(ErrID_Fatal,'MoorDyn Mooring DT must be equal to MBDyn DT',errStat,errMsg,routineName) 
         return
      end if
   
         ! Copy t inputs to t-dt, which also populates any arrays/meshes on the new set of inputs.
      call MD_CopyInput( m_OS%MD_Mooring%u(1), m_OS%MD_Mooring%u(2), MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)   
         
         ! Create the necessary Meshes to transfer motions and loads between the Platform reference point and the Mooring Fairleads
      call CreateMBDynPtfmMeshes(PtfmO, PtfmODCM, m_OS%mbdPtfmMotions, m_OS%mbdPtfmLoads, errStat, errMsg )   
         if (errStat>=AbortErrLev) return
         
        
         ! Need to transfer the MBDyn platform reference point motions to MoorDyn_Mooring module
      call MeshMapCreate( m_OS%HD%u(1)%Mesh, m_OS%MD_Mooring%u(1)%PtFairleadDisplacement, m_OS%HD_P_2_MD_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m_OS%HD_P_2_MD_M_P' )     
               if (errStat>=AbortErrLev) return
         ! Need to transfer the MoorDyn mooring fairlead point loads back the to MBDyn platform reference mesh for loads
      call MeshMapCreate( m_OS%MD_Mooring%y%PtFairleadLoad, m_OS%mbdPtfmLoads,  m_OS%MD_M_P_2_MBD_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m_OS%MD_M_P_2_MBD_P' )     
               if (errStat>=AbortErrLev) return

   end if
   
end subroutine Init_Offshore
   
subroutine KFAST_OS_Init(SimMod, dt_c, TMAX_c, numFlaps, numPylons, numComp, numCompNds, modFlags, KAD_FileName_c, IfW_FileName_c, MD_Tether_FileName_c, KFC_FileName_c, HD_FileName_c, MD_Mooring_FileName_c, &
                       outFileRoot_c, printSum, gravity, KAD_InterpOrder, FusODCM_c, numRtrPts_c, rtrPts_c, rtrMass_c, rtrI_Rot_c, rtrI_trans_c, rtrXcm_c, refPts_c, &
                       numNodePts_c, nodePts_c, nodeDCMs_c, nFusOuts_c, FusOutNd_c, nSWnOuts_c, SWnOutNd_c, &
                       nPWnOuts_c, PWnOutNd_c, nVSOuts_c, VSOutNd_c, nSHSOuts_c, SHSOutNd_c, nPHSOuts_c, PHSOutNd_c, nPylOuts_c, PylOutNd_c, PtfmO_c, PtfmODCM_c, GSRefPtR_c, numOutChan_c, chanlist_c, chanlist_len_c, errStat_c, errMsg_c ) BIND (C, NAME='KFAST_OS_Init')

   integer(C_INT),         intent(in   ) :: simMod                             ! Simulation Model: 1 = Onshore, 2 = Full Offshore, 3 = Offshore, platform and mooring only (HD + MD_Mooring)
   real(C_DOUBLE),         intent(in   ) :: dt_c                               ! Timestep size (s)
   real(C_DOUBLE),         intent(in   ) :: TMAX_c                             ! Last Timestep in simulation (s)
   integer(C_INT),         intent(in   ) :: numFlaps                           ! Number of flaps
   integer(C_INT),         intent(in   ) :: numPylons                          ! Number of pylons, per wing
   integer(C_INT),         intent(in   ) :: numComp                            ! Number of kite components
   integer(C_INT),         intent(in   ) :: numCompNds(numComp)                ! MBDyn nodes per component.  The array is populated in the following order: Fuselage, Starboard Wing, Port Wing, Vertical Stabilizer, 
                                                                               !    Starboard Horizontal Stabilizer, Port Horizontal Stabilizer, Starboard pylon, from inner to outer, and then Port pylon, from inner to outer.
   integer(C_INT),         intent(in   ) :: modFlags(6)                        ! Four element array of module flags.  0 = inactive, 1 = active. Module indices are: 0 = KiteAeroDyn, 1 = InflowWind, 2 = MoorDyn, 3 = KiteFAST Controller
   character(kind=C_CHAR), intent(in   ) :: KAD_FileName_c(IntfStrLen)         ! Full path and name of the KiteAeroDyn input file.
   character(kind=C_CHAR), intent(in   ) :: IfW_FileName_c(IntfStrLen)         ! Full path and name of the InflowWind input file.
   character(kind=C_CHAR), intent(in   ) :: MD_Tether_FileName_c(IntfStrLen)   ! Full path and name of the MoorDyn tether input file.
   character(kind=C_CHAR), intent(in   ) :: KFC_FileName_c(IntfStrLen)         ! Full path and name of the KiteFAST controller shared object file.
   character(kind=C_CHAR), intent(in   ) :: HD_FileName_c(IntfStrLen)          ! Full path and name of the HydroDyn input file.
   character(kind=C_CHAR), intent(in   ) :: MD_Mooring_FileName_c(IntfStrLen)  ! Full path and name of the MoorDyn mooring input file.
   character(kind=C_CHAR), intent(in   ) :: outFileRoot_c(IntfStrLen)          ! Full path and basename of the KiteFAST output file.
   integer(C_INT),         intent(in   ) :: printSum                           ! Print the Summary file?  1 = Yes, 0 = No.
   real(C_DOUBLE),         intent(in   ) :: gravity                            ! Scalar gravity constant.  (m/s^2)
   integer(C_INT),         intent(in   ) :: KAD_InterpOrder                    ! KiteAeroDyn outputs interpolation order. 0 = hold outputs between calls, 1 = linear interpolation, 2 = 2nd order interpolation.
   real(C_DOUBLE),         intent(in   ) :: FusODCM_c(9)                       ! Initial DCM matrix to transform the location of the Kite Fuselage reference point from global to kite coordinates.
   integer(C_INT),         intent(in   ) :: numRtrPts_c                        ! Total number of rotor points (both wings).
   real(C_DOUBLE),         intent(in   ) :: rtrPts_c(numRtrPts_c*3)            ! Initial location of each rotor's reference point [RRP] in global coordinates. (m)
   real(C_DOUBLE),         intent(in   ) :: rtrMass_c(numRtrPts_c)             ! Mass of the rotor/drivetrain (kg)
   real(C_DOUBLE),         intent(in   ) :: rtrI_Rot_c(numRtrPts_c)            ! Rotational inertia about the shaft axis of the top and bottom rotors/drivetrains on the pylons on the wing meshes (kg-m2)
   real(C_DOUBLE),         intent(in   ) :: rtrI_trans_c(numRtrPts_c)          ! Transverse inertia about the rotor reference point of the top and bottom rotors/drivetrains on the pylons on the wing meshes (kg-m2)
   real(C_DOUBLE),         intent(in   ) :: rtrXcm_c(numRtrPts_c)              ! Distance along the shaft from the rotor reference point of the top and bottom rotors/drivetrains on the pylons on the wing meshes to the center of mass of the rotor/drivetrain (positive along positive x) (m)
   real(C_DOUBLE),         intent(in   ) :: refPts_c(numComp*3)                ! Initial location of the MBDyn component reference points in the global coordinates. (m)  The length of this array comes from  numComp * 3.
   integer(C_INT),         intent(in   ) :: numNodePts_c                       ! The total number of MBDyn structural nodes.  We need this total number (which could be derived from the numCompNds array) to size the following arrays in the Fortran code. 
   real(C_DOUBLE),         intent(in   ) :: nodePts_c(numNodePts_c*3)          ! Initial location of the MBDyn structural nodes in the global coordinates. (m)  The array is populated in the same order at the numCompNds array.
   real(C_DOUBLE),         intent(in   ) :: nodeDCMs_c(numNodePts_c*9)         ! Initial DCMs matrices to transform each nodal point from global to kite coordinates.
   integer(C_INT),         intent(in   ) :: nFusOuts_c                         ! Number of user-requested output locations on the fuselage  ( 0-9 )
   integer(C_INT),         intent(in   ) :: FusOutNd_c(nFusOuts_c)             ! Node number(s) (within the component) of the requested output locations.  Structural node index for motions and KiteAeroDyn quantities,  and Gauss point index for MBDyn structural loads
   integer(C_INT),         intent(in   ) :: nSWnOuts_c                         ! Number of user-requested output locations on the starboard wing  ( 0-9 )
   integer(C_INT),         intent(in   ) :: SWnOutNd_c(nSWnOuts_c)             ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nPWnOuts_c                         ! Number of user-requested output locations on the port wing  ( 0-9 )
   integer(C_INT),         intent(in   ) :: PWnOutNd_c(nPWnOuts_c)             ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nVSOuts_c                          ! Number of user-requested output locations on the vertical stabilizer  ( 0-9 )
   integer(C_INT),         intent(in   ) :: VSOutNd_c(nVSOuts_c)               ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nSHSOuts_c                         ! Number of user-requested output locations on the starboard horizontal stabilizer  ( 0-9 )
   integer(C_INT),         intent(in   ) :: SHSOutNd_c(nSHSOuts_c)             ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nPHSOuts_c                         ! Number of user-requested output locations on the port horizontal stabilizer  ( 0-9 )
   integer(C_INT),         intent(in   ) :: PHSOutNd_c(nPHSOuts_c)             ! Node number(s) (within the component) of the requested output locations.
   integer(C_INT),         intent(in   ) :: nPylOuts_c                         ! Number of user-requested output locations on each pylon  ( 0-9 )
   integer(C_INT),         intent(in   ) :: PylOutNd_c(nPylOuts_c)             ! Node number(s) (within the component) of the requested output locations.
   real(C_DOUBLE),         intent(in   ) :: PtfmO_c(3)                         ! Initial location of the Platform reference point in global coordinates.
   real(C_DOUBLE),         intent(in   ) :: PtfmODCM_c(9)                      ! Initial DCM matrix to transform the location of the Platform reference point from global to platform coordinates.
   real(C_DOUBLE),         intent(in   ) :: GSRefPtR_c(3)                      ! Initial location of the Ground Station (GS) reference point in global coordinates.
   integer(C_INT),         intent(in   ) :: numOutChan_c                       ! Number of user-requested output channel names
   type(c_ptr)   ,target,  intent(in   ) :: chanlist_c(numOutChan_c)           ! Array of output channel names (strings)
   integer(C_INT),         intent(in   ) :: chanlist_len_c(numOutChan_c )      ! The length of each string in the above array
   integer(C_INT),         intent(  out) :: errStat_c                          ! Error code coming from KiteFAST
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)               ! Error message

      ! Local variables
   real(DbKi)                      :: dt, TMAX
   type(KAD_InitOutputType)        :: KAD_InitOut
   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   type(InflowWind_InitOutputType) :: IfW_InitOut
   type(MD_InitOutputType)         :: MD_Tether_InitOut
   type(HydroDyn_InitOutputType)   :: HD_InitOut
   type(MD_InitOutputType)         :: MD_Mooring_InitOut
   type(KFC_InitInputType)         :: KFC_InitInp
   character(*), parameter         :: routineName = 'KFAST_OS_Init'
   integer(IntKi)                  :: SumFileUnit
   CHARACTER(ChanLen)              :: OutList(MaxOutPts)              ! MaxOutPts is defined in KiteFAST_IO.f90, ChanLen defined in NWTC_Base.f90
   CHARACTER(ChanLen)              :: OnShoreOutList(MaxOutPts)
   character, pointer              :: chanName_f(:)   
   character(255)                  :: enabledModules
   character(255)                  :: disabledModules
   

   errStat = ErrID_None
   errMsg  = ''

   dt   = dt_c
   TMAX = TMAX_c
   p_OS%SimMod = SimMod
   
   call SetupSim(dt, modFlags, gravity, outFileRoot_c, numOutChan_c, chanlist_c, chanlist_len_c, p, OutList, errStat, errMsg)
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if 
   
   call SetupSim_OS(modFlags, p, errStat, errMsg)
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if 
   
   if ( simMod < 2 .or. simMod > 3 ) then
      call SetErrStat( ErrID_FATAL, 'SimMod flag must be set to a value of 1, 2, or 3', errStat, errMsg, routineName ) 
      call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
   end if
   
   if ( simMod == 2 ) then ! Full Offshore, including kite
      call Init_KiteSystem(dt_c, numFlaps, numPylons, numComp, numCompNds, modFlags, KAD_FileName_c, IfW_FileName_c, MD_Tether_FileName_c, KFC_FileName_c, &
                       outFileRoot_c, printSum, gravity, KAD_InterpOrder, FusODCM_c, numRtrPts_c, rtrPts_c, rtrMass_c, rtrI_Rot_c, rtrI_trans_c, rtrXcm_c, refPts_c, &
                       numNodePts_c, nodePts_c, nodeDCMs_c, nFusOuts_c, FusOutNd_c, nSWnOuts_c, SWnOutNd_c, &
                       nPWnOuts_c, PWnOutNd_c, nVSOuts_c, VSOutNd_c, nSHSOuts_c, SHSOutNd_c, nPHSOuts_c, PHSOutNd_c, nPylOuts_c, PylOutNd_c, KAD_InitOut, MD_Tether_InitOut, IfW_InitOut, errStat, errMsg )
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if 
      
      ! TODO: We need to create special mesh mapping between the tether fairlead points which attach to the moving,floating platform.  For the Onshore case, this mapping is not required.
      
      
   end if
   
      ! Init the Hydrodynamics + mooring system if it is enabled
   call Init_Offshore(dt, TMax, PtfmO_c, PtfmODCM_c, GSRefPtR_c, HD_FileName_c, MD_Mooring_FileName_c, p, m, OtherSt, HD_InitOut, MD_Mooring_InitOut, errStat, errMsg)
   if (errStat >= AbortErrLev ) then
      call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
      return
   end if 
   
    ! Set parameters for Offshore output channels:
   
   p_OS%numKFASTOuts = p%numKFASTOuts  ! Set in SetupSim(), at this point these are all the requested, KiteFAST-level output channels (potentially onshore and offshore)
   
   call KFAST_OS_SetOutParam(OutList, p_OS%numKFASTOuts, p_OS, errStat2, errMsg2 ) ! requires:  sets: p_OS%OutParam.
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if
      
   if ( simMod == 2 ) then ! Full Offshore, including kite
      
         ! The OutList can contain both Onshore and Offshore channel names.
         ! The valid Offshore channels were already identified in KFAST_OS_SetOutParam().  Any channels marked invalid will be passed onto the Onshore parsing code
         !   and the previously marked invalid channels will simply be removed from the Offshore list (p_OS%OutParam)
         ! On return, p_OS%numKFASTOuts will correspond to only the valid Offshore channel count, and p%numKFASTOuts will be all the invalid Offshore channels.
         !   Outlist will now only contain the names of all the invalid Offshore channels.
      call KFAST_OS_FindValidParam(OutList, p_OS%numKFASTOuts, p_OS, p%numKFASTOuts, ErrStat, ErrMsg )     
      
         ! Set parameters for onshore output channels:
      call KFAST_SetOutParam(OutList, p%numKFASTOuts, p, errStat2, errMsg2 ) ! requires:  sets: p%OutParam.
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
         

      call AllocAry( m%WriteOutput, p%numKFASTOuts, 'KFAST outputs', errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
         
         ! This sets the p%numOuts data based on p%numKFASTOuts + outputs from the submodules: KAD, MD, IfW
      call KFAST_SetNumOutputs( p, KAD_InitOut, MD_Tether_InitOut, IfW_InitOut, errStat, errMsg )
   end if
   
  
      ! This sets the p_OS%numOuts data based on p_OS%numKFASTOuts + outputs from the submodules: HD, MD
   call KFAST_OS_SetNumOutputs( p_OS, HD_InitOut, MD_Mooring_InitOut, errStat, errMsg )

   
   call AllocAry( m_OS%WriteOutput, p_OS%numKFASTOuts, 'KFAST OS_outputs', errStat2, errMsg2 )
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if
      
! -------------------------------------------------------------------------
! Open the Output file and generate header data
! ------------------------------------------------------------------------- 
   if ( (p%numOuts + p_OS%numOuts) > 0 ) then
      call KFAST_OpenOutput( KFAST_Ver, p%outFileRoot, p, errStat, errMsg ) 
      p_OS%UnOutFile = p%UnOutFile
      call KFAST_WriteOutputTimeChanName( p%UnOutFile )
      if (simMod == 2) call KFAST_WriteOutputChanNames( p, KAD_InitOut, MD_Tether_InitOut, IfW_InitOut )
      call KFAST_OS_WriteOutputChanNames( p_OS, HD_InitOut, MD_Mooring_InitOut )
      call KFAST_WriteOutputNL( p%UnOutFile )      
      
      call KFAST_WriteOutputTimeChanUnits( p%UnOutFile )
      if (simMod == 2) call KFAST_WriteOutputUnitNames( p, KAD_InitOut, MD_Tether_InitOut, IfW_InitOut )  
      call KFAST_OS_WriteOutputUnitNames( p_OS, HD_InitOut, MD_Mooring_InitOut )
      call KFAST_WriteOutputNL( p%UnOutFile )
   end if  
   
! -------------------------------------------------------------------------
! Create Summary file
! -------------------------------------------------------------------------      
      
   if (printSum == 1) then  
      call SumModuleStatus( p, enabledModules, disabledModules ) ! TODO: Add OS modules
      call KFAST_OpenSummary( KFAST_Ver, p%outFileRoot, enabledModules, disabledModules, p%dt, SumFileUnit, errStat, errMsg )
      if (simMod == 2) call KFAST_WriteSummary( SumFileUnit, p, m, KAD_InitOut, MD_Tether_InitOut, IfW_InitOut, ErrStat2, ErrMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      ! TODO: Write OS Summary info
      call KFAST_OS_WriteSummary( SumFileUnit, GSRefPtR_c, p_OS, m_OS, HD_InitOut, MD_Mooring_InitOut, ErrStat2, ErrMsg2 )
      close(SumFileUnit)
         
   end if 
   
   
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
   
end subroutine KFAST_OS_Init
               

subroutine Ass_Res_OffShore(t_c, isInitialTime_c, PtfmO_c, PtfmODCM_c, PtfmOv_c, PtfmOomegas_c, PtfmOacc_c, PtfmOalphas_c, &
                          PtfmIMUPt_c, PtfmIMUDCM_c, PtfmIMUv_c, PtfmIMUomegas_c, PtfmIMUacc_c, &
                          GSRefPt_c, GSRefDCM_c, GSRefv_c, GSRefomegas_c, GSRefacc_c, ptfmLoads_c, errStat, errMsg )
                          
   real(C_DOUBLE),         intent(in   ) :: t_c                          !  simulation time for the current timestep (s)
   integer(C_INT),         intent(in   ) :: isInitialTime_c              !  1 = first time KFAST_AssRes has been called for this particular timestep, 0 = otherwise
   real(C_DOUBLE),         intent(in   ) :: PtfmO_c(3)                   !  Current timestep position of the Platform reference point, expressed in global coordinates. (m) 
   real(C_DOUBLE),         intent(in   ) :: PtfmODCM_c(9)                !  Current timestep DCM matrix to transform the location of the Platform reference point from global to kite coordinates.
   real(C_DOUBLE),         intent(in   ) :: PtfmOv_c(3)                  !  Current timestep velocity of the Platform reference point, expressed in global coordinates. (m/s)
   real(C_DOUBLE),         intent(in   ) :: PtfmOomegas_c(3)             !  Current timestep rotational velocity of the Platform reference point, expressed in global coordinates. (rad/s) 
   real(C_DOUBLE),         intent(in   ) :: PtfmOacc_c(3)                !  Current timestep translational acceleration of the Platform reference point, expressed in global coordinates. (m/s^2) 
   real(C_DOUBLE),         intent(in   ) :: PtfmOalphas_c(3)             !  Current timestep rotational acceleration of the Platform reference point, expressed in global coordinates. (rad/s^2) 
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUPt_c(3)               !  Current timestep position of the Platform IMU point, expressed in global coordinates. (m) 
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUDCM_c(9)              !  Current timestep DCM matrix to transform the location of the Platform IMU point from global to kite coordinates.
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUv_c(3)                !  Current timestep velocity of the Platform IMU point, expressed in global coordinates. (m/s)
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUomegas_c(3)           !  Current timestep rotational velocity of the Platform IMU point, expressed in global coordinates. (rad/s) 
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUacc_c(3)              !  Current timestep translational acceleration of the Platform IMU point, expressed in global coordinates. (m/s^2) 
   real(C_DOUBLE),         intent(in   ) :: GSRefPt_c(3)                 !  Current timestep position of the ground station reference point, expressed in global coordinates. (m) 
   real(C_DOUBLE),         intent(in   ) :: GSRefDCM_c(9)                !  Current timestep DCM matrix to transform the location of the ground station reference point from global to kite coordinates.
   real(C_DOUBLE),         intent(in   ) :: GSRefv_c(3)                  !  Current timestep velocity of the ground station reference point, expressed in global coordinates. (m/s)
   real(C_DOUBLE),         intent(in   ) :: GSRefomegas_c(3)             !  Current timestep rotational velocity of the ground station reference point, expressed in global coordinates. (rad/s) 
   real(C_DOUBLE),         intent(in   ) :: GSRefacc_c(3)                !  Current timestep translational acceleration of the ground station reference point, expressed in global coordinates. (m/s^2) 
   real(C_DOUBLE),         intent(  out) :: ptfmLoads_c(6)               !  Concentrated loads at the plaform reference pont in global coordinates.  Returned from KiteFAST to MBDyn.
   integer(IntKi),         intent(  out) :: errStat                      ! Error status of the operation
   character(*),           intent(  out) :: errMsg                       ! Error message if errStat /= ErrID_None

    ! Local variables
   
   integer(IntKi)           :: n, n2, c, i, j                 ! counters
   integer(IntKi)           :: isInitialTime                  ! Is this the initial time of the simulation 1=Yes, should we update the states? 0=yes, 1=no
   real(DbKi)               :: t                              ! simulations time (s)
   real(DbKi)               :: utimes(2)                      ! t-p%dt and t timestep values (s)
   integer(IntKi)           :: errStat2              ! error status values
   character(ErrMsgLen)     :: errMsg2                ! error messages
   character(*), parameter  :: routineName = 'AssRes_Onshore'


   integer(IntKi)           :: numFairLeads
   
   errStat = ErrID_None
   errMsg  = ''

   
   isInitialTime = isInitialTime_c
   t = t_c
   
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
   
   m_OS%PtfmO         = PtfmO_c
   m_OS%PtfmODCM      = reshape(PtfmODCM_c,(/3,3/) )
   m_OS%PtfmOv        = PtfmOv_c    
   m_OS%PtfmOomegas   = PtfmOomegas_c
   m_OS%PtfmOacc      = PtfmOacc_c 
   m_OS%PtfmOalphas   = PtfmOalphas_c
   m_OS%PtfmIMUPt     = PtfmIMUPt_c
   m_OS%PtfmIMUDCM    = reshape(PtfmIMUDCM_c,(/3,3/) )
   m_OS%PtfmIMUv      = PtfmIMUv_c
   m_OS%PtfmIMUomegas = PtfmIMUomegas_c
   m_OS%PtfmIMUacc    = PtfmIMUacc_c
   m_OS%GSRefPt       = GSRefPt_c
   m_OS%GSRefDCM      = reshape(GSRefDCM_c,(/3,3/) )
   m_OS%GSRefv        = GSRefv_c  
   m_OS%GSRefomegas   = GSRefomegas_c
   m_OS%GSRefacc      = GSRefacc_c

   ! HydroDyn
   
   ! Transfer MBDyn platform motions to HD input mesh
                       
   m_OS%HD%u(1)%Mesh%Orientation  (:,:,1) = m_OS%PtfmODCM
   m_OS%HD%u(1)%Mesh%TranslationDisp(:,1) = PtfmO_c - m_OS%HD%u(1)%Mesh%Position(:,1)
   m_OS%mbdPtfmLoads%TranslationDisp(:,1) = m_OS%HD%u(1)%Mesh%TranslationDisp(:,1)
   m_OS%HD%u(1)%Mesh%TranslationVel (:,1) = PtfmOv_c
   m_OS%HD%u(1)%Mesh%RotationVel    (:,1) = PtfmOomegas_c
   m_OS%HD%u(1)%Mesh%TranslationAcc (:,1) = PtfmOacc_c
   m_OS%HD%u(1)%Mesh%RotationAcc    (:,1) = PtfmOalphas_c
   
   IF ( m_OS%HD%u(1)%Morison%LumpedMesh%Committed ) THEN 

      ! These are the motions for the lumped point loads associated viscous drag on the WAMIT body and/or filled/flooded lumped forces of the WAMIT body
      CALL Transfer_Point_to_Point( m_OS%HD%u(1)%Mesh, m_OS%HD%u(1)%Morison%LumpedMesh, m_OS%HD_P_2_HD_M_P, ErrStat2, ErrMsg2 )
         CALL SetErrStat(ErrStat2,ErrMsg2,ErrStat, ErrMsg,'Transfer_HD_to_HD (m_OS%HD%u(1)%Morison%LumpedMesh)' )
         
   END IF
   
   IF ( m_OS%HD%u(1)%Morison%DistribMesh%Committed ) THEN 
         
      ! These are the motions for the line2 (distributed) loads associated viscous drag on the WAMIT body and/or filled/flooded distributed forces of the WAMIT body
      CALL Transfer_Point_to_Line2( m_OS%HD%u(1)%Mesh, m_OS%HD%u(1)%Morison%DistribMesh, m_OS%HD_P_2_HD_M_L, ErrStat2, ErrMsg2 )
         CALL SetErrStat(ErrStat2,ErrMsg2,ErrStat, ErrMsg,'Transfer_HD_to_HD (m_OS%HD%u(1)%Morison%DistribMesh)' )

   END IF
   
   call HydroDyn_CopyContState( OtherSt_OS%HD%x, OtherSt_OS%HD%x_copy, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call HydroDyn_CopyDiscState( OtherSt_OS%HD%xd, OtherSt_OS%HD%xd_copy, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         
   if ( isInitialTime < 1 ) then
      
      call HydroDyn_CopyInput( OtherSt_OS%HD_u, m_OS%HD%u(2), MESH_NEWCOPY, errStat2, errMsg2 )
      call HydroDyn_UpdateStates( utimes(1), n, m_OS%HD%u, utimes, m_OS%HD%p, OtherSt_OS%HD%x_copy, OtherSt_OS%HD%xd_copy, m_OS%HD%z, OtherSt_OS%HD%OtherSt, m_OS%HD%m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      if (errStat >= AbortErrLev ) return      
            
   end if         
         
   call HydroDyn_CalcOutput( t, m_OS%HD%u(1), m_OS%HD%p, OtherSt_OS%HD%x_copy, OtherSt_OS%HD%xd_copy, m_OS%HD%z, OtherSt_OS%HD%OtherSt, m_OS%HD%y, m_OS%HD%m, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   if (errStat >= AbortErrLev ) return
      
   do i = 1,3
      ptfmLoads_c(i)   = m_OS%HD%y%AllHdroOrigin%Force(i,1)
      ptfmLoads_c(i+3) = m_OS%HD%y%AllHdroOrigin%Moment(i,1)      
   end do
   
   ! MoorDyn - Moorings 

   if ( p_OS%useMD_Mooring ) then
      
      call Transfer_Point_to_Point( m_OS%HD%u(1)%Mesh, m_OS%MD_Mooring%u(1)%PtFairleadDisplacement, m_OS%HD_P_2_MD_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      if (errStat >= AbortErrLev ) return


      call MD_CopyContState( OtherSt_OS%MD_Mooring%x, OtherSt_OS%MD_Mooring%x_copy, MESH_NEWCOPY, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

      if ( isInitialTime < 1 ) then
         call MD_CopyInput( OtherSt_OS%MD_Mooring_u, m_OS%MD_Mooring%u(2), MESH_NEWCOPY, errStat2, errMsg2 )
         call MD_UpdateStates( utimes(1), n, m_OS%MD_Mooring%u, utimes, m_OS%MD_Mooring%p, OtherSt_OS%MD_Mooring%x_copy, m_OS%MD_Mooring%xd, m_OS%MD_Mooring%z, OtherSt_OS%MD_Mooring%OtherSt, m_OS%MD_Mooring%m, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) return
      end if

!print *, "Beginning MD_CalcOutput for Mooring"  
!print *, " at time="//trim(num2lstr(t))
!print *, "m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(1,1): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(1,1)))//", m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(2,1): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(2,1)))//", m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(3,1): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(3,1)))
!print *, "m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(1,2): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(1,2)))//", m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(2,2): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(2,2)))//", m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(3,2): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%Position(3,2)))
!print *, "m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(1,1): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(1,1)))//", m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(2,1): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(2,1)))//", m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(3,1): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(3,1)))
!print *, "m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(1,2): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(1,2)))//", m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(2,2): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(2,2)))//", m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(3,2): "//trim(num2lstr(m_OS%MD_Mooring%u(1)%PtFairLeadDisplacement%TranslationDisp(3,2)))


      call MD_CalcOutput( t, m_OS%MD_Mooring%u(1), m_OS%MD_Mooring%p, OtherSt_OS%MD_Mooring%x_copy, m_OS%MD_Mooring%xd, m_OS%MD_Mooring%z, OtherSt_OS%MD_Mooring%OtherSt, m_OS%MD_Mooring%y, m_OS%MD_Mooring%m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      if (errStat >= AbortErrLev ) return

      ! Transfer fairlead loads to the MBDyn Platform loads mesh
      
      call Transfer_Point_to_Point( m_OS%MD_Mooring%y%PtFairLeadLoad, m_OS%mbdPtfmLoads, m_OS%MD_M_P_2_MBD_P, errStat2, errMsg2, m_OS%MD_Mooring%u(1)%PtFairleadDisplacement, m_OS%mbdPtfmLoads )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      if (errStat >= AbortErrLev ) return

      do i = 1,3
         ptfmLoads_c(i)   = ptfmLoads_c(i)   + m_OS%mbdPtfmLoads%Force(i,1)
         ptfmLoads_c(i+3) = ptfmLoads_c(i+3) + m_OS%mbdPtfmLoads%Moment(i,1)      
      end do
!print *, "Finished MD_CalcOutput" 
!print *, "m_OS%MD_Mooring%y%PtFairLeadLoad%Force(1,1): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Force(1,1)))//", m_OS%MD_Mooring%y%PtFairLeadLoad%Force(2,1): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Force(2,1)))//", m_OS%MD_Mooring%y%PtFairLeadLoad%Force(3,1): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Force(3,1)))
!print *, "m_OS%MD_Mooring%y%PtFairLeadLoad%Force(1,2): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Force(1,2)))//", m_OS%MD_Mooring%y%PtFairLeadLoad%Force(2,2): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Force(2,2)))//", m_OS%MD_Mooring%y%PtFairLeadLoad%Force(3,2): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Force(3,2)))
!print *, "m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(1,1): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(1,1)))//", m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(2,1): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(2,1)))//", m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(3,1): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(3,1)))
!print *, "m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(1,2): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(1,2)))//", m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(2,2): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(2,2)))//", m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(3,2): "//trim(num2lstr(m_OS%MD_Mooring%y%PtFairLeadLoad%Moment(3,2)))  
!m_OS%MD_Mooring%y%PtFairLeadLoad%Force(:,1)=0.0_ReKi
!m_OS%MD_Mooring%y%PtFairLeadLoad%Force(:,2)=0.0_ReKi
      
   end if

end subroutine Ass_Res_OffShore
                       
                       
subroutine KFAST_OS_AssRes(t_c, isInitialTime_c, WindPt_c, WindPtVel_c, FusO_c, FusODCM_c, FusOv_c, FusOomegas_c, FusOacc_c, FusOalphas_c, numNodePts_c, nodePts_c, &
                          nodeDCMs_c, nodeVels_c, nodeOmegas_c, nodeAccs_c,  numRtrPts_c, rtrPts_c, &
                          rtrDCMs_c, rtrVels_c, rtrOmegas_c, rtrAccs_c, rtrAlphas_c, &
                          PtfmO_c, PtfmODCM_c, PtfmOv_c, PtfmOomegas_c, PtfmOacc_c, PtfmOalphas_c, &
                          PtfmIMUPt_c, PtfmIMUDCM_c, PtfmIMUv_c, PtfmIMUomegas_c, PtfmIMUacc_c, &
                          GSRefPt_c, GSRefDCM_c, GSRefv_c, GSRefomegas_c, GSRefacc_c, nodeLoads_c, rtrLoads_c, ptfmLoads_c, errStat_c, errMsg_c ) BIND (C, NAME='KFAST_OS_AssRes')
   IMPLICIT NONE

   real(C_DOUBLE),         intent(in   ) :: t_c                          !  simulation time for the current timestep (s)
   integer(C_INT),         intent(in   ) :: isInitialTime_c              !  1 = first time KFAST_AssRes has been called for this particular timestep, 0 = otherwise
   real(C_DOUBLE),         intent(in   ) :: WindPt_c(3)                  !  Position of the ground station where the fixed wind measurement is taken, expressed in global coordinates. (m)
   real(C_DOUBLE),         intent(in   ) :: WindPtVel_c(3)               !  Velocity of the ground station where the fixed wind measurement is taken, expressed in global coordinates. (m)
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
   real(C_DOUBLE),         intent(in   ) :: PtfmO_c(3)                   !  Current timestep position of the Platform reference point, expressed in global coordinates. (m) 
   real(C_DOUBLE),         intent(in   ) :: PtfmODCM_c(9)                !  Current timestep DCM matrix to transform the location of the Platform reference point from global to kite coordinates.
   real(C_DOUBLE),         intent(in   ) :: PtfmOv_c(3)                  !  Current timestep velocity of the Platform reference point, expressed in global coordinates. (m/s)
   real(C_DOUBLE),         intent(in   ) :: PtfmOomegas_c(3)             !  Current timestep rotational velocity of the Platform reference point, expressed in global coordinates. (rad/s) 
   real(C_DOUBLE),         intent(in   ) :: PtfmOacc_c(3)                !  Current timestep translational acceleration of the Platform reference point, expressed in global coordinates. (m/s^2) 
   real(C_DOUBLE),         intent(in   ) :: PtfmOalphas_c(3)             !  Current timestep rotational acceleration of the Platform reference point, expressed in global coordinates. (rad/s^2) 
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUPt_c(3)               !  Current timestep position of the Platform IMU point, expressed in global coordinates. (m) 
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUDCM_c(9)              !  Current timestep DCM matrix to transform the location of the Platform IMU point from global to kite coordinates.
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUv_c(3)                !  Current timestep velocity of the Platform IMU point, expressed in global coordinates. (m/s)
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUomegas_c(3)           !  Current timestep rotational velocity of the Platform IMU point, expressed in global coordinates. (rad/s) 
   real(C_DOUBLE),         intent(in   ) :: PtfmIMUacc_c(3)              !  Current timestep translational acceleration of the Platform IMU point, expressed in global coordinates. (m/s^2) 
   real(C_DOUBLE),         intent(in   ) :: GSRefPt_c(3)                 !  Current timestep position of the ground station reference point, expressed in global coordinates. (m) 
   real(C_DOUBLE),         intent(in   ) :: GSRefDCM_c(9)                !  Current timestep DCM matrix to transform the location of the ground station reference point from global to kite coordinates.
   real(C_DOUBLE),         intent(in   ) :: GSRefv_c(3)                  !  Current timestep velocity of the ground station reference point, expressed in global coordinates. (m/s)
   real(C_DOUBLE),         intent(in   ) :: GSRefomegas_c(3)             !  Current timestep rotational velocity of the ground station reference point, expressed in global coordinates. (rad/s) 
   real(C_DOUBLE),         intent(in   ) :: GSRefacc_c(3)                !  Current timestep translational acceleration of the ground station reference point, expressed in global coordinates. (m/s^2) 
   real(C_DOUBLE),         intent(  out) :: nodeLoads_c(numNodePts_c*6)  !  KiteFAST loads (3 forces + 3 moments) in global coordinates ( N, N-m ) at the MBDyn structural nodes.  Sequence follows the pattern used for MBDyn structural node array.  Returned from KiteFAST to MBDyn.
   real(C_DOUBLE),         intent(  out) :: rtrLoads_c(numRtrPts_c*6)    !  Concentrated reaction loads at the nacelles on the pylons at the RRPs in global coordinates.  Length is 6 loads per rotor * number of RRPs. Returned from KiteFAST to MBDyn.
   real(C_DOUBLE),         intent(  out) :: ptfmLoads_c(6)               !  Concentrated loads at the plaform reference pont in global coordinates.  Returned from KiteFAST to MBDyn.
   integer(C_INT),         intent(  out) :: errStat_c                    !  Error code coming from KiteFAST
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)         !  Error message
   
   
   ! Local variables
   
   integer(IntKi)           :: n, n2, c, i, j                 ! counters
   integer(IntKi)           :: isInitialTime                  ! Is this the initial time of the simulation 1=Yes, should we update the states? 0=yes, 1=no
   real(DbKi)               :: t                              ! simulations time (s)
   real(DbKi)               :: utimes(2)                      ! t-p_OS%dt and t timestep values (s)
   real(ReKi)               :: AeroMoment(3)                  ! AeroDynamic moment on a rotor as computed by KAD (N-m)
   real(ReKi)               :: xhat(3)                        ! 
   real(ReKi)               :: AeroForce(3)                   ! AeroDynamic force on a rotor as computed by KAD (N)              ! 
   integer(IntKi)           :: errStat, errStat2              ! error status values
   character(ErrMsgLen)     :: errMsg, errMsg2                ! error messages
   character(*), parameter  :: routineName = 'KFAST_AssRes'
   logical                  :: test, doTransfersforKFC
   real(SiKi)               :: fracStep, t_s, dt_s
   real(ReKi)               :: IfW_ground(3)
   real(ReKi)               :: IfW_FusO(3)
   integer(IntKi)           :: numFairLeads
   
   errStat = ErrID_None
   errMsg  = ''
   t = t_c
   
   if ( p_OS%simMod < 3 ) then
      call AssRes_OnShore( t_c, isInitialTime_c, WindPt_c, FusO_c, FusODCM_c, FusOv_c, FusOomegas_c, FusOacc_c, FusOalphas_c, numNodePts_c, nodePts_c, &
                          nodeDCMs_c, nodeVels_c, nodeOmegas_c, nodeAccs_c,  numRtrPts_c, rtrPts_c, &
                          rtrDCMs_c, rtrVels_c, rtrOmegas_c, rtrAccs_c, rtrAlphas_c, nodeLoads_c, rtrLoads_c, errStat, errMsg ) 
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c) 
         return
      end if
      
   end if
   
   call Ass_Res_OffShore(t_c, isInitialTime_c, PtfmO_c, PtfmODCM_c, PtfmOv_c, PtfmOomegas_c, PtfmOacc_c, PtfmOalphas_c, &
                          PtfmIMUPt_c, PtfmIMUDCM_c, PtfmIMUv_c, PtfmIMUomegas_c, PtfmIMUacc_c, &
                          GSRefPt_c, GSRefDCM_c, GSRefv_c, GSRefomegas_c, GSRefacc_c, ptfmLoads_c, errStat, errMsg)
   
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c) 
   
end subroutine KFAST_OS_AssRes
 
subroutine KFAST_OS_AfterPredict(t_c, errStat_c, errMsg_c) BIND (C, NAME='KFAST_OS_AfterPredict')
   IMPLICIT NONE
   real(C_DOUBLE),         intent(in   ) :: t_c
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_OS_AfterPredict'
   integer(IntKi)                  :: numFairLeads, i, count, j
   real(DbKi)                      :: t
   integer(IntKi)                  :: n
   
   errStat = ErrID_None
   errMsg  = ''
   
   t = t_c  
         
   if ( p_OS%simMod < 3 ) then
      call AfterPredict_Onshore(t_c, errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   
!===================
! Offshore
!===================
   
   if ( p_OS%useMD_Mooring ) then
         ! Copy the temporary states and place them into the actual versions
      call MD_CopyContState( OtherSt_OS%MD_Mooring%x_copy, OtherSt_OS%MD_Mooring%x, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         ! Copy t inputs for use has  t-dt, historical inputs, for the next timestep
      call MD_CopyInput( m_OS%MD_Mooring%u(1), OtherSt_OS%MD_Mooring_u, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   if ( p_OS%useHD ) then
         ! Copy the temporary states and place them into the actual versions
      call HydroDyn_CopyContState( OtherSt_OS%HD%x_copy, OtherSt_OS%HD%x, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call HydroDyn_CopyDiscState( OtherSt_OS%HD%xd_copy, OtherSt_OS%HD%xd, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         ! Copy t inputs for use has  t-dt, historical inputs, for the next timestep
      call HydroDyn_CopyInput( m_OS%HD%u(1), OtherSt_OS%HD_u, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   

   
   
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
      
end subroutine KFAST_OS_AfterPredict

subroutine KFAST_OS_Output(t_c, numGaussPts_c, gaussPtLoads_c, errStat_c, errMsg_c) BIND (C, NAME='KFAST_OS_Output')
   IMPLICIT NONE
   real(C_DOUBLE),         intent(in   ) :: t_c                              ! simulation time for the current timestep (s)   
   integer(C_INT),         intent(in   ) :: numGaussPts_c                    ! Total number of gauss points in the MBDyn model
   real(C_DOUBLE),         intent(in   ) :: gaussPtLoads_c(numGaussPts_c*6)  ! Array of loads in the global coordinate system (3 forces + 3 moments) corresponding to each MBDyn gauss point. ( N, N-m )
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_OS_Output'
   real(DbKi)                      :: t
   
   errStat = ErrID_None
   errMsg  = ''
   t = real(t_c,DbKi)

   
   
   

   if (p%numOuts  > 0 .and. p_OS%simMod == 2 ) then
      
      call TransferMBDynLoadInputs( numGaussPts_c, gaussPtLoads_c, p, m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
         ! Compute the MBDyn-related outputs using the MBDyn load data and the most recently recieved motion data
      call KFAST_ProcessOutputs()
   end if
   
   if ( (p%numOuts + p_OS%numOuts) > 0 ) then

      call KFAST_WriteOutputTime( t, p%UnOutFile )
      if (p%numOuts  > 0 .and. p_OS%simMod == 2) then
         call KFAST_WriteOutput( t, p, m%KAD%y, m%MD_Tether%y, m%IfW%y, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      end if
      if ( p_OS%numOuts > 0 ) then
         call KFAST_OS_ProcessOutputs()
         call KFAST_OS_WriteOutput( t, p_OS, m_OS%HD%y, m_OS%MD_Mooring%y, errStat2, errMsg2 )
      end if
      
      call KFAST_WriteOutputNL( p%UnOutFile ) 
      
   end if
   
      ! transfer Fortran variables to C:  
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)

end subroutine KFAST_OS_Output

subroutine KFAST_OS_End(errStat_c, errMsg_c) BIND (C, NAME='KFAST_OS_End')
   IMPLICIT NONE
   
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_OS_End'

   errStat = ErrID_None
   errMsg  = ''

   if ( p_OS%simMod < 3 ) then
      ! Call the End subroutines for KiteAeroDyn, MoorDyn, InflowWind, and the Controller
      call End_Onshore(errStat, errMsg)
   end if
   
   call KFAST_OS_DestroyParam( p_OS, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KFAST_OS_DestroyMisc( m_OS, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KFAST_OS_DestroyOtherState( OtherSt_OS, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

! -------------------------------------------------------------------------
! Close the Output file
! -------------------------------------------------------------------------   
   close(p%UnOutFile)
      
      ! transfer Fortran variables to C:  
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
  
end subroutine KFAST_OS_End
end module KiteFAST_OS