module KiteFAST
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
   
   implicit none 

private

   type(ProgDesc), parameter  :: KFAST_Ver = ProgDesc( 'KiteFAST', '', '' )
   integer,        parameter  :: IntfStrLen  = 1025       ! length of strings through the C interface
   type(KFAST_ParameterType)  :: p
   type(KFAST_MiscVarType)    :: m
   type(KFAST_OtherStateType)    :: OtherSt
   !type(KFAST_InputType)      :: u
   !type(KFAST_OutputType)     :: y
   
   
   public :: KFAST_Init
   public :: KFAST_AssRes
   public :: KFAST_Output
   public :: KFAST_AfterPredict
   public :: KFAST_End


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

!====================================================================================================
subroutine KFAST_WriteOutput( t, p, y_KAD, y_MD, y_IfW, errStat, errMsg )
! This subroutine 
!----------------------------------------------------------------------------------------------------

      ! Passed variables
   real(DbKi),                    intent( in    ) :: t
   type(KFAST_ParameterType),     intent( inout ) :: p   
   type(KAD_OutPutType ),         intent( in    ) :: y_KAD          !
   type(MD_OutPutType ),          intent( in    ) :: y_MD          !
   type(InflowWind_OutPutType ),  intent( in    ) :: y_IfW              !
   integer,                       intent(   out ) :: errStat              ! a non-zero value indicates an error occurred           
   character(*),                  intent(   out ) :: errMsg               ! Error message if ErrStat /= ErrID_None
   
      ! Local variables
   integer                                        :: i                    ! Generic loop counter      
   character(1024)                                :: OutFileName          ! The name of the output file  including the full path.
   integer                                        :: errStat2              
   character(ErrMsgLen)                           :: errMsg2                ! error messages
   character(200)                                 :: Frmt                                      ! A string to hold a format specifier
   character(14)                                  :: TmpStr                                    ! temporary string to print the time output as text

   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   ErrStat = ErrID_None  
   ErrMsg  = ""

   if ( p%numOuts > 0 ) then   ! Output has been requested so let's write to the output file            
 
             ! Write one line of tabular output:
      Frmt = '"'//p%Delim//'"'//p%OutFmt      ! format for array elements from individual modules

            ! time
      write( tmpStr, '(F10.6)' ) t
      call WrFileNR( p%UnOutFile, tmpStr )

         ! write the individual module output (convert to SiKi if necessary, so that we don't need to print so many digits in the exponent)
      
      
      if ( p%numKADOuts > 0 ) then
        call WrNumAryFileNR ( p%UnOutFile, real(y_KAD%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      end if
      
      if ( p%numMDOuts > 0 ) then
            call WrNumAryFileNR ( p%UnOutFile, real(y_MD%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      end if
      if ( p%numIfWOuts > 0 ) then
            call WrNumAryFileNR ( p%UnOutFile, real(y_IfW%WriteOutput,SiKi), Frmt, errStat2, errMsg2 )
      end if
      write (p%UnOutFile,'()')

   end if   ! there are any requested outputs   

   return

end subroutine KFAST_WriteOutput
!====================================================================================================
subroutine KFAST_OpenOutput( ProgVer, OutRootName, p, KAD_InitOut, MD_InitOut, IfW_InitOut, errStat, errMsg )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

      ! Passed variables

   type(ProgDesc),                intent( in    ) :: ProgVer
   character(*),                  intent( in    ) :: OutRootName          ! Root name for the output file
   type(KFAST_ParameterType),     intent( inout ) :: p   
   type(KAD_InitOutPutType ),     intent( in    ) :: KAD_InitOut              !
   type(MD_InitOutPutType ),      intent( in    ) :: MD_InitOut              !
   type(InflowWind_InitOutPutType ),     intent( in    ) :: IfW_InitOut              !
   integer,                       intent(   out ) :: ErrStat              ! a non-zero value indicates an error occurred           
   character(*),                  intent(   out ) :: ErrMsg               ! Error message if ErrStat /= ErrID_None
   
      ! Local variables
   integer                                        :: i                    ! Generic loop counter      
   character(1024)                                :: OutFileName          ! The name of the output file  including the full path.
   character(200)                                 :: Frmt                 ! a string to hold a format statement
   integer                                        :: errStat2              
   character(ErrMsgLen)                           :: errMsg2                ! error messages
   
   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   ErrStat = ErrID_None  
   ErrMsg  = ""
   p%OutSFmt = "A10"  
   p%OutFmt  = "ES10.3E2"
   p%Delim = ' '
   
   
   !-------------------------------------------------------------------------------------------------      
   ! Open the output file, if necessary, and write the header
   !-------------------------------------------------------------------------------------------------      
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
   p%numOuts = p%numKADOuts + p%numMDOuts + p%numIfWOuts
   
   if ( p%numOuts > 0 ) then   ! Output has been requested so let's open an output file            
      
         ! Open the file for output
      OutFileName = trim(OutRootName)//'.out'
      call GetNewUnit( p%UnOutFile )
   
      call OpenFOutFile ( p%UnOutFile, OutFileName, errStat, errMsg ) 
      if ( errStat >= AbortErrLev ) then
         errMsg = ' Error opening KiteFAST output file: '//trim(errMsg)
         return
      end if
      
       
         ! Write the output file header
      
      write (p%UnOutFile,'(/,A/)', IOSTAT=errStat2)  'These predictions were generated by '//trim(GETNVD(ProgVer))//&
                      ' on '//CurDate()//' at '//CurTime()//'.'
      
      write(p%UnOutFile, '(//)') ! add 3 lines to make file format consistant with FAST v8 (headers on line 7; units on line 8) [this allows easier post-processing]

         ! Write the names of the output channels:
      
      call WrFileNR ( p%UnOutFile, '   Time   ' )
      
      if ( p%numKADOuts > 0 ) then
         do i=1,p%numKADOuts
            call WrFileNR ( p%UnOutFile, p%Delim//KAD_InitOut%WriteOutputHdr(i) )
         end do ! I
      end if
      if ( p%numMDOuts > 0 ) then
         do i=1,p%numMDOuts
            call WrFileNR ( p%UnOutFile, p%Delim//MD_InitOut%WriteOutputHdr(i) )
         end do ! I
      end if
      if ( p%numIfWOuts > 0 ) then
         do i=1,p%numIfWOuts
            call WrFileNR ( p%UnOutFile, p%Delim//IfW_InitOut%WriteOutputHdr(i) )
         end do ! I
      end if
      write (p%UnOutFile,'()')
      
      
      
      
         ! Write the names of the output channel units:
      
      call WrFileNR ( p%UnOutFile, '    s     ' )
      
      if ( p%numKADOuts > 0 ) then
         do i=1,p%numKADOuts
            call WrFileNR ( p%UnOutFile, p%Delim//KAD_InitOut%WriteOutputUnt(i) )
         end do ! I
      end if
      if ( p%numMDOuts > 0 ) then
         do i=1,p%numMDOuts
            call WrFileNR ( p%UnOutFile, p%Delim//MD_InitOut%WriteOutputUnt(i) )
         end do ! I
      end if
      if ( p%numIfWOuts > 0 ) then
         do i=1,p%numIfWOuts
            call WrFileNR ( p%UnOutFile, p%Delim//IfW_InitOut%WriteOutputUnt(i) )
         end do ! I
      end if
      write (p%UnOutFile,'()')

   end if   ! there are any requested outputs   

   return

end subroutine KFAST_OpenOutput

subroutine TransferLoadsToMBDyn( p, m, nodeLoads_c, rtrLoads_c, errStat, errMsg )
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   real(C_DOUBLE),            intent(inout) :: nodeLoads_c(:)  ! 1D array of the nodal point loads
   real(C_DOUBLE),            intent(inout) :: rtrLoads_c(:)   ! 1D array of the rotor point loads
   integer(IntKi),            intent(  out) :: errStat         !< Error status of the operation
   character(*),              intent(  out) :: errMsg          !< Error message if errStat /= ErrID_None

   integer(IntKi)  :: i, istart, j,k,c, compOffset
   integer(IntKi)           :: errStat2              ! error status values
   character(ErrMsgLen)     :: errMsg2                ! error messages
   character(*), parameter  :: routineName = 'TransferLoadsToMBDyn'
   
   errStat = ErrID_None
   errMsg  = ''

   nodeLoads_c = 0.0
   rtrLoads_c  = 0.0
   
   if ( p%useMD ) then
         ! First map MD loads back to the wing mesh
         
      !TODO: add translationdisp field to m%mbdWngLoads and manually apply the m%mbdWngMotions TranslationDisp to m%mbdWngLoads mesh
      call Transfer_Point_to_Point( m%MD%y%PtFairleadLoad, m%mbdWngLoads,  m%MD_P_2_P, errStat2, errMsg2, m%MD%u(2)%PtFairleadDisplacement, m%mbdWngLoads )
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
         
      ! Map KAD rotor loads to the corresponding MBDyn loads (no mesh mapping required)
      do i = 1, p%NumPylons *2
         j = 6*(i-1) + 1
         rtrLoads_c(j  ) = m%KAD%y%SPyRtrLoads(i)%Force(1,1)
         rtrLoads_c(j+1) = m%KAD%y%SPyRtrLoads(i)%Force(2,1)
         rtrLoads_c(j+2) = m%KAD%y%SPyRtrLoads(i)%Force(3,1)
         rtrLoads_c(j+3) = m%KAD%y%SPyRtrLoads(i)%Moment(1,1)
         rtrLoads_c(j+4) = m%KAD%y%SPyRtrLoads(i)%Moment(2,1)
         rtrLoads_c(j+5) = m%KAD%y%SPyRtrLoads(i)%Moment(3,1)
      end do
      do i = 1, p%NumPylons *2
         j = p%NumPylons*2*6 + 6*(i-1) + 1
         rtrLoads_c(j  ) = m%KAD%y%PPyRtrLoads(i)%Force(1,1)
         rtrLoads_c(j+1) = m%KAD%y%PPyRtrLoads(i)%Force(2,1)
         rtrLoads_c(j+2) = m%KAD%y%PPyRtrLoads(i)%Force(3,1)
         rtrLoads_c(j+3) = m%KAD%y%PPyRtrLoads(i)%Moment(1,1)
         rtrLoads_c(j+4) = m%KAD%y%PPyRtrLoads(i)%Moment(2,1)
         rtrLoads_c(j+5) = m%KAD%y%PPyRtrLoads(i)%Moment(3,1)
      end do
         
   end if
   if ( p%useKFC ) then
      ! TODO: Map the controller torques back to the top and bottom nodes of the pylons
      !       and to the rtrLoads data
      
         
   end if
end subroutine TransferLoadsToMBDyn
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
   position = 0.0_ReKi
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
   if ( EqualRealNos( TwoNorm( PWnPos(:,numPWnNodes) - SWnPos(:,1) ), 0.0_ReKi ) ) then
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

!subroutine CreateMBDynPtMotionsMesh(origin, positionIn, alignDCM, nodeDCM, mesh, errStat, errMsg)
!   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
!   real(ReKi),                   intent(in   )  :: positionIn(3)     !< Coordinates of the undisplaced mesh
!   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the Kite axes
!   real(R8Ki),                   intent(in   )  :: nodeDCM(3,3)      !< DCM needed to transform into a node's axes
!   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
!   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
!   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None
!
!   ! Local variables
!   real(reKi)                                   :: position(3)       ! node reference position
!   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
!   integer(intKi)                               :: errStat2          ! temporary Error status
!   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
!   character(*), parameter                      :: RoutineName = 'CreateL2MotionsMesh'
!
!      ! Initialize variables for this routine
!
!   errStat = ErrID_None
!   errMsg  = ""
!
!   call MeshCreate ( BlankMesh = mesh     &
!                     ,IOS       = COMPONENT_INPUT &
!                     ,Nnodes    = 1               &
!                     ,errStat   = errStat2        &
!                     ,ErrMess   = errMsg2         &
!                     ,Orientation     = .true.    &
!                     ,TranslationDisp = .true.    &
!                     ,TranslationVel = .true.    &
!                     )
!         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
!
!   if (errStat >= AbortErrLev) return
!            
!      ! set node initial position/orientation
!   position = 0.0_ReKi
!        
!   position = positionIn - origin
!   position = matmul(alignDCM, position) 
!   orientation = matmul(nodeDCM, transpose(alignDCM))
!      
!   call MeshPositionNode(mesh, 1, position, errStat2, errMsg2, orientation)  
!      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
!
!         
!      ! create point element
!  
!      call MeshConstructElement( mesh, ELEMENT_POINT, errStat2, errMsg2, p1=1 )
!         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
!            
!   call MeshCommit(mesh, errStat2, errMsg2 )
!      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
!            
!   if (errStat >= AbortErrLev) return
!
!      
!   mesh%Orientation     = mesh%RefOrientation
!   mesh%TranslationDisp = 0.0_R8Ki
!   mesh%TranslationVel  = 0.0_ReKi
!   
!   
!end subroutine CreateMBDynPtMotionsMesh  

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
   type(KAD_Data),           intent(in   ) :: KAD
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
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( m%mbdSWnMotions, KAD%u(1)%SWnMotions, m%SWn_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SWn_L2_L2' )     
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( m%mbdPWnMotions, KAD%u(1)%PWnMotions, m%PWn_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PWn_L2_L2' )     
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( m%mbdVSMotions, KAD%u(1)%VSMotions, m%VS_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%VS_L2_L2' )     
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( m%mbdSHSMotions, KAD%u(1)%SHSMotions, m%SHS_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SHS_L2_L2' )     
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( m%mbdPHSMotions, KAD%u(1)%PHSMotions, m%PHS_L2_L2, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PHS_L2_L2' )     
            if (ErrStat>=AbortErrLev) return

      allocate(m%SPy_L2_L2(p%NumPylons), STAT=errStat2)
         if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPy_L2_L2', errStat, errMsg, RoutineName )     
      allocate(m%PPy_L2_L2(p%NumPylons), STAT=errStat2)
         if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPy_L2_L2', errStat, errMsg, RoutineName )     
   
      do i = 1 , p%NumPylons           
         call MeshMapCreate( m%mbdSPyMotions(i), KAD%u(1)%SPyMotions(i), m%SPy_L2_L2(i), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SPy_L2_L2('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) return
            
         call MeshMapCreate( m%mbdPPyMotions(i), KAD%u(1)%PPyMotions(i), m%PPy_L2_L2(i), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PPy_L2_L2('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) return
      end do
   
   
         ! Mappings between KiteAeroDyn loads point meshes and MBDyn loads point meshes 

      call MeshMapCreate( KAD%y%FusLoads, m%mbdFusLoads, m%Fus_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%Fus_P_P' )     
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( KAD%y%SWnLoads, m%mbdSWnLoads, m%SWn_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SWn_P_P' )     
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( KAD%y%PWnLoads, m%mbdPWnLoads, m%PWn_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PWn_P_P' )     
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( KAD%y%VSLoads, m%mbdVSLoads, m%VS_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%VS_P_P' )     
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( KAD%y%SHSLoads, m%mbdSHSLoads, m%SHS_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SHS_P_P' )     
            if (ErrStat>=AbortErrLev) return
      call MeshMapCreate( KAD%y%PHSLoads, m%mbdPHSLoads, m%PHS_P_P, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PHS_P_P' )     
            if (ErrStat>=AbortErrLev) return

      allocate(m%SPy_P_P(p%NumPylons), STAT=errStat2)
         if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPy_P_P', errStat, errMsg, RoutineName )     
      allocate(m%PPy_P_P(p%NumPylons), STAT=errStat2)
         if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPy_P_P', errStat, errMsg, RoutineName )     
   
      do i = 1 , p%NumPylons           
         call MeshMapCreate( KAD%y%SPyLoads(i), m%mbdSPyLoads(i), m%SPy_P_P(i), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%SPy_P_P('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) return
            
         call MeshMapCreate( KAD%y%PPyLoads(i), m%mbdPPyLoads(i), m%PPy_P_P(i), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%PPy_P_P('//trim(num2lstr(i))//')' ) 
               if (ErrStat>=AbortErrLev) return
      end do
   end if ! if ( p%useKAD )
   
   if ( p%useMD ) then
      ! Need to transfer the MBDyn bridle point motions to MoorDyn
   call MeshMapCreate( m%mbdWngMotions, m%MD%u(2)%PtFairleadDisplacement, m%MD_L2_2_P, errStat2, errMsg )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%MD_L2_2_P' )     
            if (ErrStat>=AbortErrLev) return
      ! Need to transfer the MoorDyn bridle point loads back the to MBDyn wing mesh for loads
   call MeshMapCreate( m%MD%y%PtFairleadLoad, m%mbdWngLoads,  m%MD_P_2_P, errStat2, errMsg )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: m%MD_P_2_P' )     
            if (ErrStat>=AbortErrLev) return
   end if 
   
end subroutine CreateMeshMappings

subroutine TransferMBDynInputs2KADMeshes( FusO, p, m, errStat, errMsg )
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
   
   m%KAD%u(1)%FusOMotions%TranslationDisp(:,1) =  FusO 
   
      ! Map the motions over to the corresponding KAD input meshes
   call Transfer_Line2_to_Line2(m%mbdFusMotions, m%KAD%u(1)%FusMotions, m%Fus_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdSWnMotions, m%KAD%u(1)%SWnMotions, m%SWn_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdPWnMotions, m%KAD%u(1)%PWnMotions, m%PWn_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdVSMotions , m%KAD%u(1)%VSMotions , m%VS_L2_L2,  errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdSHSMotions, m%KAD%u(1)%SHSMotions, m%SHS_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call Transfer_Line2_to_Line2(m%mbdPHSMotions, m%KAD%u(1)%PHSMotions, m%PHS_L2_L2, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   do i = 1, p%numPylons
      call Transfer_Line2_to_Line2(m%mbdSPyMotions(i), m%KAD%u(1)%SPyMotions(i), m%SPy_L2_L2(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )    
      call Transfer_Line2_to_Line2(m%mbdPPyMotions(i), m%KAD%u(1)%PPyMotions(i), m%PPy_L2_L2(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )                
   end do
   
      ! Rotors : Directly transfer these MBDyn motions to the cooresponding KAD input meshes
   do j = 1, p%numPylons
      do i=1,2 ! two per pylon
         c = i+(j-1)*p%numPylons
         m%KAD%u(1)%SPyRtrMotions(c)%TranslationDisp(:,1) =  m%SPyRtrO(:,i,j) - m%KAD%u(1)%SPyRtrMotions(c)%Position(:,1)
         m%KAD%u(1)%SPyRtrMotions(c)%Orientation(:,:,1)   =  m%SPyRtrDCMs(:,:,i,j)
         m%KAD%u(1)%SPyRtrMotions(c)%TranslationVel(:,1)  =  m%SPyRtrVels(:,i,j)
      end do
   end do
   
   do j = 1, p%numPylons
      do i=1,2 ! two per pylon
         c = i+(j-1)*p%numPylons
         m%KAD%u(1)%PPyRtrMotions(c)%TranslationDisp(:,1) = m%PPyRtrO(:,i,j) - m%KAD%u(1)%PPyRtrMotions(c)%Position(:,1)
         m%KAD%u(1)%PPyRtrMotions(c)%Orientation(:,:,1)   = m%PPyRtrDCMs(:,:,i,j)
         m%KAD%u(1)%PPyRtrMotions(c)%TranslationVel(:,1)  = m%PPyRtrVels(:,i,j)
      end do
   end do
   
   
      
   
   
end subroutine TransferMBDynInputs2KADMeshes


subroutine TransferMBDynInitInputs( WindPt_c, FusO, numNodePtElem_c, nodePts_c, numDCMElem_c, nodeDCMs_c, rtrPts_c, p, m, errStat, errMsg )
   real(C_DOUBLE),            intent(in   ) :: WindPt_c(3)                      ! The location of the ground station point where the freestream wind is measured [global coordinates] (m)
   real(ReKi),                intent(in   ) :: FusO(3)                          ! Location of principal kite reference point in global coordinates
   integer(C_INT),            intent(in   ) :: numNodePtElem_c                  ! total number of array elements in the nodal points array
   real(C_DOUBLE),            intent(in   ) :: nodePts_c(numNodePtElem_c)       ! 1D array containing all the nodal point coordinate data
   integer(C_INT),            intent(in   ) :: numDCMElem_c                     ! total number of array elements in the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: nodeDCMs_c(numDCMElem_c)         ! 1D array containing all the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: rtrPts_c(:)                      ! 1D array of the rotor positions in global coordinates (m)
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None


   integer(IntKi)   :: i, j, c, n, v

   errStat = ErrID_None
   errMsg  = ''
   
   
   c=1
   n=1
   
   do i = 1,p%numFusNds
      m%FusNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%FusPts(:,i)            = nodePts_c(n:n+2)
      m%FusVels(:,i)           = 0.0_ReKi
      m%FusOmegas(:,i)         = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSwnNds
      m%SWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SWnPts(:,i)            = nodePts_c(n:n+2)
      m%SWnVels(:,i)           = 0.0_ReKi
      m%SWnOmegas(:,i)         = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPwnNds
      m%PWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PWnPts(:,i)            = nodePts_c(n:n+2)
      m%PWnVels(:,i)           = 0.0_ReKi
      m%PWnOmegas(:,i)         = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numVSNds
      m%VSNdDCMs(:,:,i)        = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%VSPts(:,i)             = nodePts_c(n:n+2)
      m%VSVels(:,i)            = 0.0_ReKi
      m%VSOmegas(:,i)          = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSHSNds
      m%SHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SHSPts(:,i)            = nodePts_c(n:n+2)
      m%SHSVels(:,i)           = 0.0_ReKi
      m%SHSOmegas(:,i)         = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPHSNds
      m%PHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PHSPts(:,i)            = nodePts_c(n:n+2)
      m%PHSVels(:,i)           = 0.0_ReKi
      m%PHSOmegas(:,i)         = 0.0_ReKi
      c = c + 9
      n = n + 3
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)
         m%SPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%SPyPts(:,i,j)            = nodePts_c(n:n+2)
         m%SPyVels(:,i,j)           = 0.0_ReKi
         m%SPyOmegas(:,i,j)         = 0.0_ReKi
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
         c = c + 9
         n = n + 3
      end do
   end do
   
   if ( (c-1) /= numDCMElem_c ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of DCM elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numDCMElem_c))//'.'
   end if
   
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


subroutine TransferMBDynToKAD( FusO, numNodePtElem_c, nodePts_c, numNodeVelElem_c, nodeVels_c, numNodeOmegaElem_c, nodeOmegas_c, numDCMElem_c, nodeDCMs_c, rtrPts_c, rtrVels_c, rtrDCMs_c, p, m, errStat, errMsg )
   real(ReKi),                intent(in   ) :: FusO(3)                          ! Location of principal kite reference point in global coordinates
   integer(C_INT),            intent(in   ) :: numNodePtElem_c                  ! total number of array elements in the nodal points array
   real(C_DOUBLE),            intent(in   ) :: nodePts_c(numNodePtElem_c)       ! 1D array containing all the nodal point coordinate data
   integer(C_INT),            intent(in   ) :: numNodeVelElem_c                 ! total number of array elements in the nodal translationalvelocities array
   real(C_DOUBLE),            intent(in   ) :: nodeVels_c(numNodeVelElem_c)     ! 1D array containing all the nodal translational velocities data
   integer(C_INT),            intent(in   ) :: numNodeOmegaElem_c               ! total number of array elements in the nodal angular velocities array
   real(C_DOUBLE),            intent(in   ) :: nodeOmegas_c(numNodeOmegaElem_c) ! 1D array containing all the nodal angular velocities data
   integer(C_INT),            intent(in   ) :: numDCMElem_c                     ! total number of array elements in the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: nodeDCMs_c(numDCMElem_c)         ! 1D array containing all the nodal DCM data
   real(C_DOUBLE),            intent(in   ) :: rtrPts_c(:)                      ! 1D array of the rotor positions in global coordinates (m)
   real(C_DOUBLE),            intent(in   ) :: rtrVels_c(:)                      ! 1D array of the rotor point velocities in global coordinates (m/s)
   real(C_DOUBLE),            intent(in   ) :: rtrDCMs_c(:)                      ! 1D array of the rotor point DCMs
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None


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
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSwnNds
      m%SWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SWnPts(:,i)            = nodePts_c(n:n+2)
      m%SWnVels(:,i)           = nodeVels_c(n:n+2)
      m%SWnOmegas(:,i)         = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPwnNds
      m%PWnNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PWnPts(:,i)            = nodePts_c(n:n+2)
      m%PWnVels(:,i)           = nodeVels_c(n:n+2)
      m%PWnOmegas(:,i)         = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numVSNds
      m%VSNdDCMs(:,:,i)        = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%VSPts(:,i)             = nodePts_c(n:n+2)
      m%VSVels(:,i)            = nodeVels_c(n:n+2)
      m%VSOmegas(:,i)          = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numSHSNds
      m%SHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%SHSPts(:,i)            = nodePts_c(n:n+2)
      m%SHSVels(:,i)           = nodeVels_c(n:n+2)
      m%SHSOmegas(:,i)         = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do i = 1,p%numPHSNds
      m%PHSNdDCMs(:,:,i)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
      m%PHSPts(:,i)            = nodePts_c(n:n+2)
      m%PHSVels(:,i)           = nodeVels_c(n:n+2)
      m%PHSOmegas(:,i)         = nodeOmegas_c(n:n+2)
      c = c + 9
      n = n + 3
   end do
   do j = 1, p%numPylons
      do i = 1, p%numSPyNds(j)
         m%SPyNdDCMs(:,:,i,j)       = reshape(nodeDCMs_c(c:c+8),(/3,3/))
         m%SPyPts(:,i,j)            = nodePts_c(n:n+2)
         m%SPyVels(:,i,j)           = nodeVels_c(n:n+2)
         m%SPyOmegas(:,i,j)         = nodeOmegas_c(n:n+2)
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
         c = c + 9
         n = n + 3
      end do
   end do
   
   if ( (c-1) /= numDCMElem_c ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of DCM elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numDCMElem_c))//'.'
   end if
   
      ! Decode rotor positions
   c=1
   n=1
   do i = 1, p%numPylons
      m%SPyRtrO(:,1,i)          = rtrPts_c(c:c+2)
      m%SPyRtrDCMs(:,:,1,i)     = reshape(rtrDCMs_c(n:n+8),(/3,3/))
      m%SPyRtrVels(:,1,i)       = rtrVels_c(c:c+2)
      c = c + 3
      n = n + 9
      m%SPyRtrO(:,2,i)         = rtrPts_c(c:c+2)
      m%SPyRtrDCMs(:,:,2,i)     = reshape(rtrDCMs_c(n:n+8),(/3,3/))
      m%SPyRtrVels(:,2,i)       = rtrVels_c(c:c+2)
      c = c + 3
      n = n + 9
   end do
   do i = 1, p%numPylons
      m%PPyRtrO(:,1,i)          = rtrPts_c(c:c+2)
      m%PPyRtrDCMs(:,:,1,i)     = reshape(rtrDCMs_c(n:n+8),(/3,3/))
      m%PPyRtrVels(:,1,i)       = rtrVels_c(c:c+2)
      c = c + 3
      n = n + 9
      m%PPyRtrO(:,2,i)          = rtrPts_c(c:c+2)
      m%PPyRtrDCMs(:,:,2,i)     = reshape(rtrDCMs_c(n:n+8),(/3,3/))
      m%PPyRtrVels(:,2,i)       = rtrVels_c(c:c+2)
      c = c + 3
      n = n + 9
   end do
   
      ! Now attach this motion data to the MBDyn motion meshes and transfer motions to the corresponding KAD input meshes
   call TransferMBDynInputs2KADMeshes( FusO, p, m, errStat, errMsg )
   
end subroutine TransferMBDynToKAD
 
subroutine TransferMBDynToIfW( WindPt_c, FusO, numNodePtElem_c, nodePts_c, rtrPts_c, p, m, errStat, errMsg )
   real(C_DOUBLE),            intent(in   ) :: WindPt_c(3)                      ! The location of the ground station point where the freestream wind is measured [global coordinates] (m)
   real(ReKi),                intent(in   ) :: FusO(3)                          ! Location of principal kite reference point in global coordinates
   integer(C_INT),            intent(in   ) :: numNodePtElem_c                  ! total number of array elements in the nodal points array
   real(C_DOUBLE),            intent(in   ) :: nodePts_c(numNodePtElem_c)       ! 1D array containing all the nodal point coordinate data
   real(C_DOUBLE),            intent(in   ) :: rtrPts_c(:)                      ! 1D array of the rotor positions in global coordinates (m)
   type(KFAST_ParameterType), intent(in   ) :: p
   type(KFAST_MiscVarType),   intent(inout) :: m
   integer(IntKi),            intent(  out) :: errStat                    ! Error status of the operation
   character(*),              intent(  out) :: errMsg                     ! Error message if errStat /= ErrID_None


   integer(IntKi)   :: i, j, n, c, v

   errStat = ErrID_None
   errMsg  = ''
   
   m%IfW%u%PositionXYZ(:,1) = WindPt_c
   m%IfW%u%PositionXYZ(:,2) = FusO
   
   v=3
   
   do i = 1,m%KAD%u(1)%FusMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = m%KAD%u(1)%FusMotions%Position(:,i) + m%KAD%u(1)%FusMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,m%KAD%u(1)%SWnMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = m%KAD%u(1)%SWnMotions%Position(:,i) + m%KAD%u(1)%SWnMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,m%KAD%u(1)%PWnMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = m%KAD%u(1)%PWnMotions%Position(:,i) + m%KAD%u(1)%PWnMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,m%KAD%u(1)%VSMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = m%KAD%u(1)%VSMotions%Position(:,i) + m%KAD%u(1)%VSMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,m%KAD%u(1)%SHSMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = m%KAD%u(1)%SHSMotions%Position(:,i) + m%KAD%u(1)%SHSMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do i = 1,m%KAD%u(1)%PHSMotions%NNodes
      m%IfW%u%PositionXYZ(:,v) = m%KAD%u(1)%PHSMotions%Position(:,i) + m%KAD%u(1)%PHSMotions%TranslationDisp(:,i)
      v = v + 1
   end do
   do j = 1, p%numPylons
      do i = 1, m%KAD%u(1)%SPyMotions(j)%NNodes
         m%IfW%u%PositionXYZ(:,v)   = m%KAD%u(1)%SPyMotions(j)%Position(:,i) + m%KAD%u(1)%SPyMotions(j)%TranslationDisp(:,i)
         v = v + 1
      end do
   end do
   do j = 1, p%numPylons
      do i = 1, m%KAD%u(1)%PPyMotions(j)%NNodes
         m%IfW%u%PositionXYZ(:,v)   = m%KAD%u(1)%PPyMotions(j)%Position(:,i) + m%KAD%u(1)%PPyMotions(j)%TranslationDisp(:,i)
         v = v + 1
      end do
   end do
   
      ! Decode rotor positions
   do j = 1, p%numPylons
      do i=1,2 ! two per pylon
         c = i+(j-1)*p%numPylons
         m%IfW%u%PositionXYZ(:,v) = m%KAD%u(1)%SPyRtrMotions(c)%Position(:,1) + m%KAD%u(1)%SPyRtrMotions(c)%TranslationDisp(:,1) 
         v = v + 1
      end do
   end do
   
   do j = 1, p%numPylons
      do i=1,2 ! two per pylon
         c = i+(j-1)*p%numPylons
         m%IfW%u%PositionXYZ(:,v) = m%KAD%u(1)%PPyRtrMotions(c)%Position(:,1) + m%KAD%u(1)%PPyRtrMotions(c)%TranslationDisp(:,1) 
         v = v + 1
      end do
   end do

end subroutine TransferMBDynToIfW

subroutine KFAST_Init(dt, numFlaps, numPylons, numComp, numCompNds, modFlags, KAD_FileName_c, IfW_FileName_c, MD_FileName_c, KFC_FileName_c, &
                       outFileRoot_c, gravity, WindPt_c, FusODCM_c, numRtrPtsElem_c, rtrPts_c, numRefPtElem_c, refPts_c, &
                       numNodePtElem_c, nodePts_c, numDCMElem_c, nodeDCMs_c, errStat_c, errMsg_c ) BIND (C, NAME='KFAST_Init')
   IMPLICIT NONE


   real(C_DOUBLE),         intent(in   ) :: dt                         ! simulation time step size (s)
   integer(C_INT),         intent(in   ) :: numFlaps                   ! number of flaps per wing
   integer(C_INT),         intent(in   ) :: numPylons                  ! number of pylons per wing
   integer(C_INT),         intent(in   ) :: numComp                    ! number of individual components in the kite
   integer(C_INT),         intent(in   ) :: numCompNds(numComp)        ! number of nodes per kite component
   integer(C_INT),         intent(in   ) :: modFlags(4)                ! flags indicating which modules are being used [ Index: 1=KAD, 2=IfW, 3=MD, 4=KFC; Value: 0=off,1=on]
   character(kind=C_CHAR), intent(in   ) :: KAD_FileName_c(IntfStrLen) ! name of KiteAeroDyn input file
   character(kind=C_CHAR), intent(in   ) :: IfW_FileName_c(IntfStrLen) ! name of InflowWind input file
   character(kind=C_CHAR), intent(in   ) :: MD_FileName_c(IntfStrLen)  ! name of MoorDyn input file
   character(kind=C_CHAR), intent(in   ) :: KFC_FileName_c(IntfStrLen) ! name of KiteFAST controller Shared Library file
   character(kind=C_CHAR), intent(in   ) :: outFileRoot_c(IntfStrLen)  ! root name of any output file generated by KiteFAST
   real(C_DOUBLE),         intent(in   ) :: gravity                    ! gravitational constant (m/s^2)
   real(C_DOUBLE),         intent(in   ) :: WindPt_c(3)                ! The location of the ground station point where the freestream wind is measured [global coordinates] (m)
   real(C_DOUBLE),         intent(in   ) :: FusODCM_c(9)               ! fuselage principal reference point DCM (MIP of kite)
   integer(C_INT),         intent(in   ) :: numRtrPtsElem_c            ! total number of rotor point elements
   real(C_DOUBLE),         intent(in   ) :: rtrPts_c(numRtrPtsElem_c)  ! location of the rotor points
   integer(C_INT),         intent(in   ) :: numRefPtElem_c             ! total number of array elements in the reference points array
   real(C_DOUBLE),         intent(in   ) :: refPts_c(numRefPtElem_c)   ! 1D array containing all the reference point data
   integer(C_INT),         intent(in   ) :: numNodePtElem_c            ! total number of array elements in the nodal points array
   real(C_DOUBLE),         intent(in   ) :: nodePts_c(numNodePtElem_c) ! 1D array containing all the nodal point coordinate data
   integer(C_INT),         intent(in   ) :: numDCMElem_c               ! total number of array elements in the nodal DCM data
   real(C_DOUBLE),         intent(in   ) :: nodeDCMs_c(numDCMElem_c)   ! 1D array containing all the nodal DCM data
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

      ! Local variables
   real(ReKi)                      :: FusO(3), SWnO(3), PWnO(3), VSO(3), SHSO(3), PHSO(3), zero_vec(3)
   real(R8Ki)                      :: FusODCM(3,3)
   real(ReKi)                      :: SPyO(3,numPylons), PPyO(3,numPylons)
   type(KAD_InitInputType)         :: KAD_InitInp
   type(KAD_InitOutputType)        :: KAD_InitOut
   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   type(InflowWind_InitInputType)  :: IfW_InitInp 
   type(InflowWind_InitOutputType) :: IfW_InitOut
   type(MD_InitInputType)          :: MD_InitInp
   type(MD_InitOutputType)         :: MD_InitOut
   type(KFC_InitInputType)         :: KFC_InitInp
   type(KFC_InitOutputType)        :: KFC_InitOut
   character(*), parameter         :: routineName = 'KFAST_Init'
   integer(IntKi)                  :: i,j,c, n, maxSPyNds, maxPPyNds
   real(DbKi)                      :: interval
   
   ! Initialize all the sub-modules : MoorDyn, KiteAeroDyn, Controller, and InflowWind
   ! Set KiteFAST-level parameters, misc vars
   ! Open an Output file
   errStat = ErrID_None
   errMsg  = ''
  
         ! Initialize the NWTC Subroutine Library
   call NWTC_Init( EchoLibVer=.FALSE. )

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
      p%useMD = .true.
   else
      p%useMD = .false.
   end if
   
   if ( modFlags(4) > 0 ) then
      p%useKFC = .true.
         ! Validate some of the input data
      if ( numFlaps /=  3 )  call SetErrStat( ErrID_FATAL, "Due to the Controller interface requirements, numFlaps must be set to 3", errStat, errMsg, routineName )
      if ( numPylons /=  2 ) call SetErrStat( ErrID_FATAL, "Due to the Controller interface requirements, numPylons must be set to 2", errStat, errMsg, routineName )
      if ( .not. EqualRealNos(REAL(dt, DbKi) ,0.01_DbKi) )  call SetErrStat( ErrID_FATAL, "Due to the Controller requirements, dt must be set to 0.01", errStat, errMsg, routineName )
   else
      p%useKFC = .false.
   end if
   
   
   if (errStat >= AbortErrLev ) then
      call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
      return
   end if
      
      
      ! Set KiteFAST parameters
   p%dt = dt
   
   p%outFileRoot = transfer(outFileRoot_c(1:IntfStrLen-1),p%outFileRoot)
   call RemoveNullChar(p%outFileRoot)
   
   p%numFlaps    = numFlaps
   p%numPylons   = numPylons
   
   call SetupMBDynMotionData()
   
!----------------------------------------------------------------
! Initialize the KiteAeroDyn Module
!----------------------------------------------------------------

      ! Set KiteAeroDyn initialization input data

   if (p%useKAD) then
      
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
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if
      
      KAD_InitInp%SWnOR =  SwnO - FusO
      KAD_InitInp%SWnOR = matmul(m%FusODCM,KAD_InitInp%SWnOR)
      KAD_InitInp%PWnOR =  PwnO - FusO
      KAD_InitInp%PWnOR = matmul(m%FusODCM,KAD_InitInp%PWnOR)
      KAD_InitInp%VSOR =  VSO - FusO
      KAD_InitInp%VSOR = matmul(m%FusODCM,KAD_InitInp%VSOR)
      KAD_InitInp%SHSOR =  SHSO - FusO
      KAD_InitInp%SHSOR = matmul(m%FusODCM,KAD_InitInp%SHSOR)
      KAD_InitInp%PHSOR =  PHSO - FusO
      KAD_InitInp%PHSOR = matmul(m%FusODCM,KAD_InitInp%PHSOR)

      do i = 1, numPylons
         KAD_InitInp%SPyOR(:,i) = SPyO(:,i) - FusO
         KAD_InitInp%SPyOR(:,i) = matmul(m%FusODCM,KAD_InitInp%SPyOR(:,i))
      end do
      do i = 1, numPylons
         KAD_InitInp%PPyOR(:,i) = PPyO(:,i) - FusO
         KAD_InitInp%PPyOR(:,i) = matmul(m%FusODCM,KAD_InitInp%PPyOR(:,i))
      end do
  
      do i = 1, numPylons
         KAD_InitInp%SPyRtrOR(:,1,i) = m%SPyRtrO(:,1,i) - FusO   
         KAD_InitInp%SPyRtrOR(:,1,i) = matmul(m%FusODCM,KAD_InitInp%SPyRtrOR(:,1,i))
         KAD_InitInp%SPyRtrOR(:,2,i) = m%SPyRtrO(:,2,i) - FusO   
         KAD_InitInp%SPyRtrOR(:,2,i) = matmul(m%FusODCM,KAD_InitInp%SPyRtrOR(:,2,i))     
      end do
      do i = 1, numPylons
         KAD_InitInp%PPyRtrOR(:,1,i) = m%PPyRtrO(:,1,i) - FusO   
         KAD_InitInp%PPyRtrOR(:,1,i) = matmul(m%FusODCM,KAD_InitInp%PPyRtrOR(:,1,i))
         KAD_InitInp%PPyRtrOR(:,2,i) = m%PPyRtrO(:,2,i) - FusO   
         KAD_InitInp%PPyRtrOR(:,2,i) = matmul(m%FusODCM,KAD_InitInp%PPyRtrOR(:,2,i))     
      end do
   
      call KAD_Init(KAD_InitInp, m%KAD%u(1), m%KAD%p, m%KAD%y, interval, m%KAD%x, m%KAD%xd, m%KAD%z, m%KAD%OtherSt, m%KAD%m, KAD_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
      
      p%AirDens = KAD_InitOut%AirDens
   else
      p%AirDens = 1.225_ReKi   ! Default air density
      KAD_InitOut%nIfWPts = 0  
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
                            m%IfW%OtherSt, m%IfW%y, m%IfW%m, interval, IfW_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
   end if
   
!----------------------------------------------------------------
! Initialize the MoorDyn Module
!----------------------------------------------------------------

   if (p%useMD) then
      
      MD_InitInp%FileName  = transfer(MD_FileName_c(1:IntfStrLen-1),MD_InitInp%FileName)
      call RemoveNullChar(MD_InitInp%FileName)
   
      MD_InitInp%RootName  = TRIM(p%outFileRoot)//'.MD'
         ! The platform in this application is the location of the Kite's fuselage reference point in the inertial coordinate system
      MD_InitInp%PtfmPos   = FusO
      MD_InitInp%PtfmDCM   = m%FusODCM
      MD_InitInp%g         = gravity                    ! 
      MD_InitInp%rhoW      = p%AirDens                  ! This needs to be set according to air density at the Kite      
      MD_InitInp%WtrDepth  = 0.0_ReKi                   ! No water depth in this application
      interval             = dt
   
      call MD_Init( MD_InitInp, m%MD%u(2), m%MD%p, m%MD%x, m%MD%xd, m%MD%z, &
                     m%MD%OtherSt, m%MD%y, m%MD%m, interval, MD_InitOut, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
         
            ! Copy t+dt inputs to t for the next timestep
         call MD_CopyInput( m%MD%u(2), m%MD%u(1), MESH_NEWCOPY, errStat2, errMsg2 )
   
      if ( MD_InitOut%NAnchs > 1 ) then
         call SetErrStat(ErrID_Fatal,'KiteFAST can only support a single anchor point in the MoorDyn model',errStat,errMsg,routineName)
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
      end if
   
      p%anchorPt = MD_InitOut%Anchs(:,1)  
   else
      p%anchorPt = 0.0_ReKi
   end if
   
!----------------------------------------------------------------
! Initialize the Controller Module
!----------------------------------------------------------------
   
   
   if (p%useKFC) then
      
      KFC_InitInp%DLL_FileName = transfer(KFC_FileName_c(1:IntfStrLen-1),KFC_InitInp%DLL_FileName)
      call RemoveNullChar(KFC_InitInp%DLL_FileName)

         ! Set the DCM between FAST inertial frame and the Controller ground frame
      p%DCM_Fast2Ctrl = reshape((/-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0/),(/3,3/))
      
      KFC_InitInp%numPylons    = numPylons
      KFC_InitInp%numFlaps     = numFlaps
      interval = dt
      call KFC_Init(KFC_InitInp, m%KFC%p, KFC_InitOut, interval, errStat2, errMsg2 )
         call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
   end if
   
   OtherSt%NewTime = .true.  ! This flag is needed to tell us when we have advanced time, and hence need to call the Controller's Step routine
   
! -------------------------------------------------------------------------
! Initialize mesh-mapping data
! -------------------------------------------------------------------------

   call CreateMeshMappings(m, p, m%KAD, m%MD, errStat2, errMsg2)
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if
! -------------------------------------------------------------------------
! Open the Output file and generate header data
! -------------------------------------------------------------------------      

   call KFAST_OpenOutput( KFAST_Ver, p%outFileRoot, p, KAD_InitOut, MD_InitOut, IfW_InitOut, ErrStat2, ErrMsg2 )
      call SetErrStat(errStat2,errMsg2,errStat,errMsg,routineName)
      
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
   return

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
   
      ! Allocate rotor positions
 
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
      
      ! Convert 1D float array data into specific quantities
      
   FusO = refPts_c(1:3)  
   SWnO = refPts_c(4:6)
   PWnO = refPts_c(7:9)
   VSO = refPts_c(10:12)
   SHSO = refPts_c(13:15)
   PHSO = refPts_c(16:18)
   c = 19
   do i = 1, p%numPylons
      SPyO(:,i) = refPts_c(c:c+2)
      c = c + 3
   end do
   do i = 1, p%numPylons
      PPyO(:,i) = refPts_c(c:c+2)
      c = c + 3
   end do
 
   if ( (c-1) /= numRefPtElem_c ) then
      errStat = ErrID_FATAL
      errMsg  = 'The transferred number of reference point elements,'//trim(num2lstr(c-1))//' ,did not match the expected number, '//trim(num2lstr(numRefPtElem_c))//'.'
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
   call AllocAry( m%PHSNdDCMs, 3, 3, p%numFusNds, 'PHSNdDCMs', errStat2, errMsg2 )
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
   call AllocAry( m%PHSPts, 3, p%numFusNds, 'PHSPts', errStat2, errMsg2 )
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
   call AllocAry( m%PHSVels, 3, p%numFusNds, 'PHSVels', errStat2, errMsg2 )
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
   call AllocAry( m%PHSOmegas, 3, p%numFusNds, 'PHSOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%SPyOmegas, 3, maxSPyNds, p%numPylons, 'SPyOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call AllocAry( m%PPyOmegas, 3, maxPPyNds, p%numPylons, 'PPyOmegas', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )

   ! TODO : Check ording of c data to make sure we get the expected global to local DCM
   m%FusODCM = reshape(FusODCM_c,(/3,3/))
   
   call TransferMBDynInitInputs( WindPt_c, FusO, numNodePtElem_c, nodePts_c, numDCMElem_c, nodeDCMs_c, rtrPts_c, p, m, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
  
!----------------------------------------------------------------
! Initialize the Motions meshes which correspond to the motions 
!   coming from MBDyn.  The points coming from MBDyn are assumed
!   to be in the global, inertial coordinate system and the DCMs
!   transform from global to local.
!----------------------------------------------------------------

      ! Fuselage
   call CreateMBDynL2MotionsMesh(FusO, p%numFusNds, m%FusPts, m%FusODCM, m%FusNdDCMs, m%mbdFusMotions, errStat, errMsg)
      ! Starboard Wing
   call CreateMBDynL2MotionsMesh(FusO, p%numSWnNds, m%SWnPts, m%FusODCM, m%SWnNdDCMs, m%mbdSWnMotions, errStat, errMsg)
      ! Port Wing
   call CreateMBDynL2MotionsMesh(FusO, p%numPWnNds, m%PWnPts, m%FusODCM, m%PWnNdDCMs, m%mbdPWnMotions, errStat, errMsg)
      ! Total wing mesh which is needed for MoorDyn mapping
   call CreateMBDynL2MotionsWingMesh(FusO, p%numSWnNds, m%SWnPts, p%numPWnNds, m%PWnPts, m%FusODCM, m%SWnNdDCMs, m%PWnNdDCMs, m%mbdWngMotions, errStat, errMsg)
      ! Vertical Stabilizer
   call CreateMBDynL2MotionsMesh(FusO, p%numVSNds, m%VSPts, m%FusODCM, m%VSNdDCMs, m%mbdVSMotions, errStat, errMsg)
      ! Starboard Horizontal Stabilizer
   call CreateMBDynL2MotionsMesh(FusO, p%numSHSNds, m%SHSPts, m%FusODCM, m%SHSNdDCMs, m%mbdSHSMotions, errStat, errMsg)
      ! Port Horizontal Stabilizer
   call CreateMBDynL2MotionsMesh(FusO, p%numPHSNds, m%PHSPts, m%FusODCM, m%PHSNdDCMs, m%mbdPHSMotions, errStat, errMsg)

   allocate(m%mbdSPyMotions(p%numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdSPyMotions', errStat, errMsg, RoutineName )     
   allocate(m%mbdPPyMotions(p%numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdPPyMotions', errStat, errMsg, RoutineName )  

      ! Starboard Pylons
   do i = 1, p%numPylons
      call CreateMBDynL2MotionsMesh(FusO, p%numSPyNds(i), m%SPyPts(:,:,i), m%FusODCM, m%SPyNdDCMs(:,:,:,i), m%mbdSPyMotions(i), errStat, errMsg)
   end do
      ! Port Pylons
   do i = 1, p%numPylons
      call CreateMBDynL2MotionsMesh(FusO, p%numPPyNds(i), m%PPyPts(:,:,i), m%FusODCM, m%PPyNdDCMs(:,:,:,i), m%mbdPPyMotions(i), errStat, errMsg)
   end do   
 
   ! NOTE: We don't need a set of MBDyn rotor meshes because the data is transferred directly to KiteAeroDyn and then back to MBDyn 

      ! Fuselage
   call CreateMBDynPtLoadsMesh(FusO, p%numFusNds, m%FusPts, m%FusODCM, m%FusNdDCMs, m%mbdFusLoads, errStat, errMsg)
      ! Starboard Wing
   call CreateMBDynPtLoadsMesh(FusO, p%numSWnNds, m%SWnPts, m%FusODCM, m%SWnNdDCMs, m%mbdSWnLoads, errStat, errMsg)
      ! Port Wing
   call CreateMBDynPtLoadsMesh(FusO, p%numPWnNds, m%PWnPts, m%FusODCM, m%PWnNdDCMs, m%mbdPWnLoads, errStat, errMsg)
      ! Total Wing
   call CreateMBDynPtLoadsWingMesh(FusO, p%numSWnNds, m%SWnPts, p%numPWnNds, m%PWnPts, m%FusODCM, m%SWnNdDCMs, m%PWnNdDCMs, m%mbdWngLoads, errStat, errMsg)
      ! Vertical Stabilizer
   call CreateMBDynPtLoadsMesh(FusO, p%numVSNds, m%VSPts, m%FusODCM, m%VSNdDCMs, m%mbdVSLoads, errStat, errMsg)
      ! Starboard Horizontal Stabilizer
   call CreateMBDynPtLoadsMesh(FusO, p%numSHSNds, m%SHSPts, m%FusODCM, m%SHSNdDCMs, m%mbdSHSLoads, errStat, errMsg)
      ! Port Horizontal Stabilizer
   call CreateMBDynPtLoadsMesh(FusO, p%numPHSNds, m%PHSPts, m%FusODCM, m%PHSNdDCMs, m%mbdPHSLoads, errStat, errMsg)
      ! Starboard Pylons
   allocate(m%mbdSPyLoads(p%numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdSPyLoads', errStat, errMsg, RoutineName )     
   allocate(m%mbdPPyLoads(p%numPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%mbdPPyLoads', errStat, errMsg, RoutineName )  
   do i = 1, p%numPylons
      call CreateMBDynPtLoadsMesh(FusO, p%numSPyNds(i), m%SPyPts(:,:,i), m%FusODCM, m%SPyNdDCMs(:,:,:,i), m%mbdSPyLoads(i), errStat, errMsg)
   end do
      ! Port Pylons
   do i = 1, p%numPylons
      call CreateMBDynPtLoadsMesh(FusO, p%numPPyNds(i), m%PPyPts(:,:,i), m%FusODCM, m%PPyNdDCMs(:,:,:,i), m%mbdPPyLoads(i), errStat, errMsg)
   end do  
 
   ! NOTE: We don't need a set of MBDyn rotor meshes because the data is transferred directly to KiteAeroDyn and then back to MBDyn 
   
   end subroutine SetupMBDynMotionData
                       
end subroutine KFAST_Init

subroutine KFAST_AssRes(t_c, isInitialTime_c, numRtSpdRtrElem_c, RtSpd_PyRtr_c, WindPt_c, FusO_prev_c, FusO_c, FusODCM_prev_c, FusOv_prev_c, FusOomegas_prev_c, FusOacc_prev_c, numNodePtElem_c, nodePts_c, &
                          numNodeVelElem_c, nodeVels_c, numNodeOmegaElem_c, nodeOmegas_c, numDCMElem_c, nodeDCMs_c, numRtrPtsElem_c, rtrPts_c, &
                          rtrVels_c, rtrDCMs_c, numNodeLoadsElem_c, nodeLoads_c, numRtrLoadsElem_c, rtrLoads_c, errStat_c, errMsg_c ) BIND (C, NAME='KFAST_AssRes')
   IMPLICIT NONE

   real(C_DOUBLE),         intent(in   ) :: t_c                               ! simulation time  (s)
   integer(C_INT),         intent(in   ) :: isInitialTime_c                   ! Is this the initial time of the simulation 1=Yes, should we update the states? 0=yes, 1=no
   integer(C_INT),         intent(in   ) :: numRtSpdRtrElem_c                 ! total number of array elements in the rotor rotor speeds array
   real(C_DOUBLE),         intent(in   ) :: RtSpd_PyRtr_c(numRtSpdRtrElem_c)  ! 1D array containing all the rotor speeds for the kite 
   real(C_DOUBLE),         intent(in   ) :: WindPt_c(3)                       ! The location of the ground station point where the freestream wind is measured [global coordinates] (m)
   real(C_DOUBLE),         intent(in   ) :: FusO_prev_c(3)                    ! The location of the principal Kite reference point [global coordinates] at time t (m)
   real(C_DOUBLE),         intent(in   ) :: FusO_c(3)                         ! The location of the principal Kite reference point [global coordinates] at time t+dt (m)
   real(C_DOUBLE),         intent(in   ) :: FusODCM_prev_c(9)                 ! Principal reference point DCM (MIP of kite) at time t
   real(C_DOUBLE),         intent(in   ) :: FusOv_prev_c(3)                   ! Translational velocities at the principal reference point [global coordinates] at time t (m/s)
   real(C_DOUBLE),         intent(in   ) :: FusOomegas_prev_c(3)              ! Angular velocities at the principal reference point [global coordinates] at time t (rad/s)
   real(C_DOUBLE),         intent(in   ) :: FusOacc_prev_c(3)                 ! Accelerations at the principal reference point  [global coordinates] at time t (m/s^2)
   integer(C_INT),         intent(in   ) :: numNodePtElem_c                   ! total number of array elements in the nodal points array
   real(C_DOUBLE),         intent(in   ) :: nodePts_c(numNodePtElem_c)        ! 1D array containing all the nodal point coordinate data
   integer(C_INT),         intent(in   ) :: numNodeVelElem_c                  ! total number of array elements in the nodal translational velocities array
   real(C_DOUBLE),         intent(in   ) :: nodeVels_c(numNodeVelElem_c)      ! 1D array containing all the nodal translational velocities data
   integer(C_INT),         intent(in   ) :: numNodeOmegaElem_c                ! total number of array elements in the nodal angular velocities array
   real(C_DOUBLE),         intent(in   ) :: nodeOmegas_c(numNodeOmegaElem_c) ! 1D array containing all the nodal angular velocities data
   integer(C_INT),         intent(in   ) :: numDCMElem_c                      ! total number of array elements in the nodal DCM data
   real(C_DOUBLE),         intent(in   ) :: nodeDCMs_c(numDCMElem_c)          ! 1D array containing all the nodal DCM data
   integer(C_INT),         intent(in   ) :: numRtrPtsElem_c                   ! total number of rotor point elements
   real(C_DOUBLE),         intent(in   ) :: rtrPts_c(numRtrPtsElem_c)         ! location of the rotor points
   real(C_DOUBLE),         intent(in   ) :: rtrVels_c(numRtrPtsElem_c)        ! 1D array of the rotor point velocities in global coordinates (m/s)
   real(C_DOUBLE),         intent(in   ) :: rtrDCMs_c(numRtrPtsElem_c*3)      ! 1D array of the rotor point DCMs
   integer(C_INT),         intent(in   ) :: numNodeLoadsElem_c                ! total number of nodel point load elements
   real(C_DOUBLE),         intent(inout) :: nodeLoads_c(numNodeLoadsElem_c)   ! 1D array of the nodal point loads
   integer(C_INT),         intent(in   ) :: numRtrLoadsElem_c                 ! total number of rotor point load elements
   real(C_DOUBLE),         intent(inout) :: rtrLoads_c(numRtrLoadsElem_c)     ! 1D array of the rotor point loads

   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)           :: n, c, i, j                     ! counters
   integer(IntKi)           :: isInitialTime                  ! Is this the initial time of the simulation 1=Yes, should we update the states? 0=yes, 1=no
   real(DbKi)               :: t                              ! simulations time (s)
   real(DbKi)               :: utimes(2)                      ! t and t+dt timestep values (s)
   real(ReKi)               :: FusO(3)                        ! fuselage (or kite) reference point location in global (inertial) coordinates at time t+dt (m)
   real(ReKi)               :: FusO_prev(3)                   ! fuselage (or kite) reference point location in global (inertial) coordinates at time t (m)
   real(ReKi)               :: FusOv_prev(3)                  ! fuselage reference point translational velocities in global (inertial) coordinates at time t (m)
   real(ReKi)               :: FusODCM_prev(3,3)              ! fuselage reference point DCM to transform from global to kite coordinates at time t
   real(ReKi)               :: FusOomegas_prev(3)             ! fuselage reference point rotational velocities in global coordinates at time t (m/s)
   real(ReKi)               :: FusOacc_prev(3)                ! fuselage reference point accelerations in global coordinates at time t (m/s^2)
   integer(IntKi)           :: errStat, errStat2              ! error status values
   character(ErrMsgLen)     :: errMsg, errMsg2                ! error messages
   character(*), parameter  :: routineName = 'KFAST_AssRes'
   
   errStat = ErrID_None
   errMsg  = ''
   
   isInitialTime = isInitialTime_c
   t = t_c
   
   if (isInitialTime > 0 ) then
      utimes(1) = t
   else
      utimes(1) = t - p%DT
   end if
      
   utimes(2) = t
   n = utimes(1) / p%DT
   
      ! Transfer fuselage (kite) principal point inputs
   FusO            = FusO_c
   FusO_prev       = FusO_prev_c
   FusOv_prev      = FusOv_prev_c
   FusOomegas_prev = FusOomegas_prev_c
   FusOacc_prev    = FusOacc_prev_c
   
      ! Transfer C-based nodal and rotor quantities into Fortran-based data structures (but not meshes, yet)
      !   The resulting data resides inside the MiscVars data structure (m)
   
   FusODCM_prev = reshape(FusODCM_prev_c,(/3,3/))  
   
   if ( p%useKAD ) then
      call TransferMBDynToKAD( FusO, numNodePtElem_c, nodePts_c, numNodeVelElem_c, nodeVels_c, numNodeOmegaElem_c, nodeOmegas_c, numDCMElem_c, nodeDCMs_c, rtrPts_c, rtrVels_c, rtrDCMs_c, p, m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
   end if
   

! -------------------------------------------------------------------------
! InflowWind
! -------------------------------------------------------------------------      
      ! The inputs to InflowWind are the positions where wind velocities [the outputs] are to be computed.  These inputs are set above by
      !  the TransferMBDynInputs() call.
   if ( p%useIfW ) then
      call TransferMBDynToIfW( WindPt_c, FusO, numNodePtElem_c, nodePts_c, rtrPts_c, p, m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
      call InflowWind_CalcOutput(t, m%IfW%u, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z, m%IfW%OtherSt, m%IfW%y, m%IfW%m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
   end if
   
! -------------------------------------------------------------------------
! Controller
! -------------------------------------------------------------------------      
   
   if (OtherSt%NewTime) then
      
      if ( p%useKFC ) then         
! TODO: Need to work out how we generate a controller output signal for the very first timestep (t=0.0)
         
            ! NOTE: The controller is stepping from t (GetXPrev in MBDyn) to t+dt (GetXCur in MBDyn)
            !       therefore, all inputs to KFC needs to be at time, t.
         m%KFC%u%dcm_g2b       = matmul(FusODCM_prev, transpose(p%DCM_Fast2Ctrl))
         m%KFC%u%pqr           = matmul(FusODCM_prev, FusOomegas_prev)
         m%KFC%u%acc_norm      = TwoNorm(FusOacc_prev)
         m%KFC%u%Xg            = FusO_prev - p%anchorPt
         m%KFC%u%Xg            = matmul(p%DCM_Fast2Ctrl, m%KFC%u%Xg)
         m%KFC%u%Vg            = matmul(p%DCM_Fast2Ctrl, FusOv_prev)
         m%KFC%u%Vb            = matmul(FusODCM_prev, FusOv_prev)
         m%KFC%u%Ag            = matmul(p%DCM_Fast2Ctrl, FusOacc_prev)
         m%KFC%u%Ab            = matmul(FusODCM_prev, FusOacc_prev)
         m%KFC%u%rho           = p%AirDens
         if ( isInitialTime > 0 ) then
            m%KFC%u%apparent_wind = m%IfW%y%VelocityUVW(:,2) - FusOv_prev
         else
            m%KFC%u%apparent_wind = m%IfW_FusO_prev - FusOv_prev
         end if
         m%KFC%u%apparent_wind = matmul(p%DCM_Fast2Ctrl, m%KFC%u%apparent_wind)
        ! m%KFC%u%tether_force  = 0 
         m%KFC%u%wind_g        = matmul(p%DCM_Fast2Ctrl, m%IfW_ground_prev)
      
         call KFC_Step(utimes(1), m%KFC%u, m%KFC%p, m%KFC%y, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
      
      end if
      
      OtherSt%NewTime = .false. ! only call the controller once per timestep
      
   end if
  
! -------------------------------------------------------------------------
! MoorDyn
! ------------------------------------------------------------------------- 
   
   if ( p%useMD ) then
     
         ! Update the MBDyn motions mesh associated with the total wing
         ! Nodes for this wing mesh are specified left to right (port tip across to starboard tip)
      
      ! TODO: Handle possible colocated nodes at fuselage
      c = 1   
      do i = p%numPwnNds, 1, -1
         m%mbdWngMotions%Orientation  (:,:,c) = m%PWnNdDCMs(:,:,i)
         m%mbdWngMotions%TranslationDisp(:,c) = m%PWnPts(:,i) - m%mbdPWnMotions%Position(:,c)
         m%mbdWngLoads%TranslationDisp(:,c)   = m%mbdWngMotions%TranslationDisp(:,c)
         m%mbdWngMotions%TranslationVel (:,c) = m%PWnVels(:,i)
         m%mbdWngMotions%RotationVel    (:,c) = m%PWnOmegas(:,i)
         c = c + 1
      end do
      do i = 1,p%numSwnNds
         m%mbdWngMotions%Orientation  (:,:,c) = m%SWnNdDCMs(:,:,i)
         m%mbdWngMotions%TranslationDisp(:,c) = m%SWnPts(:,i) - m%mbdSWnMotions%Position(:,c)
         m%mbdWngLoads%TranslationDisp(:,c)   = m%mbdWngMotions%TranslationDisp(:,c)
         m%mbdWngMotions%TranslationVel (:,c) = m%SWnVels(:,i)
         m%mbdWngMotions%RotationVel    (:,c) = m%SWnOmegas(:,i)
         c = c + 1
      end do
   
         ! Pos and Vel of each Fairlead (Bridle connection) at t.  
         !   We need to set the TranslationDisp, Orientation, TranslationVel, RotationVel properties on the mesh
         ! To do this we need to map the motions from a wing mesh to the specific fairlead mesh ( one node per fairlead connection point)
         ! This means we need the t and t+dt motions of this wing mesh
      call Transfer_Line2_to_Point( m%mbdWngMotions, m%MD%u(2)%PtFairleadDisplacement, m%MD_L2_2_P, errStat2, errMsg )
 
      if ( isInitialTime < 1 ) then
         call MD_CopyContState( m%MD%x, m%MD%x_copy, MESH_NEWCOPY, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         call MD_UpdateStates( utimes(1), n, m%MD%u, utimes, m%MD%p, m%MD%x_copy, m%MD%xd, m%MD%z, m%MD%OtherSt, m%MD%m, errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      end if
      
      call MD_CalcOutput( t, m%MD%u(2), m%MD%p, m%MD%x_copy, m%MD%xd, m%MD%z, m%MD%OtherSt, m%MD%y, m%MD%m, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      if (errStat >= AbortErrLev ) then
         call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
         return
      end if
   
   end if
   
      
! -------------------------------------------------------------------------
! KiteAeroDyn
! -------------------------------------------------------------------------      
   
   if ( p%useKAD ) then
         ! Outputs from Controller for KAD
      if ( p%useKFC ) then
         m%KAD%u(1)%Ctrl_SFlp = m%KFC%y%SFlp
         m%KAD%u(1)%Ctrl_PFlp = m%KFC%y%PFlp
         m%KAD%u(1)%Ctrl_Rudr = m%KFC%y%Rudr
         m%KAD%u(1)%Ctrl_SElv = m%KFC%y%SElv
         m%KAD%u(1)%Ctrl_PElv = m%KFC%y%PElv
      else
         m%KAD%u(1)%Ctrl_SFlp = 0.0_ReKi
         m%KAD%u(1)%Ctrl_PFlp = 0.0_ReKi
         m%KAD%u(1)%Ctrl_Rudr = 0.0_ReKi
         m%KAD%u(1)%Ctrl_SElv = 0.0_ReKi
         m%KAD%u(1)%Ctrl_PElv = 0.0_ReKi
      end if
      
      m%KAD%u(1)%Pitch_SPyRtr = 0.0_ReKi   ! Controller does not set these, yet.
      m%KAD%u(1)%Pitch_PPyRtr = 0.0_ReKi
   
         ! Rotor Speeds from MBDyn  [2 per pylon]
      c = 1
      do i = 1, p%numPylons ! [moving from inboard to outboard]
         m%KAD%u(1)%RtSpd_SPyRtr(1,i) = RtSpd_PyRtr_c(c)    ! top
         m%KAD%u(1)%RtSpd_SPyRtr(2,i) = RtSpd_PyRtr_c(c+1)  ! bottom
         c = c+2
      end do
      do i = 1, p%numPylons   ! [moving from inboard to outboard]
         m%KAD%u(1)%RtSpd_PPyRtr(1,i) = RtSpd_PyRtr_c(c)    ! top
         m%KAD%u(1)%RtSpd_PPyRtr(2,i) = RtSpd_PyRtr_c(c+1)  ! bottom
         c = c+2
      end do
   
   
      if ( p%useIfW ) then
         
            ! Need previous timestep's inflow at FusO for the next call to KFC calcoutput
         m%IfW_FusO_prev = m%IfW%y%VelocityUVW(:,2)
         
            ! Transfer Inflow Wind outputs to the various KAD inflow inputs
         c=3
         do i = 1,size(m%KAD%u(1)%V_Fus,2)
            m%KAD%u(1)%V_Fus(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u(1)%V_SWn,2)
            m%KAD%u(1)%V_SWn(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u(1)%V_PWn,2)
            m%KAD%u(1)%V_PWn(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u(1)%V_VS,2)
            m%KAD%u(1)%V_VS(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u(1)%V_SHS,2)
            m%KAD%u(1)%V_SHS(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,size(m%KAD%u(1)%V_PHS,2)
            m%KAD%u(1)%V_PHS(:,i)   = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do j = 1,p%numPylons
            do i = 1,size(m%KAD%u(1)%V_SPy,2)
               m%KAD%u(1)%V_SPy(:,i,j)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
         end do
         do j = 1,p%numPylons
            do i = 1,size(m%KAD%u(1)%V_PPy,2)
               m%KAD%u(1)%V_PPy(:,i,j)   = m%IfW%y%VelocityUVW(:,c)
               c = c+1
            end do
         end do   
         do i = 1,p%numPylons
            m%KAD%u(1)%V_SPyRtr(:,1,i) = m%IfW%y%VelocityUVW(:,c)
            c = c+1
            m%KAD%u(1)%V_SPyRtr(:,2,i) = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
         do i = 1,p%numPylons
            m%KAD%u(1)%V_PPyRtr(:,1,i) = m%IfW%y%VelocityUVW(:,c)
            c = c+1
            m%KAD%u(1)%V_PPyRtr(:,2,i) = m%IfW%y%VelocityUVW(:,c)
            c = c+1
         end do
      else
         m%KAD%u(1)%V_Fus     = 0.0_ReKi
         m%KAD%u(1)%V_SWn     = 0.0_ReKi
         m%KAD%u(1)%V_PWn     = 0.0_ReKi
         m%KAD%u(1)%V_VS      = 0.0_ReKi
         m%KAD%u(1)%V_SHS     = 0.0_ReKi
         m%KAD%u(1)%V_PHS     = 0.0_ReKi
         m%KAD%u(1)%V_SPy     = 0.0_ReKi
         m%KAD%u(1)%V_PPy     = 0.0_ReKi         
         m%KAD%u(1)%V_SPyRtr  = 0.0_ReKi    
         m%KAD%u(1)%V_PPyRtr  = 0.0_ReKi  
      end if
 
      

! TODO: Should we clean up/destroy the old version of m%KAD%z_copy ?
      call KAD_CopyConstrState( m%KAD%z, m%KAD%z_copy, MESH_NEWCOPY, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
         
      if ( isInitialTime < 1 ) then
            ! Copy the inputs
         call KAD_CopyInput(m%KAD%u(1),m%KAD%u(2), MESH_NEWCOPY, errStat, errMsg)
         call KAD_UpdateStates( utimes(1), n, m%KAD%u, utimes, m%KAD%p, m%KAD%x, m%KAD%xd, m%KAD%z_copy, m%KAD%OtherSt, m%KAD%m, errStat, errMsg )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
            if (errStat >= AbortErrLev ) then
               call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
               return
            end if
      end if
      
      call KAD_CalcOutput( t, m%KAD%u(1), m%KAD%p, m%KAD%x, m%KAD%xd, m%KAD%z_copy, m%KAD%OtherSt, m%KAD%y, m%KAD%m, errStat, errMsg )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
         if (errStat >= AbortErrLev ) then
            call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
            return
         end if
   end if
   
   call TransferLoadsToMBDyn(p, m, nodeLoads_c, rtrLoads_c, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
   call TransferErrors(errStat, errMsg, errStat_c, errMsg_c)
   return
   

   
   
end subroutine KFAST_AssRes

subroutine KFAST_AfterPredict(errStat_c, errMsg_c) BIND (C, NAME='KFAST_AfterPredict')
   IMPLICIT NONE

   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_AfterPredict'
   
   errStat = ErrID_None
   errMsg  = ''

   OtherSt%NewTime = .true.
   
      ! Copy the temporary states and place them into the actual versions
   call MD_CopyContState   ( m%MD%x_copy, m%MD%x, MESH_NEWCOPY, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KAD_CopyConstrState( m%KAD%z_copy, m%KAD%z, MESH_NEWCOPY, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
      ! Copy t+dt inputs to t for the next timestep
   call MD_CopyInput( m%MD%u(2), m%MD%u(1), MESH_NEWCOPY, errStat2, errMsg2 )
   
              ! transfer Fortran variables to C:  
   errStat_c = errStat
   errMsg    = trim(errMsg)//C_NULL_CHAR
   errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )
   
end subroutine KFAST_AfterPredict

subroutine KFAST_Output(t_c, errStat_c, errMsg_c) BIND (C, NAME='KFAST_Output')
   IMPLICIT NONE
   real(C_DOUBLE),         intent(in   ) :: t_c
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_Output'
   real(DbKi)                      :: t
   errStat = ErrID_None
   errMsg  = ''
   t = real(t_c,DbKi)

   ! Write any outputs to file
   ! call KFAST_WriteOutput()
   call KFAST_WriteOutput( t, p, m%KAD%y, m%MD%y, m%IfW%y, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
                 ! transfer Fortran variables to C:  
   errStat_c = errStat
   errMsg    = trim(errMsg)//C_NULL_CHAR
   errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )

end subroutine KFAST_Output

   
subroutine KFAST_End(errStat_c, errMsg_c) BIND (C, NAME='KFAST_End')
   IMPLICIT NONE
   
   integer(C_INT),         intent(  out) :: errStat_c      
   character(kind=C_CHAR), intent(  out) :: errMsg_c(IntfStrLen)   

   integer(IntKi)                  :: errStat, errStat2
   character(ErrMsgLen)            :: errMsg, errMsg2
   character(*), parameter         :: routineName = 'KFAST_End'

   errStat = ErrID_None
   errMsg  = ''

   
   ! Call the End subroutines for KiteAeroDyn, MoorDyn, InflowWind, and the Controller
   if ( p%useKAD ) then
      call KAD_DestroyInput(m%KAD%u(2), errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call KAD_End(m%KAD%u(1), m%KAD%p, m%KAD%x, m%KAD%xd, m%KAD%z, m%KAD%OtherSt, m%KAD%y, m%KAD%m, errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   
   if ( p%useMD ) then
      call MD_DestroyInput(m%MD%u(2), errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      call MD_End(m%MD%u(1), m%MD%p, m%MD%x, m%MD%xd, m%MD%z, m%MD%OtherSt, m%MD%y, m%MD%m, errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   
   if ( p%useIfW ) then

      call InflowWind_End(m%IfW%u, m%IfW%p, m%IfW%x, m%IfW%xd, m%IfW%z, m%IfW%OtherSt, m%IfW%y, m%IfW%m, errStat2 , errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   
   if ( p%useKFC ) then
      call KFC_End(m%KFC%p, errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end if
   
   
   
      
! -------------------------------------------------------------------------
! Close the Output file
! -------------------------------------------------------------------------   
      close(p%UnOutFile)
      
      ! transfer Fortran variables to C:  
   errStat_c = errStat
   errMsg    = trim(errMsg)//C_NULL_CHAR
   errMsg_c  = transfer( errMsg//C_NULL_CHAR, errMsg_c )
  
end subroutine KFAST_End
   
end module KiteFAST
   