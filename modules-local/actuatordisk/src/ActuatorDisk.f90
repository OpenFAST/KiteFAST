!**********************************************************************************************************************************
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
!> This module solves quasi-steady actuator disk to calculate the 3 forces, 3 moments, and power dependent on omega, blade pitch, inflow, and inflow skew.
module ActuatorDisk

   
   use NWTC_Library   
   use ActuatorDisk_Types
   
   implicit none 

private

   type(ProgDesc), parameter  :: ActDsk_Ver = ProgDesc( 'ActuatorDisk', '', '' )

   public :: ActDsk_Init
   public :: ActDsk_CalcOutput


   contains
   
function CoefInterp(omega, pitch, Vinf, skew, p, errStat, errMsg)
   real(ReKi),                 intent(in   )  :: omega
   real(ReKi),                 intent(in   )  :: pitch
   real(ReKi),                 intent(in   )  :: Vinf
   real(ReKi),                 intent(in   )  :: skew
   type(ActDsk_ParameterType), intent(in   )  :: p
   integer(IntKi),             intent(  out)  :: errStat     !< Error status of the operation
   character(*),               intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None
   real(ReKi)   :: CoefInterp(7)
   
   integer(IntKi)             :: iOmega_Lo, iOmega_Hi, iSkew_Lo, iSkew_Hi, iPitch_Lo, iPitch_Hi, iVinf_Lo, iVinf_Hi   ! grid indices
   real(ReKi)                 :: sOmega, sSkew, sPitch, sVinf   ! interpolant for Omega and Skew, Pitch, and Vinf, scaled between -1 and 1
   real(ReKi)                 :: m(16), v(16)
   integer(IntKi)             :: i             ! loop counter
   character(*), parameter    :: routineName = 'CoefInterp'
   
         ! Initialize variables for this routine
   errStat = ErrID_None
   errMsg  = ""
   CoefInterp = 0.0_ReKi
   
   ! Make sure the input Omega, Skew, Vinf, and Pitch angle are within our data tables.  Do not allow extrapolation.
   
   if ( ( Omega < p%Omegas(1) ) .or. ( Omega > p%Omegas(p%numOmega) ) )  then
      call SetErrStat(ErrID_Fatal, 'The input Omega of '//trim(num2lstr(Omega))//' (rad/s) is outside the values found in the Omega tables for this rotor',errStat, errMsg, routineName)
      return
   end if
   
   if ( ( pitch < p%Pitches(1) ) .or. ( pitch > p%Pitches(p%numPitch) ) ) then
      call SetErrStat(ErrID_Fatal, 'The input pitch angle of '//trim(num2lstr(pitch*R2D))//' degrees is outside the values found in the pitch tables for this rotor',errStat, errMsg, routineName)
      return
   end if

   if ( ( skew < p%Skews(1) ) .or. ( skew > p%Skews(p%numSkew) ) ) then
      call SetErrStat(ErrID_Fatal, 'The input skew angle of '//trim(num2lstr(skew*R2D))//' degrees is outside the values found in the skew tables for this rotor',errStat, errMsg, routineName)
      return
   end if
   
   if ( ( Vinf < p%Vinfs(1) ) .or. ( Vinf > p%Vinfs(p%numVinf) ) ) then
      call SetErrStat(ErrID_Fatal, 'The input Vinf of '//trim(num2lstr(Vinf))//' (m/s) is outside the values found in the Vinf tables for this rotor',errStat, errMsg, routineName)
      return
   end if
   
   
   
   
   iOmega_Lo  = 1
   iSkew_Lo = 1
   iPitch_Lo = 1
   iVinf_Lo = 1
   
     ! Let's interpolate!

   iOmega_Lo = MAX( MIN( iOmega_Lo, p%numOmega-1 ), 1 )

   do

      if ( Omega < p%Omegas(iOmega_Lo) )  then

         iOmega_Lo = iOmega_Lo - 1

      else if ( Omega >= p%Omegas(iOmega_Lo+1) )  then

         iOmega_Lo = iOmega_Lo + 1

      else

         iOmega_Hi = iOmega_Lo + 1
            ! scale sOmega between -1  and 1 where -1 = iOmega_Lo and 1 = iOmega_Hi
         sOmega = 2.0_ReKi*((Omega - p%Omegas(iOmega_Lo))/(p%Omegas(iOmega_Hi) - p%Omegas(iOmega_Lo))) -1
         exit

      end if

   end do

   
   iSkew_Lo = MAX( MIN( iSkew_Lo, p%numSkew-1 ), 1 )

   do

      if ( skew < p%Skews(iSkew_Lo) )  then

         iSkew_Lo = iSkew_Lo - 1

      else if ( skew > p%Skews(iSkew_Lo+1) )  then

         iSkew_Lo = iSkew_Lo + 1

      else

         iSkew_Hi = iSkew_Lo + 1
            ! scale sSkew between -1  and 1 where -1 = iSkew_Lo and 1 = iSkew_Hi
         sSkew = 2.0_ReKi*((skew - p%Skews(iSkew_Lo))/(p%Skews(iSkew_Hi) - p%Skews(iSkew_Lo))) -1
         exit

      end if

   end do
   
   iPitch_Lo = MAX( MIN( iPitch_Lo, p%numPitch-1 ), 1 )

   do

      if ( pitch < p%Pitches(iPitch_Lo) )  then

         iPitch_Lo = iPitch_Lo - 1

      else if ( pitch >= p%Pitches(iPitch_Lo+1) )  then

         iPitch_Lo = iPitch_Lo + 1

      else

         iPitch_Hi = iPitch_Lo + 1
            ! scale sPitch between -1  and 1 where -1 = iPitch_Lo and 1 = iPitch_Hi
         sPitch = 2.0_ReKi*((pitch - p%Pitches(iPitch_Lo))/(p%Pitches(iPitch_Hi) - p%Pitches(iPitch_Lo))) -1
         exit

      end if

   end do
   
   iVinf_Lo = MAX( MIN( iVinf_Lo, p%numVinf-1 ), 1 )

   do

      if ( Vinf < p%Vinfs(iVinf_Lo) )  then

         iVinf_Lo = iVinf_Lo - 1

      else if ( Vinf >= p%Vinfs(iVinf_Lo+1) )  then

         iVinf_Lo = iVinf_Lo + 1

      else

         iVinf_Hi = iVinf_Lo + 1
            ! scale sVinf between -1  and 1 where -1 = iVinf_Lo and 1 = iVinf_Hi
         sVinf = 2.0_ReKi*((Vinf - p%Vinfs(iVinf_Lo))/(p%Vinfs(iVinf_Hi) - p%Vinfs(iVinf_Lo))) -1
         exit

      end if

   end do
   
   !----------------------------------------------------------------------------------------------
   ! Interpolate the Omega/Pitch/Vinf/Skew table using an volume interpolation.
   !----------------------------------------------------------------------------------------------
   
      ! Setup the scaling factors.  Set the unused portion of the array to zero
   m(1)  = ( 1.0_ReKi + sOmega )*( 1.0_ReKi - sVinf )*( 1.0_ReKi - sSkew )*( 1.0_ReKi - sPitch )
   m(2)  = ( 1.0_ReKi + sOmega )*( 1.0_ReKi + sVinf )*( 1.0_ReKi - sSkew )*( 1.0_ReKi - sPitch )
   m(3)  = ( 1.0_ReKi - sOmega )*( 1.0_ReKi + sVinf )*( 1.0_ReKi - sSkew )*( 1.0_ReKi - sPitch )
   m(4)  = ( 1.0_ReKi - sOmega )*( 1.0_ReKi - sVinf )*( 1.0_ReKi - sSkew )*( 1.0_ReKi - sPitch )
   m(5)  = ( 1.0_ReKi + sOmega )*( 1.0_ReKi - sVinf )*( 1.0_ReKi + sSkew )*( 1.0_ReKi - sPitch )
   m(6)  = ( 1.0_ReKi + sOmega )*( 1.0_ReKi + sVinf )*( 1.0_ReKi + sSkew )*( 1.0_ReKi - sPitch )
   m(7)  = ( 1.0_ReKi - sOmega )*( 1.0_ReKi + sVinf )*( 1.0_ReKi + sSkew )*( 1.0_ReKi - sPitch )
   m(8)  = ( 1.0_ReKi - sOmega )*( 1.0_ReKi - sVinf )*( 1.0_ReKi + sSkew )*( 1.0_ReKi - sPitch )
   m(9)  = ( 1.0_ReKi + sOmega )*( 1.0_ReKi - sVinf )*( 1.0_ReKi - sSkew )*( 1.0_ReKi + sPitch )
   m(10) = ( 1.0_ReKi + sOmega )*( 1.0_ReKi + sVinf )*( 1.0_ReKi - sSkew )*( 1.0_ReKi + sPitch )
   m(11) = ( 1.0_ReKi - sOmega )*( 1.0_ReKi + sVinf )*( 1.0_ReKi - sSkew )*( 1.0_ReKi + sPitch )
   m(12) = ( 1.0_ReKi - sOmega )*( 1.0_ReKi - sVinf )*( 1.0_ReKi - sSkew )*( 1.0_ReKi + sPitch )
   m(13) = ( 1.0_ReKi + sOmega )*( 1.0_ReKi - sVinf )*( 1.0_ReKi + sSkew )*( 1.0_ReKi + sPitch )
   m(14) = ( 1.0_ReKi + sOmega )*( 1.0_ReKi + sVinf )*( 1.0_ReKi + sSkew )*( 1.0_ReKi + sPitch )
   m(15) = ( 1.0_ReKi - sOmega )*( 1.0_ReKi + sVinf )*( 1.0_ReKi + sSkew )*( 1.0_ReKi + sPitch )
   m(16) = ( 1.0_ReKi - sOmega )*( 1.0_ReKi - sVinf )*( 1.0_ReKi + sSkew )*( 1.0_ReKi + sPitch )

   m     =  m / 16.0_ReKi               ! normalize
   
   do i = 1,7
     
      v(1)  = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Hi, iVinf_Lo, iSkew_Lo, iPitch_Lo )
      v(2)  = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Hi, iVinf_Hi, iSkew_Lo, iPitch_Lo )
      v(3)  = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Lo, iVinf_Hi, iSkew_Lo, iPitch_Lo )
      v(4)  = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Lo, iVinf_Lo, iSkew_Lo, iPitch_Lo )
      v(5)  = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Hi, iVinf_Lo, iSkew_Hi, iPitch_Lo )
      v(6)  = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Hi, iVinf_Hi, iSkew_Hi, iPitch_Lo )
      v(7)  = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Lo, iVinf_Hi, iSkew_Hi, iPitch_Lo )
      v(8)  = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Lo, iVinf_Lo, iSkew_Hi, iPitch_Lo )
      v(9)  = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Hi, iVinf_Lo, iSkew_Lo, iPitch_Hi )
      v(10) = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Hi, iVinf_Hi, iSkew_Lo, iPitch_Hi )
      v(11) = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Lo, iVinf_Hi, iSkew_Lo, iPitch_Hi )
      v(12) = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Lo, iVinf_Lo, iSkew_Lo, iPitch_Hi )
      v(13) = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Hi, iVinf_Lo, iSkew_Hi, iPitch_Hi )
      v(14) = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Hi, iVinf_Hi, iSkew_Hi, iPitch_Hi )
      v(15) = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Lo, iVinf_Hi, iSkew_Hi, iPitch_Hi )
      v(16) = p%Omega_Ptch_Vinf_Skw_Table( i, iOmega_Lo, iVinf_Lo, iSkew_Hi, iPitch_Hi )
      
      CoefInterp(i)  =  sum ( m * v ) 
      
   end do

end function CoefInterp

!==============================================================================
subroutine ReadActDskFile(InitInp, errStat, errMsg)
! This routine is called as part of the initialization step.
! The initialization data is loaded from an input file.
! 
! Called by : ActDsk_Init
! Calls  to : GetNewUnit, OpenFInpfile, ReadCom, ReadVar, ReadAry, CleanUp
!..............................................................................

   type(ActDsk_InitInputType),       intent(inout)  :: InitInp
   integer(IntKi),                   intent(  out)  :: errStat     !< Error status of the operation
   character(*),                     intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None

         ! Local variables
   integer(IntKi)                               :: i, j, k, l, ln         ! counters

   character(ErrMsgLen)                         :: errMsg2     ! temporary Error message if ErrStat /= ErrID_None
   integer(IntKi)                               :: errStat2    ! temporary Error status of the operation
   character(*), parameter                      :: routineName = 'ReadActDskFile'
   character(1024)                              :: fileName
   integer(IntKi)                               :: UnIn, UnEc
   real(ReKi)                                   :: vals(11)    ! 11 entries in a single row of disk data
   
         ! Initialize variables for this routine
   errStat  = ErrID_None
   errMsg   = ""
   UnEc     = -1 
   fileName = trim(InitInp%Filename)
   
   call GetNewUnit( UnIn )   
  
   call OpenFInpfile(UnIn, trim(fileName), errStat, errMsg)
      if ( errStat /= ErrID_None ) then
         errStat = ErrID_Fatal
         call CleanUp()
         return
      end if
   
   !-------------------------- HEADER ---------------------------------------------
      ! Skip header lines
   do i = 1, 2
       call ReadCom( UnIn, fileName, 'ActuatorDisk input file header line '//trim(Int2LStr(i)), errStat, errMsg )
       if ( errStat /= ErrID_None ) then
          errStat = ErrID_Fatal
          call CleanUp()
          return
       end if
   end do   

   !-------------------------- ACTUATOR DISK PROPERTIES ----------------------

         ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' ACTUATOR DISK PROPERTIES ', errStat, errMsg  ) 
      if ( errStat /= ErrID_None ) then
         errStat = ErrID_Fatal
         call CleanUp()
         return
      end if

      ! Read Number of tip-speed ratios
   call ReadVar ( UnIn, fileName, InitInp%InitInpFile%numOmega, 'NumOmega', 'Number of rotor speeds', errStat, errMsg, UnEc )
      if ( errStat /= ErrID_None ) then
         errStat = ErrID_Fatal
         call CleanUp()
         return   
      end if

  
      ! Read Number of Freestream velocities
   call ReadVar ( UnIn, fileName, InitInp%InitInpFile%numVinf, 'NumVinf', 'Number of inflow velocities', errStat, errMsg, UnEc )
      if ( errStat /= ErrID_None ) then
         errStat = ErrID_Fatal
         call CleanUp()
         return   
      end if

      ! Read Number of inflow-skew angles
   call ReadVar ( UnIn, fileName, InitInp%InitInpFile%numSkew, 'NumSkew', 'Number of inflow-skew angles', errStat, errMsg, UnEc )
      if ( errStat /= ErrID_None ) then
         errStat = ErrID_Fatal
         call CleanUp()
         return   
      end if
   
          ! Read Number of pitch angles
   call ReadVar ( UnIn, fileName, InitInp%InitInpFile%numPitch, 'NumPitch', 'Number of pitch angles', errStat, errMsg, UnEc )
      if ( errStat /= ErrID_None ) then
         errStat = ErrID_Fatal
         call CleanUp()
         return   
      end if

      
      ! Allocate the data arrays
      
   allocate( InitInp%InitInpFile%Omegas (InitInp%InitInpFile%numOmega), STAT = errStat )
   allocate( InitInp%InitInpFile%Pitches(InitInp%InitInpFile%numPitch), STAT = errStat )
   allocate( InitInp%InitInpFile%Vinfs  (InitInp%InitInpFile%numVinf ), STAT = errStat )
   allocate( InitInp%InitInpFile%Skews  (InitInp%InitInpFile%numSkew ), STAT = errStat )
   allocate( InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table(7, InitInp%InitInpFile%numOmega, InitInp%InitInpFile%numVinf, InitInp%InitInpFile%numSkew, InitInp%InitInpFile%numPitch), STAT = errStat )
   
      ! Skip table header lines
   do i = 1, 2
       call ReadCom( UnIn, fileName, 'ActuatorDisk table header line '//trim(Int2LStr(i)), errStat, errMsg )
       if ( errStat /= ErrID_None ) then
          errStat = ErrID_Fatal
          call CleanUp()
          return
       end if
   end do   
    
      ! 3D Loop over the table entries
      ! The table data layout is such that we vary Omega the fastest, then Skew, and then Pitch
      ! For example, if we have 3 Omegas, 2 Pitches, 2 Vinfs, and 2 Skews the table might look like this:
      ! Omega       Pitch  Vinf   Skew    ...  Coefs
      !--------------------------------------------------------
      ! Om1         P1     V1      S1      ...
      ! Om2         P1     V1      S1      ...
      ! Om3         P1     V1      S1      ...
      ! Om1         P2     V1      S1      ...
      ! Om2         P2     V1      S1      ...
      ! Om3         P2     V1      S1      ...
      ! Om1         P1     V2      S1      ...
      ! Om2         P1     V2      S1      ...
      ! Om3         P1     V2      S1      ...
      ! Om1         P2     V2      S1      ...
      ! Om2         P2     V2      S1      ...
      ! Om3         P2     V2      S1      ...
      ! Om1         P1     V1      S2      ...
      ! Om2         P1     V1      S2      ...
      ! Om3         P1     V1      S2      ...
      ! Om1         P2     V1      S2      ...
      ! Om2         P2     V1      S2      ...
      ! Om3         P2     V1      S2      ...
      ! Om1         P1     V2      S2      ...
      ! Om2         P1     V2      S2      ...
      ! Om3         P1     V2      S2      ...
      ! Om1         P2     V2      S2      ...
      ! Om2         P2     V2      S2      ...
      ! Om3         P2     V2      S2      ...
   
   ln = 0
   do l = 1, InitInp%InitInpFile%numPitch
      do k = 1, InitInp%InitInpFile%numSkew
         do j = 1, InitInp%InitInpFile%numVinf
            do i = 1, InitInp%InitInpFile%numOmega
            
                  ! Read a row of coefficients
               call ReadAry( UnIn, fileName, vals, 11, 'Values','Table row', errStat2, errMsg2, UnEc )
                  if ( errStat2 /= ErrID_None ) then
                     call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
                     call CleanUp()
                     return
                  end if
               
                  ! Store the unique values of Omegas
               if (j == 1 .and. k==1 .and. l==1) then
                  InitInp%InitInpFile%Omegas(i) = vals(1) ! stored in rad/s, and table data is in rad/s
               end if
            
                  ! Store the unique values of Vinf
               if (i == 1 .and. k==1 .and. l==1) then
                  InitInp%InitInpFile%Vinfs(j) = vals(2) ! stored in m/s
               end if
         
                  ! Store the unique values of Skew
               if (i == 1 .and. j==1 .and. l==1) then
                  InitInp%InitInpFile%Skews(k) = vals(3)   *D2R ! stored in radians, but table data is in degrees
               end if
               
                  ! Store the unique values of Pitch
               if (i == 1 .and. j==1 .and. k==1) then
                  InitInp%InitInpFile%Pitches(l) = vals(4)*D2R ! stored in radians, but table data is in degrees 
               end if
                  ! Set all the coefficient values
               InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table(1,i,j,k,l) = vals(5)
               InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table(2,i,j,k,l) = vals(6)
               InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table(3,i,j,k,l) = vals(7)
               InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table(4,i,j,k,l) = vals(8)
               InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table(5,i,j,k,l) = vals(9)
               InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table(6,i,j,k,l) = vals(10)
               InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table(7,i,j,k,l) = vals(11)
            
  
               ln = ln + 1
            
                  ! Verify that the correct and expected values of Omega, Pitch, and Skew are found on the current row
               if ( InitInp%InitInpFile%Omegas (i) /= vals(1)      ) call SetErrStat( ErrID_Fatal, 'The omega value of '//trim(num2lstr(vals(1)))//' on row '//trim(num2lstr(ln))//' does not match the expected values of '//trim(num2lstr(InitInp%InitInpFile%Omegas (i)     )), errStat, errMsg, routineName )
               if ( InitInp%InitInpFile%Vinfs  (j) /= vals(2)      ) call SetErrStat( ErrID_Fatal, 'The Vinf value of ' //trim(num2lstr(vals(2)))//' on row '//trim(num2lstr(ln))//' does not match the expected values of '//trim(num2lstr(InitInp%InitInpFile%Vinfs  (j)     )), errStat, errMsg, routineName )
               if ( InitInp%InitInpFile%Skews  (k) /= vals(3)*D2R  ) call SetErrStat( ErrID_Fatal, 'The skew value of ' //trim(num2lstr(vals(3)))//' on row '//trim(num2lstr(ln))//' does not match the expected values of '//trim(num2lstr(InitInp%InitInpFile%Skews  (k)*R2D )), errStat, errMsg, routineName )
               if ( InitInp%InitInpFile%Pitches(l) /= vals(4)*D2R  ) call SetErrStat( ErrID_Fatal, 'The pitch value of '//trim(num2lstr(vals(4)))//' on row '//trim(num2lstr(ln))//' does not match the expected values of '//trim(num2lstr(InitInp%InitInpFile%Pitches(l)*R2D )), errStat, errMsg, routineName )
            
            end do
         end do
      end do
   end do
   
         
   call Cleanup()  
   
   contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
        
            close(UnIn)
            
         end subroutine Cleanup
         
end subroutine ReadActDskFile


!==============================================================================
subroutine ValidateInitData(InitInp, errStat, errMsg)
! This routine is called as part of the initialization step.
! The initialization data is verified here, separately from where it is set.
! This allows the initialization data to originate from various sources, but
!   all validation is done in one place, here.
! 
! Called by : ActDsk_Init
! Calls  to : SetErrStat
!..............................................................................
   type(ActDsk_InitInputType),       intent(in   )  :: InitInp     !< Initialization input data for the ActuatorDisk module
   integer(IntKi),                   intent(  out)  :: errStat     !< Error status of the operation
   character(*),                     intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None

         ! Local variables
   integer(IntKi)                               :: i,j,k,ln    ! counters
   real(ReKi)                                   :: minVal      ! temporary value
   character(ErrMsgLen)                         :: errMsg2     ! temporary Error message if ErrStat /= ErrID_None
   integer(IntKi)                               :: errStat2    ! temporary Error status of the operation
   character(*), parameter                      :: routineName = 'ValidateInitData'

         ! Initialize variables for this routine
   errStat = ErrID_None
   errMsg  = ""

   if ( InitInp%R        <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'Rotor radius must be greater than zero', errStat, errMsg, routineName )
   if ( InitInp%AirDens  <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'Air density must be greater than zero', errStat, errMsg, routineName )
   if ( InitInp%InitInpFile%numOmega   < 2         ) call SetErrStat( ErrID_Fatal, 'Number of Omega entries in the Omega/Pitch/Skew table must be greater than 1', errStat, errMsg, routineName )
   if ( InitInp%InitInpFile%numPitch < 2         ) call SetErrStat( ErrID_Fatal, 'Number of Pitch entries in the Omega/Pitch/Skew table must be greater than 1', errStat, errMsg, routineName )
   if ( InitInp%InitInpFile%numSkew  < 2         ) call SetErrStat( ErrID_Fatal, 'Number of Skew entries in the Omega/Pitch/Skew table must be greater than 1', errStat, errMsg, routineName )
   
   ! Verify that all the table data is monotonic and increasing
 
   minVal = InitInp%InitInpFile%Omegas(1)
   if ( ( minVal < 0.0_ReKi ) ) call SetErrStat( ErrID_Fatal, 'Omega values must be greater than or equal to zero', errStat, errMsg, routineName )
 
   do i = 2, InitInp%InitInpFile%numOmega
      if (  ( InitInp%InitInpFile%Omegas(i) <= minVal ) ) then
         call SetErrStat( ErrID_Fatal, 'Omega values must be monotonic and increasing', errStat, errMsg, routineName )
         exit
      end if
      minVal = InitInp%InitInpFile%Omegas(i)
   end do
  
   minVal = InitInp%InitInpFile%Pitches(1)
   if ( ( minVal < -PI ) ) call SetErrStat( ErrID_Fatal, 'Pitch values must be greater than or equal to -Pi', errStat, errMsg, routineName )
   if ( ( minVal > PI ) )       call SetErrStat( ErrID_Fatal, 'Pitch values must be less than or equal to Pi', errStat, errMsg, routineName )
   
   do i = 2, InitInp%InitInpFile%numPitch
      if (  ( InitInp%InitInpFile%Pitches(i) <= minVal ) ) then
         call SetErrStat( ErrID_Fatal, 'Pitch values must be monotonic and increasing', errStat, errMsg, routineName )
         exit
      end if
      if ( ( InitInp%InitInpFile%Pitches(i) > PI ) ) then
         call SetErrStat( ErrID_Fatal, 'Pitch values must be less than or equal to Pi', errStat, errMsg, routineName )
         exit
      end if
      
      minVal = InitInp%InitInpFile%Pitches(i)
      
   end do
   
   minVal = InitInp%InitInpFile%Skews(1)
   if ( ( minVal < 0.0_ReKi ) ) call SetErrStat( ErrID_Fatal, 'Skew values must be greater than or equal to zero', errStat, errMsg, routineName )
   if ( ( minVal > PI ) )       call SetErrStat( ErrID_Fatal, 'Skew values must be less than or equal to Pi', errStat, errMsg, routineName )
   
   do i = 2, InitInp%InitInpFile%numSkew
      if (  ( InitInp%InitInpFile%Skews(i) <= minVal ) ) then
         call SetErrStat( ErrID_Fatal, 'Skew values must be monotonic and increasing', errStat, errMsg, routineName )
         exit
      end if
      if ( ( InitInp%InitInpFile%Skews(i) > PI ) ) then
         call SetErrStat( ErrID_Fatal, 'Skew values must be less than or equal to Pi', errStat, errMsg, routineName )
         exit
      end if
      
      minVal = InitInp%InitInpFile%Skews(i)
      
   end do
   
   
   
   
end subroutine ValidateInitData

!==============================================================================
! Framework Routines                                                          !
!==============================================================================                               
      

!==============================================================================
subroutine ActDsk_Init( InitInp, u, p, y, interval, &
                              InitOut, errStat, errMsg )
! This routine is called at the start of the simulation to perform initialization steps.
! The parameters are set here and not changed during the simulation.
! The initial states and initial guess for the input are defined.
! Called by : Driver/Glue-code
! Calls  to : NWTC_Init, DispNVD, ReadActDskFile (conditional), ValidateInitData, Move_Alloc
!..............................................................................

   type(ActDsk_InitInputType),       intent(inout)  :: InitInp     !< Input data for initialization routine, needs to be inout because there is a copy of some data in InitInp in BEMT_SetParameters()
   type(ActDsk_InputType),           intent(in   )  :: u           !< An initial guess for the input; input mesh must be defined
   type(ActDsk_ParameterType),       intent(  out)  :: p           !< Parameters
   type(ActDsk_OutputType),          intent(  out)  :: y           !< Initial system outputs (outputs are not calculated;
                                                                   !<   only the output mesh is initialized)
   real(DbKi),                       intent(inout)  :: interval    !< Coupling interval in seconds: 
                                                                   !<   Input is the suggested time from the glue code;
                                                                   !<   Output is the actual coupling interval that will be used
                                                                   !<   by the glue code.
   type(ActDsk_InitOutputType),      intent(  out)  :: InitOut     !< Output for initialization routine
   integer(IntKi),                   intent(  out)  :: errStat     !< Error status of the operation
   character(*),                     intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None


      ! Local variables
   integer(IntKi)                               :: i,j, iNode, iOffset
    
      ! Initialize variables for this routine
   errStat = ErrID_None
   errMsg  = ""


      ! Initialize the NWTC Subroutine Library
   call NWTC_Init( EchoLibVer=.FALSE. )

      ! Return the module information to caller
   InitOut%Version = ActDsk_Ver
   
      ! Display the module information
   call DispNVD( ActDsk_Ver )

      ! Read Input file, otherwise the caller must have fully populated the Initialization inputs
   if ( trim(InitInp%Filename) /= '' ) then
      call ReadActDskFile(InitInp, errStat, errMsg)
         if ( errStat > ErrID_None ) return
   endif
   
      ! Validate the input file data and issue errors as needed
   call ValidateInitData(InitInp, errStat, errMsg)
      if ( errStat > ErrID_None ) return
   
      ! Set parameters based on initialization inputs
   p%R        = InitInp%R
   p%halfRhoA = 0.5_ReKi*InitInp%AirDens*pi*p%R*p%R
   p%numOmega = InitInp%InitInpFile%numOmega
   p%numPitch = InitInp%InitInpFile%numPitch
   p%numSkew  = InitInp%InitInpFile%numSkew
   p%numVinf  = InitInp%InitInpFile%numVinf
   
      ! Move Omega and Skew angles to parameters
   if (allocated(InitInp%InitInpFile%Omegas )) call Move_Alloc( InitInp%InitInpFile%Omegas ,  p%Omegas  )
   if (allocated(InitInp%InitInpFile%Pitches)) call Move_Alloc( InitInp%InitInpFile%Pitches,  p%Pitches )
   if (allocated(InitInp%InitInpFile%Skews)) call Move_Alloc( InitInp%InitInpFile%Skews,  p%Skews )
   if (allocated(InitInp%InitInpFile%Vinfs)) call Move_Alloc( InitInp%InitInpFile%Vinfs,  p%Vinfs )
   if (allocated(InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table)) call Move_Alloc( InitInp%InitInpFile%Omega_Ptch_Vinf_Skw_Table,  p%Omega_Ptch_Vinf_Skw_Table )
   
   ! TODO Set the rest of the InitOut values per plan once we see how they are used/need by KiteAeroDyn (for use in KiteVSM?) 
   
end subroutine ActDsk_Init
!==============================================================================                                  

!============================================================================== 
subroutine ActDsk_CalcOutput( u, p, y, errStat, errMsg )   
! Routine for computing outputs, used in both loose and tight coupling.
! Called by : Driver/Glue-code
! Calls  to : CoefInterp, SetErrStat
!..............................................................................
   
   type(ActDsk_InputType),           intent(in   )  :: u           !< Inputs at Time
   type(ActDsk_ParameterType),       intent(in   )  :: p           !< Parameters
   type(ActDsk_OutputType),          intent(inout)  :: y           !< Outputs computed at Time
   integer(IntKi),                   intent(  out)  :: errStat     !< Error status of the operation
   character(*),                     intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None

   
   integer(IntKi)                                   :: errStat2    ! Error status of the operation (secondary error)
   character(ErrMsgLen)                             :: errMsg2     ! Error message if errStat2 /= ErrID_None
   real(ReKi)                                       :: velsqrd     ! The square of the disk-averaged axial velocity 
   real(ReKi)                                       :: TSR         ! Tip-speed-ratio for the current time step
   real(ReKi)                                       :: coefs(7)
   character(*), parameter                          :: routineName = 'ActDsk_CalcOutput'

   
   errStat   = ErrID_None           ! no error has occurred
   errMsg    = ""
   
   
   if ( u%DiskAve_Vx_Rel < 0.0_ReKi ) then
      call SetErrStat( ErrID_Fatal, 'u%DiskAve_Vx_Rel is less than zero', errStat, errMsg, routineName ) 
      return
   end if
   
      ! If the axial component of the relative velocity is zero, all forces, moments, and power are set to zero.
   if ( EqualRealNos(u%DiskAve_Vx_Rel, 0.0_ReKi) ) then
      y%Fx = 0.0_ReKi
      y%Fy = 0.0_ReKi
      y%Fz = 0.0_ReKi
      y%Mx = 0.0_ReKi
      y%My = 0.0_ReKi
      y%Mz = 0.0_ReKi
      y%P  = 0.0_ReKi
      return  
   end if
   
   TSR = abs( u%omega*p%R / u%DiskAve_Vx_Rel ) 

      ! Use trilinearly interpolation to determine the disk coefficients associated with this operating condition
   coefs = CoefInterp(u%omega, u%pitch, u%DiskAve_Vinf_Rel, u%skew, p, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName ) 
      
   velsqrd = u%DiskAve_Vx_Rel*u%DiskAve_Vx_Rel

   y%Fx = p%halfRhoA*velsqrd*coefs(1)

   y%Fy = p%halfRhoA*velsqrd*coefs(2)

   y%Fz = p%halfRhoA*velsqrd*coefs(3)

   y%Mx = p%halfRhoA*p%R*velsqrd*coefs(4)

   y%My = p%halfRhoA*p%R*velsqrd*coefs(5)  

   y%Mz = p%halfRhoA*p%R*velsqrd*coefs(6)
   
   y%P = p%halfRhoA*velsqrd*u%DiskAve_Vx_Rel*coefs(7)
  
end subroutine ActDsk_CalcOutput

end module ActuatorDisk
