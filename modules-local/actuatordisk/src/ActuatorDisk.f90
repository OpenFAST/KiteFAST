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
!> This module solves quasi-steady actuator disk to calculate the 3 forces, 3 moments, and power dependent on RtSpd, blade pitch, inflow, and inflow skew.
module ActuatorDisk

   
   use NWTC_Library   
   use ActuatorDisk_Types
   
   implicit none 

private

   type(ProgDesc), parameter  :: ActDsk_Ver = ProgDesc( 'ActuatorDisk', '', '' )

   public :: ActDsk_Init
   public :: ActDsk_CalcOutput
   public :: ActDsk_End

   contains
   
function CoefInterp(RtSpd, pitch, Vrel, skew, p, errStat, errMsg)
   real(ReKi),                 intent(in   )  :: RtSpd
   real(ReKi),                 intent(in   )  :: pitch
   real(ReKi),                 intent(in   )  :: Vrel
   real(ReKi),                 intent(in   )  :: skew
   type(ActDsk_ParameterType), intent(in   )  :: p
   integer(IntKi),             intent(  out)  :: errStat     !< Error status of the operation
   character(*),               intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None
   real(ReKi)   :: CoefInterp(7)
   
   integer(IntKi)             :: iRtSpd_Lo, iRtSpd_Hi, iSkew_Lo, iSkew_Hi, iPitch_Lo, iPitch_Hi, iVrel_Lo, iVrel_Hi   ! grid indices
   real(ReKi)                 :: sRtSpd, sSkew, sPitch, sVrel   ! interpolant for RtSpd and Skew, Pitch, and Vrel, scaled between -1 and 1
   real(ReKi)                 :: m(16), v(16)
   integer(IntKi)             :: i             ! loop counter
   character(*), parameter    :: routineName = 'CoefInterp'
   
         ! Initialize variables for this routine
   errStat = ErrID_None
   errMsg  = ""
   CoefInterp = 0.0_ReKi
   
   ! Make sure the input RtSpd, Skew, Vrel, and Pitch angle are within our data tables.  Do not allow extrapolation.
   
   if ( ( RtSpd < p%RtSpds(1) ) .or. ( RtSpd > p%RtSpds(p%numRtSpd) ) )  then
      call SetErrStat(ErrID_Fatal, 'The input RtSpd of '//trim(num2lstr(RtSpd))//' (rad/s) is outside the values found in the RtSpd tables for this rotor',errStat, errMsg, routineName)
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
   
   if ( ( Vrel < p%Vrels(1) ) .or. ( Vrel > p%Vrels(p%numVrel) ) ) then
      call SetErrStat(ErrID_Fatal, 'The input Vrel of '//trim(num2lstr(Vrel))//' (m/s) is outside the values found in the Vrel tables for this rotor',errStat, errMsg, routineName)
      return
   end if
   
   
   
   
   iRtSpd_Lo  = 1
   iSkew_Lo = 1
   iPitch_Lo = 1
   iVrel_Lo = 1
   
     ! Let's interpolate!

   iRtSpd_Lo = MAX( MIN( iRtSpd_Lo, p%numRtSpd-1 ), 1 )

   do

      if ( RtSpd < p%RtSpds(iRtSpd_Lo) )  then

         iRtSpd_Lo = iRtSpd_Lo - 1

      else if ( RtSpd > p%RtSpds(iRtSpd_Lo+1) )  then

         iRtSpd_Lo = iRtSpd_Lo + 1

      else

         iRtSpd_Hi = iRtSpd_Lo + 1
            ! scale sRtSpd between -1  and 1 where -1 = iRtSpd_Lo and 1 = iRtSpd_Hi
         sRtSpd = 2.0_ReKi*((RtSpd - p%RtSpds(iRtSpd_Lo))/(p%RtSpds(iRtSpd_Hi) - p%RtSpds(iRtSpd_Lo))) - 1_ReKi
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
         sSkew = 2.0_ReKi*((skew - p%Skews(iSkew_Lo))/(p%Skews(iSkew_Hi) - p%Skews(iSkew_Lo))) - 1_ReKi
         exit

      end if

   end do
   
   iPitch_Lo = MAX( MIN( iPitch_Lo, p%numPitch-1 ), 1 )

   do

      if ( pitch < p%Pitches(iPitch_Lo) )  then

         iPitch_Lo = iPitch_Lo - 1

      else if ( pitch > p%Pitches(iPitch_Lo+1) )  then

         iPitch_Lo = iPitch_Lo + 1

      else

         iPitch_Hi = iPitch_Lo + 1
            ! scale sPitch between -1  and 1 where -1 = iPitch_Lo and 1 = iPitch_Hi
         sPitch = 2.0_ReKi*((pitch - p%Pitches(iPitch_Lo))/(p%Pitches(iPitch_Hi) - p%Pitches(iPitch_Lo))) - 1_ReKi
         exit

      end if

   end do
   
   iVrel_Lo = MAX( MIN( iVrel_Lo, p%numVrel-1 ), 1 )

   do

      if ( Vrel < p%Vrels(iVrel_Lo) )  then

         iVrel_Lo = iVrel_Lo - 1

      else if ( Vrel > p%Vrels(iVrel_Lo+1) )  then

         iVrel_Lo = iVrel_Lo + 1

      else

         iVrel_Hi = iVrel_Lo + 1
            ! scale sVrel between -1  and 1 where -1 = iVrel_Lo and 1 = iVrel_Hi
         sVrel = 2.0_ReKi*((Vrel - p%Vrels(iVrel_Lo))/(p%Vrels(iVrel_Hi) - p%Vrels(iVrel_Lo))) - 1_ReKi
         exit

      end if

   end do
   
   !----------------------------------------------------------------------------------------------
   ! Interpolate the RtSpd/Pitch/Vrel/Skew table using an volume interpolation.
   !----------------------------------------------------------------------------------------------
   
      ! Setup the scaling factors.  Set the unused portion of the array to zero
   m(1)  = ( 1.0_ReKi + sRtSpd )*( 1.0_ReKi - sVrel )*( 1.0_ReKi - sSkew )*( 1.0_ReKi - sPitch )
   m(2)  = ( 1.0_ReKi + sRtSpd )*( 1.0_ReKi + sVrel )*( 1.0_ReKi - sSkew )*( 1.0_ReKi - sPitch )
   m(3)  = ( 1.0_ReKi - sRtSpd )*( 1.0_ReKi + sVrel )*( 1.0_ReKi - sSkew )*( 1.0_ReKi - sPitch )
   m(4)  = ( 1.0_ReKi - sRtSpd )*( 1.0_ReKi - sVrel )*( 1.0_ReKi - sSkew )*( 1.0_ReKi - sPitch )
   m(5)  = ( 1.0_ReKi + sRtSpd )*( 1.0_ReKi - sVrel )*( 1.0_ReKi + sSkew )*( 1.0_ReKi - sPitch )
   m(6)  = ( 1.0_ReKi + sRtSpd )*( 1.0_ReKi + sVrel )*( 1.0_ReKi + sSkew )*( 1.0_ReKi - sPitch )
   m(7)  = ( 1.0_ReKi - sRtSpd )*( 1.0_ReKi + sVrel )*( 1.0_ReKi + sSkew )*( 1.0_ReKi - sPitch )
   m(8)  = ( 1.0_ReKi - sRtSpd )*( 1.0_ReKi - sVrel )*( 1.0_ReKi + sSkew )*( 1.0_ReKi - sPitch )
   m(9)  = ( 1.0_ReKi + sRtSpd )*( 1.0_ReKi - sVrel )*( 1.0_ReKi - sSkew )*( 1.0_ReKi + sPitch )
   m(10) = ( 1.0_ReKi + sRtSpd )*( 1.0_ReKi + sVrel )*( 1.0_ReKi - sSkew )*( 1.0_ReKi + sPitch )
   m(11) = ( 1.0_ReKi - sRtSpd )*( 1.0_ReKi + sVrel )*( 1.0_ReKi - sSkew )*( 1.0_ReKi + sPitch )
   m(12) = ( 1.0_ReKi - sRtSpd )*( 1.0_ReKi - sVrel )*( 1.0_ReKi - sSkew )*( 1.0_ReKi + sPitch )
   m(13) = ( 1.0_ReKi + sRtSpd )*( 1.0_ReKi - sVrel )*( 1.0_ReKi + sSkew )*( 1.0_ReKi + sPitch )
   m(14) = ( 1.0_ReKi + sRtSpd )*( 1.0_ReKi + sVrel )*( 1.0_ReKi + sSkew )*( 1.0_ReKi + sPitch )
   m(15) = ( 1.0_ReKi - sRtSpd )*( 1.0_ReKi + sVrel )*( 1.0_ReKi + sSkew )*( 1.0_ReKi + sPitch )
   m(16) = ( 1.0_ReKi - sRtSpd )*( 1.0_ReKi - sVrel )*( 1.0_ReKi + sSkew )*( 1.0_ReKi + sPitch )

   m     =  m / 16.0_ReKi               ! normalize
   
   do i = 1,7
     
      v(1)  = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Hi, iVrel_Lo, iSkew_Lo, iPitch_Lo )
      v(2)  = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Hi, iVrel_Hi, iSkew_Lo, iPitch_Lo )
      v(3)  = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Lo, iVrel_Hi, iSkew_Lo, iPitch_Lo )
      v(4)  = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Lo, iVrel_Lo, iSkew_Lo, iPitch_Lo )
      v(5)  = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Hi, iVrel_Lo, iSkew_Hi, iPitch_Lo )
      v(6)  = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Hi, iVrel_Hi, iSkew_Hi, iPitch_Lo )
      v(7)  = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Lo, iVrel_Hi, iSkew_Hi, iPitch_Lo )
      v(8)  = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Lo, iVrel_Lo, iSkew_Hi, iPitch_Lo )
      v(9)  = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Hi, iVrel_Lo, iSkew_Lo, iPitch_Hi )
      v(10) = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Hi, iVrel_Hi, iSkew_Lo, iPitch_Hi )
      v(11) = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Lo, iVrel_Hi, iSkew_Lo, iPitch_Hi )
      v(12) = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Lo, iVrel_Lo, iSkew_Lo, iPitch_Hi )
      v(13) = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Hi, iVrel_Lo, iSkew_Hi, iPitch_Hi )
      v(14) = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Hi, iVrel_Hi, iSkew_Hi, iPitch_Hi )
      v(15) = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Lo, iVrel_Hi, iSkew_Hi, iPitch_Hi )
      v(16) = p%RtSpd_Ptch_Vrel_Skw_Table( i, iRtSpd_Lo, iVrel_Lo, iSkew_Hi, iPitch_Hi )
      
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
   fileName = InitInp%Filename
   
   call GetNewUnit( UnIn )   
  
   call OpenFInpfile(UnIn, fileName, errStat, errMsg)
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
   call ReadVar ( UnIn, fileName, InitInp%InitInpFile%numRtSpd, 'NumRtSpd', 'Number of rotor speeds', errStat, errMsg, UnEc )
      if ( errStat /= ErrID_None ) then
         errStat = ErrID_Fatal
         call CleanUp()
         return   
      end if

  
      ! Read Number of Freestream velocities
   call ReadVar ( UnIn, fileName, InitInp%InitInpFile%numVrel, 'NumVrel', 'Number of inflow velocities', errStat, errMsg, UnEc )
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

   if ( InitInp%InitInpFile%numRtSpd < 2  ) call SetErrStat( ErrID_Fatal, 'Number of RtSpd entries in the RtSpd/Vrel/Skew/Pitch table must be greater than 1', errStat, errMsg, routineName )
   if ( InitInp%InitInpFile%numVrel  < 2  ) call SetErrStat( ErrID_Fatal, 'Number of Vrel entries in the RtSpd/Vrel/Skew/Pitch table must be greater than 1', errStat, errMsg, routineName )
   if ( InitInp%InitInpFile%numPitch < 2  ) call SetErrStat( ErrID_Fatal, 'Number of Pitch entries in the RtSpd/Vrel/Skew/Pitch table must be greater than 1', errStat, errMsg, routineName )
   if ( InitInp%InitInpFile%numSkew  < 2  ) call SetErrStat( ErrID_Fatal, 'Number of Skew entries in the RtSpd/Vrel/Skew/Pitch table must be greater than 1', errStat, errMsg, routineName )

   if ( errStat >= AbortErrLev ) then     
      call CleanUp()
      return   
   end if
      
      ! Allocate the data arrays
      
   allocate( InitInp%InitInpFile%RtSpds (InitInp%InitInpFile%numRtSpd), STAT = errStat )
   allocate( InitInp%InitInpFile%Pitches(InitInp%InitInpFile%numPitch), STAT = errStat )
   allocate( InitInp%InitInpFile%Vrels  (InitInp%InitInpFile%numVrel ), STAT = errStat )
   allocate( InitInp%InitInpFile%Skews  (InitInp%InitInpFile%numSkew ), STAT = errStat )
   allocate( InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table(7, InitInp%InitInpFile%numRtSpd, InitInp%InitInpFile%numVrel, InitInp%InitInpFile%numSkew, InitInp%InitInpFile%numPitch), STAT = errStat )
   
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
      ! The table data layout is such that we vary RtSpd the fastest, then Skew, and then Pitch
      ! For example, if we have 3 RtSpds, 2 Pitches, 2 Vrels, and 2 Skews the table might look like this:
      ! RtSpd       Pitch  Vrel   Skew    ...  Coefs
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
         do j = 1, InitInp%InitInpFile%numVrel
            do i = 1, InitInp%InitInpFile%numRtSpd
            
                  ! Read a row of coefficients
               call ReadAry( UnIn, fileName, vals, 11, 'Values','Table row', errStat2, errMsg2, UnEc )
                  if ( errStat2 /= ErrID_None ) then
                     call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
                     call CleanUp()
                     return
                  end if
               
                  ! Store the unique values of RtSpds
               if (j == 1 .and. k==1 .and. l==1) then
                  InitInp%InitInpFile%RtSpds(i) = vals(1) ! stored in rad/s, and table data is in rad/s
               end if
            
                  ! Store the unique values of Vrel
               if (i == 1 .and. k==1 .and. l==1) then
                  InitInp%InitInpFile%Vrels(j) = vals(2) ! stored in m/s
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
               InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table(1,i,j,k,l) = vals(5)
               InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table(2,i,j,k,l) = vals(6)
               InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table(3,i,j,k,l) = vals(7)
               InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table(4,i,j,k,l) = vals(8)
               InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table(5,i,j,k,l) = vals(9)
               InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table(6,i,j,k,l) = vals(10)
               InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table(7,i,j,k,l) = vals(11)
            
  
               ln = ln + 1
            
                  ! Verify that the correct and expected values of RtSpd, Pitch, and Skew are found on the current row
               if ( .not. EqualRealNos(InitInp%InitInpFile%RtSpds (i), vals(1)     ) ) call SetErrStat( ErrID_Fatal, 'The RtSpd value of '//trim(num2lstr(vals(1)))//' on row '//trim(num2lstr(ln))//' does not match the expected values of '//trim(num2lstr(InitInp%InitInpFile%RtSpds (i)     )), errStat, errMsg, routineName )
               if ( .not. EqualRealNos(InitInp%InitInpFile%Vrels  (j), vals(2)     ) ) call SetErrStat( ErrID_Fatal, 'The Vrel value of ' //trim(num2lstr(vals(2)))//' on row '//trim(num2lstr(ln))//' does not match the expected values of '//trim(num2lstr(InitInp%InitInpFile%Vrels  (j)     )), errStat, errMsg, routineName )
               if ( .not. EqualRealNos(InitInp%InitInpFile%Skews  (k), vals(3)*D2R ) ) call SetErrStat( ErrID_Fatal, 'The skew value of ' //trim(num2lstr(vals(3)))//' on row '//trim(num2lstr(ln))//' does not match the expected values of '//trim(num2lstr(InitInp%InitInpFile%Skews  (k)*R2D )), errStat, errMsg, routineName )
               if ( .not. EqualRealNos(InitInp%InitInpFile%Pitches(l), vals(4)*D2R ) ) call SetErrStat( ErrID_Fatal, 'The pitch value of '//trim(num2lstr(vals(4)))//' on row '//trim(num2lstr(ln))//' does not match the expected values of '//trim(num2lstr(InitInp%InitInpFile%Pitches(l)*R2D )), errStat, errMsg, routineName )
            
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
   integer(IntKi)                               :: i           ! counter
   real(ReKi)                                   :: minVal      ! temporary value
   character(*), parameter                      :: routineName = 'ValidateInitData'

         ! Initialize variables for this routine
   errStat = ErrID_None
   errMsg  = ""

   if ( InitInp%R        <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'Rotor radius must be greater than zero', errStat, errMsg, routineName )
   if ( InitInp%AirDens  <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'Air density must be greater than zero', errStat, errMsg, routineName )
   
   ! Verify that all the table data is monotonic and increasing
 
   minVal = InitInp%InitInpFile%RtSpds(1)
 
   do i = 2, InitInp%InitInpFile%numRtSpd
      if (  ( InitInp%InitInpFile%RtSpds(i) <= minVal ) ) then
         call SetErrStat( ErrID_Fatal, 'RtSpd values must be monotonic and increasing', errStat, errMsg, routineName )
         exit
      end if
      minVal = InitInp%InitInpFile%RtSpds(i)
   end do
  
   minVal = InitInp%InitInpFile%Vrels(1)
   if ( ( minVal < 0.0_ReKi ) ) call SetErrStat( ErrID_Fatal, 'Vrel values must be greater than or equal to 0.0', errStat, errMsg, routineName )
   do i = 2, InitInp%InitInpFile%numVrel
      if (  ( InitInp%InitInpFile%Vrels(i) <= minVal ) ) then
         call SetErrStat( ErrID_Fatal, 'Vrel values must be monotonic and increasing', errStat, errMsg, routineName )
         exit
      end if
      minVal = InitInp%InitInpFile%Vrels(i)
   end do
   
   minVal = InitInp%InitInpFile%Pitches(1)
   !if ( ( minVal < -PI ) ) call SetErrStat( ErrID_Fatal, 'Pitch values must be greater than or equal to -Pi', errStat, errMsg, routineName )
   !if ( ( minVal > PI ) )       call SetErrStat( ErrID_Fatal, 'Pitch values must be less than or equal to Pi', errStat, errMsg, routineName )
   
   do i = 2, InitInp%InitInpFile%numPitch
      if (  ( InitInp%InitInpFile%Pitches(i) <= minVal ) ) then
         call SetErrStat( ErrID_Fatal, 'Pitch values must be monotonic and increasing', errStat, errMsg, routineName )
         exit
      end if
      !if ( ( InitInp%InitInpFile%Pitches(i) > PI ) ) then
      !   call SetErrStat( ErrID_Fatal, 'Pitch values must be less than or equal to Pi', errStat, errMsg, routineName )
      !   exit
      !end if
      
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


   
      ! Initialize variables for this routine
   errStat = ErrID_None
   errMsg  = ""


      ! Initialize the NWTC Subroutine Library
   call NWTC_Init( EchoLibVer=.FALSE. )

      ! Return the module information to caller
   InitOut%Version = ActDsk_Ver
   
      ! Display the module information
   call DispNVD( ActDsk_Ver )

   p%RotorMod = InitInp%RotorMod
   
   if ( p%RotorMod == 1 ) then
         ! Read Input file, otherwise the caller must have fully populated the Initialization inputs
      if ( trim(InitInp%Filename) /= '' ) then
         call ReadActDskFile(InitInp, errStat, errMsg)
            if ( errStat > ErrID_None ) return
      endif
         ! Validate the input file data and issue errors as needed
      call ValidateInitData(InitInp, errStat, errMsg)
         if ( errStat > ErrID_None ) return
         
      p%numRtSpd = InitInp%InitInpFile%numRtSpd
      p%numPitch = InitInp%InitInpFile%numPitch
      p%numSkew  = InitInp%InitInpFile%numSkew
      p%numVrel  = InitInp%InitInpFile%numVrel
   
   end if
   
   
   
      ! Set parameters based on initialization inputs
   p%R        = InitInp%R
   p%halfRhoA = 0.5_ReKi*InitInp%AirDens*pi*p%R*p%R
 
      ! Move RtSpd and Skew angles to parameters
   if (allocated(InitInp%InitInpFile%RtSpds )) call Move_Alloc( InitInp%InitInpFile%RtSpds ,  p%RtSpds  )
   if (allocated(InitInp%InitInpFile%Pitches)) call Move_Alloc( InitInp%InitInpFile%Pitches,  p%Pitches )
   if (allocated(InitInp%InitInpFile%Skews)) call Move_Alloc( InitInp%InitInpFile%Skews,  p%Skews )
   if (allocated(InitInp%InitInpFile%Vrels)) call Move_Alloc( InitInp%InitInpFile%Vrels,  p%Vrels )
   if (allocated(InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table)) call Move_Alloc( InitInp%InitInpFile%RtSpd_Ptch_Vrel_Skw_Table,  p%RtSpd_Ptch_Vrel_Skw_Table )
  
end subroutine ActDsk_Init
!==============================================================================                                  

!============================================================================== 
subroutine ActDsk_CalcOutput( u, p, m, y, errStat, errMsg )   
! Routine for computing outputs, used in both loose and tight coupling.
! Called by : Driver/Glue-code
! Calls  to : CoefInterp, SetErrStat
!..............................................................................
   
   type(ActDsk_InputType),           intent(in   )  :: u           !< Inputs at Time
   type(ActDsk_ParameterType),       intent(in   )  :: p           !< Parameters
   type(ActDsk_MiscVarType),         intent(inout)  :: m           !< MiscVars
   type(ActDsk_OutputType),          intent(inout)  :: y           !< Outputs computed at Time
   integer(IntKi),                   intent(  out)  :: errStat     !< Error status of the operation
   character(*),                     intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None

   
   integer(IntKi)                                   :: errStat2    ! Error status of the operation (secondary error)
   character(ErrMsgLen)                             :: errMsg2     ! Error message if errStat2 /= ErrID_None
   real(ReKi)                                       :: velsqrd     ! The square of the disk-averaged axial velocity 
   real(ReKi)                                       :: coefs(7)
   character(*), parameter                          :: routineName = 'ActDsk_CalcOutput'
   real(ReKi)                                       :: factorF, factorM
   
   errStat   = ErrID_None           ! no error has occurred
   errMsg    = ""
   
   m%DiskAve_Vx_Rel = abs(u%DiskAve_Vrel*cos(u%skew))
   
   if ( EqualRealNos(m%DiskAve_Vx_Rel, 0.0_ReKi) ) then
      m%TSR = 0.0_ReKi
   else    
      m%TSR = abs( u%RtSpd*p%R / m%DiskAve_Vx_Rel ) 
   end if

   if (p%RotorMod == 1) then
         ! Use quadlinear interpolation to determine the disk coefficients associated with this operating condition
      coefs = CoefInterp(u%RtSpd, u%pitch, u%DiskAve_Vrel, u%skew, p, errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName ) 
      
      velsqrd  = m%DiskAve_Vx_Rel*m%DiskAve_Vx_Rel
      factorF  = p%halfRhoA*velsqrd 
      factorM  = factorF*p%R
   
      y%Fx = factorF*coefs(1)
      y%Fy = factorF*coefs(2)
      y%Fz = factorF*coefs(3)
      y%Mx = factorM*coefs(4)
      y%My = factorM*coefs(5)  
      y%Mz = factorM*coefs(6)
      y%P  = factorF*m%DiskAve_Vx_Rel*coefs(7)
      m%Cp = coefs(7)
      m%Cq = coefs(4)
      m%Ct = -coefs(1)
   else
      y%Fx = 0.0_ReKi
      y%Fy = 0.0_ReKi
      y%Fz = 0.0_ReKi
      y%Mx = 0.0_ReKi
      y%My = 0.0_ReKi
      y%Mz = 0.0_ReKi
      y%P  = 0.0_ReKi
      m%Cp = 0.0_ReKi
      m%Cq = 0.0_ReKi
      m%Ct = 0.0_ReKi
   end if
 
end subroutine ActDsk_CalcOutput

subroutine ActDsk_End(u, p, y, m, errStat, errMsg)
   type(ActDsk_InputType) ,            intent(inout) :: u
   type(ActDsk_ParameterType) ,        intent(inout) :: p
   type(ActDsk_OutputType) ,           intent(inout) :: y
   type(ActDsk_MiscVarType),           intent(inout) :: m      
   integer(IntKi),                  intent(  out) :: errStat
   character(*),                    intent(  out) :: errMsg

!      integer(IntKi)                                :: i=0

   integer(IntKi)                               :: errStat2      ! Error status of the operation
   character(ErrMsgLen)                         :: errMsg2       ! Error message if ErrStat2 /= ErrID_None
   character(*), parameter                      :: routineName = 'ActDsk_End'
   ErrStat = ErrID_None
   ErrMsg  = ""
  
      ! deallocate data structures
   call ActDsk_DestroyInput(u, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call ActDsk_DestroyParam(p, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call ActDsk_DestroyOutput(y, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call ActDsk_DestroyMisc(m, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )      

end subroutine ActDsk_End

end module ActuatorDisk
