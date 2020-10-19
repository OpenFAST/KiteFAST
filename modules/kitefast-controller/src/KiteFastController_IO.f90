!**********************************************************************************************************************************
! LICENSING
! Copyright (C) 2019  Makani
!
!
!**********************************************************************************************************************************
!> This module sets up the output-handling data structures for KiteAeroDyn.
   
module KiteFastController_IO
   
   use NWTC_Library
   use KiteFastController_Types
   
   implicit none

! ===================================================================================================
! NOTE: The following lines of code were generated by a Matlab script called "Write_ChckOutLst.m"
!      using the parameters listed in the "OutListParameters.xlsx" Excel file. Any changes to these 
!      lines should be modified in the Matlab script and/or Excel worksheet as necessary. 
! ===================================================================================================
! This code was generated by Write_ChckOutLst.m at 22-Feb-2018 08:48:34.


     ! Parameters related to output length (number of characters allowed in the output data headers):

   INTEGER(IntKi), PARAMETER      :: OutStrLenM1 = ChanLen - 1


     ! Indices for computing output channels:
     ! NOTES: 
     !    (1) These parameters are in the order stored in "OutListParameters.xlsx"
     !    (2) Array AllOuts() must be dimensioned to the value of the largest output parameter

     !  Time: 

   INTEGER(IntKi), PARAMETER      :: Time      =    0

     ! Rotor Outputs:

   INTEGER(IntKi), PARAMETER      :: Rot1GTq   = 1
   INTEGER(IntKi), PARAMETER      :: Rot2GTq   = 2
   INTEGER(IntKi), PARAMETER      :: Rot3GTq   = 3
   INTEGER(IntKi), PARAMETER      :: Rot4GTq   = 4
   INTEGER(IntKi), PARAMETER      :: Rot5GTq   = 5
   INTEGER(IntKi), PARAMETER      :: Rot6GTq   = 6
   INTEGER(IntKi), PARAMETER      :: Rot7GTq   = 7
   INTEGER(IntKi), PARAMETER      :: Rot8GTq   = 8
   INTEGER(IntKi), PARAMETER      :: Rot1Spd   = 9
   INTEGER(IntKi), PARAMETER      :: Rot2Spd   = 10
   INTEGER(IntKi), PARAMETER      :: Rot3Spd   = 11
   INTEGER(IntKi), PARAMETER      :: Rot4Spd   = 12
   INTEGER(IntKi), PARAMETER      :: Rot5Spd   = 13
   INTEGER(IntKi), PARAMETER      :: Rot6Spd   = 14
   INTEGER(IntKi), PARAMETER      :: Rot7Spd   = 15
   INTEGER(IntKi), PARAMETER      :: Rot8Spd   = 16
   INTEGER(IntKi), PARAMETER      :: Rot1ATq   = 17
   INTEGER(IntKi), PARAMETER      :: Rot2ATq   = 18
   INTEGER(IntKi), PARAMETER      :: Rot3ATq   = 19
   INTEGER(IntKi), PARAMETER      :: Rot4ATq   = 20
   INTEGER(IntKi), PARAMETER      :: Rot5ATq   = 21
   INTEGER(IntKi), PARAMETER      :: Rot6ATq   = 22
   INTEGER(IntKi), PARAMETER      :: Rot7ATq   = 23
   INTEGER(IntKi), PARAMETER      :: Rot8ATq   = 24
    ! Flap Outputs:
   INTEGER(IntKi), PARAMETER      :: Flp1Def   = 25
   INTEGER(IntKi), PARAMETER      :: Flp2Def   = 26
   INTEGER(IntKi), PARAMETER      :: Flp3Def   = 27
   INTEGER(IntKi), PARAMETER      :: Flp4Def   = 28
   INTEGER(IntKi), PARAMETER      :: Flp5Def   = 29
   INTEGER(IntKi), PARAMETER      :: Flp6Def   = 30
   INTEGER(IntKi), PARAMETER      :: Flp7Def   = 31
   INTEGER(IntKi), PARAMETER      :: Flp8Def   = 32
    ! DCM_G2B passed to CSIM controller:   
   INTEGER(IntKi), PARAMETER      :: DCMG2Bc1   = 33
   INTEGER(IntKi), PARAMETER      :: DCMG2Bc2   = 34
   INTEGER(IntKi), PARAMETER      :: DCMG2Bc3   = 35
   INTEGER(IntKi), PARAMETER      :: DCMG2Bc4   = 36
   INTEGER(IntKi), PARAMETER      :: DCMG2Bc5   = 37
   INTEGER(IntKi), PARAMETER      :: DCMG2Bc6   = 38
   INTEGER(IntKi), PARAMETER      :: DCMG2Bc7   = 39
   INTEGER(IntKi), PARAMETER      :: DCMG2Bc8   = 40
   INTEGER(IntKi), PARAMETER      :: DCMG2Bc9   = 41
   !Tether Force at the bridle knot
   INTEGER(IntKi), PARAMETER      :: TethFxb    = 42
   INTEGER(IntKi), PARAMETER      :: TethFyb    = 43
   INTEGER(IntKi), PARAMETER      :: TethFzb    = 44
     ! The maximum number of output channels which can be output by the code.
   INTEGER(IntKi), PARAMETER      :: MaxOutPts = 44

!End of code generated by Matlab script
! ===================================================================================================
   INTEGER,  PARAMETER :: GenTq(8)     = (/Rot1GTq, Rot2GTq, Rot3GTq, Rot4GTq, Rot5GTq, Rot6GTq, Rot7GTq, Rot8GTq/)
   INTEGER,  PARAMETER :: RotSpd(8)    = (/Rot1Spd, Rot2Spd, Rot3Spd, Rot4Spd, Rot5Spd, Rot6Spd, Rot7Spd, Rot8Spd/)
   INTEGER,  PARAMETER :: RotATq(8)    = (/Rot1ATq, Rot2ATq, Rot3ATq, Rot4ATq, Rot5ATq, Rot6ATq, Rot7ATq, Rot8ATq/)
   INTEGER,  PARAMETER :: FlpDef(8)    = (/Flp1Def, Flp2Def, Flp3Def, Flp4Def, Flp5Def, Flp6Def, Flp7Def, Flp8Def/)  
   INTEGER,  PARAMETER :: DCMG2Bc(9)   = (/DCMG2Bc1, DCMG2Bc2, DCMG2Bc3, DCMG2Bc4, DCMG2Bc5, DCMG2Bc6, DCMG2Bc7, DCMG2Bc8, DCMG2Bc9/)  

   contains
   

!**********************************************************************************************************************************
! NOTE: The following lines of code were generated by a Matlab script called "Write_ChckOutLst.m"
!      using the parameters listed in the "OutListParameters.xlsx" Excel file. Any changes to these 
!      lines should be modified in the Matlab script and/or Excel worksheet as necessary. 
!----------------------------------------------------------------------------------------------------------------------------------
!> This routine checks to see if any requested output channel names (stored in the OutList(:)) are invalid. It returns a 
!! warning if any of the channels are not available outputs from the module.
!!  It assigns the settings for OutParam(:) (i.e, the index, name, and units of the output channels, WriteOutput(:)).
!!  the sign is set to 0 if the channel is invalid.
!! It sets assumes the value p%NumOuts has been set before this routine has been called, and it sets the values of p%OutParam here.
!! 
!! This routine was generated by Write_ChckOutLst.m using the parameters listed in OutListParameters.xlsx at 22-Feb-2018 08:48:34.
SUBROUTINE KFC_SetOutParam(OutList, p, ErrStat, ErrMsg )
!..................................................................................................................................

   IMPLICIT                        NONE

      ! Passed variables

   CHARACTER(ChanLen),        INTENT(IN)     :: OutList(:)                        !< The list out user-requested outputs
   TYPE(KFC_ParameterType),   INTENT(INOUT)  :: p                                 !< The module parameters
   INTEGER(IntKi),            INTENT(OUT)    :: ErrStat                           !< The error status code
   CHARACTER(*),              INTENT(OUT)    :: ErrMsg                            !< The error message, if an error occurred

      ! Local variables

   INTEGER                      :: ErrStat2                                        ! temporary (local) error status
   INTEGER                      :: i, j                                            ! Generic loop-counting index
   INTEGER                      :: INDX, iNd                                       ! Index for valid arrays

   LOGICAL                      :: CheckOutListAgain                               ! Flag used to determine if output parameter starting with "M" is valid (or the negative of another parameter)
   LOGICAL                      :: InvalidOutput(0:MaxOutPts)                      ! This array determines if the output channel is valid for this configuration
   CHARACTER(ChanLen)           :: OutListTmp                                      ! A string to temporarily hold OutList(I)
   CHARACTER(*), PARAMETER      :: RoutineName = "SetOutParam"

   CHARACTER(OutStrLenM1), PARAMETER  :: ValidParamAry(MaxOutPts) =  (/ &                  ! This lists the names of the allowed parameters, which must be sorted alphabetically
                               "DCMG2BC1 ","DCMG2BC2 ","DCMG2BC3 ", "DCMG2BC4 ","DCMG2BC5 ", "DCMG2BC6 ","DCMG2BC7 ","DCMG2BC8 ", "DCMG2BC9 ",&                              
                               "FLP1DEF  ","FLP2DEF  ","FLP3DEF  ", "FLP4DEF  ","FLP5DEF  ", "FLP6DEF  ","FLP7DEF  ","FLP8DEF  ", &                            
                               "ROT1ATQ  ","ROT1GTQ  ","ROT1SPD  ", "ROT2ATQ  ","ROT2GTQ  ", "ROT2SPD  ","ROT3ATQ  ","ROT3GTQ  ","ROT3SPD  ",&
                               "ROT4ATQ  ","ROT4GTQ  ","ROT4SPD  ", "ROT5ATQ  ","ROT5GTQ  ", "ROT5SPD  ","ROT6ATQ  ","ROT6GTQ  ","ROT6SPD  ",&
                               "ROT7ATQ  ","ROT7GTQ  ","ROT7SPD  ", "ROT8ATQ  ","ROT8GTQ  ", "ROT8SPD  ","TETHFXB  ","TETHFYB  ","TETHFZB  " /)
   INTEGER(IntKi), PARAMETER :: ParamIndxAry(MaxOutPts) =  (/ &                            ! This lists the index into AllOuts(:) of the allowed parameters ValidParamAry(:)
                                DCMG2Bc1, DCMG2Bc2, DCMG2Bc3, DCMG2Bc4, DCMG2Bc5, DCMG2Bc6, DCMG2Bc7, DCMG2Bc8, DCMG2Bc9, &                             
                                Flp1Def,    Flp2Def,    Flp3Def,  Flp4Def,   Flp5Def,  Flp6Def,  Flp7Def,  Flp8Def, &
                                Rot1ATq,    Rot1GTq ,   Rot1Spd , Rot2ATq ,  Rot2GTq , Rot2Spd , Rot3ATq , Rot3GTq ,  Rot3Spd , & 
                                Rot4ATq ,   Rot4GTq ,   Rot4Spd , Rot5ATq ,  Rot5GTq , Rot5Spd , Rot6ATq , Rot6GTq ,  Rot6Spd , &
                                Rot7ATq ,   Rot7GTq ,   Rot7Spd , Rot8ATq ,  Rot8GTq,  Rot8Spd , TethFxb , TethFyb ,  TethFzb   /)
   CHARACTER(ChanLen), PARAMETER :: ParamUnitsAry(MaxOutPts) =  (/ &                     ! This lists the units corresponding to the allowed parameters
                               "(-)       ","(-)       ","(-)       ","(-)       ","(-)       ","(-)       ","(-)       ","(-)       ","(-)       ",  &                            
                               "(Rad)     ","(Rad)     ","(Rad)     ","(Rad)     ","(Rad)     ","(Rad)     ","(Rad)     ","(Rad)     ", &
                               "(Nm)      ","(Nm)      ","(Rad/s)   ","(Nm)      ","(Nm)      ","(Rad/s)   ","(Nm)      ","(Nm)      ","(Rad/s)   ",&
                               "(Nm)      ","(Nm)      ","(Rad/s)   ","(Nm)      ","(Nm)      ","(Rad/s)   ","(Nm)      ","(Nm)      ","(Rad/s)   ",&
                               "(Nm)      ","(Nm)      ","(Rad/s)   ","(Nm)      ","(Nm)      ","(Rad/s)   ","(N)       ","(N)       ","(N)       " /)

      ! Initialize values
   ErrStat = ErrID_None
   ErrMsg = ""
   InvalidOutput = .FALSE.

   
!   ..... Developer must add checking for invalid inputs here: .....

   ! 1) Cannot request an output channel for which the user did not specify a corresponding location

      ! make sure we don't ask for outputs that don't exist:
   do i = p%NRotOuts+1, 4*p%numPylons-1
      iNd = p%RotOuts(i)
      if ( (iNd < 1) .or. ( iNd > 4*p%numPylons) )  then
          InvalidOutput( GenTq(i) ) =  .TRUE.
          InvalidOutput( RotATq(i) ) =  .TRUE.
          InvalidOutput( RotSpd(i) ) =  .TRUE.
      endif    
   end do   

   !!print *, ">>>>>RRD_debug: In ",RoutineName," p%NFlpOuts, p%NumOuts=",p%NFlpOuts, p%NumOuts
   do i = p%NFlpOuts+1, 2*p%numFlaps+2
      iNd = p%FlpOuts(i)
      if ( (iNd < 1) .or. ( iNd > 2*p%numFlaps+2) )  then
          InvalidOutput( FlpDef(i) ) =  .TRUE.
      endif    
   end do   
    
  
!   ................. End of validity checking .................


   !-------------------------------------------------------------------------------------------------
   ! Allocate and set index, name, and units for the output channels
   ! If a selected output channel is not available in this module, set error flag.
   !-------------------------------------------------------------------------------------------------
   
   ALLOCATE ( p%OutParam(0:p%NumOuts) , STAT=ErrStat2 )
   IF ( ErrStat2 /= 0_IntKi )  THEN
      CALL SetErrStat( ErrID_Fatal,"Error allocating memory for the KiteFastController OutParam array.", ErrStat, ErrMsg, RoutineName )
      RETURN
   ENDIF

      ! Set index, name, and units for the time output channel:

   p%OutParam(0)%Indx  = Time
   p%OutParam(0)%Name  = "Time"    ! OutParam(0) is the time channel by default.
   p%OutParam(0)%Units = "(s)"
   p%OutParam(0)%SignM = 1


      ! Set index, name, and units for all of the output channels.
      ! If a selected output channel is not available by this module set ErrStat = ErrID_Warn.

   DO I = 1,p%NumOuts

      p%OutParam(I)%Name  = OutList(I)
      OutListTmp          = OutList(I)

      ! Reverse the sign (+/-) of the output channel if the user prefixed the
      !   channel name with a "-", "_", "m", or "M" character indicating "minus".


      CheckOutListAgain = .FALSE.

      IF      ( INDEX( "-_", OutListTmp(1:1) ) > 0 ) THEN
         p%OutParam(I)%SignM = -1                         ! ex, "-TipDxc1" causes the sign of TipDxc1 to be switched.
         OutListTmp          = OutListTmp(2:)
      ELSE IF ( INDEX( "mM", OutListTmp(1:1) ) > 0 ) THEN ! We'll assume this is a variable name for now, (if not, we will check later if OutListTmp(2:) is also a variable name)
         CheckOutListAgain   = .TRUE.
         p%OutParam(I)%SignM = 1
      ELSE
         p%OutParam(I)%SignM = 1
      END IF

      CALL Conv2UC( OutListTmp )    ! Convert OutListTmp to upper case


      Indx = IndexCharAry( OutListTmp(1:OutStrLenM1), ValidParamAry )


         ! If it started with an "M" (CheckOutListAgain) we didn't find the value in our list (Indx < 1)

      IF ( CheckOutListAgain .AND. Indx < 1 ) THEN    ! Let's assume that "M" really meant "minus" and then test again
         p%OutParam(I)%SignM = -1                     ! ex, "MTipDxc1" causes the sign of TipDxc1 to be switched.
         OutListTmp          = OutListTmp(2:)

         Indx = IndexCharAry( OutListTmp(1:OutStrLenM1), ValidParamAry )
      END IF


      IF ( Indx > 0 ) THEN ! we found the channel name
         p%OutParam(I)%Indx     = ParamIndxAry(Indx)
         IF ( InvalidOutput( ParamIndxAry(Indx) ) ) THEN  ! but, it isn't valid for these settings
            p%OutParam(I)%Units = "INVALID"
            p%OutParam(I)%SignM = 0
         ELSE
            p%OutParam(I)%Units = ParamUnitsAry(Indx) ! it's a valid output
         END IF
      
      ELSE ! this channel isn't valid
         p%OutParam(I)%Indx  = Time                 ! pick any valid channel (I just picked "Time" here because it's universal)
         p%OutParam(I)%Units = "INVALID"
         p%OutParam(I)%SignM = 0                    ! multiply all results by zero

         CALL SetErrStat(ErrID_Fatal, TRIM(p%OutParam(I)%Name)//" is not an available output channel.",ErrStat,ErrMsg,RoutineName)
      END IF

   END DO

   RETURN
END SUBROUTINE KFC_SetOutParam
!----------------------------------------------------------------------------------------------------------------------------------
!End of code generated by Matlab script
!**********************************************************************************************************************************

end module KiteFastController_IO