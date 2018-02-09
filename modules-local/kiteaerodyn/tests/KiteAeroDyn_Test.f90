!**********************************************************************************************************************************
! KiteAeroDyn_Test: This code tests the KiteAeroDyn module
!..................................................................................................................................
! LICENSING
! Copyright (C) 2017  National Renewable Energy Laboratory
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

module KiteAeroDyn_Test
   
   use KiteAeroDyn_Types
   use KiteAeroDyn
   use NWTC_Library
   
   
   implicit none

   
contains
   
   subroutine KADTest_SetBaseInitInpData(InitInData, errStat, errMsg)
      type(KAD_InitInputType), intent(inout) :: InitInData      ! Input data for initialization
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'KADTest_SetBaseInitInpData'
         ! Initialize error handling variables
      errMsg  = ''
      errStat = ErrID_None
      
      ! --- KiteAeroDyn INPUT FILE DATA ---

      InitInData%FileName    = "KiteAeroDyn.inp"   ! not using an input file for KiteAeroDyn module
      InitInData%NumFlaps    = 1      ! Number of flaps per wing
      InitInData%NumPylons   = 2     ! Number of pylons per wing

      !       ---   ---            ---   ---           !< ---   = rotors
      !    |---*-----*------||------*-----*---|        !< *     = pylons
      !    |---|XXXXX|------||------|XXXXX|---|        !< |XXX| = flap
      !                     ||                         !< ||    = fuselage
      !                     ||
      !                     ||
      !                     ||
      !            ~~~~~~~~~XX~~~~~~~~~                !< XX    = vertical stabilizer
      !                                                !< ~~~   = horizontal stabilizer
      !
      !
         ! Kite Origin Data
      
         ! allocate arrays
   
      call AllocAry( InitInData%SPyOR, 3, 2, 'SPyOR', ErrStat2, ErrMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( InitInData%PPyOR, 3, 2, 'PPyOR', ErrStat2, ErrMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( InitInData%SPyRtrOR, 3, 2, 2, 'SPyRtrOR', ErrStat2, ErrMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( InitInData%PPyRtrOR, 3, 2, 2, 'PPyRtrOR', ErrStat2, ErrMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
     
      InitInData%SWnOR           = (/0.0_ReKi, 0.0_ReKi, 0.0_ReKi/)
      InitInData%PWnOR           = (/0.0_ReKi, 0.0_ReKi, 0.0_ReKi/)
      InitInData%VSOR           = (/3.0_ReKi, 0.0_ReKi, 0.0_ReKi/)
      InitInData%SHSOR           = (/3.0_ReKi, 0.0_ReKi, 0.0_ReKi/)
      InitInData%PHSOR           = (/3.0_ReKi, 0.0_ReKi, 0.0_ReKi/)
      InitInData%SPyOR(:,1)      = (/0.0_ReKi, 2.0_ReKi, 0.0_ReKi/)
      InitInData%SPyOR(:,2)      = (/0.0_ReKi, 5.0_ReKi, 0.0_ReKi/)
      InitInData%PPyOR(:,1)      = (/0.0_ReKi,-2.0_ReKi, 0.0_ReKi/)
      InitInData%PPyOR(:,2)      = (/0.0_ReKi,-5.0_ReKi, 0.0_ReKi/)
      InitInData%SPyRtrOR(:,1,1) = (/0.0_ReKi, 2.0_ReKi, 2.0_ReKi/)
      InitInData%SPyRtrOR(:,2,1) = (/0.0_ReKi, 2.0_ReKi,-2.0_ReKi/)
      InitInData%SPyRtrOR(:,1,2) = (/0.0_ReKi, 5.0_ReKi, 2.0_ReKi/)
      InitInData%SPyRtrOR(:,2,2) = (/0.0_ReKi, 5.0_ReKi,-2.0_ReKi/)
      InitInData%PPyRtrOR(:,1,1) = (/0.0_ReKi,-2.0_ReKi, 2.0_ReKi/)
      InitInData%PPyRtrOR(:,2,1) = (/0.0_ReKi,-2.0_ReKi,-2.0_ReKi/)
      InitInData%PPyRtrOR(:,1,2) = (/0.0_ReKi,-5.0_ReKi, 2.0_ReKi/)
      InitInData%PPyRtrOR(:,2,2) = (/0.0_ReKi,-5.0_ReKi,-2.0_ReKi/)


   !   InitInData%HeaderLn   = 'Sample KiteAeroDyn input file data'
   !   
   !   ! --- SIMULATION CONTROL ---
   !   
   !   InitInData%DTAeroStr  = ''
   !   InitInData%DTAero     = dvrInitInp%DTAero
   !   InitInData%LiftMod    = 1
   !   InitInData%RotorMod   = 0
   !   InitInData%UseCm      = .false.
   !   
   !   ! --- ENVIRONMENTAL CONDITIONS ---
   !   
   !   InitInData%AirDens    = 1.225_ReKi
   !   InitInData%KinVisc    = 1.464E-05_ReKi
   !   
   !   ! --- AIRFOIL INFORMATION ---
   !   
   !   InitInData%AFTabMod   = 3
   !   InitInData%InCol_Alfa = 1
   !   InitInData%InCol_Cl   = 2
   !   InitInData%InCol_Cd   = 3
   !   InitInData%InCol_Cm   = 4
   !   InitInData%NumAFfiles = 1
   !   InitInData%AFNames = ''  ! not set because we are populating the airfoil data here without reading a file
   !   
   !      ! A generic cylinder
   !   InitInData%AFData(1)%NumTable = 1
   !   InitInData%AFData(1)%alpha0    = 0.0_ReKi*D2R
   !   InitInData%AFData(1)%alpha1    = 0.0_ReKi*D2R
   !   InitInData%AFData(1)%alpha2    = 0.0_ReKi*D2R
   !   InitInData%AFData(1)%eta_e     = 0.0_ReKi
   !   InitInData%AFData(1)%C_nalpha  = 0.0_ReKi
   !   InitInData%AFData(1)%T_f0      = 3_ReKi
   !   InitInData%AFData(1)%T_V0      = 6_ReKi
   !   InitInData%AFData(1)%T_p       = 1.7_ReKi
   !   InitInData%AFData(1)%T_VL      = 11_ReKi
   !   InitInData%AFData(1)%b1        = 0.14_ReKi
   !   InitInData%AFData(1)%b2        = 0.53_ReKi
   !   InitInData%AFData(1)%b5        = 5_ReKi
   !   InitInData%AFData(1)%A1        = 0.3_ReKi
   !   InitInData%AFData(1)%A2        = 0.7_ReKi
   !   InitInData%AFData(1)%A5        = 1.0_ReKi
   !   InitInData%AFData(1)%S1        = 0.0_ReKi
   !   InitInData%AFData(1)%S2        = 0.0_ReKi
   !   InitInData%AFData(1)%S3        = 0.0_ReKi
   !   InitInData%AFData(1)%S4        = 0.0_ReKi
   !   InitInData%AFData(1)%Cn1       = 0.0_ReKi
   !   InitInData%AFData(1)%Cn2       = 0.0_ReKi
   !   InitInData%AFData(1)%St_sh     = 0.19_ReKi
   !   InitInData%AFData(1)%Cd0       = 0.5_ReKi
   !   InitInData%AFData(1)%Cm0       = 0.0_ReKi
   !   InitInData%AFData(1)%k0        = 0.0_ReKi
   !   InitInData%AFData(1)%k1        = 0.0_ReKi
   !   InitInData%AFData(1)%k2        = 0.0_ReKi
   !   InitInData%AFData(1)%k3        = 0.0_ReKi
   !   InitInData%AFData(1)%k1_hat    = 0.0_ReKi
   !   InitInData%AFData(1)%x_cp_bar  = 0.2_ReKi
   !   InitInData%AFData(1)%UACutout  = 45.0_ReKi*D2R
   !   InitInData%AFData(1)%filtCutOff= 20.0_ReKi
   !   InitInData%AFData(1)%NumAlf    = 3
   !   InitInData%AFData(1)%Alpha     = (/-180.0_ReKi, 0.0_ReKi, 180.0_ReKi/)
   !   InitInData%AFData(1)%Cl        = (/   0.0_ReKi, 0.0_ReKi,   0.0_ReKi/)
   !   InitInData%AFData(1)%Cd        = (/   0.0_ReKi, 0.0_ReKi,   0.0_ReKi/)
   !   InitInData%AFData(1)%Cm        = (/   0.0_ReKi, 0.0_ReKi,   0.0_ReKi/)
   !
   !   
   !   ! --- FUSELAGE PROPERTIES ---
   !
   !   InitInData%FuseProps%NumNds = 9
   !   
   !      ! allocate the data based on NumNds
   !   
   !   allocate(  InitInData%FuseProps%Pos(InitInData%FuseProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%FuseProps%Twist(InitInData%FuseProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%FuseProps%Chord(InitInData%FuseProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%FuseProps%AFID(InitInData%FuseProps%NumNds), STAT = errStat )
   !   
   !   InitInData%FuseProps%Pos(:,1) = (/ -5.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%FuseProps%Pos(:,2) = (/ -4.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%FuseProps%Pos(:,3) = (/ -3.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%FuseProps%Pos(:,4) = (/ -2.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%FuseProps%Pos(:,5) = (/ -1.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%FuseProps%Pos(:,6) = (/  0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%FuseProps%Pos(:,7) = (/  1.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%FuseProps%Pos(:,8) = (/  2.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%FuseProps%Pos(:,9) = (/  3.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !
   !   
   !   InitInData%FuseProps%Twist  (:) =  0.0_ReKi
   !   InitInData%FuseProps%Chord  (:) =  0.3_ReKi
   !   InitInData%FuseProps%AFID   (:) =  1         ! cylinder
   !
   !   
   !   
   !    ! --- STARBOARD WING PROPERTIES ---
   !
   !   InitInData%SWnProps%NumNds = 8
   !   
   !      ! allocate the data based on NumNds
   !   
   !   allocate(  InitInData%SWnProps%Pos(InitInData%SWnProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%SWnProps%Twist(InitInData%SWnProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%SWnProps%Chord(InitInData%SWnProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%SWnProps%AFID(InitInData%SWnProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%SWnProps%CntrlID(InitInData%SWnProps%NumNds), STAT = errStat )
   !   
   !   InitInData%SWnProps%Pos(:,1) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%SWnProps%Pos(:,2) = (/ 0.0_ReKi, 1.0_ReKi, 0.0_ReKi /)
   !   InitInData%SWnProps%Pos(:,3) = (/ 0.0_ReKi, 2.0_ReKi, 0.0_ReKi /)
   !   InitInData%SWnProps%Pos(:,4) = (/ 0.0_ReKi, 3.0_ReKi, 0.0_ReKi /)
   !   InitInData%SWnProps%Pos(:,5) = (/ 0.0_ReKi, 4.0_ReKi, 0.0_ReKi /)
   !   InitInData%SWnProps%Pos(:,6) = (/ 0.0_ReKi, 5.0_ReKi, 0.0_ReKi /)
   !   InitInData%SWnProps%Pos(:,7) = (/ 0.0_ReKi, 6.0_ReKi, 0.0_ReKi /)
   !   InitInData%SWnProps%Pos(:,8) = (/ 0.0_ReKi, 7.0_ReKi, 0.0_ReKi /)
   !   
   !   !InitInData%SWnProps%Twist  (1) =  
   !   !InitInData%SWnProps%Twist  (2) =  
   !   !InitInData%SWnProps%Twist  (3) =  
   !   !InitInData%SWnProps%Chord  (1) =   
   !   !InitInData%SWnProps%Chord  (2) =   
   !   !InitInData%SWnProps%Chord  (3) =   
   !   !InitInData%SWnProps%AFID   (1) =  
   !   !InitInData%SWnProps%AFID   (2) = 
   !   !InitInData%SWnProps%AFID   (3) =  
   !   !InitInData%SWnProps%CntrlID(1) = 
   !   !InitInData%SWnProps%CntrlID(2) = 
   !   !InitInData%SWnProps%CntrlID(3) = 
   !
   !   
   !    ! --- PORT WING PROPERTIES ---
   !
   !   InitInData%PWnProps%NumNds = 3
   !   
   !      ! allocate the data based on NumNds
   !   
   !   allocate(  InitInData%PWnProps%Pos(InitInData%PWnProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PWnProps%Twist(InitInData%PWnProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PWnProps%Chord(InitInData%PWnProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PWnProps%AFID(InitInData%PWnProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PWnProps%CntrlID(InitInData%PWnProps%NumNds), STAT = errStat )
   !   
   !   InitInData%PWnProps%Pos(:,1) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%PWnProps%Pos(:,2) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%PWnProps%Pos(:,3) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   
   !   !InitInData%PWnProps%Twist  (1) =  
   !   !InitInData%PWnProps%Twist  (2) =  
   !   !InitInData%PWnProps%Twist  (3) =  
   !   !InitInData%PWnProps%Chord  (1) =   
   !   !InitInData%PWnProps%Chord  (2) =   
   !   !InitInData%PWnProps%Chord  (3) =   
   !   !InitInData%PWnProps%AFID   (1) =  
   !   !InitInData%PWnProps%AFID   (2) = 
   !   !InitInData%PWnProps%AFID   (3) =  
   !   !InitInData%PWnProps%CntrlID(1) = 
   !   !InitInData%PWnProps%CntrlID(2) = 
   !   !InitInData%PWnProps%CntrlID(3) = 
   !
   !   
   !   
   !   ! --- VERTICAL STABILIZER PROPERTIES ---
   !
   !   InitInData%VSProps%NumNds = 3
   !   
   !      ! allocate the data based on NumNds
   !   
   !   allocate(  InitInData%VSProps%Pos(InitInData%VSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%VSProps%Twist(InitInData%VSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%VSProps%Chord(InitInData%VSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%VSProps%AFID(InitInData%VSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%VSProps%CntrlID(InitInData%VSProps%NumNds), STAT = errStat )
   !   
   !   InitInData%VSProps%Pos(:,1) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%VSProps%Pos(:,2) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%VSProps%Pos(:,3) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   
   !   !InitInData%VSProps%Twist  (1) =  
   !   !InitInData%VSProps%Twist  (2) =  
   !   !InitInData%VSProps%Twist  (3) =  
   !   !InitInData%VSProps%Chord  (1) =   
   !   !InitInData%VSProps%Chord  (2) =   
   !   !InitInData%VSProps%Chord  (3) =   
   !   !InitInData%VSProps%AFID   (1) =  
   !   !InitInData%VSProps%AFID   (2) = 
   !   !InitInData%VSProps%AFID   (3) =  
   !   !InitInData%VSProps%CntrlID(1) = 
   !   !InitInData%VSProps%CntrlID(2) = 
   !   !InitInData%VSProps%CntrlID(3) = 
   !   !
   !   
   !   ! --- STARBOARD HORIZONTAL STABILIER PROPERTIES ---
   !
   !   InitInData%SHSProps%NumNds = 3
   !   
   !      ! allocate the data based on NumNds
   !   
   !   allocate(  InitInData%SHSProps%Pos(InitInData%SHSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%SHSProps%Twist(InitInData%SHSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%SHSProps%Chord(InitInData%SHSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%SHSProps%AFID(InitInData%SHSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%SHSProps%CntrlID(InitInData%SHSProps%NumNds), STAT = errStat )
   !   
   !   InitInData%SHSProps%Pos(:,1) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%SHSProps%Pos(:,2) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%SHSProps%Pos(:,3) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   
   !   !InitInData%SHSProps%Twist  (1) =  
   !   !InitInData%SHSProps%Twist  (2) =  
   !   !InitInData%SHSProps%Twist  (3) =  
   !   !InitInData%SHSProps%Chord  (1) =   
   !   !InitInData%SHSProps%Chord  (2) =   
   !   !InitInData%SHSProps%Chord  (3) =   
   !   !InitInData%SHSProps%AFID   (1) =  
   !   !InitInData%SHSProps%AFID   (2) = 
   !   !InitInData%SHSProps%AFID   (3) =  
   !   !InitInData%SHSProps%CntrlID(1) = 
   !   !InitInData%SHSProps%CntrlID(2) = 
   !   !InitInData%SHSProps%CntrlID(3) = 
   !
   !   
   !   
   !   ! --- PORT HORIZONTAL STABILIZER PROPERTIES ---
   !
   !   InitInData%PHSProps%NumNds = 3
   !   
   !      ! allocate the data based on NumNds
   !   
   !   allocate(  InitInData%PHSProps%Pos(InitInData%PHSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PHSProps%Twist(InitInData%PHSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PHSProps%Chord(InitInData%PHSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PHSProps%AFID(InitInData%PHSProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PHSProps%CntrlID(InitInData%PHSProps%NumNds), STAT = errStat )
   !   
   !   InitInData%PHSProps%Pos(:,1) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%PHSProps%Pos(:,2) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%PHSProps%Pos(:,3) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   
   !   !InitInData%PHSProps%Twist  (1) =  
   !   !InitInData%PHSProps%Twist  (2) =  
   !   !InitInData%PHSProps%Twist  (3) =  
   !   !InitInData%PHSProps%Chord  (1) =   
   !   !InitInData%PHSProps%Chord  (2) =   
   !   !InitInData%PHSProps%Chord  (3) =   
   !   !InitInData%PHSProps%AFID   (1) =  
   !   !InitInData%PHSProps%AFID   (2) = 
   !   !InitInData%PHSProps%AFID   (3) =  
   !   !InitInData%PHSProps%CntrlID(1) = 
   !   !InitInData%PHSProps%CntrlID(2) = 
   !   !InitInData%PHSProps%CntrlID(3) = 
   !
   !
   !   
   !   ! --- PYLON PROPERTIES ---
   !
   !   InitInData%PylProps%NumNds = 3
   !   
   !      ! allocate the data based on NumNds
   !   
   !   allocate(  InitInData%PylProps%Pos(InitInData%PylProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PylProps%Twist(InitInData%PylProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PylProps%Chord(InitInData%PylProps%NumNds), STAT = errStat )
   !   allocate(  InitInData%PylProps%AFID(InitInData%PylProps%NumNds), STAT = errStat )
   !   
   !   InitInData%PylProps%Pos(:,1) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%PylProps%Pos(:,2) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   InitInData%PylProps%Pos(:,3) = (/ 0.0_ReKi, 0.0_ReKi, 0.0_ReKi /)
   !   
   !   !InitInData%PylProps%Twist  (1) =  
   !   !InitInData%PylProps%Twist  (2) =  
   !   !InitInData%PylProps%Twist  (3) =  
   !   !InitInData%PylProps%Chord  (1) =   
   !   !InitInData%PylProps%Chord  (2) =   
   !   !InitInData%PylProps%Chord  (3) =   
   !   !InitInData%PylProps%AFID   (1) =  
   !   !InitInData%PylProps%AFID   (2) = 
   !   !InitInData%PylProps%AFID   (3) =  
   !   !
   !
   !   
   !   ! --- ROTOR PROPERTIES --- [used only when RotMod=1]
   !   
   !   !InitInData%TSRs     = (/3.0_ReKi, 15.0_ReKi/)
   !   !InitInData%Skews     = (/0.0_ReKi, 0.3_ReKi, 0.5_ReKi/)
   !   !
   !   !
   !   !InitInData%TSR_Skew_Table = RESHAPE( (/ &  !  C_Fx       C_Fy       C_Fz       C_Mx   C_My     C_Mz     C_P
   !   !                                           1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
   !   !                                           1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
   !   !                                           1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
   !   !                                           1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
   !   !                                           1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi, &
   !   !                                           1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi,1.0_ReKi &
   !   !                                      /), (/7, 2, 3/) )
   !   
end subroutine KADTest_SetBaseInitInpData
   !
   
   !subroutine KADTest_ReadFile(errStat, errMsg)
   !
   !   integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
   !   character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
   !   type(KAD_InitInputType)                :: InitInData           ! Input data for initialization
   !   type(KAD_InitOutputType)               :: InitOutData          ! Output data from initialization
   !   type(KAD_ParameterType)                :: p                    ! Parameters
   !   type(KAD_InputType)                    :: u                    ! System inputs
   !   type(KAD_OutputType)                   :: y                    ! System outputs      
   !   integer(IntKi)                            :: errStat2        ! Status of error message
   !   character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
   !   character(*), parameter                   :: routineName = 'KADTest_Basic'
   !   character(1024)                           :: tmpValues
   !   real(DbKi)                                :: interval
   !   
   !      ! Initialize error handling variables
   !   errMsg  = ''
   !   errStat = ErrID_None
   !
   !   interval            = 1.0_DbKi 
   !   InitInData%R        = 2.0_ReKi
   !   InitInData%AirDens  = 98.0_ReKi
   !   InitInData%Filename = '..\..\reg_tests\r-test\modules-local\actuatordisk\basic_fileread\KiteAeroDyn.dat'
   !   
   !      ! Initialize the KiteAeroDyn module
   !   call KAD_Init( InitInData, u, p, y, interval, InitOutData, errStat, errMsg )
   !      if ( errStat >= AbortErrLev ) then
   !         call Cleanup()
   !         stop
   !      end if
   !      
   !      ! Set Inputs
   !   u%DiskAve_Vx_Rel  = 10.0_ReKi
   !   u%omega   = 40.0_ReKi
   !   u%skew = 0.0_ReKi
   !   
   !      ! Obtain outputs from KiteAeroDyn module
   !   call KAD_CalcOutput( u, p, y, errStat, errMsg )
   !      if ( errStat >= AbortErrLev ) then
   !         call Cleanup()
   !         stop
   !      end if
   !
   !      ! Print the results to a log
   !   call WrScr('      Fx      ,      Fy      ,      Fz      ,      Mx      ,      My      ,      Mz      ,      P      ')   
   !   write(tmpValues,*) y%Fx, y%Fy, y%Fz, y%Mx, y%My, y%Mz, y%P  
   !   call WrScr(TRIM(tmpValues))
   !   return
   !   
   !   contains
   !
   !      !====================================================================================================
   !      subroutine Cleanup()
   !      !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
   !      !     any existing echo information
   !      !----------------------------------------------------------------------------------------------------  
   !   
   !         if ( errStat /= ErrID_None ) print *, errMsg       
   !   
   !      end subroutine Cleanup
   !end subroutine KADTest_ReadFile
   !
   subroutine KADTest_Basic(errStat, errMsg)
   
      integer(IntKi)            , intent(  out) :: errStat         ! Status of error message
      character(1024)           , intent(  out) :: errMsg          ! Error message if errStat /= ErrID_None
      type(KAD_InitInputType)                :: InitInData           ! Input data for initialization
      type(KAD_InitOutputType)               :: InitOutData          ! Output data from initialization
      type(KAD_ParameterType)                :: p                    ! Parameters
      type(KAD_InputType)                    :: u                    ! System inputs
      type(KAD_OutputType)                   :: y                    ! System outputs     
      type(KAD_MiscVarType)                  :: m                    ! miscvars
      integer(IntKi)                            :: errStat2        ! Status of error message
      character(1024)                           :: errMsg2         ! Error message if errStat /= ErrID_None
      character(*), parameter                   :: routineName = 'KADTest_Basic'
      real(DbKi)                                :: interval
      
         ! Initialize error handling variables
      errMsg  = ''
      errStat = ErrID_None
   
      interval            = 1.0_DbKi 
   
      call KADTest_SetBaseInitInpData(InitInData, errStat, errMsg)
        if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
        end if
      
         ! Initialize the KiteAeroDyn module
      call KAD_Init( InitInData, u, p, y, interval, m, InitOutData, errStat, errMsg )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            stop
         end if
         
         ! Set Inputs
    
      
         ! Obtain outputs from KiteAeroDyn module
      !call KAD_CalcOutput( u, p, y, errStat, errMsg )
      !   if ( errStat >= AbortErrLev ) then
      !      call Cleanup()
      !      stop
      !   end if
   
         ! Print the results to a log
      !call WrScr('      Fx      ,      Fy      ,      Fz      ,      Mx      ,      My      ,      Mz      ,      P      ')   
      !write(tmpValues,*) y%Fx, y%Fy, y%Fz, y%Mx, y%My, y%Mz, y%P  
      !call WrScr(TRIM(tmpValues))
      return
      
      contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
            if ( ErrStat /= ErrID_None ) print *, ErrMsg       
      
         end subroutine Cleanup
   end subroutine KADTest_Basic
   
end module KiteAeroDyn_Test