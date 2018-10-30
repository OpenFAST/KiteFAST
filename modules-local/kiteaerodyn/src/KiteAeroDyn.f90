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
!> This module receives kite-related motions as inputs and then computes associate aerodynamic loads as output.
!> The aerodynamic loads are computed using a Vortex Step Method (VSM) and an Actuator Disk Method.
module KiteAeroDyn

   
   use NWTC_Library   
   use KiteAeroDyn_Types
   use KiteAeroDyn_IO
   use ActuatorDisk
   use VSM
   
   implicit none 

private

   type(ProgDesc), parameter  :: KAD_Ver = ProgDesc( 'KiteAeroDyn', '', '' )

   public :: KAD_Init
   public :: KAD_CalcOutput
   public :: KAD_UpdateStates
   public :: KAD_End


   contains
   
!> Routine to compute the rotor inputs.
subroutine RotorDisk_SetInputs(c_offset, numPylons, V_PyRtr, PyRtrMotions, RtSpds, pitches, u_ActDsk, dcm, errStat, errMsg)
   integer(IntKi),             intent(in   ) :: c_offset          !< Offset into the 1D ActDsk array, either 0 (starboard wing) or numPylons (port wing)
   integer(IntKi),             intent(in   ) :: numPylons         !< Number of pylons
   real(ReKi),                 intent(in   ) :: V_PyRtr(:,:,:)    !< Undisturbed wind velocities at the rotors (1st index: u,v,w, 2nd index: 1=top, 2=bottom, 3rd index: 1,NumPylons)
   type(MeshType),             intent(in   ) :: PyRtrMotions(:)   !< Rotor point meshes
   real(ReKi),                 intent(in   ) :: RtSpds(:,:)       !< Rotor speeds in rad/s for the wing's pylons (1st index: 1=top, 2=bottom, 2nd index: 1,NumPylons)
   real(ReKi),                 intent(in   ) :: pitches(:,:)      !< Rotor pitches in rad for the wing's pylons (1st index: 1=top, 2=bottom, 2nd index: 1,NumPylons)
   type(ActDsk_InputType),     intent(inout) :: u_ActDsk(:)       !< Actuator Disk inputs, starboard wing first, followed by port wing
   real(ReKi),                 intent(inout) :: dcm(:,:,:)        !< direct cosine matrix for transforming from global coordinates into the disk local coordinates
   integer(IntKi),             intent(  out) :: errStat           !< Error status of the operation
   character(*),               intent(  out) :: errMsg            !< Error message if errStat /= ErrID_None
   
      ! Local Variables
   integer(IntKi)                         :: errStat2          ! Error status of the operation (secondary error)
   character(ErrMsgLen)                   :: errMsg2           ! Error message if errStat2 /= ErrID_None
   integer(IntKi)                         :: i,j,c, index      ! counters
   real(ReKi)                             :: Vrel(3)           ! Relative velocity
   real(ReKi)                             :: V_dot_x           ! Dot product of V and the local x-axis unit vector
   real(ReKi)                             :: x_hat_disk(3)     ! Unit vector normal to the rotor disk
   real(ReKi)                             :: y_hat_disk(3)     ! Unit vector in the plane of the rotor disk
   real(ReKi)                             :: z_hat_disk(3)     ! Unit vector in the plane of the rotor disk
   real(ReKi)                             :: tmp(3)            ! temporary vector
   
   real(ReKi)                             :: tmp_sz, tmp_sz_y  ! temporary quantities for calculations
   real(ReKi)                             :: forces(3), moments(3)
   character(*), parameter                :: routineName = 'RotorDisk_SetInputs'

   errStat   = ErrID_None           ! no error has occurred
   errMsg    = ""

   c=c_offset
   
   do j=1,numPylons
      do i=1,2
         index = i + (j-1)*numPylons
         c = c+1
            
            ! Relative averaged motion of the rotor hub
         Vrel = V_PyRtr(:,i,j) - PyRtrMotions(index)%TranslationVel(:,1)
            
            ! orientation vector along rotor disk, x-axis:
         x_hat_disk = PyRtrMotions(index)%Orientation(1,:,1)      
            
            ! Magnitude of the average motion along  x-axis        
         V_dot_x  = dot_product( Vrel, x_hat_disk )
         tmp    = V_dot_x * x_hat_disk - Vrel
         tmp_sz = TwoNorm(tmp)
         if ( EqualRealNos( tmp_sz, 0.0_ReKi ) ) then
            y_hat_disk = PyRtrMotions(index)%Orientation(2,:,1)
            z_hat_disk = PyRtrMotions(index)%Orientation(3,:,1)
         else
            y_hat_disk = tmp / tmp_sz
            z_hat_disk = cross_product( Vrel, x_hat_disk ) / tmp_sz
         end if
         
            ! "Angle between the negative vector normal to the rotor plane and the wind vector (e.g., the yaw angle in the case of no tilt)" rad 
         tmp_sz = TwoNorm( Vrel )
         if ( EqualRealNos( tmp_sz, 0.0_ReKi ) ) then
            u_ActDsk(c)%skew = 0.0_ReKi
         else
               ! make sure we don't have numerical issues that make the ratio outside +/-1
            tmp_sz_y = min(  1.0_ReKi, V_dot_x / tmp_sz )
            tmp_sz_y = max( -1.0_ReKi, tmp_sz_y )
      
            u_ActDsk(c)%skew = acos( tmp_sz_y )
      
         end if         
         
         u_ActDsk(c)%DiskAve_Vrel = tmp_sz
         u_ActDsk(c)%RtSpd = RtSpds(i,j)
         u_ActDsk(c)%pitch = pitches(i,j)
         dcm(1,:,c) = x_hat_disk
         dcm(2,:,c) = y_hat_disk
         dcm(3,:,c) = z_hat_disk
      end do
   end do

end subroutine RotorDisk_SetInputs
!> Routine to compute the rotor loads using a Actuator Disk Method on a per wing basis.
subroutine RotorDisk_CalcOutput(c_offset, numPylons, V_PyRtr, PyRtrMotions, RtSpds, pitches, dcms, u_ActDsk, p_ActDsk, m_ActDsk, y_ActDsk, PyRtrLoads, errStat, errMsg)
   integer(IntKi),             intent(in   ) :: c_offset          !< Offset into the 1D ActDsk array, either 0 (starboard wing) or numPylons (port wing)
   integer(IntKi),             intent(in   ) :: numPylons         !< Number of pylons
   real(ReKi),                 intent(in   ) :: V_PyRtr(:,:,:)    !< Undisturbed wind velocities at the rotors (1st index: u,v,w, 2nd index: 1=top, 2=bottom, 3rd index: 1,NumPylons)
   type(MeshType),             intent(in   ) :: PyRtrMotions(:)   !< Rotor point meshes
   real(ReKi),                 intent(in   ) :: RtSpds(:,:)       !< Rotor speeds in rad/s for the wing's pylons (1st index: 1=top, 2=bottom, 2nd index: 1,NumPylons)
   real(ReKi),                 intent(in   ) :: pitches(:,:)      !< Rotor pitches in rad for the wing's pylons (1st index: 1=top, 2=bottom, 2nd index: 1,NumPylons)
   real(ReKi),                 intent(in   ) :: dcms(:,:,:)       !< direct cosine matrix for transforming from global coordinates into the disk local coordinates
   type(ActDsk_InputType),     intent(in   ) :: u_ActDsk(:)       !< Actuator Disk inputs, starboard wing first, followed by port wing
   type(ActDsk_ParameterType), intent(in   ) :: p_ActDsk(:)       !< Actuator Disk parameters
   type(ActDsk_MiscVarType),   intent(inout) :: m_ActDsk(:)       !< Actuator Disk miscvars
   type(ActDsk_OutputType),    intent(inout) :: y_ActDsk(:)       !< Actuator Disk outputs
   type(MeshType),             intent(inout) :: PyRtrLoads(:)     !< Rotor loads meshes
   integer(IntKi),             intent(  out) :: errStat           !< Error status of the operation
   character(*),               intent(  out) :: errMsg            !< Error message if errStat /= ErrID_None
   
      ! Local Variables
   integer(IntKi)                         :: errStat2          ! Error status of the operation (secondary error)
   character(ErrMsgLen)                   :: errMsg2           ! Error message if errStat2 /= ErrID_None
   integer(IntKi)                         :: i,j,c, index      ! counters
   real(ReKi)                             :: Vrel(3)           ! Relative velocity
   real(ReKi)                             :: V_dot_x           ! Dot product of V and the local x-axis unit vector
   real(ReKi)                             :: x_hat_disk(3)     ! Unit vector normal to the rotor disk
   real(ReKi)                             :: y_hat_disk(3)     ! Unit vector in the plane of the rotor disk
   real(ReKi)                             :: z_hat_disk(3)     ! Unit vector in the plane of the rotor disk
   real(ReKi)                             :: tmp(3)            ! temporary vector
   real(ReKi)                             :: dcm(3,3)          ! direct cosine matrix
   real(ReKi)                             :: tmp_sz, tmp_sz_y  ! temporary quantities for calculations
   real(ReKi)                             :: forces(3), moments(3)
   character(*), parameter                :: routineName = 'RotorDisk_CalcOutput'

   errStat   = ErrID_None           ! no error has occurred
   errMsg    = ""

   c=c_offset
   
   do j=1,numPylons
      do i=1,2
         index = i + (j-1)*numPylons
         c = c+1
            
            ! Compute the outputs from the Actuator disk model for a particular rotor
         call ActDsk_CalcOutput(u_ActDsk(c), p_ActDsk(c), m_ActDsk(c), y_ActDsk(c), errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
         !dcm(1,:) = x_hat_disk
         !dcm(2,:) = y_hat_disk
         !dcm(3,:) = z_hat_disk
            
            ! dcm is transform from global to local, but we need local to global so transpose
         dcm = transpose(dcms(:,:,c))
         forces  = (/y_ActDsk(c)%Fx,y_ActDsk(c)%Fy,y_ActDsk(c)%Fz/)
         moments = (/y_ActDsk(c)%Mx,y_ActDsk(c)%My,y_ActDsk(c)%Mz/)
         
            ! Transform loads from disk to global
         forces = matmul(dcm, forces)
         moments = matmul (dcm, moments)
         PyRtrLoads(index)%Force(:,1)  = forces
         PyRtrLoads(index)%Moment(:,1) = moments
         
      end do
   end do

end subroutine RotorDisk_CalcOutput
   
!> Routine to parse the pylon component properties.
subroutine ReadPyProps( UnIn, fileName, numPylons, SPyProps, PPyProps, errStat, errMsg, UnEc )
   integer(IntKi),                     intent(in   )  :: UnIn        !< Input file unit number
   character(*),                       intent(in   )  :: filename    !< Input file name
   integer(IntKi),                     intent(in   )  :: numPylons   !< Number of pylons per wing
   type(KAD_MbrPropsType),             intent(inout)  :: SPyProps(:) !< Starboard pylon(s) data
   type(KAD_MbrPropsType),             intent(inout)  :: PPyProps(:) !< Port pylon(s) data
   integer(IntKi),                     intent(  out)  :: errStat     !< Error status of the operation
   character(*),                       intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None
   integer(IntKi),                     intent(in   )  :: UnEc        !< Echo file unit
   
   integer(intKi)                               :: i,j               ! counters
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   integer(intKi)                               :: numNodes          ! number of nodes per pylon component
   real(ReKi), allocatable                      :: vals(:)           ! a row of property data in the input file
   character(*), parameter                      :: RoutineName = 'ReadPyProps'
   
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""
   
      ! Skip the comment line and table headers.
   call ReadCom( UnIn, fileName, ' PYLON PROPERTIES ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
  

       ! Number of nodes used in the analysis
   call ReadVar ( UnIn, fileName, numNodes, 'NumNodes', 'Number of Pylon nodes used in the analysis', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   !allocate( SPyProps(numPylons), STAT = errStat2 )
   !   if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for SPyProps', errStat, errMsg, RoutineName )     
   !allocate( PPyProps(numPylons), STAT = errStat2 )
   !   if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for PPyProps', errStat, errMsg, RoutineName )     
   
   do i=1,numPylons   
      call AllocAry( SPyProps(i)%Pos, 3_IntKi, numNodes, 'SPyProps(i)%Pos', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( SPyProps(i)%Twist, numNodes, 'SPyProps(i)%Twist', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( SPyProps(i)%Chord, numNodes, 'SPyProps(i)%Chord', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( SPyProps(i)%AFID, numNodes, 'SPyProps(i)%AFID', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      call AllocAry( PPyProps(i)%Pos, 3_IntKi, numNodes, 'PPyProps(i)%Pos', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( PPyProps(i)%Twist, numNodes, 'PPyProps(i)%Twist', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( PPyProps(i)%Chord, numNodes, 'PPyProps(i)%Chord', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( PPyProps(i)%AFID, numNodes, 'PPyProps(i)%AFID', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   end do
   
   call AllocAry( vals, 6, 'vals', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    
   
   if (errStat >= AbortErrLev) return
   
      ! Skip the table header line2.
   call ReadCom( UnIn, fileName, ' Pylon property table header line 1 ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call ReadCom( UnIn, fileName, ' Pylon property table header line 2 ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   do j=1, numPylons
      SPyProps(j)%NumNds = numNodes
      do i=1, numNodes
         call ReadAry( UnIn, fileName, vals, 6, 'Values','Table row', errStat2, errMsg2, UnEc )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         SPyProps(j)%Pos(1,i) = vals(1)
         SPyProps(j)%Pos(2,i) = vals(2)
         SPyProps(j)%Pos(3,i) = vals(3)
         SPyProps(j)%Twist(i) = vals(4)*D2R
         SPyProps(j)%Chord(i) = vals(5)
         SPyProps(j)%AFID(i)  = vals(6)      
      end do
   end do
   
   do j=1, numPylons
      PPyProps(j)%NumNds = numNodes
      do i=1, numNodes
         call ReadAry( UnIn, fileName, vals, 6, 'Values','Table row', errStat2, errMsg2, UnEc )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         PPyProps(j)%Pos(1,i) = vals(1)
         PPyProps(j)%Pos(2,i) = vals(2)
         PPyProps(j)%Pos(3,i) = vals(3)
         PPyProps(j)%Twist(i) = vals(4)*D2R
         PPyProps(j)%Chord(i) = vals(5)
         PPyProps(j)%AFID(i)  = vals(6)      
      end do
   end do
   
   deallocate(vals)
   
end subroutine ReadPyProps

!> Routine to parse the rotor component properties.
subroutine ReadRotorProps( UnIn, fileName, numRotors, ActDsk_InitInp, errStat, errMsg, UnEc )
   integer(IntKi),               intent(in   )  :: UnIn              !< Input file unit number
   character(*),                 intent(in   )  :: filename          !< Input file name
   integer(IntKi),               intent(in   )  :: numRotors         !< Number of rotor for the entire kite
   type(ActDsk_InitInputType),   intent(inout)  :: ActDsk_InitInp(:) !< Actuator model initialization input data
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None
   integer(IntKi),               intent(in   )  :: UnEc              !< Echo file unit
   
   integer(intKi)                               :: i                 ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(1024)                              :: charAry(2)
   integer(intKi)                               :: numWords
   character(1024)                              :: line
   integer(intKi)                               :: IOS
   character(*), parameter                      :: RoutineName = 'ReadRotorProps'
   
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""
   
      ! Skip the comment line and table headers.
   call ReadCom( UnIn, fileName, ' ROTOR PROPERTIES ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call ReadCom( UnIn, fileName, ' Rotor table header line 1', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )   
   call ReadCom( UnIn, fileName, ' Rotor table header line 2', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )   
      
   do i = 1, numRotors
      ! Read the rotor radius and the rotor input file name
      READ (UnIn,'(A)',IOSTAT=IOS)   line
      if (IOS /= 0) then
         call SetErrStat( ErrID_Fatal, 'Could not parse the line containing rotor radius and input file from the KiteAeroDyn the input file', errStat, errMsg, RoutineName )
         return
      end if

      numWords = CountWords( line )    ! The number of words in OutLine.

      if ( numWords < 2 ) then
         call SetErrStat( ErrID_Fatal, 'Rotor property lines must contain: rotor radius and a string corresponding to the rotor input file', errStat, errMsg, RoutineName )
         return
      else
         call GetWords ( line, charAry(1:2), 2 )
         read( charAry(1), *, IOSTAT=IOS) ActDsk_InitInp(i)%R
         if (IOS /= 0) then
            call SetErrStat( ErrID_Fatal, 'Could not parse the rotor radius from the KiteAeroDyn input file', errStat, errMsg, RoutineName )
            return
         end if
         
         ActDsk_InitInp(i)%FileName = charAry(2)

         if (IOS /= 0) then
            call SetErrStat( ErrID_Fatal, 'Could not parse the rotor file name from the KiteAeroDyn input file', errStat, errMsg, RoutineName )
            return
         end if

      end if
   
   end do
   
end subroutine ReadRotorProps

!> Routine to parse kite component properties.
subroutine ReadProps( UnIn, fileName, numNodes, Props, numProps, label, errStat, errMsg, UnEc )
   integer(IntKi),               intent(in   )  :: UnIn              !< Input file unit number
   character(*),                 intent(in   )  :: filename          !< Input file name
   integer(IntKi),               intent(inout)  :: numNodes          !< Number of node in this component
   type(KAD_MbrPropsType),       intent(inout)  :: Props             !< Member properties data
   integer(IntKi),               intent(in   )  :: numProps          !< number of properties to parse from the input file for this component
   character(*),                 intent(in   )  :: label             !< Component label/identifying string
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None
   integer(IntKi),               intent(in   )  :: UnEc              !< Echo file unit
   
   integer(intKi)                               :: i                 ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   real(ReKi), allocatable                      :: vals(:)           ! Row of input file property values
   character(*), parameter                      :: RoutineName = 'ReadProps'
   
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""
   
      ! Skip the comment line.
   call ReadCom( UnIn, fileName, trim(label)//' PROPERTIES ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      ! Number of nodes used in the analysis
   call ReadVar ( UnIn, fileName, numNodes, 'NumNodes', 'Number of '//trim(label)//' nodes used in the analysis', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call AllocAry( Props%Pos, 3_IntKi, numNodes, trim(label)//'%Pos', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Props%Twist, numNodes, trim(label)//'%Twist', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      Props%Twist = Props%Twist*D2R
   call AllocAry( Props%Chord, numNodes, trim(label)//'%Chord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Props%AFID, numNodes, trim(label)//'%AFID', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   if ( numProps == 7 ) then
      call AllocAry( Props%CntrlID, numNodes, trim(label)//'%CntrlID', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      call AllocAry( vals, 7, 'vals', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )   
   else
      call AllocAry( vals, 6, 'vals', errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    
   end if
   
      ! Skip the table header line2.
   call ReadCom( UnIn, fileName, ' property table header line 1 ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call ReadCom( UnIn, fileName, ' property table header line 2 ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   do i=1, numNodes
      call ReadAry( UnIn, fileName, vals, numProps, 'Values','Table row', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      Props%Pos(1,i) = vals(1)
      Props%Pos(2,i) = vals(2)
      Props%Pos(3,i) = vals(3)
      Props%Twist(i) = vals(4)*D2R
      Props%Chord(i) = vals(5)
      Props%AFID(i)  = vals(6)
      if ( numProps == 7 ) Props%CntrlID(i) = vals(7)
   end do

   deallocate(vals)
   
end subroutine ReadProps

!> Routine to parse kite wing component properties.
subroutine ReadWngProps( UnIn, fileName, numNodes, Props, numProps, label, errStat, errMsg, UnEc )
   integer(IntKi),               intent(in   )  :: UnIn              !< Input file unit number
   character(*),                 intent(in   )  :: filename          !< Input file name
   integer(IntKi),               intent(inout)  :: numNodes          !< Number of node in this component
   type(KAD_MbrPropsType),       intent(inout)  :: Props             !< Member properties data
   integer(IntKi),               intent(in   )  :: numProps          !< number of properties to parse from the input file for this component
   character(*),                 intent(in   )  :: label             !< Component label/identifying string
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None
   integer(IntKi),               intent(in   )  :: UnEc              !< Echo file unit
   
   integer(intKi)                               :: i                 ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   real(ReKi), allocatable                      :: vals(:)           ! Row of input file property values
   character(*), parameter                      :: RoutineName = 'ReadProps'
   
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""
   
      ! Skip the comment line.
   call ReadCom( UnIn, fileName, trim(label)//' PROPERTIES ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      ! Number of nodes used in the analysis
   call ReadVar ( UnIn, fileName, numNodes, 'NumNodes', 'Number of '//trim(label)//' nodes used in the analysis', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call AllocAry( Props%Pos, 3_IntKi, numNodes, trim(label)//'%Pos', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Props%Dhdrl, numNodes, trim(label)//'%Dhdrl', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Props%Twist, numNodes, trim(label)//'%Twist', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      Props%Twist = Props%Twist*D2R
   call AllocAry( Props%Chord, numNodes, trim(label)//'%Chord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( Props%AFID, numNodes, trim(label)//'%AFID', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call AllocAry( Props%CntrlID, numNodes, trim(label)//'%CntrlID', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( vals, 8, 'vals', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )   
  
   
      ! Skip the table header line2.
   call ReadCom( UnIn, fileName, ' property table header line 1 ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call ReadCom( UnIn, fileName, ' property table header line 2 ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   do i=1, numNodes
      call ReadAry( UnIn, fileName, vals, numProps, 'Values','Table row', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      Props%Pos(1,i) = vals(1)
      Props%Pos(2,i) = vals(2)
      Props%Pos(3,i) = vals(3)
      Props%Dhdrl(i) = vals(4)*D2R
      if ( abs(Props%Dhdrl(i)) > (PI/2.0_ReKi) ) call SetErrStat( ErrID_FATAL, ' wing dihedral angles must be >= -90 deg and <= 90 deg', errStat, errMsg, RoutineName )
      Props%Twist(i) = vals(5)*D2R
      Props%Chord(i) = vals(6)
      Props%AFID(i)  = vals(7)
      Props%CntrlID(i) = vals(8)
   end do

   deallocate(vals)
   
end subroutine ReadWngProps
!> Routine to generate the line2 motions meshes for the various kite components.
subroutine CreateL2WngMotionsMesh(origin, numNodes, positions, alignDCM, dhdrls, twists, axis, mesh, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numNodes          !< Number of nodes in the mesh
   real(ReKi),                   intent(in   )  :: positions(:,:)    !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the airfoil axes
   real(R8Ki),                   intent(in   )  :: dhdrls(:)         !< Dihedral angles for each node (rad)
   real(R8Ki),                   intent(in   )  :: twists(:)         !< Twist angles for each node (rad)
   integer(IntKi),               intent(in   )  :: axis              !< Which euler angle is being set to the twist value
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: theta(3)          ! Euler angles
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   integer(intKi)                               :: j                 ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateL2WngMotionsMesh'

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
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation

   do j=1,numNodes       
      position(:) = origin(:) + positions(:,j)  
      select case( axis )
      case (1)
         theta = (/ twists(j), 0.0_R8Ki, 0.0_R8Ki/)
      case (2)  ! dhdrl only for wings
         theta = (/ dhdrls(j), twists(j), 0.0_R8Ki/)
      case (3)
         theta = (/ 0.0_R8Ki, 0.0_R8Ki, twists(j)/)
      case default
         call SetErrStat( ErrID_FATAL, "axis must be 1,2,3 for using euler angles to contruct euler matrix", errStat, errMsg, RoutineName )
         return
      end select
      
      orientation = EulerConstruct(theta)   
      orientation = matmul(alignDCM,orientation)
      call MeshPositionNode(mesh, j, position, errStat2, errMsg2, orientation)  
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   end do !j
         
      ! create line2 elements
   do j=1,numNodes-1
         ! Test for co-located nodes, which would be used when there is a step change in aerodynamic properties
      position = mesh%position(:,j+1) - mesh%position(:,j)
      if (.not. EqualRealNos ( abs(position(axis)), 0.0_ReKi ) ) then 
         call MeshConstructElement( mesh, ELEMENT_LINE2, errStat2, errMsg2, p1=j, p2=j+1 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      end if
   end do 
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return

      
   mesh%Orientation     = mesh%RefOrientation
   mesh%TranslationDisp = 0.0_ReKi
   mesh%TranslationVel  = 0.0_ReKi
   
   
   end subroutine CreateL2WngMotionsMesh
   
!> Routine to generate the line2 motions meshes for the various kite components.
subroutine CreateL2MotionsMesh(origin, numNodes, positions, alignDCM, twists, axis, mesh, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numNodes          !< Number of nodes in the mesh
   real(ReKi),                   intent(in   )  :: positions(:,:)    !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the airfoil axes
   real(R8Ki),                   intent(in   )  :: twists(:)         !< Twist angles for each node (rad)
   integer(IntKi),               intent(in   )  :: axis              !< Which euler angle is being set to the twist value
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: theta(3)          ! Euler angles
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
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation

   do j=1,numNodes       
      position(:) = origin(:) + positions(:,j)  
      select case( axis )
      case (1)
         theta = (/ twists(j), 0.0_R8Ki, 0.0_R8Ki/)
      case (2)
         theta = (/ 0.0_R8Ki, twists(j), 0.0_R8Ki/)
      case (3)
         theta = (/ 0.0_R8Ki, 0.0_R8Ki, twists(j)/)
      case default
         call SetErrStat( ErrID_FATAL, "axis must be 1,2,3 for using euler angles to contruct euler matrix", errStat, errMsg, RoutineName )
         return
      end select
      
      orientation = EulerConstruct(theta)   
      orientation = matmul(alignDCM,orientation)
      call MeshPositionNode(mesh, j, position, errStat2, errMsg2, orientation)  
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   end do !j
         
      ! create line2 elements
   do j=1,numNodes-1
         ! Test for co-located nodes, which would be used when there is a step change in aerodynamic properties
      position = mesh%position(:,j+1) - mesh%position(:,j)
      if (.not. EqualRealNos ( abs(position(axis)), 0.0_ReKi ) ) then 
         call MeshConstructElement( mesh, ELEMENT_LINE2, errStat2, errMsg2, p1=j, p2=j+1 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      end if
   end do 
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return

      
   mesh%Orientation     = mesh%RefOrientation
   mesh%TranslationDisp = 0.0_ReKi
   mesh%TranslationVel  = 0.0_ReKi
   
   
end subroutine CreateL2MotionsMesh

!> Routine to generate the point loads meshes for the various kite components.
subroutine CreatePtLoadsMesh(origin, numNodes, positions, alignDCM, twists, axis, mesh, elemLens, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numNodes          !< Number of nodes in the mesh
   real(ReKi),                   intent(in   )  :: positions(:,:)    !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the airfoil axes
   real(R8Ki),                   intent(in   )  :: twists(:)         !< Twist angles for each node (rad)
   integer(IntKi),               intent(in   )  :: axis              !< Which euler angle is being set to the twist value
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   real(ReKi),allocatable,       intent(  out)  :: elemLens(:)       !< The lengths of each element associated with this mesh data (VSM-elements)
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: theta(3)          ! Euler angles
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   
   integer(intKi)                               :: j,count           ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreatePtLoadsMesh'
   real(ReKi)                                   :: elemLen           ! Length of a VSM element associated with this data
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   ! Determine actual number of nodes required, because some may be co-located to capture discontinuities in the properties
   count = 0
   do j=1,numNodes     
      position(:) = positions(:,j+1) - positions(:,j)
      if (.not. EqualRealNos(abs(position(axis)),0.0_ReKi)) then
         count = count + 1
      end if
   end do
   
   call AllocAry(elemLens,count,'elemLens',errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   if (errStat >= AbortErrLev) return
   
   call MeshCreate ( BlankMesh = mesh     &
                     ,Nnodes    = count         &
                     ,errStat   = errStat2         &
                     ,ErrMess   = errMsg2          &
                     ,IOS       = COMPONENT_OUTPUT &
                     , force    = .true.           &
                     , moment   = .true.           &
                     , orientation = .true.        &  ! Used for computing VSM inputs and for kite load calculations
                     , translationdisp = .true.    &  ! Used by KiteFAST and for kite load calculations
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation
   count = 1
   do j=1,numNodes     
      position(:) = positions(:,j+1) - positions(:,j)
      elemLen = abs(position(axis))
      if (.not. EqualRealNos(elemLen,0.0_ReKi)) then
         elemLens(count) = elemLen
         position(:) = origin + 0.5_ReKi*(positions(:,j) + positions(:,j+1)) 
         select case( axis )
         case (1)
            theta(1) = 0.5_ReKi*(twists(j)+twists(j+1))
         case (2)
            theta(2) = 0.5_ReKi*(twists(j)+twists(j+1))
         case (3)
            theta(3) = 0.5_ReKi*(twists(j)+twists(j+1))
         case default
            call SetErrStat( ErrID_FATAL, "axis must be 1,2,3 for using twist to contruct euler matrix", errStat, errMsg, RoutineName )
            return
         end select
      
         orientation = EulerConstruct(theta)   
         orientation = matmul(alignDCM,orientation)
         call MeshPositionNode(mesh, count, position, errStat2, errMsg2, orientation)  
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         call MeshConstructElement( mesh, ELEMENT_POINT, errStat2, errMsg2, p1=count )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         count = count + 1
      end if
      
   end do !j
         
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return

   
end subroutine CreatePtLoadsMesh

!> Routine to generate the point loads meshes for the various kite components.
subroutine CreatePtWngLoadsMesh(origin, numNodes, positions, alignDCM, dhdrls, twists, axis, mesh, elemLens, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   integer(IntKi),               intent(in   )  :: numNodes          !< Number of nodes in the mesh
   real(ReKi),                   intent(in   )  :: positions(:,:)    !< Coordinates of the undisplaced mesh
   real(R8Ki),                   intent(in   )  :: alignDCM(3,3)     !< DCM needed to transform into the airfoil axes
   real(R8Ki),                   intent(in   )  :: dhdrls(:)         !< Dihedral angles for each node (rad)
   real(R8Ki),                   intent(in   )  :: twists(:)         !< Twist angles for each node (rad)
   integer(IntKi),               intent(in   )  :: axis              !< Which euler angle is being set to the twist value
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   real(ReKi),allocatable,       intent(  out)  :: elemLens(:)       !< The lengths of each element associated with this mesh data (VSM-elements)
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(reKi)                                   :: position(3)       ! node reference position
   real(R8Ki)                                   :: theta(3)          ! Euler angles
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   
   integer(intKi)                               :: j,count           ! counter for nodes
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreatePtLoadsMesh'
   real(ReKi)                                   :: elemLen           ! Length of a VSM element associated with this data
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   ! Determine actual number of nodes required, because some may be co-located to capture discontinuities in the properties
   count = 0
   do j=1,numNodes     
      position(:) = positions(:,j+1) - positions(:,j)
      if (.not. EqualRealNos(abs(position(axis)),0.0_ReKi)) then
         count = count + 1
      end if
   end do
   
   call AllocAry(elemLens,count,'elemLens',errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   if (errStat >= AbortErrLev) return
   
   call MeshCreate ( BlankMesh = mesh     &
                     ,Nnodes    = count         &
                     ,errStat   = errStat2         &
                     ,ErrMess   = errMsg2          &
                     ,IOS       = COMPONENT_OUTPUT &
                     , force    = .true.           &
                     , moment   = .true.           &
                     , orientation = .true.        &  ! Used for computing VSM inputs and for kite load calculations
                     , translationdisp = .true.    &  ! Used by KiteFAST and for kite load calculations
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation
   count = 1
   do j=1,numNodes     
      position(:) = positions(:,j+1) - positions(:,j)
      elemLen = abs(position(axis))
      if (.not. EqualRealNos(elemLen,0.0_ReKi)) then
         elemLens(count) = elemLen / cos ( 0.5_R8Ki * ( dhdrls(j) + dhdrls(j+1) ) )
         position(:) = origin + 0.5_ReKi*(positions(:,j) + positions(:,j+1)) 
         select case( axis )
         case (1)
            theta(1) = 0.5_R8Ki*(twists(j)+twists(j+1))
         case (2)
            theta(1) = 0.5_R8Ki*(dhdrls(j)+dhdrls(j+1))
            theta(2) = 0.5_R8Ki*(twists(j)+twists(j+1))
         case (3)
            theta(3) = 0.5_R8Ki*(twists(j)+twists(j+1))
         case default
            call SetErrStat( ErrID_FATAL, "axis must be 1,2,3 for using twist to contruct euler matrix", errStat, errMsg, RoutineName )
            return
         end select
      
         orientation = EulerConstruct(theta)   
         orientation = matmul(alignDCM,orientation)
         call MeshPositionNode(mesh, count, position, errStat2, errMsg2, orientation)  
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         call MeshConstructElement( mesh, ELEMENT_POINT, errStat2, errMsg2, p1=count )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         count = count + 1
      end if
      
   end do !j
         
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return

   
end subroutine CreatePtWngLoadsMesh

!> Routine to generate the point loads meshes for the rotor components.
subroutine CreateRtrPtLoadsMesh(origin, motionMesh, mesh, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
   type(MeshType),               intent(inout)  :: motionMesh        !< The sibling mesh 
   type(MeshType),               intent(  out)  :: mesh              !< The resulting mesh 
   integer(IntKi),               intent(  out)  :: errStat           !< Error status of the operation
   character(*),                 intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None

   ! Local variables
   real(R8Ki)                                   :: orientation(3,3)  ! node reference orientation
   
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'CreateRtrPtLoadsMesh'
   real(ReKi)                                   :: elemLen           ! Length of a VSM element associated with this data
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   call MeshCopy (  SrcMesh  = motionMesh        &
                  , DestMesh = mesh          &
                  , CtrlCode = MESH_SIBLING       &
                  , IOS      = COMPONENT_OUTPUT   &
                  , force    = .TRUE.             &
                  , moment   = .TRUE.             &
                  , ErrStat  = errStat2           &
                  , ErrMess  = errMsg2            )
   
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
               
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return

   
end subroutine CreateRtrPtLoadsMesh

subroutine CreateMeshMappings( u, y, p, m, errStat, errMsg )
   type(KAD_InputType),            intent(in   )  :: u               !< Module inputs
   type(KAD_OutputType),           intent(inout)  :: y               !< Module outputs
   type(KAD_ParameterType),        intent(in   )  :: p               !< Parameters
   type(KAD_MiscVarType),          intent(inout)  :: m               !< Misc Vars
   integer(IntKi),                 intent(  out)  :: errStat         !< Error status of the operation
   character(*),                   intent(  out)  :: errMsg          !< Error message if errStat /= ErrID_None

   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   integer(IntKi)                               :: i, j
   character(*), parameter                      :: routineName = 'CreateMeshMappings'
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""
   
            ! TODO: consider moving these into CreateMeshMappings
   allocate(m%SPyLoads(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPyLoads', errStat, errMsg, RoutineName )     
   allocate(m%PPyLoads(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPyLoads', errStat, errMsg, RoutineName )  
   allocate(m%SPy_L_2_P(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPy_L_2_P', errStat, errMsg, RoutineName )     
   allocate(m%PPy_L_2_P(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPy_L_2_P', errStat, errMsg, RoutineName )  
      
   allocate(m%SPy_P_2_P(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPy_P_2_P', errStat, errMsg, RoutineName )  
   allocate(m%PPy_P_2_P(p%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPy_P_2_P', errStat, errMsg, RoutineName )  
   allocate(m%SPyRtr_P_2_P(p%NumPylons*2), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPyRtr_P_2_P', errStat, errMsg, RoutineName )   
   allocate(m%PPyRtr_P_2_P(p%NumPylons*2), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPyRtr_P_2_P', errStat, errMsg, RoutineName )   

   if (errStat >= AbortErrLev) return   

      ! Create mesh mappings
   
   call MeshCopy( y%FusLoads, m%FusLoads, MESH_NEWCOPY, ErrStat2, ErrMsg2 )  ! we need to make this copy because we cannot update y in UpdateStates()
      call SetErrStat(ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName)
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( u%FusMotions, m%FusLoads, m%Fus_L_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: Fus_L_2_P' )     
         if (ErrStat>=AbortErrLev) return
   
   call MeshCopy( y%SWnLoads, m%SWnLoads, MESH_NEWCOPY, ErrStat2, ErrMsg2 )
      call SetErrStat(ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName)
         if (ErrStat>=AbortErrLev) return
   call MeshMapCreate( u%SWnMotions, m%SWnLoads, m%SWn_L_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: SWn_L_2_P' )     
         if (ErrStat>=AbortErrLev) return
         
   call MeshCopy( y%PWnLoads, m%PWnLoads, MESH_NEWCOPY, ErrStat2, ErrMsg2 )
      call SetErrStat(ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName)
         if (ErrStat>=AbortErrLev) return      
   call MeshMapCreate( u%PWnMotions, m%PWnLoads, m%PWn_L_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: PWn_L_2_P' )     
         if (ErrStat>=AbortErrLev) return
         
   call MeshCopy( y%VSLoads, m%VSLoads, MESH_NEWCOPY, ErrStat2, ErrMsg2 )
      call SetErrStat(ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName)
         if (ErrStat>=AbortErrLev) return         
   call MeshMapCreate( u%VSMotions, m%VSLoads, m%VS_L_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: VS_L_2_P' )     
         if (ErrStat>=AbortErrLev) return
         
   call MeshCopy( y%SHSLoads, m%SHSLoads, MESH_NEWCOPY, ErrStat2, ErrMsg2 )
      call SetErrStat(ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName)
         if (ErrStat>=AbortErrLev) return       
   call MeshMapCreate( u%SHSMotions, m%SHSLoads, m%SHS_L_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: SHS_L_2_P' )     
         if (ErrStat>=AbortErrLev) return
         
   call MeshCopy( y%PHSLoads, m%PHSLoads, MESH_NEWCOPY, ErrStat2, ErrMsg2 )
      call SetErrStat(ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName)
         if (ErrStat>=AbortErrLev) return       
   call MeshMapCreate( u%PHSMotions, m%PHSLoads, m%PHS_L_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: PHS_L_2_P' )     
         if (ErrStat>=AbortErrLev) return
         
   do i = 1 , p%NumPylons      
      
      call MeshCopy( y%SPyLoads(i), m%SPyLoads(i), MESH_NEWCOPY, ErrStat2, ErrMsg2 )
         call SetErrStat(ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName)
            if (ErrStat>=AbortErrLev) return 
      call MeshMapCreate( u%SPyMotions(i), m%SPyLoads(i), m%SPy_L_2_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: SPy_L_2_P('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
            
      call MeshCopy( y%PPyLoads(i), m%PPyLoads(i), MESH_NEWCOPY, ErrStat2, ErrMsg2 )
         call SetErrStat(ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName)
            if (ErrStat>=AbortErrLev) return       
      call MeshMapCreate( u%PPyMotions(i), m%PPyLoads(i), m%PPy_L_2_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: PPy_L_2_P('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
   end do
   
      ! Map loads to FusO for total integrated load calculations
   call MeshMapCreate( y%FusLoads, m%FusOLoads, m%Fus_P_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: Fus_P_2_P' )     
         if (ErrStat>=AbortErrLev) return
   
   call MeshMapCreate( y%SWnLoads, m%FusOLoads, m%SWn_P_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: SWn_P_2_P' )     
         if (ErrStat>=AbortErrLev) return
             
   call MeshMapCreate( y%PWnLoads, m%FusOLoads, m%PWn_P_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: PWn_P_2_P' )     
         if (ErrStat>=AbortErrLev) return
                
   call MeshMapCreate( y%VSLoads, m%FusOLoads, m%VS_P_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: VS_P_2_P' )     
         if (ErrStat>=AbortErrLev) return
              
   call MeshMapCreate( y%SHSLoads, m%FusOLoads, m%SHS_P_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: SHS_P_2_P' )     
         if (ErrStat>=AbortErrLev) return
              
   call MeshMapCreate( y%PHSLoads, m%FusOLoads, m%PHS_P_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: PHS_P_2_P' )     
         if (ErrStat>=AbortErrLev) return
         
    ! Pylons
    do i = 1 , p%NumPylons      
      
      call MeshMapCreate( y%SPyLoads(i), m%FusOLoads, m%SPy_P_2_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: SPy_P_2_P('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
    
      call MeshMapCreate( y%PPyLoads(i), m%FusOLoads, m%PPy_P_2_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: PPy_P_2_P('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
    end do   
    
    ! Rotors
    do i = 1 , p%NumPylons*2
       
       call MeshMapCreate( y%SPyRtrLoads(i), m%FusOLoads, m%SPyRtr_P_2_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: SPyRtr_P_2_P('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
    
      call MeshMapCreate( y%PPyRtrLoads(i), m%FusOLoads, m%PPyRtr_P_2_P(i), errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: PPyRtr_P_2_P('//trim(num2lstr(i))//')' ) 
            if (ErrStat>=AbortErrLev) return
     
    end do
    

end subroutine CreateMeshMappings

subroutine FindVSMElemBasedOnMotionNode( motionNd, motionMesh, VSMoffset, VSMElem )

   integer(IntKi),            intent(in   ) :: motionNd
   type(MeshType),           intent(in   ) :: motionMesh
   integer(IntKi),            intent(in   ) :: VSMoffset
   integer(IntKi),                intent(  out) :: VSMElem
   
   
   ! Search element list to find the requested motion node

   integer(IntKi) :: i, n1, n2
   
   i = 1 !! element index
   VSMElem = 0
   
   do while ( i <= motionMesh%NElemList ) 
      
      n1 = motionMesh%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = motionMesh%ELEMLIST(i)%ELEMENT%ELEMNODES(2) 
      
      if ( n1 == motionNd ) then
         ! left node in 1st element, no averaging
         VSMElem =VSMoffset+i
         exit
      end if
      
      i = i + 1
      
   end do
   
   
end subroutine FindVSMElemBasedOnMotionNode

subroutine MapVSMontoMotionNode( motionNd, motionMesh, VSMoffset, VSMdata, val )

   integer(IntKi),            intent(in   ) :: motionNd
   type(MeshType),           intent(in   ) :: motionMesh
   integer(IntKi),            intent(in   ) :: VSMoffset
   real(ReKi),                intent(in   ) :: VSMdata(:,:)
   real(ReKi),                intent(  out) :: val(3)
   ! Search element list to find the requested motion node and then determine whether that motion node is
   ! is connected to an adjacent element

   integer(IntKi) :: i, n1, n2
   
   i = 1 !! element index
   val = 0.0_ReKi
   
   do while ( i <= motionMesh%NElemList ) 
      
      n1 = motionMesh%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = motionMesh%ELEMLIST(i)%ELEMENT%ELEMNODES(2) 
      
      if (n1 == motionNd .and. i == 1) then
         ! left node in 1st element, no averaging
         val = VSMdata(:,VSMoffset+i)
         exit
      elseif (n2 == motionNd) then
         if ( i == motionMesh%NElemList  ) then
            ! No more elements, don't average
            val = VSMdata(:,VSMoffset+i)
            exit
         else
            ! may need to average, so look at next element
            n1 = motionMesh%ELEMLIST(i+1)%ELEMENT%ELEMNODES(1)
            val = VSMdata(:,VSMoffset+i)
            if (n1 == motionNd) then
               ! yep, average
               val = (val + VSMdata(:,VSMoffset+i+1))/2.0_ReKi
            else
               ! nope, no average
            end if
            exit
         end if
      end if
      
      i = i + 1
      
   end do
   
   
end subroutine MapVSMontoMotionNode
subroutine Map1D_VSMontoMotionNode( motionNd, motionMesh, VSMoffset, VSMdata, val )

   integer(IntKi),            intent(in   ) :: motionNd
   type(MeshType),           intent(in   ) :: motionMesh
   integer(IntKi),            intent(in   ) :: VSMoffset
   real(ReKi),                intent(in   ) :: VSMdata(:)
   real(ReKi),                intent(  out) :: val
   ! Search element list to find the requested motion node and then determine whether that motion node is
   ! is connected to an adjacent element

   integer(IntKi) :: i, n1, n2
   
   i = 1 !! element index
   val = 0.0_ReKi
   
   do while ( i <= motionMesh%NElemList ) 
      
      n1 = motionMesh%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = motionMesh%ELEMLIST(i)%ELEMENT%ELEMNODES(2) 
      
      if (n1 == motionNd ) then !.and. i == 1) then
         ! left node in 1st element, no averaging
         val = VSMdata(VSMoffset+i)
         exit
      !elseif (n2 == motionNd) then
      !   if ( i == motionMesh%NElemList  ) then
      !      ! No more elements, don't average
      !      val = VSMdata(VSMoffset+i)
      !      exit
      !   else
      !      ! may need to average, so look at next element
      !      n1 = motionMesh%ELEMLIST(i+1)%ELEMENT%ELEMNODES(1)
      !      val = VSMdata(VSMoffset+i)
      !      if (n1 == motionNd) then
      !         ! yep, average
      !         val = (val + VSMdata(VSMoffset+i+1)) / 2.0_ReKi
      !      else
      !         ! nope, no average
      !      end if
      !      exit
      !   end if
      end if
      
      i = i + 1
      
   end do
   
   
end subroutine Map1D_VSMontoMotionNode
!----------------------------------------------------------------------------------------------------------------------------------   
!> This routine initializes KiteAeroDyn meshes and output array variables for use during the simulation.
subroutine Init_y(y, u, InitInp, p, errStat, errMsg)

   type(KAD_OutputType),           intent(  out)  :: y               !< Module outputs
   type(KAD_InputType),            intent(inout)  :: u               !< Module inputs -- intent(out) because of mesh sibling copy
   type(KAD_InitInputType),        intent(in   )  :: InitInp         !< Initialization inputs
   type(KAD_ParameterType),        intent(inout)  :: p               !< Parameters
   integer(IntKi),                 intent(  out)  :: errStat         !< Error status of the operation
   character(*),                   intent(  out)  :: errMsg          !< Error message if errStat /= ErrID_None


      ! Local variables
   integer(intKi)                               :: i,j               ! loop counters
   integer(intKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'Init_y'
   real(R8Ki)                                   :: alignDCM(3,3)
   real(ReKi), allocatable                      :: elemLens(:)
   
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""
   
   ! Allocate mesh arrays
   allocate(y%SPyLoads(InitInp%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for y%SPyLoads', errStat, errMsg, RoutineName )     
   allocate(y%PPyLoads(InitInp%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for y%PPyLoads', errStat, errMsg, RoutineName )  
   allocate(y%SPyRtrLoads(InitInp%NumPylons*2), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for y%SPyRtrLoads', errStat, errMsg, RoutineName )  
   allocate(y%PPyRtrLoads(InitInp%NumPylons*2), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for y%PPyRtrLoads', errStat, errMsg, RoutineName )  
   if (errStat >= AbortErrLev) return
   
          ! Fuselage Mesh
   alignDCM = reshape( (/0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreatePtLoadsMesh((/0.0_ReKi, 0.0_ReKi,0.0_ReKi/), InitInp%InpFileData%FusProps%NumNds-1, InitInp%InpFileData%FusProps%Pos, alignDCM, InitInp%InpFileData%FusProps%Twist, 1, y%FusLoads, p%FusElemLen, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )        
      if (errStat >= AbortErrLev) return

      ! Starboard Wing Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreatePtWngLoadsMesh(InitInp%SWnOR, InitInp%InpFileData%SWnProps%NumNds-1, InitInp%InpFileData%SWnProps%Pos, alignDCM, -1.0_R8Ki*InitInp%InpFileData%SWnProps%Dhdrl, InitInp%InpFileData%SWnProps%Twist, 2, y%SWnLoads, p%SWnElemLen, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return
      
      ! Port Wing Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreatePtWngLoadsMesh(InitInp%PWnOR, InitInp%InpFileData%PWnProps%NumNds-1, InitInp%InpFileData%PWnProps%Pos, alignDCM, InitInp%InpFileData%PWnProps%Dhdrl, InitInp%InpFileData%PWnProps%Twist, 2, y%PWnLoads, p%PWnElemLen, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return
   
      ! Vertical Stabilizer Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0/), (/3,3/) )  
   call CreatePtLoadsMesh(InitInp%VSOR, InitInp%InpFileData%VSProps%NumNds-1, InitInp%InpFileData%VSProps%Pos, alignDCM, InitInp%InpFileData%VSProps%Twist, 3, y%VSLoads, p%VSElemLen, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return

      ! Starboard Horizontal Stabilizer Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreatePtLoadsMesh(InitInp%SHSOR, InitInp%InpFileData%SHSProps%NumNds-1, InitInp%InpFileData%SHSProps%Pos, alignDCM, InitInp%InpFileData%SHSProps%Twist, 2, y%SHSLoads, p%SHSElemLen, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return

      ! Port Horizontal Stabilizer Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreatePtLoadsMesh(InitInp%PHSOR, InitInp%InpFileData%PHSProps%NumNds-1, InitInp%InpFileData%PHSProps%Pos, alignDCM, InitInp%InpFileData%PHSProps%Twist, 2, y%PHSLoads, p%PHSElemLen, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return

      ! Starboard Pylons Meshes
   alignDCM = reshape( (/0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0/), (/3,3/) )  
   do j=1,InitInp%NumPylons
      
      call CreatePtLoadsMesh(InitInp%SPyOR(:,j), InitInp%InpFileData%SPyProps(j)%NumNds-1, InitInp%InpFileData%SPyProps(j)%Pos, alignDCM, InitInp%InpFileData%SPyProps(j)%Twist, 3, y%SPyLoads(j), elemLens, errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         if (errStat >= AbortErrLev) return
      p%SPyElemLen(:,j) = elemLens
   end do

      ! Port Pylons Meshes
   alignDCM = reshape( (/0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0/), (/3,3/) )  
   do j=1,InitInp%NumPylons
      call CreatePtLoadsMesh(InitInp%PPyOR(:,j), InitInp%InpFileData%PPyProps(j)%NumNds-1, InitInp%InpFileData%PPyProps(j)%Pos, alignDCM, InitInp%InpFileData%PPyProps(j)%Twist, 3, y%PPyLoads(j), elemLens, errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         if (errStat >= AbortErrLev) return
      p%PPyElemLen(:,j) = elemLens
   end do

      ! Starboard Rotor Meshes 
   do j=1,InitInp%NumPylons
      do i = 1,2
         call CreateRtrPtLoadsMesh(InitInp%SPyRtrOR(:,i,j), u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons), y%SPyRtrLoads(i+(j-1)*InitInp%NumPylons), errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            if (errStat >= AbortErrLev) return
      end do
      
   end do

      ! Port Rotor Meshes
   do j=1,InitInp%NumPylons
      do i = 1,2
         call CreateRtrPtLoadsMesh(InitInp%PPyRtrOR(:,i,j), u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons), y%PPyRtrLoads(i+(j-1)*InitInp%NumPylons), errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            if (errStat >= AbortErrLev) return
      end do
   end do
 
end subroutine Init_y

!----------------------------------------------------------------------------------------------------------------------------------
!> This routine initializes KiteAeroDyn meshes and input array variables for use during the simulation.
subroutine Init_u( u, p, InitInp, nIfWPts, errStat, errMsg )

   type(KAD_InputType),           intent(  out)  :: u                 !< Input data
   type(KAD_ParameterType),       intent(in   )  :: p                 !< Parameters
   type(KAD_InitInputType),       intent(in   )  :: InitInp           !< Input data for AD initialization routine
   integer(IntKi),                intent(  out)  :: nIfWPts         !< The number of points where we need inflow velocities
   integer(IntKi),                intent(  out)  :: errStat           !< Error status of the operation
   character(*),                  intent(  out)  :: errMsg            !< Error message if errStat /= ErrID_None


      ! Local variables
   
   integer(IntKi)                               :: i,j               ! counter for nodes
   integer(IntKi)                               :: n                 ! number of nodes
   integer(IntKi)                               :: errStat2          ! temporary Error status
   character(ErrMsgLen)                         :: errMsg2           ! temporary Error message
   character(*), parameter                      :: RoutineName = 'Init_u'
   real(R8Ki)                                   :: alignDCM(3,3)
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""

   ! -------------------------------------------
   ! Arrays for Undisturbed Inflow Wind inputs:
   ! -------------------------------------------
   
      ! Fuselage nodes
   n = InitInp%InpFileData%FusProps%NumNds
   nIfWPts = n
   call AllocAry( u%V_Fus, 3_IntKi, n, 'u%V_Fus', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      ! Starboard wing nodes
   n = InitInp%InpFileData%SWnProps%NumNds
   nIfWPts = nIfWPts + n
   call AllocAry( u%V_SWn, 3_IntKi, n, 'u%V_SWn', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      ! Port wing nodes
   n = InitInp%InpFileData%PWnProps%NumNds
   nIfWPts = nIfWPts + n
   call AllocAry( u%V_PWn, 3_IntKi, n, 'u%V_PWn', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      ! Vertical stabilizer nodes
   n = InitInp%InpFileData%VSProps%NumNds
   nIfWPts = nIfWPts + n
   call AllocAry( u%V_VS, 3_IntKi, n, 'u%V_VS', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      ! Starboard horizontal stabilizer nodes
   n = InitInp%InpFileData%SHSProps%NumNds
   nIfWPts = nIfWPts + n
   call AllocAry( u%V_SHS, 3_IntKi, n, 'u%V_SHS', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      ! Port horizontal stabilizer nodes
   n = InitInp%InpFileData%PHSProps%NumNds
   nIfWPts = nIfWPts + n
   call AllocAry( u%V_PHS, 3_IntKi, n, 'u%V_PHS', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      ! Pylon nodes
   n = InitInp%InpFileData%SPyProps(1)%NumNds  ! Right now, all pylons must have the same number of nodes
   nIfWPts = nIfWPts + n*InitInp%NumPylons*2
   call AllocAry( u%V_SPy, 3_IntKi, n, InitInp%NumPylons, 'u%V_SPy', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( u%V_PPy, 3_IntKi, n, InitInp%NumPylons, 'u%V_PPy', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      ! Rotor hub points, two per pylon
   n = InitInp%NumPylons
   nIfWPts = nIfWPts + n*4
   call AllocAry( u%V_SPyRtr, 3_IntKi, 2_IntKi, n, 'u%V_SPyRtr', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( u%V_PPyRtr, 3_IntKi, 2_IntKi, n, 'u%V_PPyRtr', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Rotor rototational speeds
   call AllocAry( u%RtSpd_SPyRtr, 2_IntKi, n, 'u%RtSpd_SPyRtr', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( u%RtSpd_PPyRtr, 2_IntKi, n, 'u%RtSpd_PPyRtr', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
      ! Rotor-collective blade-pitch angles
   call AllocAry( u%Pitch_SPyRtr, 2_IntKi, n, 'u%Pitch_SPyRtr', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( u%Pitch_PPyRtr, 2_IntKi, n, 'u%Pitch_PPyRtr', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Control Surfaces
      n = InitInp%NumFlaps
   call AllocAry( u%Ctrl_SFlp, n, 'u%Ctrl_SFlp', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( u%Ctrl_PFlp, n, 'u%Ctrl_PFlp', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( u%Ctrl_Rudr, 2_IntKi, 'u%Ctrl_Rudr', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( u%Ctrl_SElv, 2_IntKi, 'u%Ctrl_SElv', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( u%Ctrl_PElv, 2_IntKi, 'u%Ctrl_PElv', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    
      
   if (errStat >= AbortErrLev) return      
      
      ! initialize everything to 0.0
   u%V_Fus    = 0.0_ReKi
   u%V_SWn    = 0.0_ReKi
   u%V_PWn    = 0.0_ReKi
   u%V_VS    = 0.0_ReKi
   u%V_SHS    = 0.0_ReKi
   u%V_PHS    = 0.0_ReKi
   u%V_SPy    = 0.0_ReKi
   u%V_PPy    = 0.0_ReKi
   u%V_SPyRtr = 0.0_ReKi
   u%V_PPyRtr = 0.0_ReKi
   u%RtSpd_SPyRtr = 0.0_ReKi
   u%RtSpd_PPyRtr = 0.0_ReKi
   u%Pitch_SPyRtr = 0.0_ReKi
   u%Pitch_PPyRtr = 0.0_ReKi
   u%Ctrl_SFlp    = 0.0_ReKi
   u%Ctrl_PFlp    = 0.0_ReKi
   u%Ctrl_Rudr    = 0.0_ReKi
   u%Ctrl_SElv    = 0.0_ReKi
   u%Ctrl_PElv    = 0.0_ReKi
   
   !----------------------------------------
   ! Meshes for motion inputs ( from MBDyn)
   !----------------------------------------
   
      ! Point Mesh for the Fuselage origin point
   call MeshCreate ( BlankMesh = u%FusOMotions     &
                       ,IOS       = COMPONENT_INPUT &
                       ,Nnodes    = 1               &
                       ,errStat   = errStat2        &
                       ,ErrMess   = errMsg2         &
                       ,Orientation     = .true.    &  ! Used by the driver for mesh mapping work
                       ,TranslationDisp = .true.    &
                       ,TranslationVel  = .true.    &  ! Used by the driver for mesh mapping work
                       ,RotationVel     = .true.    &  ! Used by the driver for mesh mapping work
                      )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
                     
   call MeshPositionNode(u%FusOMotions, 1, (/0.0_ReKi, 0.0_ReKi,0.0_ReKi/), errStat2, errMsg2) ! Ref Orientation is Identity
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
   call MeshConstructElement( u%FusOMotions, ELEMENT_POINT, errStat2, errMsg2, p1=1 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   call MeshCommit(u%FusOMotions, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return
     
   u%FusOMotions%Orientation     = u%FusOMotions%RefOrientation
   u%FusOMotions%TranslationDisp = 0.0_ReKi
   
   ! allocate the rotor meshes
   allocate(u%SPyRtrMotions(InitInp%NumPylons*2), STAT=errStat2)
   allocate(u%PPyRtrMotions(InitInp%NumPylons*2), STAT=errStat2)
   
        ! Point Meshes for Starboard Rotors
   do j=1,InitInp%NumPylons
      do i=1,2 ! two per pylon
         call MeshCreate ( BlankMesh = u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons)     &
                       ,IOS       = COMPONENT_INPUT &
                       ,Nnodes    = 1               &
                       ,errStat   = errStat2        &
                       ,ErrMess   = errMsg2         &
                       ,Orientation     = .true.    &
                       ,TranslationDisp = .true.    &
                       ,TranslationVel  = .true.    &
                      )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

         if (errStat >= AbortErrLev) return
                     
         call MeshPositionNode(u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons), 1,InitInp%SPyRtrOR(:,i,j), errStat2, errMsg2) ! Ref Orientation is Identity
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
         call MeshConstructElement( u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons), ELEMENT_POINT, errStat2, errMsg2, p1=1 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
         call MeshCommit(u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
         if (errStat >= AbortErrLev) return
     
         u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons)%Orientation     = u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons)%RefOrientation
         u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons)%TranslationDisp = 0.0_ReKi
         u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons)%TranslationVel  = 0.0_ReKi
      end do
   end do
 
        ! Point Meshes for Port Rotors
   do j=1,InitInp%NumPylons
      do i=1,2 ! two per pylon
         call MeshCreate ( BlankMesh = u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons)     &
                       ,IOS       = COMPONENT_INPUT &
                       ,Nnodes    = 1               &
                       ,errStat   = errStat2        &
                       ,ErrMess   = errMsg2         &
                       ,Orientation     = .true.    &
                       ,TranslationDisp = .true.    &
                       ,TranslationVel  = .true.    &
                      )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

         if (errStat >= AbortErrLev) return
                     
         call MeshPositionNode(u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons), 1,InitInp%PPyRtrOR(:,i,j), errStat2, errMsg2) ! Orientation is Identity
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
         call MeshConstructElement( u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons), ELEMENT_POINT, errStat2, errMsg2, p1=1 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
         call MeshCommit(u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
         if (errStat >= AbortErrLev) return
     
         u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons)%Orientation     = u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons)%RefOrientation
         u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons)%TranslationDisp = 0.0_ReKi
         u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons)%TranslationVel  = 0.0_ReKi
      end do
   end do
   
      ! Line2 Fuselage Mesh
   alignDCM = reshape( (/0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0/), (/3,3/) ) ! Fortran uses Column-major storage
   call CreateL2MotionsMesh((/0.0_ReKi, 0.0_ReKi,0.0_ReKi/), InitInp%InpFileData%FusProps%NumNds, InitInp%InpFileData%FusProps%Pos, alignDCM, InitInp%InpFileData%FusProps%Twist, 1, u%FusMotions, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )        
      if (errStat >= AbortErrLev) return

      ! Line2 Starboard Wing Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreateL2WngMotionsMesh(InitInp%SWnOR, InitInp%InpFileData%SWnProps%NumNds, InitInp%InpFileData%SWnProps%Pos, alignDCM, -1.0_R8Ki*InitInp%InpFileData%SWnProps%Dhdrl, InitInp%InpFileData%SWnProps%Twist, 2, u%SWnMotions, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return
      
      ! Line2 Port Wing Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreateL2WngMotionsMesh(InitInp%PWnOR, InitInp%InpFileData%PWnProps%NumNds, InitInp%InpFileData%PWnProps%Pos, alignDCM, InitInp%InpFileData%PWnProps%Dhdrl, InitInp%InpFileData%PWnProps%Twist, 2, u%PWnMotions, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return
   
      ! Line2 Vertical Stabilizer Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0/), (/3,3/) )   
   call CreateL2MotionsMesh(InitInp%VSOR, InitInp%InpFileData%VSProps%NumNds, InitInp%InpFileData%VSProps%Pos, alignDCM, InitInp%InpFileData%VSProps%Twist, 3, u%VSMotions, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return

      ! Line2 Starboard Horizontal Stabilizer Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreateL2MotionsMesh(InitInp%SHSOR, InitInp%InpFileData%SHSProps%NumNds, InitInp%InpFileData%SHSProps%Pos, alignDCM, InitInp%InpFileData%SHSProps%Twist, 2, u%SHSMotions, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return

      ! Line2 Port Horizontal Stabilizer Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreateL2MotionsMesh(InitInp%PHSOR, InitInp%InpFileData%PHSProps%NumNds, InitInp%InpFileData%PHSProps%Pos, alignDCM, InitInp%InpFileData%PHSProps%Twist, 2, u%PHSMotions, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return

      ! Line2 Starboard Pylons Meshes
   allocate(u%SPyMotions(InitInp%NumPylons), STAT=errStat2)
   alignDCM = reshape( (/0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0/), (/3,3/) )   
   do j=1,InitInp%NumPylons
      call CreateL2MotionsMesh(InitInp%SPyOR(:,j), InitInp%InpFileData%SPyProps(j)%NumNds, InitInp%InpFileData%SPyProps(j)%Pos, alignDCM, InitInp%InpFileData%SPyProps(j)%Twist, 3, u%SPyMotions(j), errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         if (errStat >= AbortErrLev) return
   end do

      ! Line2 Port Pylons Meshes
   allocate(u%PPyMotions(InitInp%NumPylons), STAT=errStat2)
   alignDCM = reshape( (/0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0/), (/3,3/) )   
   do j=1,InitInp%NumPylons
      call CreateL2MotionsMesh(InitInp%PPyOR(:,j), InitInp%InpFileData%PPyProps(j)%NumNds, InitInp%InpFileData%PPyProps(j)%Pos, alignDCM, InitInp%InpFileData%PPyProps(j)%Twist, 3, u%PPyMotions(j), errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         if (errStat >= AbortErrLev) return
   end do   

end subroutine Init_u

!!----------------------------------------------------------------------------------------------------------------------------------
!!> This subroutine sets up the data structures and initializes AirfoilInfo to get the necessary Fuselage AFI parameters. 
!subroutine Init_FusAFIparams( NumAFfiles, AFfiles, FusProps, p_AFI, UnEc, errStat, errMsg )
!
!      ! Passed variables
!   type(VSM_InitInputType),             intent(inout)   :: InitInp    !< Initialization input data for VSM (intent(out) only because of the call to MOVE_ALLOC)
!   type(AFI_ParameterType), allocatable, intent(  out)   :: p_AFI(:)   !< parameters returned from the AFI (airfoil info) module
!   integer(IntKi),                       intent(in   )   :: UnEc       !< I/O unit for echo file. If > 0, file is open for writing.
!   integer(IntKi),                       intent(  out)   :: errStat    !< Error status
!   character(*),                         intent(  out)   :: errMsg     !< Error message
!
!      ! local variables
!   type(AFI_InitInputType)                  :: AFI_InitInputs     ! initialization data for the AFI routines
!   integer(IntKi)                           :: File               ! loop counter for airfoil files 
!   integer(IntKi)                           :: errStat2
!   character(ErrMsgLen)                     :: errMsg2
!   character(*), parameter                  :: RoutineName = 'Init_AFIparams'
!
!   
!   errStat = ErrID_None
!   errMsg  = ""
!   
!   ! pull out the Airfoil files associated with the fuselage.
!   numFusAFfiles = 0
!   call Allocary( fusAFFiles, NumAFfiles, "fusAFFiles", errStat, errMsg )
!   fusAFFiles = .false.
!   do i = 1, FusProps%NumNds
!      fusAFFiles( FusProps%AFIDs(i) ) = .true.
!   end do
!  ! do i = 1, 
!  ! end do
!   
!   allocate(p_AFI( NumAFfiles), STAT = ErrStat2)
!      if ( ErrStat2 /= 0 ) then
!         ErrMsg2 = 'Error allocating p_AFI'
!         ErrStat2 = ErrID_Fatal
!         call SetErrStat(ErrStat2,ErrMsg2,errStat,errMsg,RoutineName)  
!         return
!      end if
!      
!      
!      ! Setup Airfoil InitInput data structure:
!    
!   AFI_InitInputs%InCol_Alfa  = InitInp%InCol_Alfa
!   AFI_InitInputs%InCol_Cl    = InitInp%InCol_Cl
!   AFI_InitInputs%InCol_Cd    = InitInp%InCol_Cd
!   AFI_InitInputs%InCol_Cm    = InitInp%InCol_Cm
!   AFI_InitInputs%InCol_Cpmin = 0
!   !AFI_InitInputs%UA_Model	   =  0	  
!   AFI_InitInputs%AFTabMod    =  InitInp%AFTabMod   
!   
!   do File = 1, InitInp%NumAFfiles
!      
!      ! Call AFI_Init to read in and process an airfoil file.
!      ! This includes creating the spline coefficients to be used for interpolation.
!      
!      AFI_InitInputs%FileName = InitInp%AFNames(File)
!      
!      call AFI_Init ( AFI_InitInputs, p_AFI(File), ErrStat2, ErrMsg2, UnEc )
!         call SetErrStat(ErrStat2,'Problem with airfoil file #'//trim(num2lstr(File))//': '//trim(ErrMsg2), errStat, errMsg, RoutineName)   
!
!      if (errStat >= AbortErrLev) return
!      
!   end do
!
!   call AFI_DestroyInitInput( AFI_InitInputs, ErrStat2, ErrMsg2 )
!      call SetErrStat(ErrStat2,ErrMsg2, errStat, errMsg, RoutineName)   
!      if (errStat >= AbortErrLev) return
!
!end subroutine Init_FusAFIparams


subroutine ComputeKiteLoads( p, u, y, m, kiteForces, kiteMoments, errStat, errMsg)
   type(KAD_ParameterType),       intent(in   )  :: p              !< KAD Parameters
   type(KAD_InputType),           intent(in   )  :: u              !< KAD system inputs
   type(KAD_OutputType),          intent(in   )  :: y              !< KAD system outputs
   type(KAD_MiscVarType),         intent(inout)  :: m              !< KAD MiscVars for the module
   real(ReKi),                    intent(  out)  :: kiteForces(3)  !< Integrated kite loads
   real(ReKi),                    intent(  out)  :: kiteMoments(3) !< Integrated kite moments
   integer(IntKi)      ,          intent(  out)  :: errStat        !< Error status of the operation
   character(*)        ,          intent(  out)  :: errMsg         !< Error message if errStat /= ErrID_None

   integer(IntKi)                                :: i, j
   integer(IntKi)                                :: errStat2       ! temporary Error status
   character(ErrMsgLen)                          :: errMsg2        ! temporary Error message
   character(*), parameter                       :: RoutineName = 'ComputeKiteLoads'

   errStat = ErrID_None
   errMsg  = ''
   
   kiteForces  = 0.0_ReKi
   kiteMoments = 0.0_ReKi
   
   call Transfer_Point_to_Point( y%FusLoads, m%FusOLoads, m%Fus_P_2_P, errStat2, errMsg2, y%FusLoads, u%FusOMotions )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_Fus_P_2_P' )    
   kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
   kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
   call Transfer_Point_to_Point( y%SWnLoads, m%FusOLoads, m%SWn_P_2_P, errStat2, errMsg2, y%SWnLoads, u%FusOMotions )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_SWn_P_2_P' )    
   kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
   kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
   call Transfer_Point_to_Point( y%PWnLoads, m%FusOLoads, m%PWn_P_2_P, errStat2, errMsg2, y%PWnLoads, u%FusOMotions )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_PWn_P_2_P' )    
   kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
   kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
   call Transfer_Point_to_Point( y%VSLoads, m%FusOLoads, m%VS_P_2_P, errStat2, errMsg2, y%VSLoads, u%FusOMotions )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_VS_P_2_P' )    
   kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
   kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
   call Transfer_Point_to_Point( y%SHSLoads, m%FusOLoads, m%SHS_P_2_P, errStat2, errMsg2, y%SHSLoads, u%FusOMotions )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_SHS_P_2_P' )    
   kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
   kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
   call Transfer_Point_to_Point( y%PHSLoads, m%FusOLoads, m%PHS_P_2_P, errStat2, errMsg2, y%PHSLoads, u%FusOMotions )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_PHS_P_2_P' )    
   kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
   kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
   
   do i = 1, p%NumPylons
      call Transfer_Point_to_Point( y%SPyLoads(i), m%FusOLoads, m%SPy_P_2_P(i), errStat2, errMsg2, y%SPyLoads(i), u%FusOMotions )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_m%SPy_P_2_P' )    
      kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
      kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
      call Transfer_Point_to_Point( y%PPyLoads(i), m%FusOLoads, m%PPy_P_2_P(i), errStat2, errMsg2, y%PPyLoads(i), u%FusOMotions )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_m%PPy_P_2_P' )    
      kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
      kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
      
      j = (i-1)*p%NumPylons + 1
      call Transfer_Point_to_Point( y%SPyRtrLoads(j), m%FusOLoads, m%SPyRtr_P_2_P(j), errStat2, errMsg2, u%SPyRtrMotions(j), u%FusOMotions )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_m%SPyRtr_P_2_P' )    
      kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
      kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
      call Transfer_Point_to_Point( y%SPyRtrLoads(j+1), m%FusOLoads, m%SPyRtr_P_2_P(j+1), errStat2, errMsg2, u%SPyRtrMotions(j+1), u%FusOMotions )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_m%SPyRtr_P_2_P' )    
      kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
      kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
      
      call Transfer_Point_to_Point( y%PPyRtrLoads(j), m%FusOLoads, m%PPyRtr_P_2_P(j), errStat2, errMsg2, u%PPyRtrMotions(j), u%FusOMotions )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_m%PPyRtr_P_2_P' )    
      kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
      kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
      call Transfer_Point_to_Point( y%PPyRtrLoads(j+1), m%FusOLoads, m%PPyRtr_P_2_P(j+1), errStat2, errMsg2, u%PPyRtrMotions(j+1), u%FusOMotions )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' ComputeKiteLoads: Transfer_m%PPyRtr_P_2_P' )    
      kiteForces  = kiteForces  + m%FusOLoads%Force (:,1)
      kiteMoments = kiteMoments + m%FusOLoads%Moment(:,1)
      
   end do
   
end subroutine ComputeKiteLoads

subroutine ComputeAeroOnMotionNodes(i, outNds, motionMesh, VSMoffset, u_VSM, y_VSM, u_V, VSMchords, VSMelemLens, airDens, kinVisc, &
                                    speedOfSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, Cl, Cd, Cm, Fl, Fd, &
                                    Mm, Cn, Cc, Fn, Fc, errStat, errMsg )
            
   integer(IntKi)      , intent(in   ) :: i 
   integer(IntKi)      , intent(in   ) :: outNds(:)
   type(MeshType)      , intent(in   ) :: motionMesh
   integer(IntKi)      , intent(in   ) :: VSMoffset
   type(VSM_InputType) , intent(in   ) :: u_VSM
   type(VSM_OutputType), intent(in   ) :: y_VSM
   real(ReKi)          , intent(in   ) :: u_V(:,:)
   real(ReKi)          , intent(in   ) :: VSMchords(:)
   real(ReKi)          , intent(in   ) :: VSMelemLens(:)
   real(ReKi)          , intent(in   ) :: airDens
   real(ReKi)          , intent(in   ) :: kinVisc
   real(ReKi)          , intent(in   ) :: speedOfSound
   real(ReKi)          , intent(  out) :: Vinf_v(3)
   real(ReKi)          , intent(  out) :: Vstruct_v(3)
   real(ReKi)          , intent(  out) :: Vind_v(3)
   real(ReKi)          , intent(  out) :: Vrel
   real(ReKi)          , intent(  out) :: DynP
   real(ReKi)          , intent(  out) :: Re
   real(ReKi)          , intent(  out) :: XM
   real(ReKi)          , intent(  out) :: AoA
   real(ReKi)          , intent(  out) :: Cl
   real(ReKi)          , intent(  out) :: Cd
   real(ReKi)          , intent(  out) :: Cm
   real(ReKi)          , intent(  out) :: Fl
   real(ReKi)          , intent(  out) :: Fd
   real(ReKi)          , intent(  out) :: Mm
   real(ReKi)          , intent(  out) :: Cn
   real(ReKi)          , intent(  out) :: Cc
   real(ReKi)          , intent(  out) :: Fn
   real(ReKi)          , intent(  out) :: Fc
   integer(IntKi)      , intent(  out) :: errStat     !< Error status of the operation
   character(*)        , intent(  out) :: errMsg      !< Error message if errStat /= ErrID_None

   ! NOTE: All of the Motion node output quantities are computed via the underlying VSM elements
   !       In order to avoid inconsistencies in mapping these element-level quantities to related
   !       motion nodes [user-inputted nodes], we are simply finding the single VSM element
   !       which contains the motion node and applying quantities of the VSM element directly to
   !       the motion node.  This is obviously a compromise.  Where physically possible the motion node will
   !       be the inboard node of a chosen VSM element (not the outboard node).  We cannot therefore compute
   !       Aero quantities for the most outboard motion node on a span.  In the case where the geometry is
   !       not defined inboard to outboard, the node closer to the start of the user-defined geometry node-list
   !       will be selected, and hence the last node in the list cannot be selected for outputs

      ! Local Variables
   real(ReKi)       :: DCM(3,3)
   real(ReKi)       :: chord, factor, elemLen, F2d, Vx, Vy
   real(ReKi)       :: Vrel_v(3), Vrel_hat(3), forces(3), moments(3), d_v(3), d_hat(3), l_hat(3)
   integer(IntKi)   :: VSMElemIndx, compElemIndx, n1, n2
   character(*), parameter   :: routineName = 'ComputeAeroOnMotionNodes'
   
         ! Initialize variables for this routine
   errStat  = ErrID_None
   errMsg   = ""
   
   call FindVSMElemBasedOnMotionNode( outNds(i), motionMesh, VSMoffset, VSMElemIndx)
   
   if ( VSMElemIndx == 0 ) then
      errMsg  = 'node '//trim(num2lstr(outNds(i)))//' cannot be related to a VSM element.  You cannot output data for the last node in a component list, or the first of two co-located nodes.'
      errStat = ErrID_Fatal
      return
   end if
   
   compElemIndx = VSMElemIndx - VSMoffset
   
     
      ! Orientation is transform from global to local
      ! Find DCM for VSM element closest to the requested output node
   DCM(1,:) = u_VSM%x_hat(:,VSMElemIndx)
   DCM(2,:) = u_VSM%y_hat(:,VSMElemIndx)
   DCM(3,:) = u_VSM%z_hat(:,VSMElemIndx)
  
       
      ! Average the structural velocities of the nodes making up this VSM element in global coords
   n1 = motionMesh%ELEMLIST(compElemIndx)%ELEMENT%ELEMNODES(1)
   n2 = motionMesh%ELEMLIST(compElemIndx)%ELEMENT%ELEMNODES(2)     
   Vstruct_v = (motionMesh%TranslationVel(:,n1) + motionMesh%TranslationVel(:,n2)) / 2.0_ReKi

      ! Gobal coord inflow  minus the average structural motion is the VSM element inflow, so we need to add back the structural motions 
   Vinf_v = u_VSM%U_Inf_v(:,VSMElemIndx) + Vstruct_v

      ! Obtain Induced velocity for the Motion node in global coordinates
   Vind_v = y_VSM%Vind(:,VSMElemIndx)

      ! Relative velocity in global coordinates
   Vrel_v   = u_VSM%U_Inf_v(:,VSMElemIndx) + Vind_v
   
         ! Local coord ambient inflow, structural motion, relative and induced velocity
   Vinf_v    = matmul( DCM,Vinf_v    )
   Vstruct_v = matmul( DCM,Vstruct_v )
   Vind_v    = matmul( DCM,Vind_v    ) 
   Vrel_v    = matmul( DCM,Vrel_v    ) 
   Vx        = Vrel_v(1)
   Vy        = Vrel_v(2)
   Vrel     = sqrt(Vx**2 + Vy**2)
   !Vrel_hat = Vrel_v / Vrel
   chord    = VSMchords(VSMElemIndx)
   elemLen  = VSMelemLens(VSMElemIndx)
   Re       = Vrel * chord / (1e6*kinVisc)  ! in millions
   XM       = Vrel / speedOfSound
   DynP     = 0.5 * airDens * Vrel**2
   
   Cl  = y_VSM%Cl(VSMElemIndx)
   Cd  = y_VSM%Cd(VSMElemIndx)
   Cm  = y_VSM%Cm(VSMElemIndx)
   AoA = y_VSM%AoA(VSMElemIndx)
   Cn  =  Cl*cos(AoA)+Cd*sin(AoA)
   Cc  = -Cl*sin(AoA)+Cd*cos(AoA)  ! This is negative of the original Ct definition
   
      ! Forces per unit length
   Fc = DynP*Cc
   Fn = DynP*Cn
   Fl = DynP*Cl
   Fd = DynP*Cd
   Mm = DynP*Cm*chord
   
         
end subroutine ComputeAeroOnMotionNodes

!----------------------------------------------------------------------------------------------------------------------------------
!> This routine parse the KiteAeroDyn input file.
subroutine ReadKADFile(InitInp, interval, errStat, errMsg)

   type(KAD_InitInputType),          intent(inout)  :: InitInp     !< Initialization input data for KiteAeroDyn
   real(DbKi),                       intent(in   )  :: interval    !< Default time step for aerodynamics calculations, s
   integer(IntKi),                   intent(  out)  :: errStat     !< Error status of the operation
   character(*),                     intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None

         ! Local variables
   integer(IntKi)            :: i                ! loop counter
   character(ErrMsgLen)      :: errMsg2          ! temporary Error message if errStat /= ErrID_None
   integer(IntKi)            :: errStat2, IOS    ! temporary Error status of the operation
   character(1024)           :: fileName         ! file name
   integer(IntKi)            :: UnIn, UnEc       ! file units
   character(1024)           :: FTitle           ! "File Title": the 2nd line of the input file, which contains a description of its contents
   character(200)            :: Line             ! temporary storage of a line from the input file (to compare with "default")
   logical                   :: Echo             ! echo flag, true=echo the file
   character(*), parameter   :: routineName = 'ReadKADFile'
   
         ! Initialize variables for this routine
   errStat  = ErrID_None
   errMsg   = ""
   UnEc     = -1 
   Echo = .false.   
   fileName = trim(InitInp%Filename)
   
   call GetNewUnit( UnIn )   
  
   call OpenFInpfile(UnIn, trim(fileName), errStat, errMsg)
      if ( errStat /= ErrID_None ) then
         errStat = ErrID_Fatal
         call CleanUp()
         return
      end if
 
   ! Read the lines up/including to the "Echo" simulation control variable
   ! If echo is FALSE, don't write these lines to the echo file. 
   ! If Echo is TRUE, rewind and write on the second try.
   
   i = 1 !set the number of times we've read the file
   do 
   !----------- HEADER -------------------------------------------------------------
   
      call ReadCom( UnIn, fileName, 'KiteAeroDyn input file header (line 1)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
      call ReadStr( UnIn, fileName, FTitle, 'FTitle', 'KiteAeroDyn input file header: File Description (line 2)', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         if ( errStat >= AbortErrLev ) then
            call Cleanup()
            return
         end if
   

   !-------------------------- SIMULATION CONTROL ------------------------

      ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' SIMULATION CONTROL ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      
      ! Echo - Echo input to "<RootName>.AD.ech".
   
   call ReadVar( UnIn, fileName, Echo, 'Echo',   'Echo flag', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
   
   if (.NOT. Echo .OR. i > 1) EXIT !exit this loop
   
      ! Otherwise, open the echo file, then rewind the input file and echo everything we've read
      
   i = i + 1         ! make sure we do this only once (increment counter that says how many times we've read this file)
   
   call OpenEcho ( UnEc, trim(filename)//'.ech', errStat2, errMsg2, KAD_Ver )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if ( errStat >= AbortErrLev ) then
         call Cleanup()
         return
      end if
   
   if ( UnEc > 0 )  write (UnEc,'(/,A,/)')  'Data from '//trim(KAD_Ver%Name)//' primary input file "'//trim( fileName )//'":'
         
   rewind( UnIn, IOSTAT=errStat2 )  
      if (errStat2 /= 0_IntKi ) then
         call SetErrStat( ErrID_Fatal, 'Error rewinding file "'//trim(fileName)//'".', errStat, errMsg, RoutineName )
         call Cleanup()
         return
      end if           
   end do    

   if (NWTC_VerboseLevel == NWTC_Verbose) then
      call WrScr( ' Heading of the '//trim(KAD_Ver%Name)//' input file: ' )      
      call WrScr( '   '//trim( FTitle ) )
   end if
   
   
      ! DTAero - Time interval for aerodynamic calculations {or default} (s):
   
   call ReadVarWDefault( UnIn, fileName, InitInp%InpFileData%DTAero, "DTAero", "Time interval for aerodynamic calculations {or default} (s)", interval, errStat2, errMsg2, UnEc)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! The Lift Model
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%LiftMod, 'LiftMod', 'Lifting-line calculation model (-) (switch) {1:geometric AoA, 2:vortex method}', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Rotor calculation model
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%RotorMod, 'RotorMod', 'Rotor calculation model (-) (switch) {0:none, 1:actuator disk}', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Include aerodynamic pitching moment in calculations?
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%UseCm, 'UseCm', 'Include aerodynamic pitching moment in calculations?', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
  
   !-------------------------- ENVIRONMENTAL CONDITIONS ------------------------

      ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' ENVIRONMENTAL CONDITIONS ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Air density
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%AirDens, 'AirDens', 'Air density', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Kinematic air viscosity
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%KinVisc, 'KinVisc', 'Kinematic air viscosity', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
         ! Speed of Sound
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%SpdSound, 'SpdSound', 'Speed of Sound', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   !-------------------------- LIFTING LINE VORTEX METHOD OPTIONS ------------------------

      ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' LIFTING LINE VORTEX METHOD OPTIONS ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%VSMMod, 'VSMMod', 'Trailing vortices alignment model (-) (switch) {1:chord, 2: local free stream}', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   call ReadVarWDefault ( UnIn, fileName, InitInp%InpFileData%VSMToler, 'VSMToler', 'Tolerance in the Newton iterations (m^2/s) or DEFAULT', 0.0001, errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   call ReadVarWDefault ( UnIn, fileName, InitInp%InpFileData%VSMMaxIter, 'VSMMaxIter', 'Maximum number of Newton iterations (-) or DEFAULT', 40, errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   call ReadVarWDefault ( UnIn, fileName, InitInp%InpFileData%VSMPerturb, 'VSMPerturb', 'Perturbation size for computing the Jacobian in the Newton iterations (m^2/s) or DEFAULT', 0.05, errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   !-------------------------- AIRFOIL INFORMATION ------------------------

      ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' AIRFOIL INFORMATION ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Airfoil table interpolation model
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%AFTabMod, 'AFTabMod', 'Airfoil table interpolation model (-) (switch) {1:1D on AoA, 2:2D on AoA and Re, 3:2D on AoA and Ctrl}', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! The column in the airfoil tables that contains the angle of attack
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%InCol_Alfa, 'InCol_Alfa', 'The column in the airfoil tables that contains the angle of attack', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! The column in the airfoil tables that contains the lift coefficient
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%InCol_Cl, 'InCol_Cl', 'The column in the airfoil tables that contains the lift coefficient', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! The column in the airfoil tables that contains the drag coefficient
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%InCol_Cd, 'InCol_Cd', 'The column in the airfoil tables that contains the drag coefficient', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! The column in the airfoil tables that contains the pitching-moment coefficient
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%InCol_Cm, 'InCol_Cm', 'The column in the airfoil tables that contains the pitching-moment coefficient', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

      ! Number of airfoil files used
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%NumAFfiles, 'NumAFfiles', 'Number of airfoil files used', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   call AllocAry( InitInp%InpFileData%AFNames, InitInp%InpFileData%NumAFfiles, 'InitInp%InpFileData%AFNames', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   do i=1, InitInp%InpFileData%NumAFfiles
      call ReadVar ( UnIn, fileName, InitInp%InpFileData%AFNames(i), 'AFNames', 'Airfoil filename', errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )   
   end do
   
   !-------------------------- FUSELAGE PROPERTIES ------------------------
   call ReadProps( UnIn, fileName, InitInp%InpFileData%FusProps%NumNds, InitInp%InpFileData%FusProps, 6, 'Fuselage', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   !-------------------------- STARBOARD WING PROPERTIES ------------------------   
   call ReadWngProps( UnIn, fileName, InitInp%InpFileData%SWnProps%NumNds, InitInp%InpFileData%SWnProps, 7, 'Starboard Wing', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    
      
   !-------------------------- PORT WING PROPERTIES ------------------------   
   call ReadWngProps( UnIn, fileName, InitInp%InpFileData%PWnProps%NumNds, InitInp%InpFileData%PWnProps, 7, 'Port Wing', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    
     
   !-------------------------- VERTICAL STABILIZER PROPERTIES ------------------------   
   call ReadProps( UnIn, fileName, InitInp%InpFileData%VSProps%NumNds, InitInp%InpFileData%VSProps, 7, 'Vertical Stabilizer', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    

   !-------------------------- STARBOARD HORIZONTAL STABILIER PROPERTIES ------------------------   
   call ReadProps( UnIn, fileName, InitInp%InpFileData%SHSProps%NumNds, InitInp%InpFileData%SHSProps, 7, 'Starboard Horizontal stabilizer', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    

   !-------------------------- PORT HORIZONTAL STABILIER PROPERTIES ------------------------   
   call ReadProps( UnIn, fileName, InitInp%InpFileData%PHSProps%NumNds, InitInp%InpFileData%PHSProps, 7, 'Port Horizontal stabilizer', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    

   allocate( InitInp%InpFileData%SPyProps(InitInp%NumPylons), STAT= errStat2)
      if ( errStat2 /= 0 ) call SetErrStat( ErrID_FATAL, 'Could not allocate memory for SPyProps', errStat, errMsg, RoutineName )
   allocate( InitInp%InpFileData%PPyProps(InitInp%NumPylons), STAT= errStat2)
      if ( errStat2 /= 0 ) call SetErrStat( ErrID_FATAL, 'Could not allocate memory for PPyProps', errStat, errMsg, RoutineName )

   !-------------------------- PYLON PROPERTIES ------------------------   
      call ReadPyProps( UnIn, fileName, InitInp%NumPylons, InitInp%InpFileData%SPyProps, InitInp%InpFileData%PPyProps, errStat2, errMsg2, UnEc )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    

   !-------------------------- ROTOR PROPERTIES ------------------------   
   allocate( InitInp%InpFileData%RtrProps(InitInp%NumPylons*2*2), STAT= errStat2)
      if ( errStat2 /= 0 ) call SetErrStat( ErrID_FATAL, 'Could not allocate memory for RtrProps', errStat, errMsg, RoutineName )
   
   call ReadRotorProps( UnIn, fileName, InitInp%NumPylons*2*2, InitInp%InpFileData%RtrProps, errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    

   
!---------------------- OUTPUT --------------------------------------------------
   CALL ReadCom( UnIn, fileName, 'Section Header: Output', ErrStat2, ErrMsg2, UnEc )
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      ! SumPrint - Print summary data to <RootName>.sum (flag):
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%SumPrint, "SumPrint", "Print summary data to <RootName>.sum (flag)", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      ! OutSwtch - Switch to determine where output will be placed: (1: in module output file only; 2: in glue code output file only; 3: both) (-):
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%OutSwtch, "OutSwtch", "Switch to determine where output will be placed: (1: in module output file only; 2: in glue code output file only; 3: both) (-)", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

   !   ! TabDelim - Flag to cause tab-delimited text output (delimited by space otherwise) (flag):
   !CALL ReadVar( UnIn, fileName, InitInp%InpFileData%TabDelim, "TabDelim", "Flag to cause tab-delimited text output (delimited by space otherwise) (flag)", ErrStat2, ErrMsg2, UnEc)
   !   CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
   !   IF ( ErrStat >= AbortErrLev ) THEN
   !      CALL Cleanup()
   !      RETURN
   !   END IF

      ! OutFmt - Format used for module's text tabular output (except time); resulting field should be 10 characters (-):
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%OutFmt, "OutFmt", "Format used for module's text tabular output (except time); resulting field should be 10 characters (-)", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      
      ! NFusOuts - Number of fuselage node outputs (-) [0 to 9]:
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%NFusOuts, "NFusOuts", "Number of fuselage node outputs (-) [0 to 9]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      IF ( InitInp%InpFileData%NFusOuts > SIZE(InitInp%InpFileData%FusOutNd) ) THEN
         CALL SetErrStat( ErrID_Warn, 'Number of fuselage node outputs exceeds '// &
                                      TRIM(Num2LStr(SIZE(InitInp%InpFileData%FusOutNd)))//'.', ErrStat, ErrMsg, RoutineName )
         IF ( ErrStat >= AbortErrLev ) THEN
            CALL Cleanup()
            RETURN
         END IF
         InitInp%InpFileData%NFusOuts = SIZE(InitInp%InpFileData%FusOutNd)
      END IF
      
      ! FusOutNd - List of fuselage nodes whose values will be output (-) [1 to NFusOuts] [unused for NFusOuts=0]:
   CALL ReadAry( UnIn, fileName, InitInp%InpFileData%FusOutNd, InitInp%InpFileData%NFusOuts, "FusOutNd", "List of fuselage nodes whose values will be output (-) [1 to NFusOuts] [unused for NFusOuts=0]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF
      
      ! NSWnOuts - Number of starboard wing node outputs (-) [0 to 9]:
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%NSWnOuts, "NSWnOuts", "Number of starboard wing node outputs (-) [0 to 9]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      IF ( InitInp%InpFileData%NSWnOuts > SIZE(InitInp%InpFileData%SWnOutNd) ) THEN
         CALL SetErrStat( ErrID_Warn, 'Number of starboard wing node outputs exceeds '// &
                                      TRIM(Num2LStr(SIZE(InitInp%InpFileData%SWnOutNd)))//'.', ErrStat, ErrMsg, RoutineName )
         IF ( ErrStat >= AbortErrLev ) THEN
            CALL Cleanup()
            RETURN
         END IF
         InitInp%InpFileData%NSWnOuts = SIZE(InitInp%InpFileData%SWnOutNd)
      END IF
      
      ! SWnOutNd - List of starboard wing nodes whose values will be output (-) [1 to NSWnOuts] [unused for NSWnOuts=0]:
   CALL ReadAry( UnIn, fileName, InitInp%InpFileData%SWnOutNd, InitInp%InpFileData%NSWnOuts, "SWnOutNd", "List of starboard wing nodes whose values will be output (-) [1 to NSWnOuts] [unused for NSWnOuts=0]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      ! NPWnOuts - Number of port wing node outputs (-) [0 to 9]:
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%NPWnOuts, "NPWnOuts", "Number of port wing node outputs (-) [0 to 9]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      IF ( InitInp%InpFileData%NPWnOuts > SIZE(InitInp%InpFileData%PWnOutNd) ) THEN
         CALL SetErrStat( ErrID_Warn, 'Number of port wing node outputs exceeds '// &
                                      TRIM(Num2LStr(SIZE(InitInp%InpFileData%PWnOutNd)))//'.', ErrStat, ErrMsg, RoutineName )
         IF ( ErrStat >= AbortErrLev ) THEN
            CALL Cleanup()
            RETURN
         END IF
         InitInp%InpFileData%NPWnOuts = SIZE(InitInp%InpFileData%PWnOutNd)
      END IF
      
      ! PWnOutNd - List of port wing nodes whose values will be output (-) [1 to NPWnOuts] [unused for NPWnOuts=0]:
   CALL ReadAry( UnIn, fileName, InitInp%InpFileData%PWnOutNd, InitInp%InpFileData%NPWnOuts, "PWnOutNd", "List of port wing nodes whose values will be output (-) [1 to NPWnOuts] [unused for NPWnOuts=0]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      ! NVSOuts - Number of vertical stabilizer node outputs (-) [0 to 9]:
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%NVSOuts, "NVSOuts", "Number of vertical stabilizer node outputs (-) [0 to 9]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      IF ( InitInp%InpFileData%NVSOuts > SIZE(InitInp%InpFileData%VSOutNd) ) THEN
         CALL SetErrStat( ErrID_Warn, 'Number of vertical stabilizer node outputs exceeds '// &
                                      TRIM(Num2LStr(SIZE(InitInp%InpFileData%VSOutNd)))//'.', ErrStat, ErrMsg, RoutineName )
         IF ( ErrStat >= AbortErrLev ) THEN
            CALL Cleanup()
            RETURN
         END IF
         InitInp%InpFileData%NVSOuts = SIZE(InitInp%InpFileData%VSOutNd)
      END IF
      
      ! VSOutNd - List of fuselage nodes whose values will be output (-) [1 to NVSOuts] [unused for NVSOuts=0]:
   CALL ReadAry( UnIn, fileName, InitInp%InpFileData%VSOutNd, InitInp%InpFileData%NVSOuts, "VSOutNd", "List of vertical stabilizer nodes whose values will be output (-) [1 to NVSOuts] [unused for NVSOuts=0]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      ! NSHSOuts - Number of starboard horizontal stabilizer node outputs (-) [0 to 9]:
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%NSHSOuts, "NSHSOuts", "Number of starboard horizontal stabilizer node outputs (-) [0 to 9]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      IF ( InitInp%InpFileData%NSHSOuts > SIZE(InitInp%InpFileData%SHSOutNd) ) THEN
         CALL SetErrStat( ErrID_Warn, 'Number of starboard horizontal stabilizer node outputs exceeds '// &
                                      TRIM(Num2LStr(SIZE(InitInp%InpFileData%SHSOutNd)))//'.', ErrStat, ErrMsg, RoutineName )
         IF ( ErrStat >= AbortErrLev ) THEN
            CALL Cleanup()
            RETURN
         END IF
         InitInp%InpFileData%NSHSOuts = SIZE(InitInp%InpFileData%SHSOutNd)
      END IF
      
      ! SHSOutNd - List of starboard horizontal stabilizer nodes whose values will be output (-) [1 to NSHSOuts] [unused for NSHSOuts=0]:
   CALL ReadAry( UnIn, fileName, InitInp%InpFileData%SHSOutNd, InitInp%InpFileData%NSHSOuts, "SHSOutNd", "List of starboard horizontal stabilizer nodes whose values will be output (-) [1 to NSHSOuts] [unused for NSHSOuts=0]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      ! NPHSOuts - Number of port horizontal stabilizer node outputs (-) [0 to 9]:
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%NPHSOuts, "NPHSOuts", "Number of port horizontal stabilizer node outputs (-) [0 to 9]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      IF ( InitInp%InpFileData%NPHSOuts > SIZE(InitInp%InpFileData%PHSOutNd) ) THEN
         CALL SetErrStat( ErrID_Warn, 'Number of port horizontal stabilizer node outputs exceeds '// &
                                      TRIM(Num2LStr(SIZE(InitInp%InpFileData%PHSOutNd)))//'.', ErrStat, ErrMsg, RoutineName )
         IF ( ErrStat >= AbortErrLev ) THEN
            CALL Cleanup()
            RETURN
         END IF
         InitInp%InpFileData%NPHSOuts = SIZE(InitInp%InpFileData%PHSOutNd)
      END IF
      
      ! PHSOutNd - List of port horizontal stabilizer nodes whose values will be output (-) [1 to NPHSOuts] [unused for NPHSOuts=0]:
   CALL ReadAry( UnIn, fileName, InitInp%InpFileData%PHSOutNd, InitInp%InpFileData%NPHSOuts, "PHSOutNd", "List of port horizontal stabilizer nodes whose values will be output (-) [1 to NPHSOuts] [unused for NPHSOuts=0]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      ! NPylOuts - Number of pylon node outputs (-) [0 to 9]:
   CALL ReadVar( UnIn, fileName, InitInp%InpFileData%NPylOuts, "NPylOuts", "Number of pylon node outputs (-) [0 to 9]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      IF ( InitInp%InpFileData%NPylOuts > SIZE(InitInp%InpFileData%PylOutNd) ) THEN
         CALL SetErrStat( ErrID_Warn, 'Number of pylon node outputs exceeds '// &
                                      TRIM(Num2LStr(SIZE(InitInp%InpFileData%PylOutNd)))//'.', ErrStat, ErrMsg, RoutineName )
         IF ( ErrStat >= AbortErrLev ) THEN
            CALL Cleanup()
            RETURN
         END IF
         InitInp%InpFileData%NPylOuts = SIZE(InitInp%InpFileData%PylOutNd)
      END IF
      
      ! PylOutNd - List of fuselage nodes whose values will be output (-) [1 to NPylOuts] [unused for NPylOuts=0]:
   CALL ReadAry( UnIn, fileName, InitInp%InpFileData%PylOutNd, InitInp%InpFileData%NPylOuts, "PylOutNd", "List of pylon nodes whose values will be output (-) [1 to NPylOuts] [unused for NPylOuts=0]", ErrStat2, ErrMsg2, UnEc)
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF


   !---------------------- OUTLIST  --------------------------------------------
   CALL ReadCom( UnIn, fileName, 'Section Header: OutList', errStat2, errMsg2, UnEc )
      CALL SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

      ! OutList - List of user-requested output channels (-):
   CALL AllocAry( InitInp%InpFileData%OutList, MaxOutPts, "KiteAeroDyn Input File's Outlist", ErrStat2, ErrMsg2 )
      CALL SetErrStat( ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName )
      IF ( ErrStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

   CALL ReadOutputList ( UnIn, fileName, InitInp%InpFileData%OutList, InitInp%InpFileData%NumOuts, 'OutList', "List of user-requested output channels", errStat2, errMsg2, UnEc  )     ! Routine in NWTC Subroutine Library
      CALL SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      IF ( errStat >= AbortErrLev ) THEN
         CALL Cleanup()
         RETURN
      END IF

   
contains
   
         !====================================================================================================
         subroutine Cleanup()
         !     The routine cleans up the module echo file and resets the NWTC_Library, reattaching it to 
         !     any existing echo information
         !----------------------------------------------------------------------------------------------------  
      
                 
            close(UnIn)
            close(UnEc)
            
         end subroutine Cleanup
         
end subroutine ReadKADFile


!----------------------------------------------------------------------------------------------------------------------------------
!< This routine is called as part of the initialization step.
!< The parameters are set here and not changed during the simulation.
!< The initial states and initial guess for the input are defined.
subroutine ValidateInitData(InitInp, errStat, errMsg)

   type(KAD_InitInputType),          intent(in   )  :: InitInp
   integer(IntKi),                   intent(  out)  :: errStat     !< Error status of the operation
   character(*),                     intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None

         ! Local variables
   character(*), parameter                      :: routineName = 'ValidateInitData'
   type(KAD_InputFile)                          :: InpData
   integer(IntKi)                               :: i
   
         ! Initialize variables for this routine
   errStat = ErrID_None
   errMsg  = ""
   InpData = InitInp%InpFileData
   
   if ( InpData%DTAero   <= 0.0_DbKi ) call SetErrStat( ErrID_Fatal, 'Simulation timestep must be greater than zero', errStat, errMsg, routineName )
   if ( (InpData%LiftMod   < 1) .or. (InpData%LiftMod   > 2) ) call SetErrStat( ErrID_Fatal, 'LiftMod must be set to 1 (geometric AoA) or 2 (vortex-step)', errStat, errMsg, routineName )
   if ( (InpData%RotorMod  < 0) .or. (InpData%RotorMod  > 1) ) call SetErrStat( ErrID_Fatal, 'RotorMod must be set to 0 (none) or 1 (actuator disk)', errStat, errMsg, routineName )
   if ( InpData%AirDens   <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'Air Density must be greater than zero', errStat, errMsg, routineName )
   if ( InpData%KinVisc   <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'Kinematic air viscosity must be greater than zero', errStat, errMsg, routineName )
   if ( InpData%SpdSound   <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'Speed of sound must be greater than zero', errStat, errMsg, routineName )
   if ( InpData%VSMToler   <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'VSM tolerance in the Newton iterations must be greater than zero', errStat, errMsg, routineName )
   if ( InpData%VSMMaxIter   <= 0 ) call SetErrStat( ErrID_Fatal, 'VSM maximum number of Newton iterations must be greater than zero', errStat, errMsg, routineName )
   if ( InpData%VSMPerturb   <= 0.0_ReKi ) call SetErrStat( ErrID_Fatal, 'VSM perturbation size for computing the Jacobian in the Newton iterations must be greater than zero', errStat, errMsg, routineName )
   if ( (InpData%VSMMod  < 1) .or. (InpData%VSMMod  > 2) ) call SetErrStat( ErrID_Fatal, 'VSMMod must be set to 1 (chord) or 2 (local free stream)', errStat, errMsg, routineName )
   if ( (InpData%AFTabMod   < 1) .or. (InpData%AFTabMod   > 3) ) call SetErrStat( ErrID_Fatal, 'AFTabMod must be set to 1 (1D on AoA) or 2 (2D on AoA and Re) or 3 (2D on AoA and Ctrl)', errStat, errMsg, routineName )
   if ( InpData%FusProps%NumNds < 2 ) call SetErrStat( ErrID_Fatal, 'Number of fuselage nodes must be greater than 1', errStat, errMsg, routineName )
   if ( InpData%SWnProps%NumNds < 2 ) call SetErrStat( ErrID_Fatal, 'Number of starboard wing nodes must be greater than 1', errStat, errMsg, routineName )
   if ( InpData%PWnProps%NumNds < 2 ) call SetErrStat( ErrID_Fatal, 'Number of port wing nodes must be greater than 1', errStat, errMsg, routineName )
   if ( InpData%VSProps%NumNds  < 2 ) call SetErrStat( ErrID_Fatal, 'Number of vertical stabilizer nodes must be greater than 1', errStat, errMsg, routineName )
   if ( InpData%SHSProps%NumNds < 2 ) call SetErrStat( ErrID_Fatal, 'Number of starboard horizontal stabilizer nodes must be greater than 1', errStat, errMsg, routineName )
   if ( InpData%PHSProps%NumNds < 2 ) call SetErrStat( ErrID_Fatal, 'Number of port horizontal stabilizer nodes must be greater than 1', errStat, errMsg, routineName )
   if ( InpData%SPyProps(1)%NumNds < 2 ) call SetErrStat( ErrID_Fatal, 'Number of pylon nodes must be greater than 1', errStat, errMsg, routineName )
   if ( (InpData%OutSwtch   < 1) .or. (InpData%OutSwtch   > 3) ) call SetErrStat( ErrID_Fatal, 'OutSwtch must be set to 1=KiteAeroDyn.out, 2=GlueCode.out, 3=both files', errStat, errMsg, routineName )
   
   
   ! TODO: Decide what we will check in terms of the node coordinates and viability of component geometry
   
end subroutine ValidateInitData

!====================================================================================================
SUBROUTINE KAD_OpenOutput( ProgVer, OutRootName,  p, InitOut, ErrStat, ErrMsg )
! This subroutine initialized the output module, checking if the output parameter list (OutList)
! contains valid names, and opening the output file if there are any requested outputs
!----------------------------------------------------------------------------------------------------

   

      ! Passed variables

   TYPE(ProgDesc),                INTENT( IN    ) :: ProgVer
   CHARACTER(*),                  INTENT( IN    ) :: OutRootName          ! Root name for the output file
   TYPE(KAD_ParameterType),        INTENT( INOUT ) :: p   
   TYPE(KAD_InitOutPutType ),      INTENT( IN    ) :: InitOut              !
   INTEGER,                       INTENT(   OUT ) :: ErrStat              ! a non-zero value indicates an error occurred           
   CHARACTER(*),                  INTENT(   OUT ) :: ErrMsg               ! Error message if ErrStat /= ErrID_None
   
      ! Local variables
   INTEGER                                        :: I                    ! Generic loop counter      
   CHARACTER(1024)                                :: OutFileName          ! The name of the output file  including the full path.
   CHARACTER(200)                                 :: Frmt                 ! a string to hold a format statement
   INTEGER                                        :: ErrStat2              
   
   !-------------------------------------------------------------------------------------------------      
   ! Initialize local variables
   !-------------------------------------------------------------------------------------------------      
   ErrStat = ErrID_None  
   ErrMsg  = ""
   p%OutSFmt = "A10"      
   !-------------------------------------------------------------------------------------------------      
   ! Open the output file, if necessary, and write the header
   !-------------------------------------------------------------------------------------------------      
   
   IF ( ALLOCATED( p%OutParam ) .AND. p%NumOuts > 0 ) THEN           ! Output has been requested so let's open an output file            
      
         ! Open the file for output
      OutFileName = TRIM(OutRootName)//'.KAD.out'
      CALL GetNewUnit( p%UnOutFile )
   
      CALL OpenFOutFile ( p%UnOutFile, OutFileName, ErrStat, ErrMsg ) 
      IF ( ErrStat >= AbortErrLev ) THEN
         ErrMsg = ' Error opening KiteAeroDyn-level output file: '//TRIM(ErrMsg)
         RETURN
      END IF
      
       
         ! Write the output file header
      
      WRITE (p%UnOutFile,'(/,A/)', IOSTAT=ErrStat2)  'These predictions were generated by '//TRIM(GETNVD(ProgVer))//&
                      ' on '//CurDate()//' at '//CurTime()//'.'
      
      WRITE(p%UnOutFile, '(//)') ! add 3 lines to make file format consistant with FAST v8 (headers on line 7; units on line 8) [this allows easier post-processing]
      
         ! Write the names of the output parameters:
      
      Frmt = '(A8,'//TRIM(Int2LStr(p%NumOuts))//'(:,A,'//TRIM( p%OutSFmt )//'))'
   
      WRITE(p%UnOutFile,Frmt, IOSTAT=ErrStat2)  TRIM( 'Time' ), ( p%Delim, TRIM( InitOut%WriteOutputHdr(I) ), I=1,p%NumOuts )
                        
      
         ! Write the units of the output parameters:                 
      WRITE(p%UnOutFile,Frmt, IOSTAT=ErrStat2)  TRIM( 's'), ( p%Delim, TRIM( InitOut%WriteOutputUnt(I) ), I=1,p%NumOuts )
                     
   
      
   END IF   ! there are any requested outputs   

   RETURN

END SUBROUTINE KAD_OpenOutput

!====================================================================================================

!====================================================================================================
SUBROUTINE KAD_WrOutputLine( t, p, WriteOutputs, ErrStat, ErrMsg)
! This routine writes the module output to the output file.
!----------------------------------------------------------------------------------------------------

   IMPLICIT                        NONE
   
      ! Passed variables
   REAL(DbKi), INTENT(IN)                  :: t                                  !< Current simulation time, in seconds
   TYPE(KAD_ParameterType), INTENT(IN)     :: p                                  !< KAD parameters
   REAL(ReKi),               INTENT(IN)    :: WriteOutputs (:)                   !< KAD WriteOutput values
   INTEGER(IntKi),           INTENT(OUT)   :: ErrStat                            !< Error status
   CHARACTER(*),             INTENT(OUT)   :: ErrMsg                             !< Error message

      ! Local variables.

   CHARACTER(200)                   :: Frmt                                      ! A string to hold a format specifier
   CHARACTER(10)                    :: TmpStr                                    ! temporary string to print the time output as text

   
   ErrStat = ErrID_None
   ErrMsg  = ''
   
 
         ! Write one line of tabular output:
   !   Frmt = '(F8.3,'//TRIM(Num2LStr(p%NumOuts))//'(:,A,'//TRIM( p%OutFmt )//'))'
      Frmt = '"'//p%Delim//'"'//p%OutFmt      ! format for array elements from individual modules

            ! time
      WRITE( TmpStr, '(F10.4)' ) t
      CALL WrFileNR( p%UnOutFile, TmpStr )

         ! write the individual module output (convert to SiKi if necessary, so that we don't need to print so many digits in the exponent)
      CALL WrNumAryFileNR ( p%UnOutFile, REAL(WriteOutputs,SiKi), Frmt, ErrStat, ErrMsg )
         !IF ( ErrStat >= AbortErrLev ) RETURN
      
         ! write a new line (advance to the next line)
      WRITE (p%UnOutFile,'()')
      
END SUBROUTINE KAD_WrOutputLine

!====================================================================================================
SUBROUTINE KAD_CloseOutput ( p, ErrStat, ErrMsg )
! This function cleans up after running the SubDyn output module. It closes the output file,
! releases memory, and resets the number of outputs requested to 0.
!----------------------------------------------------------------------------------------------------

         ! Passed variables

   TYPE(KAD_ParameterType),  INTENT( INOUT )  :: p                    ! data for this instance of the floating platform module        
   INTEGER,                       INTENT(   OUT ) :: ErrStat              ! a non-zero value indicates an error occurred           
   CHARACTER(*),                  INTENT(   OUT ) :: ErrMsg               ! Error message if ErrStat /= ErrID_None

!      ! Internal variables
   LOGICAL                               :: Err


   !-------------------------------------------------------------------------------------------------
   ! Initialize error information
   !-------------------------------------------------------------------------------------------------
   ErrStat = 0
   ErrMsg  = ""
   
   Err     = .FALSE.

   !-------------------------------------------------------------------------------------------------
   ! Close our output file
   !-------------------------------------------------------------------------------------------------
   CLOSE( p%UnOutFile, IOSTAT = ErrStat )
   IF ( ErrStat /= 0 ) Err = .TRUE.

  
 
   !-------------------------------------------------------------------------------------------------
   ! Make sure ErrStat is non-zero if an error occurred
   !-------------------------------------------------------------------------------------------------
   IF ( Err ) ErrStat = ErrID_Fatal
   
   RETURN

END SUBROUTINE KAD_CloseOutput
!====================================================================================================   
SUBROUTINE Transfer_Motions_Line2_to_Point( Src, Dest, MeshMap, ErrStat, ErrMsg )

   TYPE(MeshType),                 INTENT(IN   )  :: Src       !< The source (Line2) mesh with motion fields allocated
   TYPE(MeshType),                 INTENT(INOUT)  :: Dest      !< The destination mesh

   TYPE(MeshMapType),              INTENT(INOUT)  :: MeshMap   !< The mapping data

   INTEGER(IntKi),                 INTENT(  OUT)  :: ErrStat   !< Error status of the operation
   CHARACTER(*),                   INTENT(  OUT)  :: ErrMsg    !< Error message if ErrStat /= ErrID_None

      ! local variables
   INTEGER(IntKi)            :: i , j                          ! counter over the nodes
   INTEGER(IntKi)            :: k                              ! counter components
   INTEGER(IntKi)            :: n, n1, n2                      ! temporary space for node numbers
   REAL(R8Ki)                :: FieldValueN1(3)                ! Temporary variable to store field values on element nodes
   REAL(R8Ki)                :: FieldValueN2(3)                ! Temporary variable to store field values on element nodes
   REAL(ReKi)                :: TmpVec(3)
   REAL(R8Ki)                :: RotationMatrix(3,3)

   REAL(DbKi)                :: FieldValue(3,2)                ! Temporary variable to store values for DCM interpolation
   REAL(DbKi)                :: RotationMatrixD(3,3)
   REAL(DbKi)                :: tensor_interp(3)
   

   ErrStat = ErrID_None
   ErrMsg  = ""

  !> Define \f$ \phi_1 = 1-\bar{l}^S \f$  and 
  !!        \f$ \phi_2 =   \bar{l}^S \f$.

!bjj: FieldValueN1 and FieldValueN2 should really be one matrix of DIM (3,2) now that we've modified some of the other data structures....
             
      ! ---------------------------- Translation ------------------------------------------------
      !> Translational Displacement: \f$\vec{u}^D = \sum\limits_{i=1}^{2}\left( 
      !!              \vec{u}^S_{eSn_i} + \left[\left[\theta^S_{eSn_i}\right]^T \theta^{SR}_{eSn_i} - I\right]\left\{\vec{p}^{ODR}-\vec{p}^{OSR}_{eSn_i}\right\}
      !!              \right) \phi_i\f$

      ! u_Dest1 = u_Src + [Orientation_Src^T * RefOrientation_Src - I] * [p_Dest - p_Src] at Source Node n1
      ! u_Dest2 = u_Src + [Orientation_Src^T * RefOrientation_Src - I] * [p_Dest - p_Src] at Source Node n2
      ! u_Dest = (1.-elem_position)*u_Dest1 + elem_position*u_Dest2
   if ( Src%FieldMask(MASKID_TranslationDisp) .AND. Dest%FieldMask(MASKID_TranslationDisp) ) then
      do i=1, Dest%Nnodes
         !if ( MeshMap%MapMotions(i)%OtherMesh_Element < 1 )  CYCLE

            ! add the translation displacement portion part
         do j=1,NumNodes(ELEMENT_LINE2) ! number of nodes per line2 element
            n = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(j)
         
            FieldValue(:,j) = Src%TranslationDisp(:,n)
         end do
            

            ! if Src mesh has orientation, superpose Dest displacement with translation due to rotation and couple arm
         if ( Src%FieldMask(MASKID_Orientation) ) then

            do j=1,NumNodes(ELEMENT_LINE2) ! number of nodes per line2 element
               n = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(j)
         
                  !Calculate RotationMatrix as O_S^T*O_SR
               RotationMatrix = TRANSPOSE( Src%Orientation(:,:,n ) )
               RotationMatrix = MATMUL( RotationMatrix, Src%RefOrientation(:,:,n) )

                  ! subtract I
               do k=1,3
                  RotationMatrix(k,k)= RotationMatrix(k,k) - 1.0_ReKi
               end do
               
               FieldValue(:,j) = FieldValue(:,j) + MATMUL(RotationMatrix,(Dest%Position(:,i)-Src%Position(:,n)))
                              
            end do
                        
         end if

            ! now form a weighted average of the two points:
         Dest%TranslationDisp(:,i) = MeshMap%MapMotions(i)%shape_fn(1)*FieldValue(:,1)  &
                                   + MeshMap%MapMotions(i)%shape_fn(2)*FieldValue(:,2)

      end do

   end if

      ! ---------------------------- ORIENTATION/Direction Cosine Matrix   ----------------------
      !> Orientation: \f$\theta^D = \Lambda\left( \sum\limits_{i=1}^{2} 
      !!              \log\left( \theta^{DR}\left[\theta^{SR}_{eSn_i}\right]^T\theta^S_{eSn_i} \right)
      !!              \phi_i \right)\f$
      !! where \f$\log()\f$ is nwtc_num::dcm_logmap and \f$\Lambda()\f$ is nwtc_num::dcm_exp
   
   
   
      ! transfer direction cosine matrix, aka orientation

   if ( Src%FieldMask(MASKID_Orientation) .AND. Dest%FieldMask(MASKID_Orientation) ) then

      do i=1, Dest%Nnodes
         !if ( MeshMap%MapMotions(i)%OtherMesh_Element < 1 )  CYCLE
                  
         n1 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(1)
         n2 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(2)

            ! bjj: added this IF statement because of numerical issues when the angle of rotation is pi, 
            !      (where DCM_exp( DCM_logmap (x) ) isn't quite x
         if ( EqualRealNos( MeshMap%MapMotions(i)%shape_fn(1), 1.0_ReKi ) ) then
            
            RotationMatrixD = MATMUL( TRANSPOSE( Src%RefOrientation(:,:,n1) ), Src%Orientation(:,:,n1) )
            RotationMatrixD = MATMUL( Dest%RefOrientation(:,:,i), RotationMatrixD )
      
         elseif ( EqualRealNos( MeshMap%MapMotions(i)%shape_fn(2), 1.0_ReKi ) ) then
            
            RotationMatrixD = MATMUL( TRANSPOSE( Src%RefOrientation(:,:,n2) ), Src%Orientation(:,:,n2) )
            RotationMatrixD = MATMUL( Dest%RefOrientation(:,:,i), RotationMatrixD )
      
         else

               ! calculate Rotation matrix for FieldValueN1 and convert to tensor:
            RotationMatrixD = MATMUL( TRANSPOSE( Src%RefOrientation(:,:,n1) ), Src%Orientation(:,:,n1) )
            RotationMatrixD = MATMUL( Dest%RefOrientation(:,:,i), RotationMatrixD )

            CALL DCM_logmap( RotationMatrixD, FieldValue(:,1), ErrStat, ErrMsg )
            IF (ErrStat >= AbortErrLev) RETURN

               ! calculate Rotation matrix for FieldValueN2 and convert to tensor:
            RotationMatrixD = MATMUL( TRANSPOSE( Src%RefOrientation(:,:,n2) ), Src%Orientation(:,:,n2) )
            RotationMatrixD = MATMUL( Dest%RefOrientation(:,:,i), RotationMatrixD )
         
            CALL DCM_logmap( RotationMatrixD, FieldValue(:,2), ErrStat, ErrMsg )                  
            IF (ErrStat >= AbortErrLev) RETURN
         
            CALL DCM_SetLogMapForInterp( FieldValue )  ! make sure we don't cross a 2pi boundary
         
         
               ! interpolate tensors: 
            tensor_interp =   MeshMap%MapMotions(i)%shape_fn(1)*FieldValue(:,1)  &
                            + MeshMap%MapMotions(i)%shape_fn(2)*FieldValue(:,2)    
                  
               ! convert back to DCM:
            RotationMatrixD = DCM_exp( tensor_interp )
                        
         end if
         
         Dest%Orientation(:,:,i) = REAL( RotationMatrixD, R8Ki )
             
      end do

   endif

      ! ---------------------------- Calculated total displaced positions  ---------------------
      ! these values are used in both the translational velocity and translational acceleration
      ! calculations. The calculations rely on the TranslationDisp fields, which are calculated
      ! earlier in this routine.
   IF ( Src%FieldMask(MASKID_TranslationVel) .OR. Src%FieldMask(MASKID_TranslationAcc) ) THEN
      DO i = 1,Dest%Nnodes
         !if ( MeshMap%MapMotions(i)%OtherMesh_Element < 1 )  CYCLE
      
         DO j=1,NumNodes(ELEMENT_LINE2) ! number of nodes per line2 element
            n = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(j)
         
            MeshMap%DisplacedPosition(:,i,j) =    Src%Position(:,n) +  Src%TranslationDisp(:,n)  &
                                               - Dest%Position(:,i) - Dest%TranslationDisp(:,i)  
         end do
      
      END DO   
   END IF
   
      ! ---------------------------- TranslationVel  --------------------------------------------
      !> Translational Velocity: \f$\vec{v}^D = \sum\limits_{i=1}^{2}\left( 
      !!              \vec{v}^S_{eSn_i} 
      !!              + \left\{ \left\{ \vec{p}^{OSR}_{eSn_i} + \vec{u}^S_{eSn_i} \right\} - \left\{ \vec{p}^{ODR} + \vec{u}^D \right\} \right\} \times \vec{\omega}^S_{eSn_i}
      !!              \right) \phi_i\f$
   
   
   if ( Src%FieldMask(MASKID_TranslationVel) .AND. Dest%FieldMask(MASKID_TranslationVel) ) then
      do i=1, Dest%Nnodes
         !if ( MeshMap%MapMotions(i)%OtherMesh_Element < 1 )  CYCLE

         n1 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(1)
         n2 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(2)

         FieldValueN1 = Src%TranslationVel(:,n1)
         FieldValueN2 = Src%TranslationVel(:,n2)

         if ( Src%FieldMask(MASKID_RotationVel) ) then
            FieldValueN1 = FieldValueN1 + cross_product ( MeshMap%DisplacedPosition(:,i,1), Src%RotationVel(:,n1) )
            FieldValueN2 = FieldValueN2 + cross_product ( MeshMap%DisplacedPosition(:,i,2), Src%RotationVel(:,n2) )
         endif

         Dest%TranslationVel(:,i) = MeshMap%MapMotions(i)%shape_fn(1)*FieldValueN1  &
                                  + MeshMap%MapMotions(i)%shape_fn(2)*FieldValueN2


      end do

   endif

      ! ---------------------------- RotationVel  -----------------------------------------------
      !> Rotational Velocity: \f$\vec{\omega}^D = \sum\limits_{i=1}^{2} 
      !!              \vec{\omega}^S_{eSn_i} 
      !!              \phi_i\f$

   
   if ( Src%FieldMask(MASKID_RotationVel) .AND. Dest%FieldMask(MASKID_RotationVel) ) then
      do i=1, Dest%Nnodes
         !if ( MeshMap%MapMotions(i)%OtherMesh_Element < 1 )  CYCLE

         n1 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(1)
         n2 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(2)

         Dest%RotationVel(:,i) = MeshMap%MapMotions(i)%shape_fn(1)*Src%RotationVel(:,n1)  &
                               + MeshMap%MapMotions(i)%shape_fn(2)*Src%RotationVel(:,n2)
      end do

   end if

      ! ---------------------------- TranslationAcc -----------------------------------------------
      !> Translational Acceleration: \f$\vec{a}^D = \sum\limits_{i=1}^{2}\left( 
      !!              \vec{a}^S_{eSn_i} 
      !!            + \left\{ \left\{ \vec{p}^{OSR}_{eSn_i} + \vec{u}^S_{eSn_i} \right\} - \left\{ \vec{p}^{ODR} + \vec{u}^D \right\} \right\} \times \vec{\alpha}^S_{eSn_i}
      !!            + \vec{\omega}^S_{eSn_i} \times \left\{
      !!              \left\{ \left\{ \vec{p}^{OSR}_{eSn_i} + \vec{u}^S_{eSn_i} \right\} - \left\{ \vec{p}^{ODR} + \vec{u}^D \right\} \right\} \times \vec{\omega}^S_{eSn_i}
      !!              \right\}
      !!              \right) \phi_i\f$

   
   
   
   if ( Src%FieldMask(MASKID_TranslationAcc) .AND. Dest%FieldMask(MASKID_TranslationAcc) ) then
      do i=1, Dest%Nnodes
         !if ( MeshMap%MapMotions(i)%OtherMesh_Element < 1 )  CYCLE

         n1 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(1)
         n2 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(2)

         FieldValueN1 = Src%TranslationAcc(:,n1)
         FieldValueN2 = Src%TranslationAcc(:,n2)


         if ( Src%FieldMask(MASKID_RotationAcc) )  then
            FieldValueN1 = FieldValueN1 + cross_product( MeshMap%DisplacedPosition(:,i,1), Src%RotationAcc(:,n1) )
            FieldValueN2 = FieldValueN2 + cross_product( MeshMap%DisplacedPosition(:,i,2), Src%RotationAcc(:,n2) )
         endif

         if ( Src%FieldMask(MASKID_RotationVel) )  then
            TmpVec = cross_product( MeshMap%DisplacedPosition(:,i,1), Src%RotationVel(:,n1) )
            FieldValueN1 =  FieldValueN1 + cross_product( Src%RotationVel(:,n1), TmpVec )
            
            TmpVec = cross_product( MeshMap%DisplacedPosition(:,i,2), Src%RotationVel(:,n2) )
            FieldValueN2 =  FieldValueN2 + cross_product( Src%RotationVel(:,n2), TmpVec )
                                           
         endif

         Dest%TranslationAcc(:,i) = MeshMap%MapMotions(i)%shape_fn(1)*FieldValueN1  &
                                  + MeshMap%MapMotions(i)%shape_fn(2)*FieldValueN2

      end do
   endif


      ! ---------------------------- RotationAcc  -----------------------------------------------
      !> Rotational Acceleration: \f$\vec{\alpha}^D = \sum\limits_{i=1}^{2} 
      !!              \vec{\alpha}^S_{eSn_i} 
      !!              \phi_i\f$

   if (Src%FieldMask(MASKID_RotationAcc) .AND. Dest%FieldMask(MASKID_RotationAcc) ) then
      do i=1, Dest%Nnodes
         !if ( MeshMap%MapMotions(i)%OtherMesh_Element < 1 )  CYCLE

         n1 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(1)
         n2 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(2)

         Dest%RotationAcc(:,i) = MeshMap%MapMotions(i)%shape_fn(1)*Src%RotationAcc(:,n1)  &
                               + MeshMap%MapMotions(i)%shape_fn(2)*Src%RotationAcc(:,n2)


      end do
   end if

      ! ---------------------------- Scalars  -----------------------------------------------
      !> Scalar: \f$S^D = \sum\limits_{i=1}^{2} 
      !!              S^S_{eSn_i} 
      !!              \phi_i\f$

   if (Src%FieldMask(MASKID_SCALAR) .AND. Dest%FieldMask(MASKID_SCALAR) ) then
      do i=1, Dest%Nnodes
         !if ( MeshMap%MapMotions(i)%OtherMesh_Element < 1 )  CYCLE

         n1 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(1)
         n2 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(2)

         Dest%Scalars(:,i) = MeshMap%MapMotions(i)%shape_fn(1)*Src%Scalars(:,n1)  &
                           + MeshMap%MapMotions(i)%shape_fn(2)*Src%Scalars(:,n2)

      end do
   end if


END SUBROUTINE Transfer_Motions_Line2_to_Point
subroutine Transfer_Orientation(Src, Dest, MeshMap, ErrStat, ErrMsg )

   TYPE(MeshType),                 INTENT(IN   )  :: Src       !< The source (Line2) mesh with motion fields allocated
   TYPE(MeshType),                 INTENT(INOUT)  :: Dest      !< The destination mesh

   TYPE(MeshMapType),              INTENT(INOUT)  :: MeshMap   !< The mapping data

   INTEGER(IntKi),                 INTENT(  OUT)  :: ErrStat   !< Error status of the operation
   CHARACTER(*),                   INTENT(  OUT)  :: ErrMsg    !< Error message if ErrStat /= ErrID_None

      ! local variables
   INTEGER(IntKi)            :: i                              ! counter over the nodes
   INTEGER(IntKi)            :: n1, n2                      ! temporary space for node numbers

   REAL(DbKi)                :: FieldValue(3,2)                ! Temporary variable to store values for DCM interpolation
   REAL(DbKi)                :: RotationMatrixD(3,3)
   REAL(DbKi)                :: tensor_interp(3)
   

   ErrStat = ErrID_None
   ErrMsg  = ""

    ! transfer direction cosine matrix, aka orientation

   if ( Src%FieldMask(MASKID_Orientation) .AND. Dest%FieldMask(MASKID_Orientation) ) then

      do i=1, Dest%Nnodes
         !if ( MeshMap%MapMotions(i)%OtherMesh_Element < 1 )  CYCLE
                  
         n1 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(1)
         n2 = Src%ElemTable(ELEMENT_LINE2)%Elements(MeshMap%MapMotions(i)%OtherMesh_Element)%ElemNodes(2)

            ! bjj: added this IF statement because of numerical issues when the angle of rotation is pi, 
            !      (where DCM_exp( DCM_logmap (x) ) isn't quite x
         if ( EqualRealNos( MeshMap%MapMotions(i)%shape_fn(1), 1.0_ReKi ) ) then
            !MeshMap%MapMotions(i)%shape_fn(1) = 0.5
            !MeshMap%MapMotions(i)%shape_fn(2) = 0.5
            RotationMatrixD = MATMUL( TRANSPOSE( Src%RefOrientation(:,:,n1) ), Src%Orientation(:,:,n1) )
            RotationMatrixD = MATMUL( Dest%RefOrientation(:,:,i), RotationMatrixD )
      
         elseif ( EqualRealNos( MeshMap%MapMotions(i)%shape_fn(2), 1.0_ReKi ) ) then
            !MeshMap%MapMotions(i)%shape_fn(1) = 0.5
            !MeshMap%MapMotions(i)%shape_fn(2) = 0.5
            RotationMatrixD = MATMUL( TRANSPOSE( Src%RefOrientation(:,:,n2) ), Src%Orientation(:,:,n2) )
            RotationMatrixD = MATMUL( Dest%RefOrientation(:,:,i), RotationMatrixD )
      
         else
        ! endif
         

               ! calculate Rotation matrix for FieldValueN1 and convert to tensor:
            RotationMatrixD = MATMUL( TRANSPOSE( Src%RefOrientation(:,:,n1) ), Src%Orientation(:,:,n1) )
            RotationMatrixD = MATMUL( Dest%RefOrientation(:,:,i), RotationMatrixD )

            CALL DCM_logmap( RotationMatrixD, FieldValue(:,1), ErrStat, ErrMsg )
            IF (ErrStat >= AbortErrLev) RETURN

               ! calculate Rotation matrix for FieldValueN2 and convert to tensor:
            RotationMatrixD = MATMUL( TRANSPOSE( Src%RefOrientation(:,:,n2) ), Src%Orientation(:,:,n2) )
            RotationMatrixD = MATMUL( Dest%RefOrientation(:,:,i), RotationMatrixD )
         
            CALL DCM_logmap( RotationMatrixD, FieldValue(:,2), ErrStat, ErrMsg )                  
            IF (ErrStat >= AbortErrLev) RETURN
         
            CALL DCM_SetLogMapForInterp( FieldValue )  ! make sure we don't cross a 2pi boundary
         
         
               ! interpolate tensors: 
            tensor_interp =   MeshMap%MapMotions(i)%shape_fn(1)*FieldValue(:,1)  &
                            + MeshMap%MapMotions(i)%shape_fn(2)*FieldValue(:,2)    
                  
               ! convert back to DCM:
            RotationMatrixD = DCM_exp( tensor_interp )
                        
         end if
         
         Dest%Orientation(:,:,i) = REAL( RotationMatrixD, R8Ki )
             
      end do
      
   endif
   
end subroutine Transfer_Orientation



!====================================================================================================
subroutine KAD_WriteSummary( outRootName, VSM_ElemPts, VSMnumCompElems, errStat, errMsg )
! This subroutine writes the data stored in WriteOutputs (and indexed in OutParam) to the file
! opened in VSM_Init()
!---------------------------------------------------------------------------------------------------- 

      ! Passed variables 
   character(*),                intent( in    ) :: outRootName
   real(ReKi),                  intent( in    ) :: VSM_ElemPts(:,:,:)
   integer(IntKi),              intent( in    ) :: VSMnumCompElems(:)
   !type(KAD_VSM_Data),          intent( in    ) :: VSM                    ! VSM module's output data
  ! type(VSM_ParameterType),      intent( in    ) :: p                    ! VSM module's parameter data
   integer,                     intent(   out ) :: errStat              ! returns a non-zero value when an error occurs  
   character(*),                intent(   out ) :: errMsg               ! Error message if ErrStat /= ErrID_None
   
      ! Local variables
   integer                                :: i,k,l                          ! Generic loop counter
   integer                                :: numComp
   character(1024)                        :: OutFileName
   integer                                :: Un
   
   OutFileName = TRIM(outRootName)//'.KAD.sum'
   call GetNewUnit( Un )
   
   call OpenFOutFile ( Un, OutFileName, errStat, errMsg ) 
   if (ErrStat >=AbortErrLev) return
      
      
      
         ! Write the output file header
      
   write (Un,'(/,A/)', IOSTAT=errStat)  'This file were generated by '//TRIM(KAD_Ver%Name)//&
                      ' on '//CurDate()//' at '//CurTime()//'.'
   
         ! Write the names of the output parameters:
   write (Un,'(A)', IOSTAT=errStat) '------------------------------------------------------------------------'
   write (Un,'(A)', IOSTAT=errStat) '----                         VSM Data                               ----'
   write (Un,'(A/)', IOSTAT=errStat) '------------------------------------------------------------------------'
   write(Un,'(A9,1X,A9,6(1X,A11))')  'Element #', 'CompElm #', 'PointAX', 'PointAY', 'PointAZ', 'PointBX', 'PointBY', 'PointBZ'
      
   i = 1
   do l = 1, size(VSMnumCompElems,1)
      do k = 1, VSMnumCompElems(l)
         write(Un, '(I9,1X,I9,6(1X,F11.4))') i, k, VSM_ElemPts(1,1,i), VSM_ElemPts(2,1,i), VSM_ElemPts(3,1,i), VSM_ElemPts(1,2,i), VSM_ElemPts(2,2,i), VSM_ElemPts(3,2,i)
         i = i + 1
      end do
   end do
   
   close(Un)   
      
end subroutine KAD_WriteSummary

subroutine FixUnitVectors(vec)
   real(ReKi), intent(inout) :: vec(3)
   real(ReKi) :: tmp
      ! TODO: Perhaps revisit this
      ! This was added because we lost precision going from the R8Ki DCM to the individual unit vectors of ReKi
      ! This means vectors may not quite be orthogonal, though
   if ( EqualRealNos(vec(1),0.0_ReKi) ) vec(1) = 0.0_ReKi
   if ( EqualRealNos(vec(2),0.0_ReKi) ) vec(2) = 0.0_ReKi
   if ( EqualRealNos(vec(3),0.0_ReKi) ) vec(3) = 0.0_ReKi
   
   tmp = TwoNorm(vec)
   if ( tmp > 0.0_ReKi ) vec = vec/tmp
end subroutine FixUnitVectors

subroutine Set_VSM_Inputs(u, m, p, u_VSM, errStat, errMsg)
   type(KAD_InputType),           intent(in   )  :: u           !< KiteAeroDyn inputs
   type(KAD_ParameterType),       intent(in   )  :: p           !< KiteAeroDyn parameters
   type(KAD_MiscVarType),         intent(inout)  :: m           !< KiteAeroDyn misc vars
   type(VSM_InputType),           intent(inout)  :: u_VSM       !< VSM inputs
   integer(IntKi),                intent(  out)  :: errStat     !< Error status of the operation
   character(*),                  intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None

   integer(IntKi)                                :: i, j, count, n1, n2
   character(ErrMsgLen)                          :: errMsg2     ! temporary Error message if errStat /= ErrID_None
   integer(IntKi)                                :: errStat2    ! temporary Error status of the operation
   character(*), parameter                       :: routineName = 'Set_VSM_Inputs'

      ! Initialize variables for this routine
   errStat         = ErrID_None
   errMsg          = "" 

      !  Map the orientations from the Inputs line2 mesh to the Output point mesh
  ! call Transfer_Orientation( u%FusMotions, m%FusLoads, m%Fus_L_2_P, errStat2, errMsg2 )
   call Transfer_Motions_Line2_to_Point( u%FusMotions, m%FusLoads, m%Fus_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' Set_VSM_Inputs: Transfer_Fus_L_2_P' )      
   !call Transfer_Orientation( u%SWnMotions, m%SWnLoads, m%SWn_L_2_P, errStat2, errMsg2 )
   call Transfer_Motions_Line2_to_Point( u%SWnMotions, m%SWnLoads, m%SWn_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' Set_VSM_Inputs: Transfer_SWn_L_2_P' )      
  ! call Transfer_Orientation( u%PWnMotions, m%PWnLoads, m%PWn_L_2_P, errStat2, errMsg2 )
   call Transfer_Motions_Line2_to_Point( u%PWnMotions, m%PWnLoads, m%PWn_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' Set_VSM_Inputs: Transfer_PWn_L_2_P' )      
  ! call Transfer_Orientation( u%VSMotions, m%VSLoads, m%VS_L_2_P, errStat2, errMsg2 )
   call Transfer_Motions_Line2_to_Point( u%VSMotions, m%VSLoads, m%VS_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' Set_VSM_Inputs: Transfer_VS_L_2_P' )      
   !call Transfer_Orientation( u%SHSMotions, m%SHSLoads, m%SHS_L_2_P, errStat2, errMsg2 )
   call Transfer_Motions_Line2_to_Point( u%SHSMotions, m%SHSLoads, m%SHS_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' Set_VSM_Inputs: Transfer_SHS_L_2_P' )      
  ! call Transfer_Orientation( u%PHSMotions, m%PHSLoads, m%PHS_L_2_P, errStat2, errMsg2 )
   call Transfer_Motions_Line2_to_Point( u%PHSMotions, m%PHSLoads, m%PHS_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' Set_VSM_Inputs: Transfer_PHS_L_2_P' )      
   do i = 1, p%NumPylons
      !call Transfer_Orientation( u%SPyMotions(i), m%SPyLoads(i), m%SPy_L_2_P(i), errStat2, errMsg2 )
      call Transfer_Motions_Line2_to_Point( u%SPyMotions(i), m%SPyLoads(i), m%SPy_L_2_P(i), errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' Set_VSM_Inputs: Transfer_SPy_L_2_P' )      
      !call Transfer_Orientation( u%PPyMotions(i), m%PPyLoads(i), m%PPy_L_2_P(i), errStat2, errMsg2 )
      call Transfer_Motions_Line2_to_Point( u%PPyMotions(i), m%PPyLoads(i), m%PPy_L_2_P(i), errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' Set_VSM_Inputs: Transfer_PPy_L_2_P' )      
   end do
   
      ! Loop over kite components to set these values
   count = 1
   
   do i = 1, u%FusMotions%NElemList
      n1 = u%FusMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%FusMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      u_VSM%PtA    (:,count) = u%FusMotions%Position(:,n1) + u%FusMotions%TranslationDisp(:,n1)
      u_VSM%PtB    (:,count) = u%FusMotions%Position(:,n2) + u%FusMotions%TranslationDisp(:,n2)
      u_VSM%U_Inf_v(:,count) = ( u%V_Fus(:,n1) + u%V_Fus(:,n2) ) / 2.0   -  ( u%FusMotions%TranslationVel(:,n1  ) + u%FusMotions%TranslationVel(:,n2) ) / 2.0
      u_VSM%x_hat  (:,count) = m%FusLoads%Orientation(1,:,i)
      u_VSM%y_hat  (:,count) = m%FusLoads%Orientation(2,:,i)
      u_VSM%z_hat  (:,count) = m%FusLoads%Orientation(3,:,i)
      u_VSM%Deltaf (  count) = 0.0_ReKi
      count = count + 1
   end do  
   
   do i = 1, u%SWnMotions%NElemList
      n1 = u%SWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%SWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      u_VSM%PtA    (:,count) = u%SWnMotions%Position(:,n1) + u%SWnMotions%TranslationDisp(:,n1)
      u_VSM%PtB    (:,count) = u%SWnMotions%Position(:,n2) + u%SWnMotions%TranslationDisp(:,n2)
      u_VSM%U_Inf_v(:,count) = ( u%V_SWn(:,n1) + u%V_SWn(:,n2) ) / 2.0   -  ( u%SWnMotions%TranslationVel(:,n1  ) + u%SWnMotions%TranslationVel(:,n2) ) / 2.0
      u_VSM%x_hat  (:,count) = m%SWnLoads%Orientation(1,:,i)
      u_VSM%y_hat  (:,count) = m%SWnLoads%Orientation(2,:,i)
      u_VSM%z_hat  (:,count) = m%SWnLoads%Orientation(3,:,i)
      if ( p%SWnCtrlID(n1) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_SFlp(p%SWnCtrlID(n1))
      else   
         u_VSM%Deltaf (  count) = 0.0_ReKi
      end if
      
      count = count + 1
   end do
   
   do i = 1, u%PWnMotions%NElemList
      n1 = u%PWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%PWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      u_VSM%PtA    (:,count) = u%PWnMotions%Position(:,n1) + u%PWnMotions%TranslationDisp(:,n1)
      u_VSM%PtB    (:,count) = u%PWnMotions%Position(:,n2) + u%PWnMotions%TranslationDisp(:,n2)
      u_VSM%U_Inf_v(:,count) = ( u%V_PWn(:,n1) + u%V_PWn(:,n2) ) / 2.0   -  ( u%PWnMotions%TranslationVel(:,n1  ) + u%PWnMotions%TranslationVel(:,n2) ) / 2.0
      u_VSM%x_hat  (:,count) = m%PWnLoads%Orientation(1,:,i)
      u_VSM%y_hat  (:,count) = m%PWnLoads%Orientation(2,:,i)
      u_VSM%z_hat  (:,count) = m%PWnLoads%Orientation(3,:,i)
      if ( p%PWnCtrlID(n1) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_PFlp(p%PWnCtrlID(n1))
      else   
         u_VSM%Deltaf (  count) = 0.0_ReKi
      end if
      count = count + 1
   end do
   
   do i = 1, u%VSMotions%NElemList
      n1 = u%VSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%VSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      u_VSM%PtA    (:,count) = u%VSMotions%Position(:,n1) + u%VSMotions%TranslationDisp(:,n1)
      u_VSM%PtB    (:,count) = u%VSMotions%Position(:,n2) + u%VSMotions%TranslationDisp(:,n2)
      u_VSM%U_Inf_v(:,count) = ( u%V_VS(:,n1) + u%V_VS(:,n2) ) / 2.0   -  ( u%VSMotions%TranslationVel(:,n1  ) + u%VSMotions%TranslationVel(:,n2) ) / 2.0
      u_VSM%x_hat  (:,count) = m%VSLoads%Orientation(1,:,i)
      u_VSM%y_hat  (:,count) = m%VSLoads%Orientation(2,:,i)
      u_VSM%z_hat  (:,count) = m%VSLoads%Orientation(3,:,i)
      if ( p%VSCtrlID(n1) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_Rudr(p%VSCtrlID(n1))
      else   
         u_VSM%Deltaf (  count) = 0.0_ReKi
      end if
      count = count + 1
   end do
   
   do i = 1, u%SHSMotions%NElemList
      n1 = u%SHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%SHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      u_VSM%PtA    (:,count) = u%SHSMotions%Position(:,n1) + u%SHSMotions%TranslationDisp(:,n1)
      u_VSM%PtB    (:,count) = u%SHSMotions%Position(:,n2) + u%SHSMotions%TranslationDisp(:,n2)
      u_VSM%U_Inf_v(:,count) = ( u%V_SHS(:,n1) + u%V_SHS(:,n2) ) / 2.0   -  ( u%SHSMotions%TranslationVel(:,n1  ) + u%SHSMotions%TranslationVel(:,n2) ) / 2.0
      u_VSM%x_hat  (:,count) = m%SHSLoads%Orientation(1,:,i)
      u_VSM%y_hat  (:,count) = m%SHSLoads%Orientation(2,:,i)
      u_VSM%z_hat  (:,count) = m%SHSLoads%Orientation(3,:,i)
      if ( p%SHSCtrlID(n1) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_SElv(p%SHSCtrlID(n1))
      else   
         u_VSM%Deltaf (  count) = 0.0_ReKi
      end if
      count = count + 1
   end do
   
   do i = 1, u%PHSMotions%NElemList
      n1 = u%PHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%PHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      u_VSM%PtA    (:,count) = u%PHSMotions%Position(:,n1) + u%PHSMotions%TranslationDisp(:,n1)
      u_VSM%PtB    (:,count) = u%PHSMotions%Position(:,n2) + u%PHSMotions%TranslationDisp(:,n2)
      u_VSM%U_Inf_v(:,count) = ( u%V_PHS(:,n1) + u%V_PHS(:,n2) ) / 2.0   -  ( u%PHSMotions%TranslationVel(:,n1  ) + u%PHSMotions%TranslationVel(:,n2) ) / 2.0
      u_VSM%x_hat  (:,count) = m%PHSLoads%Orientation(1,:,i)
      u_VSM%y_hat  (:,count) = m%PHSLoads%Orientation(2,:,i)
      u_VSM%z_hat  (:,count) = m%PHSLoads%Orientation(3,:,i)
      if ( p%PHSCtrlID(n1) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_PElv(p%PHSCtrlID(n1))
      else   
         u_VSM%Deltaf (  count) = 0.0_ReKi
      end if
      count = count + 1
   end do
   
   do j = 1, p%NumPylons
      do i = 1, u%SPyMotions(j)%NElemList
         n1 = u%SPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
         n2 = u%SPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
         u_VSM%PtA    (:,count) = u%SPyMotions(j)%Position(:,n1) + u%SPyMotions(j)%TranslationDisp(:,n1)
         u_VSM%PtB    (:,count) = u%SPyMotions(j)%Position(:,n2) + u%SPyMotions(j)%TranslationDisp(:,n2)
         u_VSM%U_Inf_v(:,count) = ( u%V_SPy(:,n1,j) + u%V_SPy(:,n2,j) ) / 2.0   -  ( u%SPyMotions(j)%TranslationVel(:,n1  ) + u%SPyMotions(j)%TranslationVel(:,n2) ) / 2.0
         u_VSM%x_hat  (:,count) = m%SPyLoads(j)%Orientation(1,:,i)
         u_VSM%y_hat  (:,count) = m%SPyLoads(j)%Orientation(2,:,i)
         u_VSM%z_hat  (:,count) = m%SPyLoads(j)%Orientation(3,:,i)
         u_VSM%Deltaf (  count) = 0.0_ReKi
         count = count + 1
      end do
   end do
   
   do j = 1, p%NumPylons
      do i = 1, u%PPyMotions(j)%NElemList
         n1 = u%PPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
         n2 = u%PPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
         u_VSM%PtA    (:,count) = u%PPyMotions(j)%Position(:,n1) + u%PPyMotions(j)%TranslationDisp(:,n1)
         u_VSM%PtB    (:,count) = u%PPyMotions(j)%Position(:,n2) + u%PPyMotions(j)%TranslationDisp(:,n2)
         u_VSM%U_Inf_v(:,count) = ( u%V_PPy(:,n1,j) + u%V_PPy(:,n2,j) ) / 2.0   -  ( u%PPyMotions(j)%TranslationVel(:,n1  ) + u%PPyMotions(j)%TranslationVel(:,n2) ) / 2.0
         u_VSM%x_hat  (:,count) = m%PPyLoads(j)%Orientation(1,:,i)
         u_VSM%y_hat  (:,count) = m%PPyLoads(j)%Orientation(2,:,i)
         u_VSM%z_hat  (:,count) = m%PPyLoads(j)%Orientation(3,:,i)
         u_VSM%Deltaf (  count) = 0.0_ReKi
         count = count + 1
      end do
   end do
   
   ! Fix unit vectors to be normalized and set terms close to zero = to zero
   do i = 1, size(u_VSM%x_hat,2)
      call FixUnitVectors(u_VSM%x_hat(:,i))
      call FixUnitVectors(u_VSM%y_hat(:,i))
      call FixUnitVectors(u_VSM%z_hat(:,i))
   end do
   
end subroutine Set_VSM_Inputs

subroutine KAD_MapOutputs(p, u, u_VSM, y, y_VSM, p_VSM, u_ActDsk, y_ActDsk, m, z, errStat, errMsg)
   type(KAD_ParameterType),       intent(in   )  :: p           !< KAD Parameters
   type(KAD_InputType),           intent(in   )  :: u           !< KAD inputs
   type(VSM_InputType),           intent(in   )  :: u_VSM       !< VSM inputs
   type(KAD_OutputType),          intent(in   )  :: y           !< KAD system outputs 
   type(VSM_OutputType),          intent(in   )  :: y_VSM       !< VSM system outputs 
   type(VSM_ParameterType),       intent(in   )  :: p_VSM       !< VSM parameters
   type(ActDsk_InputType),        intent(in   )  :: u_ActDsk(:) !< actuator disk system inputs
   type(ActDsk_OutputType),       intent(in   )  :: y_ActDsk(:) !< actuator disk system outputs
   type(KAD_MiscVarType),         intent(inout)  :: m           !< KAD MiscVars for the module
   type(KAD_ConstraintStateType), intent(in   )  :: z           !< KAD Constraint states                                                                
   integer(IntKi),                intent(  out)  :: errStat     !< Error status of the operation
   character(*),                  intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None
   
   real   (ReKi)               :: Vinf_v(3)
   real   (ReKi)               :: DCM(3,3)  
   real   (ReKi)               :: Vstruct_v(3)
   real   (ReKi)               :: Vrel   
   real   (ReKi)               :: Vind, Vrel_v(3), Vind_v(3), AoA, Cl, Cd, Cm, Fl, Fd, Mm, Cn, Cc, Fn, Fc
   real   (ReKi)               :: Re     
   real   (ReKi)               :: XM     
   real   (ReKi)               :: DynP  
   integer(IntKi)              :: VSMoffset
   integer(IntKi)              :: i, j
   real   (ReKi)               :: chord
   real   (ReKi), allocatable  :: Vinfs_v(:,:)
   real   (ReKi), allocatable  :: chords(:)
   real   (ReKi), allocatable  :: elemLens(:)
   real   (ReKi), allocatable  :: forces(:,:), moments(:,:)
   real   (ReKi)               :: kiteForces(3), kiteMoments(3)
   integer(IntKi)              :: n, offset
   character(*), parameter     :: RoutineName = 'KAD_MapOutputs'

   errStat = ErrID_None
   errMsg  = ''
   
   VSMoffset    = 0
   
   n = size(u%V_SPy,2)
   call AllocAry(Vinfs_v, 3, n, 'Vinfs', errStat, errMsg)
   
   n = size(p%SPyChord,1)
   call AllocAry(chords, n, 'chords', errStat, errMsg) 
   call AllocAry(elemLens, n, 'elemLens', errStat, errMsg) 
         
   !!=======================================
   !! Fuselage-related outputs
   !! NOTE: currently these nodes are not included in the VSM calculations
   !!        as a result, we will compute the uninduced loads here
   !!=======================================   
   do i = 1, p%NFusOuts
               
      call ComputeAeroOnMotionNodes(i, p%FusOutNd, u%FusMotions, VSMoffset, u_VSM, y_VSM, u%V_Fus, &
                                     p_VSM%Chords, p_VSM%elemLens, p%AirDens, p%KinVisc, p%SpdSound, &
                                     Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, Cl, Cd, Cm, &
                                     Fl, Fd, Mm, Cn, Cc, Fn, Fc, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The fuselage ', errStat, errMsg, RoutineName )
            return
         end if
      
      m%AllOuts( FusVAmbn(i) ) = Vinf_v(1)
      m%AllOuts( FusVAmbc(i) ) = Vinf_v(2)
      m%AllOuts( FusVAmbs(i) ) = Vinf_v(3)
      m%AllOuts( FusSTVn (i) ) = Vstruct_v(1)
      m%AllOuts( FusSTVc (i) ) = Vstruct_v(2)
      m%AllOuts( FusSTVs (i) ) = Vstruct_v(3)
      m%AllOuts( FusVrel (i) ) = Vrel
      m%AllOuts( FusDynP (i) ) = DynP
      m%AllOuts( FusRe   (i) ) = Re
      m%AllOuts( FusM    (i) ) = XM
      m%AllOuts( FusVIndn(i) ) = Vind_v(1)
      m%AllOuts( FusVIndc(i) ) = Vind_v(2)
      m%AllOuts( FusVInds(i) ) = Vind_v(3)     
      m%AllOuts( FusAlpha(i) ) = AoA*R2D
      m%AllOuts( FusCl   (i) ) = Cl
      m%AllOuts( FusCd   (i) ) = Cd
      m%AllOuts( FusCm   (i) ) = Cm
      m%AllOuts( FusCn   (i) ) = Cn
      m%AllOuts( FusCc   (i) ) = Cc
      m%AllOuts( FusFl   (i) ) = Fl
      m%AllOuts( FusFd   (i) ) = Fd
      m%AllOuts( FusMm   (i) ) = Mm
      m%AllOuts( FusFn   (i) ) = Fn
      m%AllOuts( FusFc   (i) ) = Fc
   
   end do
   VSMoffset = u%FusMotions%NElemList
   
   !=======================================
   ! Starboard wing-related outputs
   !=======================================   
   do i = 1, p%NSWnOuts
      
      call ComputeAeroOnMotionNodes(i, p%SWnOutNd, u%SWnMotions, VSMoffset, u_VSM, y_VSM, u%V_SWn,  &
                                      p_VSM%Chords, p_VSM%elemLens, p%AirDens, p%KinVisc, p%SpdSound, Vinf_v, Vstruct_v, Vind_v, &
                                      Vrel, DynP, Re, XM, AoA, Cl, Cd, Cm, Fl, Fd, Mm, Cn, Cc, Fn, Fc, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The starboard wing ', errStat, errMsg, RoutineName )
            return
         end if
         
      m%AllOuts( SWnVAmbn(i) ) = Vinf_v(1)
      m%AllOuts( SWnVAmbc(i) ) = Vinf_v(2)
      m%AllOuts( SWnVAmbs(i) ) = Vinf_v(3)
      m%AllOuts( SWnSTVn (i) ) = Vstruct_v(1)
      m%AllOuts( SWnSTVc (i) ) = Vstruct_v(2)
      m%AllOuts( SWnSTVs (i) ) = Vstruct_v(3)
      m%AllOuts( SWnVrel (i) ) = Vrel
      m%AllOuts( SWnDynP (i) ) = DynP
      m%AllOuts( SWnRe   (i) ) = Re
      m%AllOuts( SWnM    (i) ) = XM
      m%AllOuts( SWnVIndn(i) ) = Vind_v(1)
      m%AllOuts( SWnVIndc(i) ) = Vind_v(2)
      m%AllOuts( SWnVInds(i) ) = Vind_v(3)       
      m%AllOuts( SWnAlpha(i) ) = AoA*R2D
      m%AllOuts( SWnCl   (i) ) = Cl
      m%AllOuts( SWnCd   (i) ) = Cd
      m%AllOuts( SWnCm   (i) ) = Cm
      m%AllOuts( SWnCn   (i) ) = Cn
      m%AllOuts( SWnCc   (i) ) = Cc
      m%AllOuts( SWnFl   (i) ) = Fl
      m%AllOuts( SWnFd   (i) ) = Fd
      m%AllOuts( SWnMm   (i) ) = Mm
      m%AllOuts( SWnFn   (i) ) = Fn
      m%AllOuts( SWnFc   (i) ) = Fc
      if (p%SWnCtrlID(p%SWnOutNd(i)) > 0 ) then
         m%AllOuts(SWnCtrl(i)) = u%Ctrl_SFlp(p%SWnCtrlID(p%SWnOutNd(i)))
      else    
         m%AllOuts(SWnCtrl(i)) = 0.0_ReKi
      end if
      
   end do
   VSMoffset = VSMoffset + u%SWnMotions%NElemList
   
   !=======================================
   ! Port wing-related outputs
   !=======================================   
   do i = 1, p%NPWnOuts
         
      call ComputeAeroOnMotionNodes(i, p%PWnOutNd, u%PWnMotions, VSMoffset, u_VSM, y_VSM, u%V_PWn, p_VSM%Chords, &
                                    p_VSM%elemLens, p%AirDens, p%KinVisc, p%SpdSound, Vinf_v, Vstruct_v, Vind_v, &
                                    Vrel, DynP, Re, XM, AoA, Cl, Cd, Cm, Fl, Fd, Mm, Cn, Cc, Fn, Fc, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The port wing ', errStat, errMsg, RoutineName )
            return
         end if
 
      m%AllOuts( PWnVAmbn(i) ) = Vinf_v(1)
      m%AllOuts( PWnVAmbc(i) ) = Vinf_v(2)
      m%AllOuts( PWnVAmbs(i) ) = Vinf_v(3)
      m%AllOuts( PWnSTVn (i) ) = Vstruct_v(1)
      m%AllOuts( PWnSTVc (i) ) = Vstruct_v(2)
      m%AllOuts( PWnSTVs (i) ) = Vstruct_v(3)
      m%AllOuts( PWnVrel (i) ) = Vrel
      m%AllOuts( PWnDynP (i) ) = DynP
      m%AllOuts( PWnRe   (i) ) = Re
      m%AllOuts( PWnM    (i) ) = XM
      m%AllOuts( PWnVIndn(i) ) = Vind_v(1)
      m%AllOuts( PWnVIndc(i) ) = Vind_v(2)
      m%AllOuts( PWnVInds(i) ) = Vind_v(3)       
      m%AllOuts( PWnAlpha(i) ) = AoA*R2D
      m%AllOuts( PWnCl   (i) ) = Cl
      m%AllOuts( PWnCd   (i) ) = Cd
      m%AllOuts( PWnCm   (i) ) = Cm
      m%AllOuts( PWnCn   (i) ) = Cn
      m%AllOuts( PWnCc   (i) ) = Cc
      m%AllOuts( PWnFl   (i) ) = Fl 
      m%AllOuts( PWnFd   (i) ) = Fd 
      m%AllOuts( PWnMm   (i) ) = Mm 
      m%AllOuts( PWnFn   (i) ) = Fn
      m%AllOuts( PWnFc   (i) ) = Fc
      if (p%PWnCtrlID(p%PWnOutNd(i)) > 0 ) then
         m%AllOuts(PWnCtrl(i)) = u%Ctrl_PFlp(p%PWnCtrlID(p%PWnOutNd(i)))
      else    
         m%AllOuts(PWnCtrl(i)) = 0.0_ReKi
      end if
   end do
   VSMoffset = VSMoffset + u%PWnMotions%NElemList
   
   !=======================================
   ! Vertical stabilizer-related outputs
   !=======================================   
   do i = 1, p%NVSOuts
         
      call ComputeAeroOnMotionNodes(i, p%VSOutNd, u%VSMotions, VSMoffset, u_VSM, y_VSM, u%V_VS, p_VSM%Chords, p_VSM%elemLens, &
                                    p%AirDens, p%KinVisc, p%SpdSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                    Cl, Cd, Cm, Fl, Fd, Mm, Cn, Cc, Fn, Fc, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The vertical stabilizer ', errStat, errMsg, RoutineName )
            return
         end if

      m%AllOuts( VSVAmbn(i) ) = Vinf_v(1)
      m%AllOuts( VSVAmbc(i) ) = Vinf_v(2)
      m%AllOuts( VSVAmbs(i) ) = Vinf_v(3)
      m%AllOuts( VSSTVn (i) ) = Vstruct_v(1)
      m%AllOuts( VSSTVc (i) ) = Vstruct_v(2)
      m%AllOuts( VSSTVs (i) ) = Vstruct_v(3)
      m%AllOuts( VSVrel (i) ) = Vrel
      m%AllOuts( VSDynP (i) ) = DynP
      m%AllOuts( VSRe   (i) ) = Re
      m%AllOuts( VSMa   (i) ) = XM
      m%AllOuts( VSVIndn(i) ) = Vind_v(1)
      m%AllOuts( VSVIndc(i) ) = Vind_v(2)
      m%AllOuts( VSVInds(i) ) = Vind_v(3)              
      m%AllOuts( VSAlpha(i) ) = AoA*R2D 
      m%AllOuts( VSCl   (i) ) = Cl 
      m%AllOuts( VSCd   (i) ) = Cd 
      m%AllOuts( VSCm   (i) ) = Cm 
      m%AllOuts( VSCn   (i) ) = Cn
      m%AllOuts( VSCc   (i) ) = Cc
      m%AllOuts( VSFl   (i) ) = Fl
      m%AllOuts( VSFd   (i) ) = Fd
      m%AllOuts( VSMm   (i) ) = Mm
      m%AllOuts( VSFn   (i) ) = Fn
      m%AllOuts( VSFc   (i) ) = Fc
      if (p%VSCtrlID(p%VSOutNd(i)) > 0 ) then
         m%AllOuts(RudrCtrl(i)) = u%Ctrl_Rudr(p%VSCtrlID(p%VSOutNd(i)))
      else    
         m%AllOuts(RudrCtrl(i)) = 0.0_ReKi
      end if
   end do
   VSMoffset = VSMoffset + u%VSMotions%NElemList
   
   !=======================================
   ! Starboard horizontal stabilizer-related outputs
   !=======================================   
   do i = 1, p%NSHSOuts
         
      call ComputeAeroOnMotionNodes(i, p%SHSOutNd, u%SHSMotions, VSMoffset, u_VSM, y_VSM, u%V_SHS, p_VSM%Chords, p_VSM%elemLens, & 
                                    p%AirDens, p%KinVisc, p%SpdSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                    Cl, Cd, Cm, Fl, Fd, Mm, Cn, Cc, Fn, Fc, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The starboard horizontal stabilizer ', errStat, errMsg, RoutineName )
            return
         end if

      m%AllOuts( SHSVAmbn(i) ) = Vinf_v(1)
      m%AllOuts( SHSVAmbc(i) ) = Vinf_v(2)
      m%AllOuts( SHSVAmbs(i) ) = Vinf_v(3)
      m%AllOuts( SHSSTVn (i) ) = Vstruct_v(1)
      m%AllOuts( SHSSTVc (i) ) = Vstruct_v(2)
      m%AllOuts( SHSSTVs (i) ) = Vstruct_v(3)
      m%AllOuts( SHSVrel (i) ) = Vrel
      m%AllOuts( SHSDynP (i) ) = DynP
      m%AllOuts( SHSRe   (i) ) = Re
      m%AllOuts( SHSM    (i) ) = XM
      m%AllOuts( SHSVIndn(i) ) = Vind_v(1)
      m%AllOuts( SHSVIndc(i) ) = Vind_v(2)
      m%AllOuts( SHSVInds(i) ) = Vind_v(3)                     
      m%AllOuts( SHSAlpha(i) ) = AoA*R2D  
      m%AllOuts( SHSCl   (i) ) = Cl  
      m%AllOuts( SHSCd   (i) ) = Cd  
      m%AllOuts( SHSCm   (i) ) = Cm  
      m%AllOuts( SHSCn   (i) ) = Cn
      m%AllOuts( SHSCc   (i) ) = Cc
      m%AllOuts( SHSFl   (i) ) = Fl
      m%AllOuts( SHSFd   (i) ) = Fd
      m%AllOuts( SHSMm   (i) ) = Mm
      m%AllOuts( SHSFn   (i) ) = Fn
      m%AllOuts( SHSFc   (i) ) = Fc
      if (p%SHSCtrlID(p%SHSOutNd(i)) > 0 ) then
         m%AllOuts(SElvCtrl(i)) = u%Ctrl_SElv(p%SHSCtrlID(p%SHSOutNd(i)))
      else    
         m%AllOuts(SElvCtrl(i)) = 0.0_ReKi
      end if
   end do
   VSMoffset = VSMoffset + u%SHSMotions%NElemList
   
   !=======================================
   ! Port horizontal stabilizer-related outputs
   !=======================================   
   do i = 1, p%NPHSOuts
      
      call ComputeAeroOnMotionNodes(i, p%PHSOutNd, u%PHSMotions, VSMoffset, u_VSM, y_VSM, u%V_PHS, p_VSM%Chords, p_VSM%elemLens, &
                                    p%AirDens, p%KinVisc, p%SpdSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                    Cl, Cd, Cm, Fl, Fd, Mm, Cn, Cc, Fn, Fc, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The port horizontal stabilizer ', errStat, errMsg, RoutineName )
            return
         end if

      m%AllOuts( PHSVAmbn(i) ) = Vinf_v(1)
      m%AllOuts( PHSVAmbc(i) ) = Vinf_v(2)
      m%AllOuts( PHSVAmbs(i) ) = Vinf_v(3)
      m%AllOuts( PHSSTVn (i) ) = Vstruct_v(1)
      m%AllOuts( PHSSTVc (i) ) = Vstruct_v(2)
      m%AllOuts( PHSSTVs (i) ) = Vstruct_v(3)
      m%AllOuts( PHSVrel (i) ) = Vrel
      m%AllOuts( PHSDynP (i) ) = DynP
      m%AllOuts( PHSRe   (i) ) = Re
      m%AllOuts( PHSM    (i) ) = XM
      m%AllOuts( PHSVIndn(i) ) = Vind_v(1)
      m%AllOuts( PHSVIndc(i) ) = Vind_v(2)
      m%AllOuts( PHSVInds(i) ) = Vind_v(3)                     
      m%AllOuts( PHSAlpha(i) ) = AoA*R2D  
      m%AllOuts( PHSCl   (i) ) = Cl  
      m%AllOuts( PHSCd   (i) ) = Cd  
      m%AllOuts( PHSCm   (i) ) = Cm  
      m%AllOuts( PHSCn   (i) ) = Cn
      m%AllOuts( PHSCc   (i) ) = Cc
      m%AllOuts( PHSFl   (i) ) = Fl
      m%AllOuts( PHSFd   (i) ) = Fd
      m%AllOuts( PHSMm   (i) ) = Mm
      m%AllOuts( PHSFn   (i) ) = Fn
      m%AllOuts( PHSFc   (i) ) = Fc
      if (p%PHSCtrlID(p%PHSOutNd(i)) > 0 ) then
         m%AllOuts(PElvCtrl(i)) = u%Ctrl_PElv(p%PHSCtrlID(p%PHSOutNd(i)))
      else    
         m%AllOuts(PElvCtrl(i)) = 0.0_ReKi
      end if
   end do
   VSMoffset = VSMoffset + u%PHSMotions%NElemList
   
   !=======================================
   ! Starboard, pylon-related outputs
   !=======================================   
   do j = 1, p%NumPylons
      do i = 1, p%NPylOuts
         
         Vinfs_v = u%V_SPy(:, :,j)
         call ComputeAeroOnMotionNodes(i, p%PylOutNd, u%SPyMotions(j), VSMoffset, u_VSM, y_VSM, Vinfs_v, p_VSM%Chords, p_VSM%elemLens, &
                                       p%AirDens, p%KinVisc, p%SpdSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                       Cl, Cd, Cm, Fl, Fd, Mm, Cn, Cc, Fn, Fc, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The starboard pylon ', errStat, errMsg, RoutineName )
            return
         end if

         m%AllOuts( SPVAmbn(i,j) ) = Vinf_v(1)
         m%AllOuts( SPVAmbc(i,j) ) = Vinf_v(2)
         m%AllOuts( SPVAmbs(i,j) ) = Vinf_v(3)
         m%AllOuts( SPSTVn (i,j) ) = Vstruct_v(1)
         m%AllOuts( SPSTVc (i,j) ) = Vstruct_v(2)
         m%AllOuts( SPSTVs (i,j) ) = Vstruct_v(3)
         m%AllOuts( SPVrel (i,j) ) = Vrel
         m%AllOuts( SPDynP (i,j) ) = DynP
         m%AllOuts( SPRe   (i,j) ) = Re
         m%AllOuts( SPM    (i,j) ) = XM
         m%AllOuts( SPVIndn(i,j) ) = Vind_v(1)
         m%AllOuts( SPVIndc(i,j) ) = Vind_v(2)
         m%AllOuts( SPVInds(i,j) ) = Vind_v(3)                    
         m%AllOuts( SPAlpha(i,j) ) = AoA*R2D 
         m%AllOuts( SPCl   (i,j) ) = Cl 
         m%AllOuts( SPCd   (i,j) ) = Cd 
         m%AllOuts( SPCm   (i,j) ) = Cm 
         m%AllOuts( SPCn   (i,j) ) = Cn
         m%AllOuts( SPCc   (i,j) ) = Cc
         m%AllOuts( SPFl   (i,j) ) = Fl
         m%AllOuts( SPFd   (i,j) ) = Fd
         m%AllOuts( SPMm   (i,j) ) = Mm
         m%AllOuts( SPFn   (i,j) ) = Fn
         m%AllOuts( SPFc   (i,j) ) = Fc

      end do
      VSMoffset = VSMoffset + u%SPyMotions(j)%NElemList
   end do
   
   !=======================================
   ! Starboard, pylon-related outputs
   !=======================================   
   do j = 1, p%NumPylons
      do i = 1, p%NPylOuts
         
         Vinfs_v = u%V_PPy(:, :,j)
         call ComputeAeroOnMotionNodes(i, p%PylOutNd, u%PPyMotions(j), VSMoffset, u_VSM, y_VSM, Vinfs_v, p_VSM%Chords, p_VSM%elemLens, &
                                       p%AirDens, p%KinVisc, p%SpdSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                        Cl, Cd, Cm, Fl, Fd, Mm, Cn, Cc, Fn, Fc, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The port pylon ', errStat, errMsg, RoutineName )
            return
         end if


         m%AllOuts( PPVAmbn(i,j) ) = Vinf_v(1)
         m%AllOuts( PPVAmbc(i,j) ) = Vinf_v(2)
         m%AllOuts( PPVAmbs(i,j) ) = Vinf_v(3)
         m%AllOuts( PPSTVn (i,j) ) = Vstruct_v(1)
         m%AllOuts( PPSTVc (i,j) ) = Vstruct_v(2)
         m%AllOuts( PPSTVs (i,j) ) = Vstruct_v(3)
         m%AllOuts( PPVrel (i,j) ) = Vrel
         m%AllOuts( PPDynP (i,j) ) = DynP
         m%AllOuts( PPRe   (i,j) ) = Re
         m%AllOuts( PPM    (i,j) ) = XM
         m%AllOuts( PPVIndn(i,j) ) = Vind_v(1)
         m%AllOuts( PPVIndc(i,j) ) = Vind_v(2)
         m%AllOuts( PPVInds(i,j) ) = Vind_v(3)                   
         m%AllOuts( PPAlpha(i,j) ) = AoA*R2D 
         m%AllOuts( PPCl   (i,j) ) = Cl 
         m%AllOuts( PPCd   (i,j) ) = Cd 
         m%AllOuts( PPCm   (i,j) ) = Cm 
         m%AllOuts( PPCn   (i,j) ) = Cn
         m%AllOuts( PPCc   (i,j) ) = Cc
         m%AllOuts( PPFl   (i,j) ) = Fl 
         m%AllOuts( PPFd   (i,j) ) = Fd 
         m%AllOuts( PPMm   (i,j) ) = Mm 
         m%AllOuts( PPFn   (i,j) ) = Fn
         m%AllOuts( PPFc   (i,j) ) = Fc
      end do
      VSMoffset = VSMoffset + u%PPyMotions(j)%NElemList
   end do
   

   ! Actuator Disks
   ! Starboard Pylon 1   Top and Bottom rotors
  
   call AllocAry(forces, 3, p%NumPylons*2, 'forces', errStat, errMsg) 
   call AllocAry(moments, 3, p%NumPylons*2, 'moments', errStat, errMsg) 
   
   ! Orientation is transform from global to local
   do i = 1, 2*p%NumPylons
      DCM = u%SPyRtrMotions(i)%Orientation(:,:,1)
      forces(:,i)  = matmul(DCM, y%SPyRtrLoads(i)%Force(:,1))
      moments(:,i) = matmul (DCM,y%SPyRtrLoads(i)%Moment(:,1))
   end do
   
 !m%AllOuts( SP1TVRelx) = m%ActDsk(1)%DiskAve_Vx_Rel
 !m%AllOuts( SP1BVRelx) = m%ActDsk(2)%DiskAve_Vx_Rel
 m%AllOuts( SP1TRtSpd ) = u_ActDsk(1)%RtSpd
 m%AllOuts( SP1BRtSpd ) = u_ActDsk(2)%RtSpd
 m%AllOuts( SP1TSkew  ) = u_ActDsk(1)%skew*R2D
 m%AllOuts( SP1BSkew  ) = u_ActDsk(2)%skew*R2D
 m%AllOuts( SP1TPitch ) = u_ActDsk(1)%pitch*R2D
 m%AllOuts( SP1BPitch ) = u_ActDsk(2)%pitch*R2D
 m%AllOuts( SP1TTSR   ) = m%ActDsk(1)%TSR
 m%AllOuts( SP1BTSR   ) = m%ActDsk(2)%TSR

 m%AllOuts( SP1TVrel ) = u_ActDsk(1)%DiskAve_Vrel
 m%AllOuts( SP1BVrel ) = u_ActDsk(2)%DiskAve_Vrel
 m%AllOuts( SP1TCp   ) = m%ActDsk(1)%Cp
 m%AllOuts( SP1BCp   ) = m%ActDsk(2)%Cp
 m%AllOuts( SP1TCq   ) = m%ActDsk(1)%Cq
 m%AllOuts( SP1BCq   ) = m%ActDsk(2)%Cq
 m%AllOuts( SP1TCt   ) = m%ActDsk(1)%Ct
 m%AllOuts( SP1BCt   ) = m%ActDsk(2)%Ct
! TODO: Should we simply be using the y_ActDsk(1)%Fx, etc for these outputs and not the ones in the Rotor Node coordinate system?
! TODO:  this node system could be different than the Actuator Disk system which was used to compute y_ActDsk(1)%Fx
 m%AllOuts( SP1TFx   ) = forces(1,1)
 m%AllOuts( SP1BFx   ) = forces(1,2)
 m%AllOuts( SP1TFy   ) = forces(2,1)
 m%AllOuts( SP1BFy   ) = forces(2,2)
 m%AllOuts( SP1TFz   ) = forces(3,1)
 m%AllOuts( SP1BFz   ) = forces(3,2)
 m%AllOuts( SP1TMx   ) = moments(1,1)
 m%AllOuts( SP1BMx   ) = moments(1,2)
 m%AllOuts( SP1TMy   ) = moments(2,1)
 m%AllOuts( SP1BMy   ) = moments(2,2)
 m%AllOuts( SP1TMz   ) = moments(3,1)
 m%AllOuts( SP1BMz   ) = moments(3,2)
 m%AllOuts( SP1TPwr  ) = y_ActDsk(1)%P
 m%AllOuts( SP1BPwr  ) = y_ActDsk(2)%P

   ! Starboard Pylon 2   Top and Bottom rotors
 !m%AllOuts( SP2TVRelx) = m%ActDsk(3)%DiskAve_Vx_Rel
 !m%AllOuts( SP2BVRelx) = m%ActDsk(4)%DiskAve_Vx_Rel
 if (p%NumPylons == 2 ) then
 m%AllOuts( SP2TPitch) = u_ActDsk(3)%pitch*R2D
 m%AllOuts( SP2BPitch) = u_ActDsk(4)%pitch*R2D
 m%AllOuts( SP2TTSR  ) = m%ActDsk(3)%TSR
 m%AllOuts( SP2BTSR  ) = m%ActDsk(4)%TSR
 m%AllOuts( SP2TRtSpd) = u_ActDsk(3)%RtSpd
 m%AllOuts( SP2BRtSpd) = u_ActDsk(4)%RtSpd
 m%AllOuts( SP2TSkew ) = u_ActDsk(3)%skew*R2D
 m%AllOuts( SP2BSkew ) = u_ActDsk(4)%skew*R2D
 m%AllOuts( SP2TVrel ) = u_ActDsk(3)%DiskAve_Vrel
 m%AllOuts( SP2BVrel ) = u_ActDsk(4)%DiskAve_Vrel
 m%AllOuts( SP2TCp   ) = m%ActDsk(3)%Cp
 m%AllOuts( SP2BCp   ) = m%ActDsk(4)%Cp
 m%AllOuts( SP2TCq   ) = m%ActDsk(3)%Cq
 m%AllOuts( SP2BCq   ) = m%ActDsk(4)%Cq
 m%AllOuts( SP2TCt   ) = m%ActDsk(3)%Ct
 m%AllOuts( SP2BCt   ) = m%ActDsk(4)%Ct
 m%AllOuts( SP2TFx   ) = forces(1,3)
 m%AllOuts( SP2BFx   ) = forces(1,4)
 m%AllOuts( SP2TFy   ) = forces(2,3)
 m%AllOuts( SP2BFy   ) = forces(2,4)
 m%AllOuts( SP2TFz   ) = forces(3,3)
 m%AllOuts( SP2BFz   ) = forces(3,4)
 m%AllOuts( SP2TMx   ) = moments(1,3)
 m%AllOuts( SP2BMx   ) = moments(1,4)
 m%AllOuts( SP2TMy   ) = moments(2,3)
 m%AllOuts( SP2BMy   ) = moments(2,4)
 m%AllOuts( SP2TMz   ) = moments(3,3)
 m%AllOuts( SP2BMz   ) = moments(3,4)
 m%AllOuts( SP2TPwr  ) = y_ActDsk(3)%P
 m%AllOuts( SP2BPwr  ) = y_ActDsk(4)%P

 end if
 offset = p%NumPylons*2
 
 ! Orientation is transform from global to local
do i = 1, 2*p%NumPylons
   DCM = u%PPyRtrMotions(i)%Orientation(:,:,1)
   forces(:,i)  = matmul(DCM, y%PPyRtrLoads(i)%Force(:,1))
   moments(:,i) = matmul (DCM,y%PPyRtrLoads(i)%Moment(:,1))
end do
   
   ! Port Pylon 1   Top and Bottom rotors
 !m%AllOuts( PP1TVRelx) = m%ActDsk(1+offset)%m%DiskAve_Vx_Rel
 !m%AllOuts( PP1BVRelx) = m%ActDsk(2+offset)%m%DiskAve_Vx_Rel
 m%AllOuts( PP1TPitch) = u_ActDsk(1+offset)%pitch*R2D
 m%AllOuts( PP1BPitch) = u_ActDsk(2+offset)%pitch*R2D
 m%AllOuts( PP1TTSR  ) = m%ActDsk(1+offset)%TSR
 m%AllOuts( PP1BTSR  ) = m%ActDsk(2+offset)%TSR
 m%AllOuts( PP1TRtSpd) = u_ActDsk(1+offset)%RtSpd
 m%AllOuts( PP1BRtSpd) = u_ActDsk(2+offset)%RtSpd
 m%AllOuts( PP1TSkew ) = u_ActDsk(1+offset)%skew*R2D
 m%AllOuts( PP1BSkew ) = u_ActDsk(2+offset)%skew*R2D
 m%AllOuts( PP1TVrel ) = u_ActDsk(1+offset)%DiskAve_Vrel
 m%AllOuts( PP1BVrel ) = u_ActDsk(2+offset)%DiskAve_Vrel
 m%AllOuts( PP1TCp   ) = m%ActDsk(1+offset)%Cp
 m%AllOuts( PP1BCp   ) = m%ActDsk(2+offset)%Cp
 m%AllOuts( PP1TCq   ) = m%ActDsk(1+offset)%Cq
 m%AllOuts( PP1BCq   ) = m%ActDsk(2+offset)%Cq
 m%AllOuts( PP1TCt   ) = m%ActDsk(1+offset)%Ct
 m%AllOuts( PP1BCt   ) = m%ActDsk(2+offset)%Ct
 m%AllOuts( PP1TFx   ) = forces(1,1)
 m%AllOuts( PP1BFx   ) = forces(1,2)
 m%AllOuts( PP1TFy   ) = forces(2,1)
 m%AllOuts( PP1BFy   ) = forces(2,2)
 m%AllOuts( PP1TFz   ) = forces(3,1)
 m%AllOuts( PP1BFz   ) = forces(3,2)
 m%AllOuts( PP1TMx   ) = moments(1,1)
 m%AllOuts( PP1BMx   ) = moments(1,2)
 m%AllOuts( PP1TMy   ) = moments(2,1)
 m%AllOuts( PP1BMy   ) = moments(2,2)
 m%AllOuts( PP1TMz   ) = moments(3,1)
 m%AllOuts( PP1BMz   ) = moments(3,2)
 m%AllOuts( PP1TPwr  ) = y_ActDsk(1+offset)%P
 m%AllOuts( PP1BPwr  ) = y_ActDsk(2+offset)%P

   ! Port Pylon 2   Top and Bottom rotors
 !m%AllOuts( PP2TVRelx) = m%ActDsk(3+offset)%DiskAve_Vx_Rel
 !m%AllOuts( PP2BVRelx) = m%ActDsk(4+offset)%DiskAve_Vx_Rel
 if (p%NumPylons == 2 ) then
 m%AllOuts( PP2TPitch) = u_ActDsk(3+offset)%pitch*R2D
 m%AllOuts( PP2BPitch) = u_ActDsk(4+offset)%pitch*R2D
 m%AllOuts( PP2TTSR  ) = m%ActDsk(3+offset)%TSR
 m%AllOuts( PP2BTSR  ) = m%ActDsk(4+offset)%TSR
 m%AllOuts( PP2TRtSpd) = u_ActDsk(3+offset)%RtSpd
 m%AllOuts( PP2BRtSpd) = u_ActDsk(4+offset)%RtSpd
 m%AllOuts( PP2TSkew ) = u_ActDsk(3+offset)%skew*R2D
 m%AllOuts( PP2BSkew ) = u_ActDsk(4+offset)%skew*R2D
 m%AllOuts( PP2TVrel ) = u_ActDsk(3+offset)%DiskAve_Vrel
 m%AllOuts( PP2BVrel ) = u_ActDsk(4+offset)%DiskAve_Vrel
 m%AllOuts( PP2TCp   ) = m%ActDsk(3+offset)%Cp
 m%AllOuts( PP2BCp   ) = m%ActDsk(4+offset)%Cp
 m%AllOuts( PP2TCq   ) = m%ActDsk(3+offset)%Cq
 m%AllOuts( PP2BCq   ) = m%ActDsk(4+offset)%Cq
 m%AllOuts( PP2TCt   ) = m%ActDsk(3+offset)%Ct
 m%AllOuts( PP2BCt   ) = m%ActDsk(4+offset)%Ct
 m%AllOuts( PP2TFx   ) = forces(1,3)
 m%AllOuts( PP2BFx   ) = forces(1,4)
 m%AllOuts( PP2TFy   ) = forces(2,3)
 m%AllOuts( PP2BFy   ) = forces(2,4)
 m%AllOuts( PP2TFz   ) = forces(3,3)
 m%AllOuts( PP2BFz   ) = forces(3,4)
 m%AllOuts( PP2TMx   ) = moments(1,3)
 m%AllOuts( PP2BMx   ) = moments(1,4)
 m%AllOuts( PP2TMy   ) = moments(2,3)
 m%AllOuts( PP2BMy   ) = moments(2,4)
 m%AllOuts( PP2TMz   ) = moments(3,3)
 m%AllOuts( PP2BMz   ) = moments(3,4)
 m%AllOuts( PP2TPwr  ) = y_ActDsk(3+offset)%P
 m%AllOuts( PP2BPwr  ) = y_ActDsk(4+offset)%P
 end if
 ! Kite Loads
 call ComputeKiteLoads( p, u, y, m, kiteForces, kiteMoments, errStat, errMsg)
   if (errStat >= AbortErrLev) return
 
 m%AllOuts( KiteFxi ) = kiteForces(1)
 m%AllOuts( KiteFyi ) = kiteForces(2)
 m%AllOuts( KiteFzi ) = kiteForces(3)
 m%AllOuts( KiteMxi ) = kiteMoments(1)
 m%AllOuts( KiteMyi ) = kiteMoments(2)
 m%AllOuts( KiteMzi ) = kiteMoments(3)
 
end subroutine KAD_MapOutputs

!==============================================================================
! Framework Routines                                                          !
!==============================================================================                               
      

!----------------------------------------------------------------------------------------------------------------------------------
!< This routine is called at the start of the simulation to perform initialization steps.
!! The parameters are set here and not changed during the simulation.
!! The initial states and initial guess for the input are defined.
subroutine KAD_Init( InitInp, u, p, y, interval, x, xd, z, OtherState, m, InitOut, errStat, errMsg )

   type(KAD_InitInputType),       intent(inout)  :: InitInp     !< Input data for initialization routine, needs to be inout because there is a copy of some data in InitInp in BEMT_SetParameters()
   type(KAD_InputType),           intent(inout)  :: u           !< An initial guess for the input; input mesh must be defined
   type(KAD_ParameterType),       intent(  out)  :: p           !< Parameters
   type(KAD_OutputType),          intent(  out)  :: y           !< Initial system outputs (outputs are not calculated;
                                                                   !<   only the output mesh is initialized)
   real(DbKi),                    intent(inout)  :: interval    !< Coupling interval in seconds: 
                                                                !<   Input is the suggested time from the glue code;
                                                                !<   Output is the actual coupling interval that will be used
                                                                !<   by the glue code.
   type(KAD_ContinuousStateType), intent(inout)  :: x           !< Input: Continuous states at t;
                                                                       !!   Output: Continuous states at t + Interval
   type(KAD_DiscreteStateType),   intent(inout)  :: xd          !< Input: Discrete states at t;
                                                                       !!   Output: Discrete states at t + Interval
   type(KAD_ConstraintStateType), intent(inout)  :: z           !< Input: Constraint states at t;
                                                                       !!   Output: Constraint states at t + Interval
   type(KAD_OtherStateType),      intent(inout)  :: OtherState  !< Other states: Other states at t;
                                                                       !!   Output: Other states at t + Interval
   type(KAD_MiscVarType),          intent(  out)  :: m           !< MiscVars for the module
   
   type(KAD_InitOutputType),      intent(  out)  :: InitOut     !< Output for initialization routine
   integer(IntKi),                intent(  out)  :: errStat     !< Error status of the operation
   character(*),                  intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None


      ! Local variables
   type(ActDsk_InitOutputType)                  :: ActDsk_InitOut
   character(ErrMsgLen)                         :: errMsg2     ! temporary Error message if errStat /= ErrID_None
   integer(IntKi)                               :: errStat2    ! temporary Error status of the operation
   character(*), parameter                      :: routineName = 'KAD_Init'
   integer(IntKi)                               :: i, j, n, count, numElem, n1, n2, nIfWPts
   type(VSM_InitOutputType)                     :: VSM_InitOut
   type(VSM_InitInputType)                      :: VSM_InitInp
   integer(IntKi), allocatable                  :: VSM_numCompElems(:)
   integer(IntKi)                               :: c
   real(ReKi), allocatable                      :: VSM_ElemPts(:,:,:)
   
      ! Initialize variables for this routine
   errStat         = ErrID_None
   errMsg          = "" 
   
      ! Initialize the NWTC Subroutine Library
   call NWTC_Init( EchoLibVer=.FALSE. )

      ! Return module version information 
   InitOut%Version = KAD_Ver

      ! Display the module information
   call DispNVD( KAD_Ver )

      ! Read Input file, otherwise the caller must have fully populated the Initialization inputs
   if ( trim(InitInp%Filename) /= '' ) then
      call ReadKADFile(InitInp, interval, errStat, errMsg)
         if ( errStat > ErrID_None ) return
   endif
   
      ! Validate the input file data and issue errors as needed
   call ValidateInitData(InitInp, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return
   
      !----------------------------------
      ! Initialize the ActuatorDisk modules
      !----------------------------------
      
      ! Allocate miscvars arrays for actuator disk models
   allocate(m%ActDsk( InitInp%NumPylons*2*2 ) , STAT = errStat2 )
   if (errStat2 /=0) then
      call SetErrStat( ErrID_FATAL, 'Could not allocate memory for m%ActDsk', errStat, errMsg, routineName )
      return
   end if
   allocate(m%u_ActDsk( InitInp%NumPylons*2*2 ) , STAT = errStat2 )
   if (errStat2 /=0) then
      call SetErrStat( ErrID_FATAL, 'Could not allocate memory for m%u_ActDsk', errStat, errMsg, routineName )
      return
   end if
   allocate(m%y_ActDsk( InitInp%NumPylons*2*2 ) , STAT = errStat2 )
   if (errStat2 /=0) then
      call SetErrStat( ErrID_FATAL, 'Could not allocate memory for m%y_ActDsk', errStat, errMsg, routineName )
      return
   end if
   allocate(p%ActDsk( InitInp%NumPylons*2*2 ) , STAT = errStat2 )
   if (errStat2 /=0) then
      call SetErrStat( ErrID_FATAL, 'Could not allocate memory for p%ActDsk', errStat, errMsg, routineName )
      return
   end if
   
   
   
   do i=1,InitInp%NumPylons*2*2
         ! Set the Air Density
      InitInp%InpFileData%RtrProps(i)%AirDens = InitInp%InpFileData%AirDens
      InitInp%InpFileData%RtrProps(i)%RotorMod = InitInp%InpFileData%RotorMod
      call ActDsk_Init( InitInp%InpFileData%RtrProps(i), m%u_ActDsk(i), p%ActDsk(i), m%y_ActDsk(i), InitInp%InpFileData%DTAero, ActDsk_InitOut, errStat, errMsg )
         if ( errStat >= AbortErrLev ) return
   end do
   
      ! Set parameters based on initialization inputs
   p%NumFlaps  = InitInp%NumFlaps
   p%NumPylons = InitInp%NumPylons
   p%LiftMod   = InitInp%InpFileData%LiftMod
   p%RotorMod  = InitInp%InpFileData%RotorMod   
   p%DTAero    = InitInp%InpFileData%DTAero
   p%AirDens   = InitInp%InpFileData%AirDens
   p%KinVisc   = InitInp%InpFileData%KinVisc
   p%SpdSound  = InitInp%InpFileData%SpdSound
      ! Override the driver-requested timestep and send the module timestep back to driver
   interval    = p%DTAero
   
   n = InitInp%InpFileData%FusProps%NumNds
   call AllocAry( p%FusChord, n, 'p%FusChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do i=1,n
      p%FusChord (i) = InitInp%InpFileData%FusProps%Chord(i)      
   end do
   
   n = InitInp%InpFileData%SWnProps%NumNds
   call AllocAry( p%SWnCtrlID, n, 'p%SWnCtrlID', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( p%SWnChord, n, 'p%SWnChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do i=1,n
      p%SWnChord (i) = InitInp%InpFileData%SWnProps%Chord(i)
      p%SWnCtrlID(i) = InitInp%InpFileData%SWnProps%CntrlID(i)
   end do
   
   n = InitInp%InpFileData%PWnProps%NumNds
   call AllocAry( p%PWnCtrlID, n, 'p%PWnCtrlID', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( p%PWnChord, n, 'p%PWnChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do i=1,n
      p%PWnChord (i) = InitInp%InpFileData%PWnProps%Chord(i)
      p%PWnCtrlID(i) = InitInp%InpFileData%PWnProps%CntrlID(i)
   end do
   
   n = InitInp%InpFileData%VSProps%NumNds
   call AllocAry( p%VSCtrlID, n, 'p%VSCtrlID', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( p%VSChord, n, 'p%VSChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do i=1,n
      p%VSChord (i) = InitInp%InpFileData%VSProps%Chord(i)
      p%VSCtrlID(i) = InitInp%InpFileData%VSProps%CntrlID(i)
   end do
   
   n = InitInp%InpFileData%SHSProps%NumNds
   call AllocAry( p%SHSCtrlID, n, 'p%SHSCtrlID', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( p%SHSChord, n, 'p%SHSChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do i=1,n
      p%SHSChord (i) = InitInp%InpFileData%SHSProps%Chord(i)
      p%SHSCtrlID(i) = InitInp%InpFileData%SHSProps%CntrlID(i)
   end do
   
   n = InitInp%InpFileData%PHSProps%NumNds
   call AllocAry( p%PHSCtrlID, n, 'p%PHSCtrlID', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( p%PHSChord, n, 'p%PHSChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do i=1,n
      p%PHSChord (i) = InitInp%InpFileData%PHSProps%Chord(i)
      p%PHSCtrlID(i) = InitInp%InpFileData%PHSProps%CntrlID(i)
   end do
 
   n = 0
   do j=1,p%NumPylons
      n = max(n,InitInp%InpFileData%SPyProps(j)%NumNds)
   end do
   call AllocAry( p%SPyChord, n, p%NumPylons, 'p%SPyChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( p%SPyElemLen, n-1, p%NumPylons, 'p%SPyElemLen', errStat2, errMsg2 ) ! Need to pre-allocate this because it is 2D unlike the other component versions
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do j=1,p%NumPylons
      do i=1,InitInp%InpFileData%SPyProps(j)%NumNds
         p%SPyChord(i,j) = InitInp%InpFileData%SPyProps(j)%Chord(i)
      end do
   end do
   
   n = 0
   do j=1,p%NumPylons
      n = max(n,InitInp%InpFileData%PPyProps(j)%NumNds)
   end do
   call AllocAry( p%PPyChord, n, p%NumPylons, 'p%PPyChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( p%PPyElemLen, n-1, p%NumPylons, 'p%PPyElemLen', errStat2, errMsg2 ) ! Need to pre-allocate this because it is 2D unlike the other component versions
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do j=1,p%NumPylons
      do i=1,InitInp%InpFileData%PPyProps(j)%NumNds
         p%PPyChord(i,j) = InitInp%InpFileData%PPyProps(j)%Chord(i)
      end do
   end do
   
   !----------------------------------
   ! Create and initialize the Inputs
   !----------------------------------
   
   call Init_u( u, p, InitInp, nIfWPts, errStat, errMsg )
      if ( errStat >= AbortErrLev ) return
      
   
   call KAD_CopyInput( u, m%u_Interp, MESH_NEWCOPY, errStat, errMsg )
      if ( errStat >= AbortErrLev ) return
      
   ! Create a MiscVar sibling of the FusOMotions for integrating kite loads
   call MeshCopy (  SrcMesh  = u%FusOMotions      &
                  , DestMesh = m%FusOLoads        &
                  , CtrlCode = MESH_SIBLING       &
                  , IOS      = COMPONENT_OUTPUT   &
                  , force    = .TRUE.             &
                  , moment   = .TRUE.             &
                  , ErrStat  = errStat2           &
                  , ErrMess  = errMsg2            )
   
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return
               
   call MeshCommit(m%FusOLoads, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )          
      if (errStat >= AbortErrLev) return
      
      
      
   !----------------------------------
   ! Create and initialize the Outputs
   !----------------------------------
      
   call Init_y( y, u, InitInp, p, errStat, errMsg )
      if ( errStat >= AbortErrLev ) return
   
   
   call CreateMeshMappings( u, y, p, m, errStat, errMsg )
      if ( errStat >= AbortErrLev ) return
      
      
   !----------------------------------
   ! Create the file-related output data
   !----------------------------------
   allocate( m%AllOuts(0:MaxOutPts), STAT=errStat2 ) ! allocate starting at zero to account for invalid output channels
   IF ( ErrStat /= 0 )  THEN
      errStat = ErrID_Fatal
      errMsg  = ' Error allocating memory for the AllOuts array.'
      RETURN
   ENDIF   
   m%AllOuts   = 0.0_ReKi
   p%OutSwtch  = InitInp%InpFileData%OutSwtch
   p%NumOuts   = InitInp%InpFileData%NumOuts
   p%OutFmt    = InitInp%InpFileData%OutFmt
   p%NFusOuts  = InitInp%InpFileData%NFusOuts
   p%FusOutNd  = InitInp%InpFileData%FusOutNd
   p%NSWnOuts  = InitInp%InpFileData%NSWnOuts
   p%SWnOutNd  = InitInp%InpFileData%SWnOutNd
   p%NPWnOuts  = InitInp%InpFileData%NPWnOuts      
   p%PWnOutNd  = InitInp%InpFileData%PWnOutNd
   p%NVSOuts   = InitInp%InpFileData%NVSOuts
   p%VSOutNd   = InitInp%InpFileData%VSOutNd   
   p%NSHSOuts  = InitInp%InpFileData%NSHSOuts
   p%SHSOutNd  = InitInp%InpFileData%SHSOutNd
   p%NPHSOuts  = InitInp%InpFileData%NPHSOuts
   p%PHSOutNd  = InitInp%InpFileData%PHSOutNd
   p%NPylOuts  = InitInp%InpFileData%NPylOuts
   p%PylOutNd  = InitInp%InpFileData%PylOutNd

         ! Set parameters for output channels:
   CALL KAD_SetOutParam(InitInp%InpFileData%OutList, p, errStat, errMsg ) ! requires: p%NumOuts, p%NumBl, p%NBlGages, p%NTwGages; sets: p%OutParam.
      IF (ErrStat >= AbortErrLev) RETURN

   IF ( InitInp%InpFileData%TabDelim ) THEN
      p%Delim = TAB
   ELSE
      p%Delim = ' '
   END IF

   CALL AllocAry( InitOut%WriteOutputHdr, p%NumOuts, 'WriteOutputHdr', ErrStat, ErrMsg )
      IF (ErrStat >= AbortErrLev) RETURN
   CALL AllocAry( InitOut%WriteOutputUnt, p%NumOuts, 'WriteOutputUnt', ErrStat, ErrMsg )
      
      IF (ErrStat >= AbortErrLev) RETURN

   do i=1,p%NumOuts
      InitOut%WriteOutputHdr(i) = p%OutParam(i)%Name
      InitOut%WriteOutputUnt(i) = p%OutParam(i)%Units
   end do

   
   ! Open and initialize the output file/data
   p%OutFileRoot = InitInp%OutFileRoot
   
   if (p%OutSwtch /= 2) then   
      call KAD_OpenOutput( KAD_Ver, p%OutFileRoot,  p, InitOut, ErrStat, ErrMsg )
         if (ErrStat >= AbortErrLev) return
   end if
   
   call AllocAry( y%WriteOutput, p%NumOuts, 'y%WriteOutput', ErrStat, ErrMsg )   
      if (ErrStat >= AbortErrLev) return
   y%WriteOutput = 0.0_ReKi
   
   
   !----------------------------------
   ! Initialize the VSM module
   !----------------------------------  
   VSM_InitInp%OutRootname = 'Kite_Test' 
   VSM_InitInp%AirDens     = InitInp%InpFileData%AirDens
   VSM_InitInp%KinVisc     = InitInp%InpFileData%KinVisc
   VSM_InitInp%LiftMod     = InitInp%InpFileData%LiftMod
   VSM_InitInp%VSMToler    = InitInp%InpFileData%VSMToler
   VSM_InitInp%VSMMaxIter  = InitInp%InpFileData%VSMMaxIter
   VSM_InitInp%VSMPerturb  = InitInp%InpFileData%VSMPerturb
   VSM_InitInp%CtrlPtMod   = InitInp%InpFileData%VSMMod
   VSM_InitInp%AFTabMod    = InitInp%InpFileData%AFTabMod  
   VSM_InitInp%InCol_Alfa  = InitInp%InpFileData%InCol_Alfa
   VSM_InitInp%InCol_Cl    = InitInp%InpFileData%InCol_Cl  
   VSM_InitInp%InCol_Cd    = InitInp%InpFileData%InCol_Cd  
   VSM_InitInp%InCol_Cm    = InitInp%InpFileData%InCol_Cm  
   VSM_InitInp%NumAFfiles  = InitInp%InpFileData%NumAFfiles
   VSM_InitInp%NumVolElem  = u%FusMotions%NElemList
   
   call AllocAry( VSM_InitInp%AFNames, VSM_InitInp%NumAFfiles, 'VSM_InitInp%AFNames', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   VSM_InitInp%AFNames     = InitInp%InpFileData%AFNames
   
   call AllocAry( VSM_numCompElems, 6+2*p%NumPylons, 'VSM_numCompElems', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   VSM_numCompElems(1) = u%FusMotions%NElemList  
   VSM_numCompElems(2) = u%SWnMotions%NElemList
   VSM_numCompElems(3) = u%PWnMotions%NElemList
   VSM_numCompElems(4) = u%VSMotions%NElemList
   VSM_numCompElems(5) = u%SHSMotions%NElemList
   VSM_numCompElems(6) = u%PHSMotions%NElemList
   c = 7
   numElem = u%FusMotions%NElemList + u%SWnMotions%NElemList + u%PWnMotions%NElemList + u%VSMotions%NElemList + u%SHSMotions%NElemList + u%PHSMotions%NElemList
   do i = 1, + p%NumPylons
      VSM_numCompElems(c) = u%SPyMotions(i)%NElemList
      c = c + 1
      numElem = numElem + u%SPyMotions(i)%NElemList
   end do
   do i = 1, + p%NumPylons
      VSM_numCompElems(c) = u%PPyMotions(i)%NElemList
      c = c + 1
      numElem = numElem + u%PPyMotions(i)%NElemList
   end do
   VSM_InitInp%NumElem     = numElem
   
   call AllocAry( VSM_InitInp%Chords, numElem, 'VSM_InitInp%Chords', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( VSM_InitInp%AFIDs, numElem, 'VSM_InitInp%AFIDs', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( VSM_ElemPts, 3,2,numElem, 'VSM_ElemPts', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( VSM_InitInp%ElemLens, numElem, 'VSM_InitInp%ElemLens', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName ) 
      
   ! Loop over kite components to set these values
   count = 1
   
   ! TODO: For now we are including the fuselage in the VSM, but we are handling them differently than the other elements, so 
   !       they are first in the list and we need to tell the module how many of these we have.  GJH 1/16/2018
   
   do i = 1, u%FusMotions%NElemList
      n1 = u%FusMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%FusMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)      = u%FusMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)      = u%FusMotions%Position(:,n2)
      VSM_InitInp%Chords(count)   = (InitInp%InpFileData%FusProps%Chord(n1) + InitInp%InpFileData%FusProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)    = InitInp%InpFileData%FusProps%AFID(n1)
      VSM_InitInp%ElemLens(count) = p%FusElemLen(i)
      count = count + 1
   end do  
   
   do i = 1, u%SWnMotions%NElemList
      n1 = u%SWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%SWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)    
      VSM_ElemPts(:,1,count)      = u%SWnMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)      = u%SWnMotions%Position(:,n2)
      VSM_InitInp%Chords(count)   = (InitInp%InpFileData%SWnProps%Chord(n1) + InitInp%InpFileData%SWnProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)    = InitInp%InpFileData%SWnProps%AFID(n1)
      VSM_InitInp%ElemLens(count) = p%SWnElemLen(i)
      count = count + 1
   end do
   do i = 1, u%PWnMotions%NElemList
      n1 = u%PWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%PWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)      = u%PWnMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)      = u%PWnMotions%Position(:,n2)
      VSM_InitInp%Chords(count)   = (InitInp%InpFileData%PWnProps%Chord(n1) + InitInp%InpFileData%PWnProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)    = InitInp%InpFileData%PWnProps%AFID(n1)
      VSM_InitInp%ElemLens(count) = p%PWnElemLen(i)
      count = count + 1
   end do
   do i = 1, u%VSMotions%NElemList
      n1 = u%VSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%VSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)      = u%VSMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)      = u%VSMotions%Position(:,n2)
      VSM_InitInp%Chords(count)   = (InitInp%InpFileData%VSProps%Chord(n1) + InitInp%InpFileData%VSProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)    = InitInp%InpFileData%VSProps%AFID(n1)
      VSM_InitInp%ElemLens(count) = p%VSElemLen(i)
      count = count + 1
   end do
   do i = 1, u%SHSMotions%NElemList
      n1 = u%SHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%SHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)      = u%SHSMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)      = u%SHSMotions%Position(:,n2)
      VSM_InitInp%Chords(count)   = (InitInp%InpFileData%SHSProps%Chord(n1) + InitInp%InpFileData%SHSProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)    = InitInp%InpFileData%SHSProps%AFID(n1)
      VSM_InitInp%ElemLens(count) = p%SHSElemLen(i)
      count = count + 1
   end do
   do i = 1, u%PHSMotions%NElemList
      n1 = u%PHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%PHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)      = u%PHSMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)      = u%PHSMotions%Position(:,n2)
      VSM_InitInp%Chords(count)   = (InitInp%InpFileData%PHSProps%Chord(n1) + InitInp%InpFileData%PHSProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)    = InitInp%InpFileData%PHSProps%AFID(n1)
      VSM_InitInp%ElemLens(count) = p%PHSElemLen(i)
      count = count + 1
   end do
   do j = 1, p%NumPylons
      do i = 1, u%SPyMotions(j)%NElemList
         n1 = u%SPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
         n2 = u%SPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
         VSM_ElemPts(:,1,count)      = u%SPyMotions(j)%Position(:,n1)
         VSM_ElemPts(:,2,count)      = u%SPyMotions(j)%Position(:,n2)
         VSM_InitInp%Chords(count)   = (InitInp%InpFileData%SPyProps(j)%Chord(n1) + InitInp%InpFileData%SPyProps(j)%Chord(n2)) / 2.0
         VSM_InitInp%AFIDs(count)    = InitInp%InpFileData%SPyProps(j)%AFID(n1)
         VSM_InitInp%ElemLens(count) = p%SPyElemLen(i,j)
         count = count + 1
      end do
   end do
   do j = 1, p%NumPylons
      do i = 1, u%PPyMotions(j)%NElemList
         n1 = u%PPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
         n2 = u%PPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
         VSM_ElemPts(:,1,count)      = u%PPyMotions(j)%Position(:,n1)
         VSM_ElemPts(:,2,count)      = u%PPyMotions(j)%Position(:,n2)
         VSM_InitInp%Chords(count)   = (InitInp%InpFileData%PPyProps(j)%Chord(n1) + InitInp%InpFileData%PPyProps(j)%Chord(n2)) / 2.0
         VSM_InitInp%AFIDs(count)    = InitInp%InpFileData%PPyProps(j)%AFID(n1)
         VSM_InitInp%ElemLens(count) = p%PPyElemLen(i,j)
         count = count + 1
      end do
   end do

   call VSM_Init( VSM_InitInp, m%u_VSM, p%VSM, z%VSM, OtherState%VSM, m%VSM, m%y_VSM, interval, VSM_InitOut, errStat, errMsg )
      if ( errStat >= AbortErrLev ) return
      
   InitOut%AirDens = p%AirDens
   InitOut%nIfWPts = nIfWPts   
   
   if ( InitInp%InpFileData%SumPrint ) call KAD_WriteSummary( p%OutFileRoot, VSM_ElemPts, VSM_numCompElems, errStat, errMsg )
   
end subroutine KAD_Init
        
                              
!----------------------------------------------------------------------------------------------------------------------------------
!> Loose coupling routine for solving constraint states, integrating continuous states, and updating discrete states.
!! Continuous, constraint, and discrete states are updated to values at t + Interval.
subroutine KAD_UpdateStates( t, n, Inputs, InputTimes, p, x, xd, z, OtherState, m, errStat, errMsg )

   real(DbKi),                    intent(in   )  :: t              !< Current simulation time in seconds
   integer(IntKi),                intent(in   )  :: n              !< Current step of the simulation: t = n*Interval
   type(KAD_InputType),           intent(inout)  :: Inputs(:)      !< Inputs at Time
   real(DbKi),                    intent(in   )  :: InputTimes(:)  !< Times in seconds associated with Inputs
   type(KAD_ParameterType),       intent(in   )  :: p              !< Parameters
   type(KAD_ContinuousStateType), intent(inout)  :: x              !< Input: Continuous states at t;
                                                                          !!   Output: Continuous states at t + Interval
   type(KAD_DiscreteStateType),   intent(inout)  :: xd             !< Input: Discrete states at t;
                                                                          !!   Output: Discrete states at t + Interval
   type(KAD_ConstraintStateType), intent(inout)  :: z              !< Input: Constraint states at t;
                                                                          !!   Output: Constraint states at t + Interval
   type(KAD_OtherStateType),      intent(inout)  :: OtherState     !< Other states: Other states at t;
                                                                          !!   Output: Other states at t + Interval
   type(KAD_MiscVarType),         intent(inout)  :: m              !< Initial misc/optimization variables           
   integer(IntKi),                intent(  out)  :: errStat        !< Error status of the operation
   character(*),                  intent(  out)  :: errMsg         !< Error message if errStat /= ErrID_None

   
   character(*), parameter                :: routineName = 'KAD_UpdateStates'
   real(DbKi)                             :: tnext

   
   
   errStat   = ErrID_None           ! no error has occurred
   errMsg    = ""
   
      ! We only need inputs at t + dt for the VSM module's UpdateStates
   tnext = t + p%DTAero
   call KAD_Input_ExtrapInterp(Inputs, InputTimes, m%u_Interp, tnext, errStat, errMsg)
         if ( errStat >= AbortErrLev ) return
      
      ! Now that we have the KAD inputs at t + dt, construct the VSM inputs
   call Set_VSM_Inputs(m%u_Interp, m, p, m%u_VSM, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return
        
   call VSM_UpdateStates( t, n, m%u_VSM, p%VSM, z%VSM, OtherState%VSM, m%VSM, errStat, errMsg )
   
   
end subroutine KAD_UpdateStates


!----------------------------------------------------------------------------------------------------------------------------------
!> Routine for computing outputs, used in both loose and tight coupling.
subroutine KAD_CalcOutput( Time, u, p, x, xd, z, OtherState, y, m, errStat, errMsg )   

   real(DbKi),                    intent(in   )  :: Time        !< Current simulation time in seconds
   type(KAD_InputType),           intent(inout)  :: u           !< Inputs at Time (note that this is intent out because we're copying the u%mesh into m%u_wamit%mesh)
   type(KAD_ParameterType),       intent(in   )  :: p           !< Parameters
   type(KAD_ContinuousStateType), intent(in   )  :: x           !< Continuous states at Time
   type(KAD_DiscreteStateType),   intent(in   )  :: xd          !< Discrete states at Time
   type(KAD_ConstraintStateType), intent(in   )  :: z           !< Constraint states at Time
   type(KAD_OtherStateType),      intent(in   )  :: OtherState  !< Other states at Time
   type(KAD_OutputType),          intent(inout)  :: y           !< Outputs computed at Time (Input only so that mesh con-
                                                                     !!   nectivity information does not have to be recalculated)
   type(KAD_MiscVarType),         intent(inout)  :: m           !< Initial misc/optimization variables           
   integer(IntKi),                intent(  out)  :: errStat     !< Error status of the operation
   character(*),                  intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None

   
   character(*), parameter                :: routineName = 'KAD_CalcOutput'
   integer(IntKi)                         :: n, i, j, c
   real(ReKi)                             :: forces(3), moments(3), dcm(3,3)
   real(ReKi)                             :: rotorDCMs(3,3,p%NumPylons*4)
   character(ErrMsgLen)                   :: errMsg2     ! temporary Error message if errStat /= ErrID_None
   integer(IntKi)                         :: errStat2    ! temporary Error status of the operation

   errStat   = ErrID_None           ! no error has occurred
   errMsg    = ""
   
   

   n = Time / p%DTAero
      ! Now that we have the KAD inputs at t + dt, construct the VSM inputs
   call Set_VSM_Inputs(u, m, p, m%u_VSM, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return
   call VSM_CalcOutput( Time, n, m%u_VSM, p%VSM, z%VSM, OtherState%VSM, m%VSM, m%y_VSM, errStat, errMsg )   
      if ( errStat >= AbortErrLev ) return
      
      ! Transfer motions needed for mesh mapping from the miscvar meshes to the output meshes
   call Transfer_Motions_Line2_to_Point( u%FusMotions, y%FusLoads, m%Fus_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' KAD_CalcOutput: Transfer_Fus_L_2_P' )      
   call Transfer_Motions_Line2_to_Point( u%SWnMotions, y%SWnLoads, m%SWn_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' KAD_CalcOutput: Transfer_SWn_L_2_P' )      
   call Transfer_Motions_Line2_to_Point( u%PWnMotions, y%PWnLoads, m%PWn_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' KAD_CalcOutput: Transfer_PWn_L_2_P' )      
   call Transfer_Motions_Line2_to_Point( u%VSMotions, y%VSLoads, m%VS_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' KAD_CalcOutput: Transfer_VS_L_2_P' )      
   call Transfer_Motions_Line2_to_Point( u%SHSMotions, y%SHSLoads, m%SHS_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' KAD_CalcOutput: Transfer_SHS_L_2_P' )      
   call Transfer_Motions_Line2_to_Point( u%PHSMotions, y%PHSLoads, m%PHS_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' KAD_CalcOutput: Transfer_PHS_L_2_P' )      
   do i = 1, p%NumPylons
      call Transfer_Motions_Line2_to_Point( u%SPyMotions(i), y%SPyLoads(i), m%SPy_L_2_P(i), errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' KAD_CalcOutput: Transfer_SPy_L_2_P' )      
      call Transfer_Motions_Line2_to_Point( u%PPyMotions(i), y%PPyLoads(i), m%PPy_L_2_P(i), errStat2, errMsg2 )
         call SetErrStat(errStat2, errMsg2, errStat, errMsg,' KAD_CalcOutput: Transfer_PPy_L_2_P' )      
   end do  
      
      
      ! Transfer the VSM loads to the output meshes as point loads
   c = 1
   do i = 1, y%FusLoads%nNodes
      y%FusLoads%Force(:,i)  = m%y_VSM%Loads(1:3,c)
      y%FusLoads%Moment(:,i) = m%y_VSM%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%SWnLoads%nNodes
      y%SWnLoads%Force(:,i)  = m%y_VSM%Loads(1:3,c)
      y%SWnLoads%Moment(:,i) = m%y_VSM%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%PWnLoads%nNodes
      y%PWnLoads%Force(:,i)  = m%y_VSM%Loads(1:3,c)
      y%PWnLoads%Moment(:,i) = m%y_VSM%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%VSLoads%nNodes
      y%VSLoads%Force(:,i)  = m%y_VSM%Loads(1:3,c)
      y%VSLoads%Moment(:,i) = m%y_VSM%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%SHSLoads%nNodes
      y%SHSLoads%Force(:,i)  = m%y_VSM%Loads(1:3,c)
      y%SHSLoads%Moment(:,i) = m%y_VSM%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%PHSLoads%nNodes
      y%PHSLoads%Force(:,i)  = m%y_VSM%Loads(1:3,c)
      y%PHSLoads%Moment(:,i) = m%y_VSM%Loads(4:6,c)
      c = c + 1
   end do
   do j = 1, p%NumPylons
      do i = 1, y%SPyLoads(j)%nNodes
         y%SPyLoads(j)%Force(:,i)  = m%y_VSM%Loads(1:3,c)
         y%SPyLoads(j)%Moment(:,i) = m%y_VSM%Loads(4:6,c)
         c = c + 1
      end do
   end do
   do j = 1, p%NumPylons
      do i = 1, y%PPyLoads(j)%nNodes
         y%PPyLoads(j)%Force(:,i)  = m%y_VSM%Loads(1:3,c)
         y%PPyLoads(j)%Moment(:,i) = m%y_VSM%Loads(4:6,c)
         c = c + 1
      end do
   end do

      ! Rotor Inputs and Outputs
   
   call RotorDisk_SetInputs(0,             p%NumPylons, u%V_SPyRtr, u%SPyRtrMotions, u%RtSpd_SPyRtr, u%Pitch_SPyRtr, m%u_ActDsk, rotorDCMs, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return
   call RotorDisk_SetInputs(p%NumPylons*2, p%NumPylons, u%V_PPyRtr, u%PPyRtrMotions, u%RtSpd_PPyRtr, u%Pitch_PPyRtr, m%u_ActDsk, rotorDCMs, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return 
      
   call RotorDisk_CalcOutput(0,             p%NumPylons, u%V_SPyRtr, u%SPyRtrMotions, u%RtSpd_SPyRtr, u%Pitch_SPyRtr, rotorDCMs, m%u_ActDsk, p%ActDsk, m%ActDsk, m%y_ActDsk, y%SPyRtrLoads, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return
   call RotorDisk_CalcOutput(p%NumPylons*2, p%NumPylons, u%V_PPyRtr, u%PPyRtrMotions, u%RtSpd_PPyRtr, u%Pitch_PPyRtr, rotorDCMs, m%u_ActDsk, p%ActDsk, m%ActDsk, m%y_ActDsk, y%PPyRtrLoads, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return     
   
   ! Map the output quantities to the AllOuts array
   
   call KAD_MapOutputs(p, u, m%u_VSM, y, m%y_VSM, p%VSM, m%u_ActDsk, m%y_ActDsk, m, z, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return
      
   !...............................................................................................................................
   ! Place the selected output channels into the WriteOutput(:) array with the proper sign:
   !...............................................................................................................................

   dO i = 1,p%NumOuts  ! Loop through all selected output channels
      y%WriteOutput(i) = p%OutParam(i)%SignM * m%AllOuts( p%OutParam(i)%Indx )
   end do             ! i - All selected output channels
   

   if (p%OutSwtch /= 2) call KAD_WrOutputLine(Time, p, y%WriteOutput, errStat, errMsg)
   
end subroutine KAD_CalcOutput

subroutine KAD_End(u, p, x, xd, z, other, y, m, errStat , errMsg)
   type(KAD_InputType) ,            intent(inout) :: u
   type(KAD_ParameterType) ,        intent(inout) :: p
   type(KAD_ContinuousStateType) ,  intent(inout) :: x
   type(KAD_DiscreteStateType) ,    intent(inout) :: xd
   type(KAD_ConstraintStateType) ,  intent(inout) :: z
   type(KAD_OtherStateType) ,       intent(inout) :: other
   type(KAD_OutputType) ,           intent(inout) :: y
   type(KAD_MiscVarType),           intent(inout) :: m      
   integer(IntKi),                  intent(  out) :: errStat
   character(*),                    intent(  out) :: errMsg

!      integer(IntKi)                                :: i=0
   integer(IntKi)                               :: i
   integer(IntKi)                               :: errStat2      ! Error status of the operation
   character(ErrMsgLen)                         :: errMsg2       ! Error message if ErrStat2 /= ErrID_None
   character(*), parameter                      :: routineName = 'KAD_End'
   ErrStat = ErrID_None
   ErrMsg  = ""

      ! End all ActDsk modules
   do i = 1,p%NumPylons*4
      call ActDsk_End( m%u_ActDsk(i), p%ActDsk(i), m%y_ActDsk(i), m%ActDsk(i), errStat2, errMsg2)
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   end do
   
      ! End the VSM module
   call VSM_End( m%u_VSM, p%VSM, z%VSM, m%y_VSM, other%VSM, m%VSM, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
      ! deallocate data structures
   call KAD_DestroyInput(u, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KAD_DestroyParam(p, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KAD_DestroyContState(x, ErrStat2, ErrMsg2)  
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KAD_DestroyDiscState(xd, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KAD_DestroyConstrState(z, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KAD_DestroyOtherState(other,ErrStat2,ErrMsg2) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KAD_DestroyOutput(y, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
   call KAD_DestroyMisc(m, ErrStat2, ErrMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, routineName )
      
      ! close output file
   if (p%OutSwtch /= 2) then
      call KAD_CloseOutput ( p, errStat, errMsg )
   end if
   
      
end subroutine KAD_End

end module KiteAeroDyn
