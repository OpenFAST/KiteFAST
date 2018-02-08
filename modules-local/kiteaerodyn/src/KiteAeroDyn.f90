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


   contains

!> Routine to compute the rotor loads using a Actuator Disk Method on a per wing basis.
subroutine RotorDisk_CalcOutput(c_offset, numPylons, V_PyRtr, PyRtrMotions, omegas, pitches, ActDsk, PyRtrLoads, errStat, errMsg)
   integer(IntKi),          intent(in   ) :: c_offset          !< Offset into the 1D ActDsk array, either 0 (starboard wing) or numPylons (port wing)
   integer(IntKi),          intent(in   ) :: numPylons         !< Number of pylons
   real(ReKi),              intent(in   ) :: V_PyRtr(:,:,:)    !< Undisturbed wind velocities at the rotors (1st index: u,v,w, 2nd index: 1=top, 2=bottom, 3rd index: 1,NumPylons)
   type(MeshType),          intent(in   ) :: PyRtrMotions(:)   !< Rotor point meshes
   real(ReKi),              intent(in   ) :: omegas(:,:)       !< Rotor speeds in rad/s for the wing's pylons (1st index: 1=top, 2=bottom, 2nd index: 1,NumPylons)
   real(ReKi),              intent(in   ) :: pitches(:,:)      !< Rotor pitches in rad for the wing's pylons (1st index: 1=top, 2=bottom, 2nd index: 1,NumPylons)
   type(KAD_ActDsk_Data),   intent(inout) :: ActDsk(:)         !< Framework data associated with all the Actuator Disk models, starboard wing first, followed by port wing
   type(MeshType),          intent(inout) :: PyRtrLoads(:)     !< Rotor loads meshes
   integer(IntKi),          intent(  out) :: errStat           !< Error status of the operation
   character(*),            intent(  out) :: errMsg            !< Error message if errStat /= ErrID_None
   
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
            
            ! Relative averaged motion of the rotor hub
         Vrel = V_PyRtr(:,i,j) - PyRtrMotions(index)%TranslationVel(:,1)
            
            ! orientation vector along rotor disk negative, x-axis:
         x_hat_disk = -PyRtrMotions(index)%Orientation(1,:,1)      
            
            ! Magnitude of the average motion along  negative x-axis
         tmp = dot_product(Vrel, x_hat_disk)
         
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
            ActDsk(c)%u%skew = 0.0_ReKi
         else
               ! make sure we don't have numerical issues that make the ratio outside +/-1
            tmp_sz_y = min(  1.0_ReKi, V_dot_x / tmp_sz )
            tmp_sz_y = max( -1.0_ReKi, tmp_sz_y )
      
            ActDsk(c)%u%skew = acos( tmp_sz_y )
      
         end if         
         
         ActDsk(c)%u%DiskAve_Vx_Rel = abs(V_dot_x)
         ActDsk(c)%u%DiskAve_Vinf_Rel = tmp_sz
         ActDsk(c)%u%omega = omegas(i,j)
         ActDsk(c)%u%pitch = pitches(i,j)
       
            ! Compute the outputs from the Actuator disk model for a particular rotor
         call ActDsk_CalcOutput(ActDsk(c)%u, ActDsk(c)%p, ActDsk(c)%m, ActDsk(c)%y, errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
         dcm(1,:) = x_hat_disk
         dcm(2,:) = y_hat_disk
         dcm(3,:) = z_hat_disk
            ! dcm is transform from global to local, but we need local to global so transpose
         dcm = transpose(dcm)
         forces  = (/ActDsk(c)%y%Fx,ActDsk(c)%y%Fy,ActDsk(c)%y%Fz/)
         moments = (/ActDsk(c)%y%Mx,ActDsk(c)%y%My,ActDsk(c)%y%Mz/)
         
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
                     ,TranslationVel = .true.    &
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation
   position = 0.0_ReKi
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
      if (.not. EqualRealNos ( TwoNorm(position), 0.0_ReKi ) ) then 
         call MeshConstructElement( mesh, ELEMENT_LINE2, errStat2, errMsg2, p1=j, p2=j+1 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      end if
   end do 
            
   call MeshCommit(mesh, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return

      
   mesh%Orientation     = mesh%RefOrientation
   mesh%TranslationDisp = 0.0_R8Ki
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
      if (.not. EqualRealNos(TwoNorm(position),0.0_ReKi)) then
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
                     , orientation = .true.        &
                     , translationdisp = .true.        &
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
            
      ! set node initial position/orientation
   position = 0.0_ReKi
   theta    = 0.0_R8Ki
   count = 1
   do j=1,numNodes     
      position(:) = positions(:,j+1) - positions(:,j)
      elemLen = TwoNorm(position)
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

!> Routine to generate the point loads meshes for the rotor components.
subroutine CreateRtrPtLoadsMesh(origin, mesh, errStat, errMsg)
   real(ReKi),                   intent(in   )  :: origin(3)         !< Reference position for the mesh in global coordinates
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

   
   
   call MeshCreate ( BlankMesh = mesh     &
                     ,Nnodes    = 1         &
                     ,errStat   = errStat2         &
                     ,ErrMess   = errMsg2          &
                     ,IOS       = COMPONENT_OUTPUT &
                     , force    = .true.           &
                     , moment   = .true.           &
                     , orientation = .true.        &
                     )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
            
   
   call Eye(orientation, errStat, errMsg) 
         
   call MeshPositionNode(mesh, 1, origin, errStat2, errMsg2, orientation)  
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call MeshConstructElement( mesh, ELEMENT_POINT, errStat2, errMsg2, p1=1 )
   call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   
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
   integer(IntKi)                               :: i
   character(*), parameter                      :: routineName = 'CreateMeshMappings'
      ! Initialize variables for this routine

   errStat = ErrID_None
   errMsg  = ""
   
   
      ! Create mesh mappings
   
   call MeshCopy( y%FusLoads, m%FusLoads, MESH_NEWCOPY, ErrStat2, ErrMsg2 )
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
         
   call MeshCopy( y%VSPLoads, m%VSPLoads, MESH_NEWCOPY, ErrStat2, ErrMsg2 )
      call SetErrStat(ErrStat2, ErrMsg2, ErrStat, ErrMsg, RoutineName)
         if (ErrStat>=AbortErrLev) return         
   call MeshMapCreate( u%VSPMotions, m%VSPLoads, m%VSP_L_2_P, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, ' CreateMeshMappings: VSP_L_2_P' )     
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
   call CreatePtLoadsMesh(InitInp%SWnOR, InitInp%InpFileData%SWnProps%NumNds-1, InitInp%InpFileData%SWnProps%Pos, alignDCM, InitInp%InpFileData%SWnProps%Twist, 2, y%SWnLoads, p%SWnElemLen, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return
      
      ! Port Wing Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreatePtLoadsMesh(InitInp%PWnOR, InitInp%InpFileData%PWnProps%NumNds-1, InitInp%InpFileData%PWnProps%Pos, alignDCM, InitInp%InpFileData%PWnProps%Twist, 2, y%PWnLoads, p%PWnElemLen, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return
   
      ! Vertical Stabilizer Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0/), (/3,3/) )  
   call CreatePtLoadsMesh(InitInp%VSPOR, InitInp%InpFileData%VSPProps%NumNds-1, InitInp%InpFileData%VSPProps%Pos, alignDCM, InitInp%InpFileData%VSPProps%Twist, 3, y%VSPLoads, p%VSPElemLen, errStat2, errMsg2)
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
         call CreateRtrPtLoadsMesh(InitInp%SPyRtrOR(:,i,j), y%SPyRtrLoads(i+(j-1)*InitInp%NumPylons), errStat2, errMsg2)
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            if (errStat >= AbortErrLev) return
      end do
      
   end do

      ! Port Rotor Meshes
   do j=1,InitInp%NumPylons
      do i = 1,2
         call CreateRtrPtLoadsMesh(InitInp%PPyRtrOR(:,i,j), y%PPyRtrLoads(i+(j-1)*InitInp%NumPylons), errStat2, errMsg2)
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
   n = InitInp%InpFileData%VSPProps%NumNds
   nIfWPts = nIfWPts + n
   call AllocAry( u%V_VSP, 3_IntKi, n, 'u%V_VSP', errStat2, errMsg2 )
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
   call AllocAry( u%Omega_SPyRtr, 2_IntKi, n, 'u%Omega_SPyRtr', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( u%Omega_PPyRtr, 2_IntKi, n, 'u%Omega_PPyRtr', errStat2, errMsg2 )
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
   u%V_VSP    = 0.0_ReKi
   u%V_SHS    = 0.0_ReKi
   u%V_PHS    = 0.0_ReKi
   u%V_SPy    = 0.0_ReKi
   u%V_PPy    = 0.0_ReKi
   u%V_SPyRtr = 0.0_ReKi
   u%V_PPyRtr = 0.0_ReKi
   u%Omega_SPyRtr = 0.0_ReKi
   u%Omega_PPyRtr = 0.0_ReKi
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
                       ,Orientation     = .true.    &
                       ,TranslationDisp = .true.    &
                       ,TranslationVel  = .true.    &
                       ,RotationVel     = .true.    &
                      )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

   if (errStat >= AbortErrLev) return
                     
   call MeshPositionNode(u%FusOMotions, 1, (/0.0_ReKi, 0.0_ReKi,0.0_ReKi/), errStat2, errMsg2) ! Orientation is Identity
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
   call MeshConstructElement( u%FusOMotions, ELEMENT_POINT, errStat2, errMsg2, p1=1 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   call MeshCommit(u%FusOMotions, errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
   if (errStat >= AbortErrLev) return
     
   u%FusOMotions%Orientation     = u%FusOMotions%RefOrientation
   u%FusOMotions%TranslationDisp = 0.0_R8Ki
   
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
                      )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )

         if (errStat >= AbortErrLev) return
                     
         call MeshPositionNode(u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons), 1,InitInp%SPyRtrOR(:,i,j), errStat2, errMsg2) ! Orientation is Identity
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
         call MeshConstructElement( u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons), ELEMENT_POINT, errStat2, errMsg2, p1=1 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
         call MeshCommit(u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons), errStat2, errMsg2 )
            call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
            
         if (errStat >= AbortErrLev) return
     
         u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons)%Orientation     = u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons)%RefOrientation
         u%SPyRtrMotions(i+(j-1)*InitInp%NumPylons)%TranslationDisp = 0.0_R8Ki
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
         u%PPyRtrMotions(i+(j-1)*InitInp%NumPylons)%TranslationDisp = 0.0_R8Ki
      end do
   end do
   
      ! Line2 Fuselage Mesh
   alignDCM = reshape( (/0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreateL2MotionsMesh((/0.0_ReKi, 0.0_ReKi,0.0_ReKi/), InitInp%InpFileData%FusProps%NumNds, InitInp%InpFileData%FusProps%Pos, alignDCM, InitInp%InpFileData%FusProps%Twist, 1, u%FusMotions, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )        
      if (errStat >= AbortErrLev) return

      ! Line2 Starboard Wing Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreateL2MotionsMesh(InitInp%SWnOR, InitInp%InpFileData%SWnProps%NumNds, InitInp%InpFileData%SWnProps%Pos, alignDCM, InitInp%InpFileData%SWnProps%Twist, 2, u%SWnMotions, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return
      
      ! Line2 Port Wing Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0/), (/3,3/) )
   call CreateL2MotionsMesh(InitInp%PWnOR, InitInp%InpFileData%PWnProps%NumNds, InitInp%InpFileData%PWnProps%Pos, alignDCM, InitInp%InpFileData%PWnProps%Twist, 2, u%PWnMotions, errStat2, errMsg2)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      if (errStat >= AbortErrLev) return
   
      ! Line2 Vertical Stabilizer Mesh
   alignDCM = reshape( (/0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0/), (/3,3/) )   
   call CreateL2MotionsMesh(InitInp%VSPOR, InitInp%InpFileData%VSPProps%NumNds, InitInp%InpFileData%VSPProps%Pos, alignDCM, InitInp%InpFileData%VSPProps%Twist, 3, u%VSPMotions, errStat2, errMsg2)
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

subroutine ComputeAeroOnMotionNodes(i, outNds, motionMesh, VSMoffset, u_VSM, y_VSM, u_V, chords, elemLens,airDens, kinVisc, &
                                    speedOfSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, Cl, Cd, Cm, Fl, Fd, &
                                    Mm, Cn, Ct, Fn, Ft, errStat, errMsg )
            
   integer(IntKi)      , intent(in   ) :: i 
   integer(IntKi)      , intent(in   ) :: outNds(:)
   type(MeshType)      , intent(in   ) :: motionMesh
   integer(IntKi)      , intent(in   ) :: VSMoffset
   type(VSM_InputType) , intent(in   ) :: u_VSM
   type(VSM_OutputType), intent(in   ) :: y_VSM
   real(ReKi)          , intent(in   ) :: u_V(:,:)
   real(ReKi)          , intent(in   ) :: chords(:)
   real(ReKi)          , intent(in   ) :: elemLens(:)
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
   real(ReKi)          , intent(  out) :: Ct
   real(ReKi)          , intent(  out) :: Fn
   real(ReKi)          , intent(  out) :: Ft
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
      errMsg  = 'node '//trim(num2lstr(outNds(i)))//'cannot be related to a VSM element.  You cannot output data for the last node in a component list.'
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
   chord    = chords(outNds(i))
   elemLen  = elemLens(outNds(i))
   Re       = Vrel * chord / (1e6*kinVisc)  ! in millions
   XM       = Vrel / speedOfSound
   DynP     = 0.5 * airDens * Vrel**2
 
      ! Now compute the equivalent loads at the motion node
   !call MapVSMloadsOntoMotionNode()
   !forces  = y_VSM%Loads(1:3,VSMElemIndx)
   !moments = y_VSM%Loads(4:6,VSMElemIndx)
   !
   !   ! Now transform these loads into the motion node (airfoil) coordinate system
   !forces  = matmul( DCM, forces  ) / elemLen
   !moments = matmul( DCM, moments ) / elemLen
   
   Cl  = y_VSM%Cl(VSMElemIndx)
   Cd  = y_VSM%Cd(VSMElemIndx)
   Cm  = y_VSM%Cm(VSMElemIndx)
   AoA = y_VSM%AoA(VSMElemIndx)
   Cn  = Cl*cos(AoA)+Cd*sin(AoA)
   Ct  = Cl*sin(AoA)-Cd*cos(AoA)
   
      ! Forces per unit length
   Ft = DynP*Ct
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
   Line = ""
   call ReadVar( UnIn, fileName, Line, "DTAero", "Time interval for aerodynamic calculations {or default} (s)", errStat2, errMsg2, UnEc)
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
         
   call Conv2UC( Line )
   if ( index(Line, "DEFAULT" ) /= 1 ) then ! If it's not "default", read this variable; otherwise use the value already stored in fileNameData%DTAero
      READ( Line, *, IOSTAT=IOS) InitInp%InpFileData%DTAero
         call CheckIOS ( IOS, fileName, 'DTAero', NumType, errStat2, errMsg2 )
         call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   else
      InitInp%InpFileData%DTAero = interval
   end if   

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

   !-------------------------- LIFTING LINE VORTEX METHOD OPTIONS ------------------------

      ! Skip the comment line.
   call ReadCom( UnIn, fileName, ' LIFTING LINE VORTEX METHOD OPTIONS ', errStat2, errMsg2  ) 
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%VSMMod, 'VSMMod', 'Trailing vortices alignment model (-) (switch) {1:chord, 2: local free stream}', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%VSMToler, 'VSMToler', 'Tolerance in the Newton iterations (m^2/s) or DEFAULT', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%VSMMaxIter, 'VSMMaxIter', 'Maximum number of Newton iterations (-) or DEFAULT', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   call ReadVar ( UnIn, fileName, InitInp%InpFileData%VSMPerturb, 'VSMPerturb', 'Perturbation size for computing the Jacobian in the Newton iterations (m^2/s) or DEFAULT', errStat2, errMsg2, UnEc )
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
   call ReadProps( UnIn, fileName, InitInp%InpFileData%SWnProps%NumNds, InitInp%InpFileData%SWnProps, 7, 'Starboard Wing', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    
      
   !-------------------------- PORT WING PROPERTIES ------------------------   
   call ReadProps( UnIn, fileName, InitInp%InpFileData%PWnProps%NumNds, InitInp%InpFileData%PWnProps, 7, 'Port Wing', errStat2, errMsg2, UnEc )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )    
     
   !-------------------------- VERTICAL STABILIZER PROPERTIES ------------------------   
   call ReadProps( UnIn, fileName, InitInp%InpFileData%VSPProps%NumNds, InitInp%InpFileData%VSPProps, 7, 'Vertical Stabilizer', errStat2, errMsg2, UnEc )
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
         InitInp%InpFileData%NFusOuts = SIZE(InitInp%InpFileData%VSOutNd)
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
   CALL AllocAry( InitInp%InpFileData%OutList, MaxOutPts, "ElastoDyn Input File's Outlist", ErrStat2, ErrMsg2 )
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

         ! Initialize variables for this routine
   errStat = ErrID_None
   errMsg  = ""

   !if ( InitInp%R       <= 0.0_ReKi )call SetErrStat( ErrID_Fatal, 'Rotor radius must be greater than zero', errStat, errMsg, routineName )
   !if ( InitInp%AirDens <= 0.0_ReKi) call SetErrStat( ErrID_Fatal, 'Air density must be greater than zero', errStat, errMsg, routineName )
   !if ( InitInp%numTSR  < 1 )        call SetErrStat( ErrID_Fatal, 'Number of TSR entries in the TSR/Skew table must be greater than 1', errStat, errMsg, routineName )
   !if ( InitInp%numSkew < 1 )        call SetErrStat( ErrID_Fatal, 'Number of Skew entries in the TSR/Skew table must be greater than 1', errStat, errMsg, routineName )
   !
   ! ! Verify that all the table data is monotonic and increasing
   !
   !
   !minVal = InitInp%TSRs(1)
   !if ( ( minVal < 0.0_ReKi ) ) call SetErrStat( ErrID_Fatal, 'TSR values must be greater than or equal to zero', errStat, errMsg, routineName )
   !
   !do i = 2, InitInp%numTSR
   !   if (  ( InitInp%TSRs(i) <= minVal ) ) then
   !      call SetErrStat( ErrID_Fatal, 'TSR values must be monotonic and increasing', errStat, errMsg, routineName )
   !      exit
   !   end if
   !   minVal = InitInp%TSRs(i)
   !end do
   !
   !minVal = InitInp%Skews(1)
   !if ( ( minVal < 0.0_ReKi ) ) call SetErrStat( ErrID_Fatal, 'Skew values must be greater than or equal to zero', errStat, errMsg, routineName )
   !if ( ( minVal > PI ) )       call SetErrStat( ErrID_Fatal, 'Skew values must be less than or equal to Pi', errStat, errMsg, routineName )
   !
   !do i = 2, InitInp%numSkew
   !   if (  ( InitInp%Skews(i) <= minVal ) ) then
   !      call SetErrStat( ErrID_Fatal, 'Skew values must be monotonic and increasing', errStat, errMsg, routineName )
   !      exit
   !   end if
   !   if ( ( InitInp%Skews(i) > PI ) ) then
   !      call SetErrStat( ErrID_Fatal, 'Skew values must be less than or equal to Pi', errStat, errMsg, routineName )
   !      exit
   !   end if
   !   
   !   minVal = InitInp%Skews(i)
   !   
   !end do
   
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
         ErrMsg = ' Error opening SubDyn-level output file: '//TRIM(ErrMsg)
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
subroutine KAD_WriteSummary( outRootName, VSM_ElemPts, VSMnumCompElems, VSM, errStat, errMsg )
! This subroutine writes the data stored in WriteOutputs (and indexed in OutParam) to the file
! opened in VSM_Init()
!---------------------------------------------------------------------------------------------------- 

      ! Passed variables 
   character(*),                intent( in    ) :: outRootName
   real(ReKi),                  intent( in    ) :: VSM_ElemPts(:,:,:)
   integer(IntKi),              intent( in    ) :: VSMnumCompElems(:)
   type(KAD_VSM_Data),          intent( in    ) :: VSM                    ! VSM module's output data
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
  ! call Transfer_Orientation( u%VSPMotions, m%VSPLoads, m%VSP_L_2_P, errStat2, errMsg2 )
   call Transfer_Motions_Line2_to_Point( u%VSPMotions, m%VSPLoads, m%VSP_L_2_P, errStat2, errMsg2 )
      call SetErrStat(errStat2, errMsg2, errStat, errMsg,' Set_VSM_Inputs: Transfer_VSP_L_2_P' )      
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
   
   do i = 1, u%FusMotions%Nnodes - 1
      n1 = u%FusMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%FusMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      u_VSM%PtA    (:,count) = u%FusMotions%Position(:,n1) + u%FusMotions%TranslationDisp(:,n1)
      u_VSM%PtB    (:,count) = u%FusMotions%Position(:,n2) + u%FusMotions%TranslationDisp(:,n2)
      u_VSM%U_Inf_v(:,count) = ( u%V_Fus(:,i) + u%V_Fus(:,n2) ) / 2.0   -  ( u%FusMotions%TranslationVel(:,n1  ) + u%FusMotions%TranslationVel(:,n2) ) / 2.0
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
      u_VSM%x_hat  (:,count) = u%SWnMotions%Orientation(1,:,n1)
      u_VSM%y_hat  (:,count) = u%SWnMotions%Orientation(2,:,n1)
      u_VSM%z_hat  (:,count) = u%SWnMotions%Orientation(3,:,n1)
      if ( p%SWnCtrlID(i) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_SFlp(p%SWnCtrlID(i))
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
      u_VSM%x_hat  (:,count) = u%PWnMotions%Orientation(1,:,n1)
      u_VSM%y_hat  (:,count) = u%PWnMotions%Orientation(2,:,n1)
      u_VSM%z_hat  (:,count) = u%PWnMotions%Orientation(3,:,n1)
      if ( p%PWnCtrlID(i) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_PFlp(p%PWnCtrlID(i))
      else   
         u_VSM%Deltaf (  count) = 0.0_ReKi
      end if
      count = count + 1
   end do
   
   do i = 1, u%VSPMotions%NElemList
      n1 = u%VSPMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%VSPMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      u_VSM%PtA    (:,count) = u%VSPMotions%Position(:,n1) + u%VSPMotions%TranslationDisp(:,n1)
      u_VSM%PtB    (:,count) = u%VSPMotions%Position(:,n2) + u%VSPMotions%TranslationDisp(:,n2)
      u_VSM%U_Inf_v(:,count) = ( u%V_VSP(:,n1) + u%V_VSP(:,n2) ) / 2.0   -  ( u%VSPMotions%TranslationVel(:,n1  ) + u%VSPMotions%TranslationVel(:,n2) ) / 2.0
      u_VSM%x_hat  (:,count) = u%VSPMotions%Orientation(1,:,n1)
      u_VSM%y_hat  (:,count) = u%VSPMotions%Orientation(2,:,n1)
      u_VSM%z_hat  (:,count) = u%VSPMotions%Orientation(3,:,n1)
      if ( p%VSPCtrlID(i) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_Rudr(p%VSPCtrlID(i))
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
      u_VSM%x_hat  (:,count) = u%SHSMotions%Orientation(1,:,n1)
      u_VSM%y_hat  (:,count) = u%SHSMotions%Orientation(2,:,n1)
      u_VSM%z_hat  (:,count) = u%SHSMotions%Orientation(3,:,n1)
      if ( p%SHSCtrlID(i) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_SElv(p%SHSCtrlID(i))
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
      u_VSM%x_hat  (:,count) = u%PHSMotions%Orientation(1,:,n1)
      u_VSM%y_hat  (:,count) = u%PHSMotions%Orientation(2,:,n1)
      u_VSM%z_hat  (:,count) = u%PHSMotions%Orientation(3,:,n1)
      if ( p%PHSCtrlID(i) > 0 ) then
         u_VSM%Deltaf (  count) = u%Ctrl_PElv(p%PHSCtrlID(i))
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
         u_VSM%U_Inf_v(:,count) = ( u%V_SPy(:,i,j) + u%V_SPy(:,n2,j) ) / 2.0   -  ( u%SPyMotions(j)%TranslationVel(:,n1  ) + u%SPyMotions(j)%TranslationVel(:,n2) ) / 2.0
         u_VSM%x_hat  (:,count) = u%SPyMotions(j)%Orientation(1,:,n1)
         u_VSM%y_hat  (:,count) = u%SPyMotions(j)%Orientation(2,:,n1)
         u_VSM%z_hat  (:,count) = u%SPyMotions(j)%Orientation(3,:,n1)
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
         u_VSM%U_Inf_v(:,count) = ( u%V_PPy(:,i,j) + u%V_PPy(:,n2,j) ) / 2.0   -  ( u%PPyMotions(j)%TranslationVel(:,n1  ) + u%PPyMotions(j)%TranslationVel(:,n2) ) / 2.0
         u_VSM%x_hat  (:,count) = u%PPyMotions(j)%Orientation(1,:,n1)
         u_VSM%y_hat  (:,count) = u%PPyMotions(j)%Orientation(2,:,n1)
         u_VSM%z_hat  (:,count) = u%PPyMotions(j)%Orientation(3,:,n1)
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

subroutine KAD_MapOutputs(p, u, u_VSM, y, y_VSM, m, z, errStat, errMsg)
   type(KAD_InputType),           intent(in   )  :: u           !< An initial guess for the input; input mesh must be defined
   type(VSM_InputType),           intent(in   )  :: u_VSM       !< 
   type(KAD_ParameterType),       intent(in   )  :: p           !< Parameters
   type(KAD_OutputType),          intent(in   )  :: y           !< Initial system outputs (outputs are not calculated;
   type(VSM_OutputType),          intent(in   )  :: y_VSM       !< Initial system outputs (outputs are not calculated;
   type(KAD_ConstraintStateType), intent(in   )  :: z           !< Input: Constraint states at t;                                                                
   type(KAD_MiscVarType),         intent(inout)  :: m           !< MiscVars for the module
   integer(IntKi),                intent(  out)  :: errStat     !< Error status of the operation
   character(*),                  intent(  out)  :: errMsg      !< Error message if errStat /= ErrID_None
   
   real   (ReKi)               :: Vinf_v(3)
   real   (ReKi)               :: DCM(3,3)  
   real   (ReKi)               :: Vstruct_v(3)
   real   (ReKi)               :: Vrel   
   real   (ReKi)               :: Vind, Vrel_v(3), Vind_v(3), AoA, Cl, Cd, Cm, Fl, Fd, Mm, Cn, Ct, Fn, Ft
   real   (ReKi)               :: Re     
   real   (ReKi)               :: XM     
   real   (ReKi)               :: DynP  
   integer(IntKi)              :: VSMoffset
   integer(IntKi)              :: i, j
   real   (ReKi)               :: SpeedOfSound
   real   (ReKi)               :: chord
   real   (ReKi), allocatable  :: Vinfs_v(:,:)
   real   (ReKi), allocatable  :: chords(:)
   real   (ReKi), allocatable  :: elemLens(:)
   real   (ReKi), allocatable  :: forces(:,:), moments(:,:)
   integer(IntKi)              :: n, offset
   character(*), parameter     :: RoutineName = 'KAD_MapOutputs'

   errStat = ErrID_None
   errMsg  = ''
   
   SpeedOfSound = 343.0  ! m/s   ! TODO: This should be a parameter 
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
                                     p%FusChord, p%FusElemLen, p%AirDens, p%KinVisc, speedOfSound, &
                                     Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, Cl, Cd, Cm, &
                                     Fl, Fd, Mm, Cn, Ct, Fn, Ft, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The fuselage ', errStat, errMsg, RoutineName )
            return
         end if
      
      m%AllOuts( FusVAmbx(i) ) = Vinf_v(1)
      m%AllOuts( FusVAmby(i) ) = Vinf_v(2)
      m%AllOuts( FusVAmbz(i) ) = Vinf_v(3)
      m%AllOuts( FusSTVx (i) ) = Vstruct_v(1)
      m%AllOuts( FusSTVy (i) ) = Vstruct_v(2)
      m%AllOuts( FusSTVz (i) ) = Vstruct_v(3)
      m%AllOuts( FusVrel (i) ) = Vrel
      m%AllOuts( FusDynP (i) ) = DynP
      m%AllOuts( FusRe   (i) ) = Re
      m%AllOuts( FusM    (i) ) = XM
      m%AllOuts( FusVIndx(i) ) = Vind_v(1)
      m%AllOuts( FusVIndy(i) ) = Vind_v(2)
      m%AllOuts( FusVIndz(i) ) = Vind_v(3)     
      m%AllOuts( FusAlpha(i) ) = AoA*R2D
      m%AllOuts( FusCl   (i) ) = Cl
      m%AllOuts( FusCd   (i) ) = Cd
      m%AllOuts( FusCm   (i) ) = Cm
      m%AllOuts( FusCn   (i) ) = Cn
      m%AllOuts( FusCt   (i) ) = Ct
      m%AllOuts( FusFl   (i) ) = Fl
      m%AllOuts( FusFd   (i) ) = Fd
      m%AllOuts( FusMm   (i) ) = Mm
      m%AllOuts( FusFn   (i) ) = 0.0_ReKi
      m%AllOuts( FusFt   (i) ) = 0.0_ReKi
   
   end do
   VSMoffset = u%FusMotions%NElemList
   
   !=======================================
   ! Starboard wing-related outputs
   !=======================================   
   do i = 1, p%NSWnOuts
      
      call ComputeAeroOnMotionNodes(i, p%SWnOutNd, u%SWnMotions, VSMoffset, u_VSM, y_VSM, u%V_SWn, p%SWnChord, &
                                      p%SWnElemLen, p%AirDens, p%KinVisc, speedOfSound, Vinf_v, Vstruct_v, Vind_v, &
                                      Vrel, DynP, Re, XM, AoA, Cl, Cd, Cm, Fl, Fd, Mm, Cn, Ct, Fn, Ft, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The starboard wing ', errStat, errMsg, RoutineName )
            return
         end if
         
      m%AllOuts( SWnVAmbx(i) ) = Vinf_v(1)
      m%AllOuts( SWnVAmby(i) ) = Vinf_v(2)
      m%AllOuts( SWnVAmbz(i) ) = Vinf_v(3)
      m%AllOuts( SWnSTVx (i) ) = Vstruct_v(1)
      m%AllOuts( SWnSTVy (i) ) = Vstruct_v(2)
      m%AllOuts( SWnSTVz (i) ) = Vstruct_v(3)
      m%AllOuts( SWnVrel (i) ) = Vrel
      m%AllOuts( SWnDynP (i) ) = DynP
      m%AllOuts( SWnRe   (i) ) = Re
      m%AllOuts( SWnM    (i) ) = XM
      m%AllOuts( SWnVIndx(i) ) = Vind_v(1)
      m%AllOuts( SWnVIndy(i) ) = Vind_v(2)
      m%AllOuts( SWnVIndz(i) ) = Vind_v(3)       
      m%AllOuts( SWnAlpha(i) ) = AoA*R2D
      m%AllOuts( SWnCl   (i) ) = Cl
      m%AllOuts( SWnCd   (i) ) = Cd
      m%AllOuts( SWnCm   (i) ) = Cm
      m%AllOuts( SWnCn   (i) ) = Cn
      m%AllOuts( SWnCt   (i) ) = Ct
      m%AllOuts( SWnFl   (i) ) = Fl
      m%AllOuts( SWnFd   (i) ) = Fd
      m%AllOuts( SWnMm   (i) ) = Mm
      m%AllOuts( SWnFn   (i) ) = Fn
      m%AllOuts( SWnFt   (i) ) = Ft

   end do
   VSMoffset = VSMoffset + u%SWnMotions%NElemList
   
   !=======================================
   ! Port wing-related outputs
   !=======================================   
   do i = 1, p%NPWnOuts
         
      call ComputeAeroOnMotionNodes(i, p%PWnOutNd, u%PWnMotions, VSMoffset, u_VSM, y_VSM, u%V_PWn, p%PWnChord, &
                                    p%PWnElemLen, p%AirDens, p%KinVisc, speedOfSound, Vinf_v, Vstruct_v, Vind_v, &
                                    Vrel, DynP, Re, XM, AoA, Cl, Cd, Cm, Fl, Fd, Mm, Cn, Ct, Fn, Ft, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The port wing ', errStat, errMsg, RoutineName )
            return
         end if
 
      m%AllOuts( PWnVAmbx(i) ) = Vinf_v(1)
      m%AllOuts( PWnVAmby(i) ) = Vinf_v(2)
      m%AllOuts( PWnVAmbz(i) ) = Vinf_v(3)
      m%AllOuts( PWnSTVx (i) ) = Vstruct_v(1)
      m%AllOuts( PWnSTVy (i) ) = Vstruct_v(2)
      m%AllOuts( PWnSTVz (i) ) = Vstruct_v(3)
      m%AllOuts( PWnVrel (i) ) = Vrel
      m%AllOuts( PWnDynP (i) ) = DynP
      m%AllOuts( PWnRe   (i) ) = Re
      m%AllOuts( PWnM    (i) ) = XM
      m%AllOuts( PWnVIndx(i) ) = Vind_v(1)
      m%AllOuts( PWnVIndy(i) ) = Vind_v(2)
      m%AllOuts( PWnVIndz(i) ) = Vind_v(3)       
      m%AllOuts( PWnAlpha(i) ) = AoA*R2D
      m%AllOuts( PWnCl   (i) ) = Cl
      m%AllOuts( PWnCd   (i) ) = Cd
      m%AllOuts( PWnCm   (i) ) = Cm
      m%AllOuts( PWnCn   (i) ) = Cn
      m%AllOuts( PWnCt   (i) ) = Ct
      m%AllOuts( PWnFl   (i) ) = Fl 
      m%AllOuts( PWnFd   (i) ) = Fd 
      m%AllOuts( PWnMm   (i) ) = Mm 
      m%AllOuts( PWnFn   (i) ) = Fn
      m%AllOuts( PWnFt   (i) ) = Ft

   end do
   VSMoffset = VSMoffset + u%PWnMotions%NElemList
   
   !=======================================
   ! Vertical stabilizer-related outputs
   !=======================================   
   do i = 1, p%NVSOuts
         
      call ComputeAeroOnMotionNodes(i, p%VSOutNd, u%VSPMotions, VSMoffset, u_VSM, y_VSM, u%V_VSP, p%VSPChord, p%VSPElemLen, &
                                    p%AirDens, p%KinVisc, speedOfSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                    Cl, Cd, Cm, Fl, Fd, Mm, Cn, Ct, Fn, Ft, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The vertical stabilizer ', errStat, errMsg, RoutineName )
            return
         end if

      m%AllOuts( VSVAmbx(i) ) = Vinf_v(1)
      m%AllOuts( VSVAmby(i) ) = Vinf_v(2)
      m%AllOuts( VSVAmbz(i) ) = Vinf_v(3)
      m%AllOuts( VSSTVx (i) ) = Vstruct_v(1)
      m%AllOuts( VSSTVy (i) ) = Vstruct_v(2)
      m%AllOuts( VSSTVz (i) ) = Vstruct_v(3)
      m%AllOuts( VSVrel (i) ) = Vrel
      m%AllOuts( VSDynP (i) ) = DynP
      m%AllOuts( VSRe   (i) ) = Re
      m%AllOuts( VSMa   (i) ) = XM
      m%AllOuts( VSVIndx(i) ) = Vind_v(1)
      m%AllOuts( VSVIndy(i) ) = Vind_v(2)
      m%AllOuts( VSVIndz(i) ) = Vind_v(3)              
      m%AllOuts( VSAlpha(i) ) = AoA*R2D 
      m%AllOuts( VSCl   (i) ) = Cl 
      m%AllOuts( VSCd   (i) ) = Cd 
      m%AllOuts( VSCm   (i) ) = Cm 
      m%AllOuts( VSCn   (i) ) = Cn
      m%AllOuts( VSCt   (i) ) = Ct
      m%AllOuts( VSFl   (i) ) = Fl
      m%AllOuts( VSFd   (i) ) = Fd
      m%AllOuts( VSMm   (i) ) = Mm
      m%AllOuts( VSFn   (i) ) = Fn
      m%AllOuts( VSFt   (i) ) = Ft

   end do
   VSMoffset = VSMoffset + u%VSPMotions%NElemList
   
   !=======================================
   ! Starboard horizontal stabilizer-related outputs
   !=======================================   
   do i = 1, p%NSHSOuts
         
      call ComputeAeroOnMotionNodes(i, p%SHSOutNd, u%SHSMotions, VSMoffset, u_VSM, y_VSM, u%V_SHS, p%SHSChord, p%SHSElemLen, & 
                                    p%AirDens, p%KinVisc, speedOfSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                    Cl, Cd, Cm, Fl, Fd, Mm, Cn, Ct, Fn, Ft, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The starboard horizontal stabilizer ', errStat, errMsg, RoutineName )
            return
         end if

      m%AllOuts( SHSVAmbx(i) ) = Vinf_v(1)
      m%AllOuts( SHSVAmby(i) ) = Vinf_v(2)
      m%AllOuts( SHSVAmbz(i) ) = Vinf_v(3)
      m%AllOuts( SHSSTVx (i) ) = Vstruct_v(1)
      m%AllOuts( SHSSTVy (i) ) = Vstruct_v(2)
      m%AllOuts( SHSSTVz (i) ) = Vstruct_v(3)
      m%AllOuts( SHSVrel (i) ) = Vrel
      m%AllOuts( SHSDynP (i) ) = DynP
      m%AllOuts( SHSRe   (i) ) = Re
      m%AllOuts( SHSM    (i) ) = XM
      m%AllOuts( SHSVIndx(i) ) = Vind_v(1)
      m%AllOuts( SHSVIndy(i) ) = Vind_v(2)
      m%AllOuts( SHSVIndz(i) ) = Vind_v(3)                     
      m%AllOuts( SHSAlpha(i) ) = AoA*R2D  
      m%AllOuts( SHSCl   (i) ) = Cl  
      m%AllOuts( SHSCd   (i) ) = Cd  
      m%AllOuts( SHSCm   (i) ) = Cm  
      m%AllOuts( SHSCn   (i) ) = Cn
      m%AllOuts( SHSCt   (i) ) = Ct
      m%AllOuts( SHSFl   (i) ) = Fl
      m%AllOuts( SHSFd   (i) ) = Fd
      m%AllOuts( SHSMm   (i) ) = Mm
      m%AllOuts( SHSFn   (i) ) = Fn
      m%AllOuts( SHSFt   (i) ) = Ft

   end do
   VSMoffset = VSMoffset + u%SHSMotions%NElemList
   
   !=======================================
   ! Port horizontal stabilizer-related outputs
   !=======================================   
   do i = 1, p%NPHSOuts
      
      call ComputeAeroOnMotionNodes(i, p%PHSOutNd, u%PHSMotions, VSMoffset, u_VSM, y_VSM, u%V_PHS, p%PHSChord, p%PHSElemLen, &
                                    p%AirDens, p%KinVisc, speedOfSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                    Cl, Cd, Cm, Fl, Fd, Mm, Cn, Ct, Fn, Ft, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The port horizontal stabilizer ', errStat, errMsg, RoutineName )
            return
         end if

      m%AllOuts( PHSVAmbx(i) ) = Vinf_v(1)
      m%AllOuts( PHSVAmby(i) ) = Vinf_v(2)
      m%AllOuts( PHSVAmbz(i) ) = Vinf_v(3)
      m%AllOuts( PHSSTVx (i) ) = Vstruct_v(1)
      m%AllOuts( PHSSTVy (i) ) = Vstruct_v(2)
      m%AllOuts( PHSSTVz (i) ) = Vstruct_v(3)
      m%AllOuts( PHSVrel (i) ) = Vrel
      m%AllOuts( PHSDynP (i) ) = DynP
      m%AllOuts( PHSRe   (i) ) = Re
      m%AllOuts( PHSM    (i) ) = XM
      m%AllOuts( PHSVIndx(i) ) = Vind_v(1)
      m%AllOuts( PHSVIndy(i) ) = Vind_v(2)
      m%AllOuts( PHSVIndz(i) ) = Vind_v(3)                     
      m%AllOuts( PHSAlpha(i) ) = AoA*R2D  
      m%AllOuts( PHSCl   (i) ) = Cl  
      m%AllOuts( PHSCd   (i) ) = Cd  
      m%AllOuts( PHSCm   (i) ) = Cm  
      m%AllOuts( PHSCn   (i) ) = Cn
      m%AllOuts( PHSCt   (i) ) = Ct
      m%AllOuts( PHSFl   (i) ) = Fl
      m%AllOuts( PHSFd   (i) ) = Fd
      m%AllOuts( PHSMm   (i) ) = Mm
      m%AllOuts( PHSFn   (i) ) = Fn
      m%AllOuts( PHSFt   (i) ) = Ft

   end do
   VSMoffset = VSMoffset + u%PHSMotions%NElemList
   
   !=======================================
   ! Starboard, pylon-related outputs
   !=======================================   
   do j = 1, p%NumPylons
      do i = 1, p%NPylOuts
         
         Vinfs_v = u%V_SPy(:, :,j)
         chords  = p%SPyChord(:,j) 
         elemLens = p%SPyElemLen(:,j)
         call ComputeAeroOnMotionNodes(i, p%PylOutNd, u%SPyMotions(j), VSMoffset, u_VSM, y_VSM, Vinfs_v, chords, elemLens, &
                                       p%AirDens, p%KinVisc, speedOfSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                       Cl, Cd, Cm, Fl, Fd, Mm, Cn, Ct, Fn, Ft, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The starboard pylon ', errStat, errMsg, RoutineName )
            return
         end if

         m%AllOuts( SPVAmbx(i,j) ) = Vinf_v(1)
         m%AllOuts( SPVAmby(i,j) ) = Vinf_v(2)
         m%AllOuts( SPVAmbz(i,j) ) = Vinf_v(3)
         m%AllOuts( SPSTVx (i,j) ) = Vstruct_v(1)
         m%AllOuts( SPSTVy (i,j) ) = Vstruct_v(2)
         m%AllOuts( SPSTVz (i,j) ) = Vstruct_v(3)
         m%AllOuts( SPVrel (i,j) ) = Vrel
         m%AllOuts( SPDynP (i,j) ) = DynP
         m%AllOuts( SPRe   (i,j) ) = Re
         m%AllOuts( SPM    (i,j) ) = XM
         m%AllOuts( SPVIndx(i,j) ) = Vind_v(1)
         m%AllOuts( SPVIndy(i,j) ) = Vind_v(2)
         m%AllOuts( SPVIndz(i,j) ) = Vind_v(3)                    
         m%AllOuts( SPAlpha(i,j) ) = AoA*R2D 
         m%AllOuts( SPCl   (i,j) ) = Cl 
         m%AllOuts( SPCd   (i,j) ) = Cd 
         m%AllOuts( SPCm   (i,j) ) = Cm 
         m%AllOuts( SPCn   (i,j) ) = Cn
         m%AllOuts( SPCt   (i,j) ) = Ct
         m%AllOuts( SPFl   (i,j) ) = Fl
         m%AllOuts( SPFd   (i,j) ) = Fd
         m%AllOuts( SPMm   (i,j) ) = Mm
         m%AllOuts( SPFn   (i,j) ) = Fn
         m%AllOuts( SPFt   (i,j) ) = Ft

      end do
      VSMoffset = VSMoffset + u%SPyMotions(j)%NElemList
   end do
   
   !=======================================
   ! Starboard, pylon-related outputs
   !=======================================   
   do j = 1, p%NumPylons
      do i = 1, p%NPylOuts
         
         Vinfs_v = u%V_PPy(:, :,j)
         chords  = p%PPyChord(:,j) 
         elemLens = p%PPyElemLen(:,j)
         call ComputeAeroOnMotionNodes(i, p%PylOutNd, u%PPyMotions(j), VSMoffset, u_VSM, y_VSM, Vinfs_v, chords, elemLens, &
                                       p%AirDens, p%KinVisc, speedOfSound, Vinf_v, Vstruct_v, Vind_v, Vrel, DynP, Re, XM, AoA, &
                                        Cl, Cd, Cm, Fl, Fd, Mm, Cn, Ct, Fn, Ft, errStat, errMsg )
         if (errStat >= AbortErrLev) then
            call SetErrStat( ErrID_Fatal, 'The port pylon ', errStat, errMsg, RoutineName )
            return
         end if


         m%AllOuts( PPVAmbx(i,j) ) = Vinf_v(1)
         m%AllOuts( PPVAmby(i,j) ) = Vinf_v(2)
         m%AllOuts( PPVAmbz(i,j) ) = Vinf_v(3)
         m%AllOuts( PPSTVx (i,j) ) = Vstruct_v(1)
         m%AllOuts( PPSTVy (i,j) ) = Vstruct_v(2)
         m%AllOuts( PPSTVz (i,j) ) = Vstruct_v(3)
         m%AllOuts( PPVrel (i,j) ) = Vrel
         m%AllOuts( PPDynP (i,j) ) = DynP
         m%AllOuts( PPRe   (i,j) ) = Re
         m%AllOuts( PPM    (i,j) ) = XM
         m%AllOuts( PPVIndx(i,j) ) = Vind_v(1)
         m%AllOuts( PPVIndy(i,j) ) = Vind_v(2)
         m%AllOuts( PPVIndz(i,j) ) = Vind_v(3)                   
         m%AllOuts( PPAlpha(i,j) ) = AoA*R2D 
         m%AllOuts( PPCl   (i,j) ) = Cl 
         m%AllOuts( PPCd   (i,j) ) = Cd 
         m%AllOuts( PPCm   (i,j) ) = Cm 
         m%AllOuts( PPCn   (i,j) ) = Cn
         m%AllOuts( PPCt   (i,j) ) = Ct
         m%AllOuts( PPFl   (i,j) ) = Fl 
         m%AllOuts( PPFd   (i,j) ) = Fd 
         m%AllOuts( PPMm   (i,j) ) = Mm 
         m%AllOuts( PPFn   (i,j) ) = Fn
         m%AllOuts( PPFt   (i,j) ) = Ft
      end do
      VSMoffset = VSMoffset + u%PPyMotions(j)%NElemList
   end do
   
   ! Actuator Disks
   ! Starboard Pylon 1   Top and Bottom rotors
  
   call AllocAry(forces, 3, p%NumPylons*2, 'forces', errStat, errMsg) 
   call AllocAry(moments, 3, p%NumPylons*2, 'moments', errStat, errMsg) 
   
   ! Orientation is transform from global to local
   do i = 1, 2*p%NumPylons
      DCM = y%SPyRtrLoads(i)%Orientation(:,:,1)
      forces(:,i)  = matmul(DCM, y%SPyRtrLoads(i)%Force(:,1))
      moments(:,i) = matmul (DCM,y%SPyRtrLoads(i)%Moment(:,1))
   end do
   
 m%AllOuts( SP1TVRelx) = m%ActDsk(1)%u%DiskAve_Vx_Rel
 m%AllOuts( SP1BVRelx) = m%ActDsk(2)%u%DiskAve_Vx_Rel
 m%AllOuts( SP1TRtSpd) = m%ActDsk(1)%u%omega
 m%AllOuts( SP1BRtSpd) = m%ActDsk(2)%u%omega
 m%AllOuts( SP1TSkew ) = m%ActDsk(1)%u%skew*R2D
 m%AllOuts( SP1BSkew ) = m%ActDsk(2)%u%skew*R2D
 m%AllOuts( SP1TVrel ) = m%ActDsk(1)%u%DiskAve_Vinf_Rel
 m%AllOuts( SP1BVrel ) = m%ActDsk(2)%u%DiskAve_Vinf_Rel
 m%AllOuts( SP1TCp   ) = m%ActDsk(1)%m%Cp
 m%AllOuts( SP1BCp   ) = m%ActDsk(2)%m%Cp
 m%AllOuts( SP1TCq   ) = m%ActDsk(1)%m%Cq
 m%AllOuts( SP1BCq   ) = m%ActDsk(2)%m%Cq
 m%AllOuts( SP1TCt   ) = m%ActDsk(1)%m%Ct
 m%AllOuts( SP1BCt   ) = m%ActDsk(2)%m%Ct
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
 m%AllOuts( SP1TPwr  ) = m%ActDsk(1)%y%P
 m%AllOuts( SP1BPwr  ) = m%ActDsk(2)%y%P

   ! Starboard Pylon 2   Top and Bottom rotors
 m%AllOuts( SP2TVRelx) = m%ActDsk(3)%u%DiskAve_Vx_Rel
 m%AllOuts( SP2BVRelx) = m%ActDsk(4)%u%DiskAve_Vx_Rel
 m%AllOuts( SP2TRtSpd) = m%ActDsk(3)%u%omega
 m%AllOuts( SP2BRtSpd) = m%ActDsk(4)%u%omega
 m%AllOuts( SP2TSkew ) = m%ActDsk(3)%u%skew*R2D
 m%AllOuts( SP2BSkew ) = m%ActDsk(4)%u%skew*R2D
 m%AllOuts( SP2TVrel ) = m%ActDsk(3)%u%DiskAve_Vinf_Rel
 m%AllOuts( SP2BVrel ) = m%ActDsk(4)%u%DiskAve_Vinf_Rel
 m%AllOuts( SP2TCp   ) = m%ActDsk(3)%m%Cp
 m%AllOuts( SP2BCp   ) = m%ActDsk(4)%m%Cp
 m%AllOuts( SP2TCq   ) = m%ActDsk(3)%m%Cq
 m%AllOuts( SP2BCq   ) = m%ActDsk(4)%m%Cq
 m%AllOuts( SP2TCt   ) = m%ActDsk(3)%m%Ct
 m%AllOuts( SP2BCt   ) = m%ActDsk(4)%m%Ct
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
 m%AllOuts( SP2TPwr  ) = m%ActDsk(3)%y%P
 m%AllOuts( SP2BPwr  ) = m%ActDsk(4)%y%P

 offset = p%NumPylons*2
 
 ! Orientation is transform from global to local
do i = 1, 2*p%NumPylons
   DCM = y%PPyRtrLoads(i)%Orientation(:,:,1)
   forces(:,i)  = matmul(DCM, y%PPyRtrLoads(i)%Force(:,1))
   moments(:,i) = matmul (DCM,y%PPyRtrLoads(i)%Moment(:,1))
end do
   
   ! Port Pylon 1   Top and Bottom rotors
 m%AllOuts( PP1TVRelx) = m%ActDsk(1+offset)%u%DiskAve_Vx_Rel
 m%AllOuts( PP1BVRelx) = m%ActDsk(2+offset)%u%DiskAve_Vx_Rel
 m%AllOuts( PP1TRtSpd) = m%ActDsk(1+offset)%u%omega
 m%AllOuts( PP1BRtSpd) = m%ActDsk(2+offset)%u%omega
 m%AllOuts( PP1TSkew ) = m%ActDsk(1+offset)%u%skew*R2D
 m%AllOuts( PP1BSkew ) = m%ActDsk(2+offset)%u%skew*R2D
 m%AllOuts( PP1TVrel ) = m%ActDsk(1+offset)%u%DiskAve_Vinf_Rel
 m%AllOuts( PP1BVrel ) = m%ActDsk(2+offset)%u%DiskAve_Vinf_Rel
 m%AllOuts( PP1TCp   ) = m%ActDsk(1+offset)%m%Cp
 m%AllOuts( PP1BCp   ) = m%ActDsk(2+offset)%m%Cp
 m%AllOuts( PP1TCq   ) = m%ActDsk(1+offset)%m%Cq
 m%AllOuts( PP1BCq   ) = m%ActDsk(2+offset)%m%Cq
 m%AllOuts( PP1TCt   ) = m%ActDsk(1+offset)%m%Ct
 m%AllOuts( PP1BCt   ) = m%ActDsk(2+offset)%m%Ct
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
 m%AllOuts( PP1TPwr  ) = m%ActDsk(1+offset)%y%P
 m%AllOuts( PP1BPwr  ) = m%ActDsk(2+offset)%y%P

   ! Port Pylon 2   Top and Bottom rotors
 m%AllOuts( PP2TVRelx) = m%ActDsk(3+offset)%u%DiskAve_Vx_Rel
 m%AllOuts( PP2BVRelx) = m%ActDsk(4+offset)%u%DiskAve_Vx_Rel
 m%AllOuts( PP2TRtSpd) = m%ActDsk(3+offset)%u%omega
 m%AllOuts( PP2BRtSpd) = m%ActDsk(4+offset)%u%omega
 m%AllOuts( PP2TSkew ) = m%ActDsk(3+offset)%u%skew*R2D
 m%AllOuts( PP2BSkew ) = m%ActDsk(4+offset)%u%skew*R2D
 m%AllOuts( PP2TVrel ) = m%ActDsk(3+offset)%u%DiskAve_Vinf_Rel
 m%AllOuts( PP2BVrel ) = m%ActDsk(4+offset)%u%DiskAve_Vinf_Rel
 m%AllOuts( PP2TCp   ) = m%ActDsk(3+offset)%m%Cp
 m%AllOuts( PP2BCp   ) = m%ActDsk(4+offset)%m%Cp
 m%AllOuts( PP2TCq   ) = m%ActDsk(3+offset)%m%Cq
 m%AllOuts( PP2BCq   ) = m%ActDsk(4+offset)%m%Cq
 m%AllOuts( PP2TCt   ) = m%ActDsk(3+offset)%m%Ct
 m%AllOuts( PP2BCt   ) = m%ActDsk(4+offset)%m%Ct
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
 m%AllOuts( PP2TPwr  ) = m%ActDsk(3+offset)%y%P
 m%AllOuts( PP2BPwr  ) = m%ActDsk(4+offset)%y%P
 
end subroutine KAD_MapOutputs

!==============================================================================
! Framework Routines                                                          !
!==============================================================================                               
      

!----------------------------------------------------------------------------------------------------------------------------------
!< This routine is called at the start of the simulation to perform initialization steps.
!! The parameters are set here and not changed during the simulation.
!! The initial states and initial guess for the input are defined.
subroutine KAD_Init( InitInp, u, p, y, interval, m, InitOut, errStat, errMsg )

   type(KAD_InitInputType),       intent(inout)  :: InitInp     !< Input data for initialization routine, needs to be inout because there is a copy of some data in InitInp in BEMT_SetParameters()
   type(KAD_InputType),           intent(inout)  :: u           !< An initial guess for the input; input mesh must be defined
   type(KAD_ParameterType),       intent(  out)  :: p           !< Parameters
   type(KAD_OutputType),          intent(  out)  :: y           !< Initial system outputs (outputs are not calculated;
                                                                   !<   only the output mesh is initialized)
   real(DbKi),                    intent(inout)  :: interval    !< Coupling interval in seconds: 
                                                                   !<   Input is the suggested time from the glue code;
                                                                   !<   Output is the actual coupling interval that will be used
                                                                   !<   by the glue code.
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
      
   do i=1,InitInp%NumPylons*2*2
         ! Set the Air Density
      InitInp%InpFileData%RtrProps(i)%AirDens = InitInp%InpFileData%AirDens
      call ActDsk_Init( InitInp%InpFileData%RtrProps(i), m%ActDsk(i)%u, m%ActDsk(i)%p, m%ActDsk(i)%y, InitInp%InpFileData%DTAero, ActDsk_InitOut, errStat, errMsg )
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
   
   n = InitInp%InpFileData%VSPProps%NumNds
   call AllocAry( p%VSPCtrlID, n, 'p%VSPCtrlID', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( p%VSPChord, n, 'p%VSPChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do i=1,n
      p%VSPChord (i) = InitInp%InpFileData%VSPProps%Chord(i)
      p%VSPCtrlID(i) = InitInp%InpFileData%VSPProps%CntrlID(i)
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
   call AllocAry( p%SPyElemLen, n-1, p%NumPylons, 'p%SPyElemLen', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do j=1,p%NumPylons
      do i=1,n
         p%SPyChord(i,j) = InitInp%InpFileData%SPyProps(j)%Chord(i)
      end do
   end do
   
   n = 0
   do j=1,p%NumPylons
      n = max(n,InitInp%InpFileData%PPyProps(j)%NumNds)
   end do
   call AllocAry( p%PPyChord, n, p%NumPylons, 'p%PPyChord', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   call AllocAry( p%PPyElemLen, n-1, p%NumPylons, 'p%PPyElemLen', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   do j=1,p%NumPylons
      do i=1,n
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
   !----------------------------------
   ! Create and initialize the Outputs
   !----------------------------------
      
   call Init_y( y, u, InitInp, p, errStat, errMsg )
      if ( errStat >= AbortErrLev ) return
   
   allocate(m%SPyLoads(InitInp%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPyLoads', errStat, errMsg, RoutineName )     
   allocate(m%PPyLoads(InitInp%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPyLoads', errStat, errMsg, RoutineName )  
   allocate(m%SPy_L_2_P(InitInp%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%SPy_L_2_P', errStat, errMsg, RoutineName )     
   allocate(m%PPy_L_2_P(InitInp%NumPylons), STAT=errStat2)
      if (errStat2 /= 0) call SetErrStat( ErrID_Fatal, 'Could not allocate memory for m%PPy_L_2_P', errStat, errMsg, RoutineName )  
   if (errStat >= AbortErrLev) return   
   
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
   call GetRoot( InitInp%FileName, p%OutFileRoot )
   call KAD_OpenOutput( KAD_Ver, p%OutFileRoot,  p, InitOut, ErrStat, ErrMsg )
      IF (ErrStat >= AbortErrLev) RETURN
   CALL AllocAry( y%WriteOutput, p%NumOuts, 'y%WriteOutput', ErrStat, ErrMsg )   
      IF (ErrStat >= AbortErrLev) RETURN
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
   VSM_InitInp%NumVolElem = u%FusMotions%NElemList
   
   call AllocAry( VSM_InitInp%AFNames, VSM_InitInp%NumAFfiles, 'VSM_InitInp%AFNames', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
      
   VSM_InitInp%AFNames     = InitInp%InpFileData%AFNames
   
   call AllocAry( VSM_numCompElems, 6+2*p%NumPylons, 'VSM_numCompElems', errStat2, errMsg2 )
      call SetErrStat( errStat2, errMsg2, errStat, errMsg, RoutineName )
   VSM_numCompElems(1) = u%FusMotions%NElemList  
   VSM_numCompElems(2) = u%SWnMotions%NElemList
   VSM_numCompElems(3) = u%PWnMotions%NElemList
   VSM_numCompElems(4) = u%VSPMotions%NElemList
   VSM_numCompElems(5) = u%SHSMotions%NElemList
   VSM_numCompElems(6) = u%PHSMotions%NElemList
   c = 7
   numElem = u%FusMotions%NElemList + u%SWnMotions%NElemList + u%PWnMotions%NElemList + u%VSPMotions%NElemList + u%SHSMotions%NElemList + u%PHSMotions%NElemList
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
      
   ! Loop over kite components to set these values
   count = 1
   
   ! TODO: For now we are including the fuselage in the VSM, but we are handling them differently than the other elements, so 
   !       they are first in the list and we need to tell the module how many of these we have.  GJH 1/16/2018
   
   do i = 1, u%FusMotions%NElemList
      n1 = u%FusMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%FusMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)    = u%FusMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)    = u%FusMotions%Position(:,n2)
      VSM_InitInp%Chords(count) = (InitInp%InpFileData%FusProps%Chord(n1) + InitInp%InpFileData%FusProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)  = InitInp%InpFileData%FusProps%AFID(n1)
      count = count + 1
   end do  
   
   do i = 1, u%SWnMotions%NElemList
      n1 = u%SWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%SWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)    
      VSM_ElemPts(:,1,count)    = u%SWnMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)    = u%SWnMotions%Position(:,n2)
      VSM_InitInp%Chords(count) = (InitInp%InpFileData%SWnProps%Chord(n1) + InitInp%InpFileData%SWnProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)  = InitInp%InpFileData%SWnProps%AFID(n1)
      count = count + 1
   end do
   do i = 1, u%PWnMotions%NElemList
      n1 = u%PWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%PWnMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)    = u%PWnMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)    = u%PWnMotions%Position(:,n2)
      VSM_InitInp%Chords(count) = (InitInp%InpFileData%PWnProps%Chord(n1) + InitInp%InpFileData%PWnProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)  = InitInp%InpFileData%PWnProps%AFID(n1)
      count = count + 1
   end do
   do i = 1, u%VSPMotions%NElemList
      n1 = u%VSPMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%VSPMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)    = u%VSPMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)    = u%VSPMotions%Position(:,n2)
      VSM_InitInp%Chords(count) = (InitInp%InpFileData%VSPProps%Chord(n1) + InitInp%InpFileData%VSPProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)  = InitInp%InpFileData%VSPProps%AFID(n1)
      count = count + 1
   end do
   do i = 1, u%SHSMotions%NElemList
      n1 = u%SHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%SHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)    = u%SHSMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)    = u%SHSMotions%Position(:,n2)
      VSM_InitInp%Chords(count) = (InitInp%InpFileData%SHSProps%Chord(n1) + InitInp%InpFileData%SHSProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)  = InitInp%InpFileData%SHSProps%AFID(n1)
      count = count + 1
   end do
   do i = 1, u%PHSMotions%NElemList
      n1 = u%PHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
      n2 = u%PHSMotions%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
      VSM_ElemPts(:,1,count)    = u%PHSMotions%Position(:,n1)
      VSM_ElemPts(:,2,count)    = u%PHSMotions%Position(:,n2)
      VSM_InitInp%Chords(count) = (InitInp%InpFileData%PHSProps%Chord(n1) + InitInp%InpFileData%PHSProps%Chord(n2)) / 2.0
      VSM_InitInp%AFIDs(count)  = InitInp%InpFileData%PHSProps%AFID(n1)
      count = count + 1
   end do
   do j = 1, p%NumPylons
      do i = 1, u%SPyMotions(j)%NElemList
         n1 = u%SPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
         n2 = u%SPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
         VSM_ElemPts(:,1,count)    = u%SPyMotions(j)%Position(:,n1)
         VSM_ElemPts(:,2,count)    = u%SPyMotions(j)%Position(:,n2)
         VSM_InitInp%Chords(count) = (InitInp%InpFileData%SPyProps(j)%Chord(n1) + InitInp%InpFileData%SPyProps(j)%Chord(n2)) / 2.0
         VSM_InitInp%AFIDs(count)  = InitInp%InpFileData%SPyProps(j)%AFID(n1)
         count = count + 1
      end do
   end do
   do j = 1, p%NumPylons
      do i = 1, u%PPyMotions(j)%NElemList
         n1 = u%PPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(1)
         n2 = u%PPyMotions(j)%ELEMLIST(i)%ELEMENT%ELEMNODES(2)     
         VSM_ElemPts(:,1,count)    = u%PPyMotions(j)%Position(:,n1)
         VSM_ElemPts(:,2,count)    = u%PPyMotions(j)%Position(:,n2)
         VSM_InitInp%Chords(count) = (InitInp%InpFileData%PPyProps(j)%Chord(n1) + InitInp%InpFileData%PPyProps(j)%Chord(n2)) / 2.0
         VSM_InitInp%AFIDs(count)  = InitInp%InpFileData%PPyProps(j)%AFID(n1)
         count = count + 1
      end do
   end do

   call VSM_Init( VSM_InitInp, m%VSM%u, m%VSM%p, m%VSM%z, m%VSM%m, m%VSM%y, interval, VSM_InitOut, errStat, errMsg )
      if ( errStat >= AbortErrLev ) return
      
   InitOut%AirDens = p%AirDens
   InitOut%nIfWPts = nIfWPts   
   
   call KAD_WriteSummary( p%OutFileRoot, VSM_ElemPts, VSM_numCompElems, m%VSM, errStat, errMsg )
   
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
   call Set_VSM_Inputs(m%u_Interp, m, p, m%VSM%u, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return
        
   call VSM_UpdateStates( t, n, m%VSM%u, m%VSM%p, m%VSM%z, m%VSM%m, errStat, errMsg )
   
   
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
   
   
   errStat   = ErrID_None           ! no error has occurred
   errMsg    = ""
   
   

   n = Time / p%DTAero
      ! Now that we have the KAD inputs at t + dt, construct the VSM inputs
   call Set_VSM_Inputs(u, m, p, m%VSM%u, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return
   call VSM_CalcOutput( Time, n, m%VSM%u, m%VSM%p, m%VSM%z, m%VSM%m, m%VSM%y, errStat, errMsg )   
      if ( errStat >= AbortErrLev ) return
      
      ! Transfer the VSM loads to the output meshes as point loads
   c = 1
   do i = 1, y%FusLoads%nNodes
      y%FusLoads%Force(:,i)  = m%VSM%y%Loads(1:3,c)
      y%FusLoads%Moment(:,i) = m%VSM%y%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%SWnLoads%nNodes
      y%SWnLoads%Force(:,i)  = m%VSM%y%Loads(1:3,c)
      y%SWnLoads%Moment(:,i) = m%VSM%y%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%PWnLoads%nNodes
      y%PWnLoads%Force(:,i)  = m%VSM%y%Loads(1:3,c)
      y%PWnLoads%Moment(:,i) = m%VSM%y%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%VSPLoads%nNodes
      y%VSPLoads%Force(:,i)  = m%VSM%y%Loads(1:3,c)
      y%VSPLoads%Moment(:,i) = m%VSM%y%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%SHSLoads%nNodes
      y%SHSLoads%Force(:,i)  = m%VSM%y%Loads(1:3,c)
      y%SHSLoads%Moment(:,i) = m%VSM%y%Loads(4:6,c)
      c = c + 1
   end do
   do i = 1, y%PHSLoads%nNodes
      y%PHSLoads%Force(:,i)  = m%VSM%y%Loads(1:3,c)
      y%PHSLoads%Moment(:,i) = m%VSM%y%Loads(4:6,c)
      c = c + 1
   end do
   do j = 1, p%NumPylons
      do i = 1, y%SPyLoads(j)%nNodes
         y%SPyLoads(j)%Force(:,i)  = m%VSM%y%Loads(1:3,c)
         y%SPyLoads(j)%Moment(:,i) = m%VSM%y%Loads(4:6,c)
         c = c + 1
      end do
   end do
   do j = 1, p%NumPylons
      do i = 1, y%PPyLoads(j)%nNodes
         y%PPyLoads(j)%Force(:,i)  = m%VSM%y%Loads(1:3,c)
         y%PPyLoads(j)%Moment(:,i) = m%VSM%y%Loads(4:6,c)
         c = c + 1
      end do
   end do
   
   ! FusOLoads  - where do these loads come from?
   
   if ( p%RotorMod == 1 ) then
      call RotorDisk_CalcOutput(0,             p%NumPylons, u%V_SPyRtr, u%SPyRtrMotions, u%Omega_SPyRtr, u%Pitch_SPyRtr, m%ActDsk, y%SPyRtrLoads, errStat, errMsg)
         if ( errStat >= AbortErrLev ) return
      call RotorDisk_CalcOutput(p%NumPylons*2, p%NumPylons, u%V_PPyRtr, u%PPyRtrMotions, u%Omega_PPyRtr, u%Pitch_PPyRtr, m%ActDsk, y%PPyRtrLoads, errStat, errMsg)
         if ( errStat >= AbortErrLev ) return
         
          ! Rotor Loads in global
      do i = 1, p%NumPylons*2
         !forces  = (/m%ActDsk(i)%y%Fx,m%ActDsk(i)%y%Fy,m%ActDsk(i)%y%Fz/)
         !moments = (/m%ActDsk(i)%y%Mx,m%ActDsk(i)%y%My,m%ActDsk(i)%y%Mz/)
         !   ! Orientation is transform from global to local, but we need local to global so transpose
         !DCM = transpose(y%SPyRtrLoads(i)%Orientation(:,:,1))
         !forces = matmul(DCM, forces)
         !moments = matmul (DCM, moments)
         !y%SPyRtrLoads(i)%Force(:,1)  = forces
         !y%SPyRtrLoads(i)%Moment(:,1) = moments
      end do
      do i = 1, p%NumPylons*2
         j = i + p%NumPylons*2
         !forces  = (/m%ActDsk(j)%y%Fx,m%ActDsk(j)%y%Fy,m%ActDsk(j)%y%Fz/)
         !moments = (/m%ActDsk(j)%y%Mx,m%ActDsk(j)%y%My,m%ActDsk(j)%y%Mz/)
         !   ! Orientation is transform from global to local, but we need local to global so transpose
         !
         !DCM = transpose(y%PPyRtrLoads(i)%Orientation(:,:,1))
         !
         !forces = matmul(DCM, forces)
         !moments = matmul (DCM, moments)
         !y%PPyRtrLoads(i)%Force(:,1)  = forces
         !y%PPyRtrLoads(i)%Moment(:,1) = moments
      end do
   
   else
    
   end if
   
   ! Map the output quantities to the AllOuts array
   
   call KAD_MapOutputs(p, u, m%VSM%u, y, m%VSM%y, m, z, errStat, errMsg)
      if ( errStat >= AbortErrLev ) return
      
   !...............................................................................................................................
   ! Place the selected output channels into the WriteOutput(:) array with the proper sign:
   !...............................................................................................................................

   DO i = 1,p%NumOuts  ! Loop through all selected output channels

      y%WriteOutput(i) = p%OutParam(i)%SignM * m%AllOuts( p%OutParam(i)%Indx )

   ENDDO             ! I - All selected output channels

   call KAD_WrOutputLine(Time, p, y%WriteOutput, ErrStat, ErrMsg)
   
end subroutine KAD_CalcOutput

end module KiteAeroDyn
