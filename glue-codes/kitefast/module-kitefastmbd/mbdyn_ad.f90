! MBDyn (C) is a multibody analysis code. 
! http://www.mbdyn.org
! 
! Copyright (C) 1996-2017
! 
! Pierangelo Masarati	<masarati@aero.polimi.it>
! Paolo Mantegazza	<mantegazza@aero.polimi.it>
! 
! Dipartimento di Ingegneria Aerospaziale - Politecnico di Milano
! via La Masa, 34 - 20156 Milano, Italy
! http://www.aero.polimi.it
! 
! Changing this copyright notice is forbidden.
! 
! This program is free software; you can redistribute it and/or modify
! it under the terms of the GNU General Public License as published by
! the Free Software Foundation (version 2 of the License).
! 
! 
! This program is distributed in the hope that it will be useful,
! but WITHOUT ANY WARRANTY; without even the implied warranty of
! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
! GNU General Public License for more details.
! 
! You should have received a copy of the GNU General Public License
! along with this program; if not, write to the Free Software
! Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
! 
! Interface to NREL's AeroDyn library

! This subroutine is to pass the current simulation time 
! of MBDyn to AeroDyn!
! c_time: current time

SUBROUTINE MBDyn_sim_time(c_time)
  IMPLICIT NONE

  REAL c_time, TIME
  TIME = c_time

  RETURN
END SUBROUTINE MBDyn_sim_time

! This subroutine is to pass the current simulation time step 
! of MBDyn to AeroDyn!
! dt: time step

SUBROUTINE MBDyn_time_step(time_step)
  IMPLICIT NONE

  REAL time_step, DT
  DT = time_step

  RETURN
END SUBROUTINE MBDyn_time_step

subroutine test_value_transfer(value)
  implicit none
  real value
  print *, "test value transfer"
  print *, value
end subroutine

subroutine test_array_transfer(array, size)
  implicit none
  real*8 array
  integer*4 size
  dimension array(size)
  print *, "test array transfer"
  print *, array
end

subroutine test_testme(in, out)
  use newtonraphson
  implicit none
  integer, intent(in) :: in
  integer, intent(out) :: out
  call testme(in, out)
  print *, out
end subroutine
