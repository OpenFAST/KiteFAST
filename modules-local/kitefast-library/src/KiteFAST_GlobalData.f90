module KiteFAST_GlobalData

   use NWTC_Library 
   use KiteFAST_Types
   
   implicit none 

public

   type(ProgDesc), parameter        :: KFAST_Ver = ProgDesc( 'KiteFASTMBD', '', '' )
   integer,        parameter        :: IntfStrLen  = 1025       ! length of strings through the C interface
   type(KFAST_ParameterType), save  :: p
   type(KFAST_MiscVarType), save    :: m
   type(KFAST_OtherStateType), save :: OtherSt
   !type(KFAST_InputType)      :: u
   !type(KFAST_OutputType)     :: y
   
 end module KiteFAST_GlobalData  