#ifdef __cplusplus
#define EXTERNAL_ROUTINE extern "C"
#else
#define EXTERNAL_ROUTINE extern
#endif

EXTERNAL_ROUTINE void KFAST_OS_Init(
   int*       simMod,           // Simulation modle. 1=onshore, 2=full offshore, 3=offshore, but no kite
   double*    dt,               // Timestep size (s)
   double*    TMAX,             // Length of simulation (s)
   int*       numFlaps,         // Number of flaps
   int*       numPylons,        // Number of pylons, per wing
   int*       numComp,          // Number of kite components
   int        numCompNds[],     // MBDyn nodes per component.  The array is populated in the following order: Fuselage, Starboard Wing, Port Wing, Vertical Stabilizer, 
   //    Starboard Horizontal Stabilizer, Port Horizontal Stabilizer, Starboard pylon, from inner to outer, and then Port pylon, from inner to outer.
   int        modFlags[],       // Four element array of module flags.  0 = inactive, 1 = active. Module indices are: 0 = KiteAeroDyn, 1 = InflowWind, 2 = MoorDyn, 3 = KiteFAST Controller
   const char KAD_FileName[],   // Full path and name of the KiteAeroDyn input file.
   const char IfW_FileName[],   // Full path and name of the InflowWind input file.
   const char MD_Tether_FileName[],    // Full path and name of the MoorDyn tether input file.
   const char KFC_FileName[],   // Full path and name of the KiteFAST controller shared object file.
   const char HD_FileName[],    // Full path and name of the HydroDyn input file.
   const char MD_Mooring_FileName[],    // Full path and name of the MoorDyn mooring input file.
   const char outFileRoot[],    // Full path and basename of the KiteFAST output file.
   int*       printSum,         // Print the Summary file?  1 = Yes, 0 = No.
   double*    gravity,          // Scalar gravity constant.  (m/s^2)
   int*       KAD_InterpOrder,  // KiteAeroDyn Interpolation order.  0=hold, 1=linear, 2=2nd order.
   double     FusODCM[],        // Initial DCM matrix to transform the location of the Kite Fuselage reference point from global to kite coordinates.
   int*       numRtrPts,        // Total number of rotor points (both wings).
   double     rtrPts[],         // Initial location of each rotor's reference point [RRP] in global coordinates. (m)
   double     rtrMass[],        // Mass of the rotor/drivetrain (kg)
   double     rtrI_Rot[],       // Rotational inertia about the shaft axis of the top and bottom rotors/drivetrains on the pylons on the wing meshes (kg�m2)
   double     rtrI_Trans[],     // Transverse inertia about the rotor reference point of the top and bottom rotors/drivetrains on the pylons on the wing meshes (kg�m2)
   double     rtrXcm[],         // Distance along the shaft from the rotor reference point of the top and bottom rotors/drivetrains on the pylons on the wing meshes to the center of mass of the rotor/drivetrain (positive along positive x) (m)
   double     refPts[],         // First entry is the Kite Fus0 location in global coordinates.  The remaining entries are the location of the MBDyn component reference points in the kite coordinates. (m)  In kite coordinates, FusO = (0,0,0).  The length of this array comes from  numComp * 3.
   int*       numNodePts,       // The total number of MBDyn structural nodes.  We need this total number (which could be derived from the numCompNds array) to size the following arrays in the Fortran code. 
   double     nodePts[],        // Initial location of the MBDyn structural nodes in the global coordinates. (m)  The array is populated in the same order at the numCompNds array.
   double     nodeDCMs[],       // Initial DCMs matrices to transform each nodal point from global to kite coordinates.
   int*       nFusOuts,         // Number of user-requested output locations on the fuselage  ( 0-9 )
   int        FusOutNd[],       // Node number(s) (within the component) of the requested output locations.  Structural node index for motions and KiteAeroDyn quantities,  and Gauss point index for MBDyn structural loads
   int*       nSWnOuts,         // Number of user-requested output locations on the starboard wing  ( 0-9 )
   int        SWnOutNd[],       // Node number(s) (within the component) of the requested output locations.
   int*       nPWnOuts,         // Number of user-requested output locations on the port wing  ( 0-9 )
   int        PWnOutNd[],       // Node number(s) (within the component) of the requested output locations.
   int*       nVSOuts,          // Number of user-requested output locations on the vertical stabilizer  ( 0-9 )
   int        VSOutNd[],        // Node number(s) (within the component) of the requested output locations.
   int*       nSHSOuts,         // Number of user-requested output locations on the starboard horizontal stabilizer  ( 0-9 )
   int        SHSOutNd[],       // Node number(s) (within the component) of the requested output locations.
   int*       nPHSOuts,         // Number of user-requested output locations on the port horizontal stabilizer  ( 0-9 )
   int        PHSOutNd[],       // Node number(s) (within the component) of the requested output locations.
   int*       nPylOuts,         // Number of user-requested output locations on each pylon  ( 0-9 )
   int        PylOutNd[],       // Node number(s) (within the component) of the requested output locations.
   double     PtfmO[],          // Initial location of the Platform reference point in global coordinates.
   double     PtfmODCM[],       // Initial DCM matrix to transform the location of the Platform reference point from global to platform coordinates.
   double     GSRefPtR[],       // Initial location of the Ground Station(GS) reference point in global coordinates.
   int*       numOutChan,       // Number of user-requested output channel names
   char*      chanList[],       // Array of output channel names (strings)
   int        chanList_len[],   // Array containing the lengths of each string element in the chanList array
   int*       errStat,          // Error code coming from KiteFAST
   char       errMsg[]          // Error message
   );

EXTERNAL_ROUTINE void KFAST_OS_AssRes(
   double *t,                   // simulation time for the current timestep (s)
   int*   isInitialTime,        // Is this the initial time of the simulation (1st timestep) 1=Yes, should we update the states? 0=yes, 1=no
   double WindPt[],             // Position of the ground station where the fixed wind measurement is taken, expressed in global coordinates. (m)
   double WindPtVel[],          // Velocity of the ground station where the fixed wind measurement is taken, expressed in global coordinates. (m)
   double FusO[],               // Current  timestep position of the Fuselage reference point, expressed in global coordinates. (m) 
   double FusODCM[],            // Current  timestep DCM matrix to transform the location of the Fuselage reference point from global to kite coordinates.
   double FusOv[],              // Current timestep velocity of the Fuselage reference point, expressed in global coordinates. (m/s) 
   double FusOomegas[],         // Current timestep rotational velocity of the Fuselage reference point, expressed in global coordinates. (rad/s) 
   double FusOacc[],            // Current timestep translational acceleration of the Fuselage reference point, expressed in global coordinates. (m/s^2) 
   double FusOalphas[],         // Current timestep rotational acceleration of the Fuselage reference point, expressed in global coordinates. (rad/s^2) 
   int*   numNodePts,           // Total umber of MBDyn structural nodes. This must match what was sent during KFAST_Init, but is useful here for sizing Fortran arrays.
   double nodePts[],            // Location of the MBDyn structural nodes for the current timestep, expressed in the global coordinates. (m)  
   //    The array is populated in the following order: Fuselage, Starboard Wing, Port Wing, Vertical Stabilizer, 
   //      Starboard Horizontal Stabilizer, Port Horizontal Stabilizer, Starboard pylon, from inner to outer, and then
   //      Port pylon, from inner to outer.
   double nodeDCMs[],           // DCMs matrices to transform each nodal point from global to kite coordinates.
   double nodeVels[],           // Translational velocities of each nodal point in global coordinates. (m/s)
   double nodeOmegas[],         // Rotational velocities of each nodal point in global coordinates. (rad/s)
   double nodeAccs[],           // Translational accelerations of each nodal point in global coordinates. (m/s^2)
   int*   numRtrPts,            // Total number of rotor points.  This must match what was sent during KFAST_Init, but is used here for straigh-forward declaration of array sizes on the Fortran side.
   double rtrPts[],             // Location of each rotor's reference point [RRP] in global coordinates. (m)  The order of these points follows this sequence:
   //   Start on starboard side moving from the inner pylon outward to the most outboard pylon.  Within a plyon, start with the top rotor and then the bottom rotor
   //     then repeat this sequence for the port side.
   double rtrDCMs[],            // DCMs matrices to transform each RRP point from global to kite coordinates.
   double rtrVels[],            // Translational velocity of the nacelle (RRP) in global coordinates. (m/s)
   double rtrOmegas[],          // Rotational velocity of the nacelle (RRP) in global coordinates. (rad/s)
   double rtrAccs[],            // Translational accelerations of the nacelle (RRP) in global coordinates. (m/s^2)
   double rtrAlphas[],          // Rotational accelerations of the nacelle (RRP) in global coordinates. (rad/s^2)
   double PtfmO[],              // Current timestep position of the Platform reference point, expressed in global coordinates. (m)
   double PtfmODCM[],           // Current timestep DCM matrix to transform the location of the Platform reference point from global to kite coordinates.
   double PtfmOv[],             // Current timestep velocity of the Platform reference point, expressed in global coordinates. (m / s)
   double PtfmOomegas[],        // Current timestep rotational velocity of the Platform reference point, expressed in global coordinates. (rad / s)
   double PtfmOacc[],           // Current timestep translational acceleration of the Platform reference point, expressed in global coordinates. (m / s ^ 2)
   double PtfmOalphas[],        // Current timestep rotational acceleration of the Platform reference point, expressed in global coordinates. (rad / s ^ 2)
   double PtfmIMUPt[],          // Current timestep position of the Platform IMU point, expressed in global coordinates. (m)
   double PtfmIMUDCM[],         // Current timestep DCM matrix to transform the location of the Platform IMU point from global to kite coordinates.
   double PtfmIMUv[],           // Current timestep velocity of the Platform IMU point, expressed in global coordinates. (m / s)
   double PtfmIMUomegas[],      // Current timestep rotational velocity of the Platform IMU point, expressed in global coordinates. (rad / s)
   double PtfmIMUacc[],         // Current timestep translational acceleration of the Platform IMU point, expressed in global coordinates. (m / s ^ 2)
   double GSRefPt[],            // Current timestep position of the ground station reference point, expressed in global coordinates. (m)
   double GSRefDCM[],           // Current timestep DCM matrix to transform the location of the ground station reference point from global to kite coordinates.
   double GSRefv[],             // Current timestep velocity of the ground station reference point, expressed in global coordinates. (m / s)
   double GSRefomegas[],        // Current timestep rotational velocity of the ground station reference point, expressed in global coordinates. (rad / s)
   double GSRefacc[],           // Current timestep translational acceleration of the ground station reference point, expressed in global coordinates. (m / s ^ 2)
   double nodeLoads[],          // KiteFAST loads (3 forces + 3 moments) in global coordinates ( N, N-m ) at the MBDyn structural nodes.  Sequence follows the pattern used for MBDyn structural node array.  Returned from KiteFAST to MBDyn.
   double rtrLoads[],           // Concentrated reaction loads at the nacelles on the pylons at the RRPs in global coordinates.  Length is 6 loads per rotor * number of RRPs. Returned from KiteFAST to MBDyn.
   double ptfmLoads[],        // Concentrated loads at the plaform reference pont in global coordinates.Returned from KiteFAST to MBDyn.
   int*   errStat,              // Error code coming from KiteFAST
   char   errMsg[]              // Error message
   );

EXTERNAL_ROUTINE void KFAST_OS_AfterPredict(double *t, int *errStat, char errMsg[]);
EXTERNAL_ROUTINE void KFAST_OS_Output(
   double *t,                   // simulation time for the current timestep (s)
   int *numGaussLoadPts,        // Total number of gauss points in the MBDyn model
   double gaussPtLoads[],       // Array of loads in the global coordinate system (3 forces + 3 moments) corresponding to each MBDyn gauss point. ( N, N-m )
   int *errStat,
   char errMsg[]
   );
EXTERNAL_ROUTINE void KFAST_OS_End(int *errStat, char errMsg[]);