
#include "mbconfig.h" /* This goes first in every *.c,*.cc file */
#include "dataman.h"

struct KiteFASTNode
{
    StructNode *pNode;
    doublereal *GetCurrentPosition();
    doublereal *GetCurrentVelocity();
    doublereal *GetCurrentAngularVelocity();
    doublereal *GetCurrentAcceleration();
    doublereal *GetCurrentAngularAcceleration();
    doublereal *GetCurrentDCM();
};

doublereal *KiteFASTNode::GetCurrentPosition()
{
    Vec3 xcurr = this->pNode->GetXCurr();
    static doublereal position[3];
    position[0] = xcurr[0];
    position[1] = xcurr[1];
    position[2] = xcurr[2];
    return position;
};

doublereal *KiteFASTNode::GetCurrentVelocity()
{
    Vec3 vcurr = this->pNode->GetVCurr();
    static doublereal velocity[3];
    velocity[0] = vcurr[0];
    velocity[1] = vcurr[1];
    velocity[2] = vcurr[2];
    return velocity;
};

doublereal *KiteFASTNode::GetCurrentAngularVelocity()
{
    Vec3 wcurr = this->pNode->GetWCurr();
    static doublereal angular_velocity[3];
    angular_velocity[0] = wcurr[0];
    angular_velocity[1] = wcurr[1];
    angular_velocity[2] = wcurr[2];
    return angular_velocity;
};

doublereal *KiteFASTNode::GetCurrentAcceleration()
{
    Vec3 acccurr = this->pNode->GetXPPCurr();
    static doublereal acceleration[3];
    acceleration[0] = acccurr[0];
    acceleration[1] = acccurr[1];
    acceleration[2] = acccurr[2];
    return acceleration;
};

doublereal *KiteFASTNode::GetCurrentAngularAcceleration()
{
    Vec3 angacccurr = this->pNode->GetWPCurr();
    static doublereal angular_acceleration[3];
    angular_acceleration[0] = angacccurr[0];
    angular_acceleration[1] = angacccurr[1];
    angular_acceleration[2] = angacccurr[2];
    return angular_acceleration;
};

doublereal *KiteFASTNode::GetCurrentDCM()
{
    static doublereal dcm[9];
    Mat3x3 rcurr = this->pNode->GetRCurr();
    // these are dGet(row, col)
    dcm[0] = rcurr.dGet(1, 1);
    dcm[1] = rcurr.dGet(1, 2);
    dcm[2] = rcurr.dGet(1, 3);
    dcm[3] = rcurr.dGet(2, 1);
    dcm[4] = rcurr.dGet(2, 2);
    dcm[5] = rcurr.dGet(2, 3);
    dcm[6] = rcurr.dGet(3, 1);
    dcm[7] = rcurr.dGet(3, 2);
    dcm[8] = rcurr.dGet(3, 3);
    return dcm;
};

