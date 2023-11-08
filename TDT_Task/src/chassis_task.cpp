#include "chassis_task.h"
Chassis::Chassis(/* args */)
{
}
void Chassis::chassisInit()
{

    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        chssisMotor[i] = new Motor(M3508, CAN1, 0x201 + i);
        legMotor[i] = new UT_Motor(i,i,1);
    }
    
}
void Chassis::chassisCtrlTorque(float torque[2])
{
    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        chssisMotor[i]->ctrlCurrent(torque[i]);
    }
}
void Chassis::legCtrlTorque(float torque[2])
{
    for (u8 i = 0; i < 2; i++)
    {
        /* code */
        legMotor[i]->motorCtrl(torque[i],0,0,0,0);
    }
}