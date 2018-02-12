

namespace invensense
{
    
    
    enum class commonSensorRegister
    {

        PWR_MGMT_1 = 0x6B,
        PWR_MGMT_2 =0x6C
        WHO_AM_I = 0x75,
    
    };


    
    enum class PWR_MGMT_1_REG
    {
        NOTHING = 0,
        CLKSEL = (3),
        PD_PTAT = (1<<3).
        GYRO_STANDBY = (1<<4),
        CYCLE = 1 <<5), 
        SLEEP = (1<<6),
        H_RESET = (1<<7)
    };
    
    enum class PWR_MGMT_2_REG
    {
        NOTHING = 0,

    };    
    
    enum class mpu9250Register 
    {
        SELF_TEST_X_GYRO    = 0,
        SELF_TEST_Y_GYRO    = 1
        SELF_TEST_Z_GYRO    = 2,
        
    };
    
}