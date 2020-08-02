/*
 * A dummy sensor for Mbed LoRa Test Application
 * add parking test related info to this file
 * 
 */
 

#ifndef MBED_LORAWAN_DUMMYSENSOR_H_
#define MBED_LORAWAN_DUMMYSENSOR_H_


class DS1820 {
public:
    DS1820(uint32_t)
    {
        value = 1;
    };
    bool begin()
    {
        return true;
    };
    void startConversion() {};
    int32_t read()
    {
        value += 2;
        return value;
    }

private:
    int32_t value;
};



#endif /* MBED_LORAWAN_DUMMYSENSOR_H_ */
