#ifndef CAN_BUS_H_
#define CAN_BUS_H_

#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/twai.h>

class CanBus
{
public:
    /// @brief Constructs a new synchroneous UART interface
    /// @param 
    CanBus(gpio_num_t twaiRX, gpio_num_t twaiTX);
    virtual ~CanBus();

    // bool Init();

    esp_err_t TransmitMessage(const twai_message_t *message);

    esp_err_t ReceiveMessage(twai_message_t *message, unsigned int timeoutMs = 100);

    static const char * ErrorToMessage(esp_err_t error);

    uint32_t ReadAlerts();

    void GetStatusInfo(twai_status_info_t* statusInfo);
    twai_status_info_t statusInfo;


    friend class CanDevice;

protected:
};

class CanDevice
{
public:
    CanDevice(CanBus &rCanBus, const char *deviceName) : mrCanBus(rCanBus), mcstrDeviceName(deviceName){};
    virtual ~CanDevice()
    {
        // Destructor implementation goes here
    }

    virtual esp_err_t Init() = 0; // pure virtual function

protected:
    CanBus &mrCanBus;
    const char *mcstrDeviceName;
   
};


#endif
