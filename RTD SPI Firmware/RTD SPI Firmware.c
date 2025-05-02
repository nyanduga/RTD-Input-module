// STM32G431KBU6 C++ Firmware for AD7124-4BRUZ
// RTD Reader with Pt100/Pt1000 selection via SPDT switch on GPIOA pin 0

#include "main.h"
#include "stm32g4xx_hal.h"
#include <cstdint>
#include <cmath>

#define READ_RTD_SWITCH() (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)

class AD7124 {
public:
    static constexpr float V_REF = 2.5f;
    static constexpr float GAIN_PT100 = 32.0f;
    static constexpr float GAIN_PT1000 = 2.0f;
    static constexpr float R_REF_PT100 = 250.0f;
    static constexpr float R_REF_PT1000 = 2500.0f;
    static constexpr float ADC_MAX = 16777215.0f;
    static constexpr float RTD_A = 3.9083e-3f;
    static constexpr float RTD_B = -5.775e-7f;

    enum class RTDType { PT100, PT1000 };

    AD7124(SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin)
        : hspi(spi), csPort(csPort), csPin(csPin) {}

    void init();
    float readTemperature();

private:
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* csPort;
    uint16_t csPin;

    void csLow() { HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET); }
    void csHigh() { HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET); }
    float adcToResistance(uint32_t adcVal, float gain, float rRef);
    float resistanceToTemperature(float resistance);
    uint32_t readRegister(uint8_t reg, uint8_t size);
};

void AD7124::init() {
    csHigh();
    // Additional configuration if required
}

uint32_t AD7124::readRegister(uint8_t reg, uint8_t size) {
    uint8_t tx[1] = {static_cast<uint8_t>(0x40 | reg)};
    uint8_t rx[4] = {0};
    csLow();
    HAL_SPI_Transmit(hspi, tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, rx, size, HAL_MAX_DELAY);
    csHigh();
    uint32_t result = 0;
    for (int i = 0; i < size; ++i) {
        result = (result << 8) | rx[i];
    }
    return result;
}

float AD7124::adcToResistance(uint32_t adcVal, float gain, float rRef) {
    float voltage = ((float)adcVal / ADC_MAX) * V_REF;
    return (voltage * rRef) / (V_REF / gain);
}

float AD7124::resistanceToTemperature(float R) {
    return (-RTD_A + std::sqrt(RTD_A * RTD_A - 4 * RTD_B * (1 - R / 100.0f))) / (2 * RTD_B);
}

float AD7124::readTemperature() {
    uint32_t raw = readRegister(0x02, 3);

    RTDType rtdType = READ_RTD_SWITCH() ? RTDType::PT100 : RTDType::PT1000;
    float gain = (rtdType == RTDType::PT100) ? GAIN_PT100 : GAIN_PT1000;
    float rRef = (rtdType == RTDType::PT100) ? R_REF_PT100 : R_REF_PT1000;

    float resistance = adcToResistance(raw, gain, rRef);
    return resistanceToTemperature(resistance);
}

// ---- Hardware Init ----
SPI_HandleTypeDef hspi1;

void SPI1_Init() {
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // Configure GPIOA pin 0 as input for RTD switch
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    HAL_SPI_Init(&hspi1);
}

extern "C" int main() {
    HAL_Init();
    SystemClock_Config();
    SPI1_Init();

    AD7124 rtdReader(&hspi1, GPIOA, GPIO_PIN_4);
    rtdReader.init();

    while (true) {
        float temp = rtdReader.readTemperature();
        HAL_Delay(1000);
    }
}
