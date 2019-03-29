// more fun with c++ classes and templates
class GPIOPort :
  public gpio_reg_map {
public:
  void high(const uint32_t pin) {
    BSRR = 1 << pin;
  }
  void low(const uint32_t pin) {
    BRR = 1 << pin;
  }
  void pinMode(const uint32_t pin, gpio_pin_mode mode) {
      volatile uint32_t *cr = &CRL + (pin >> 3);
      uint32_t shift = (pin & 0x7) * 4;
      uint32_t tmp = *cr;
      tmp &= ~(0xF << shift);
      tmp |= (mode == GPIO_INPUT_PU ? GPIO_INPUT_PD : mode) << shift;
      *cr = tmp;

      if (mode == GPIO_INPUT_PD) {
          ODR &= ~(1u << pin);
      } else if (mode == GPIO_INPUT_PU) {
          ODR |= (1u << pin);
      }
  }
  int value(const uint32_t pin) {
      return (IDR & (1u<< pin) ? 1 : 0);
  }

};

#define GPIOPORT_REF(a) *((GPIOPort * const)(a))

static GPIOPort & gPortA = GPIOPORT_REF(0x40010800);
static GPIOPort & gPortB = GPIOPORT_REF(0x40010C00);
static GPIOPort & gPortC = GPIOPORT_REF(0x40011000);

template<const uint32_t PIN>
class GPIOPortCPin {
public:
    void high() { gPortC.high(PIN); }
    void low() { gPortC.low(PIN); }
    void operator=(const int value) {
        if ( value )
            gPortC.high(PIN);
        else
            gPortC.low(PIN);
    }

    operator int() {
        return gPortC.value(PIN);
    }

    void pinMode(WiringPinMode mode) {
        gpio_pin_mode gpio_mode;
        bool pwm = false;

        switch(mode) {
        case OUTPUT:
            gpio_mode = GPIO_OUTPUT_PP;
            break;
        case OUTPUT_OPEN_DRAIN:
            gpio_mode = GPIO_OUTPUT_OD;
            break;
        case INPUT:
        case INPUT_FLOATING:
            gpio_mode = GPIO_INPUT_FLOATING;
            break;
        case INPUT_ANALOG:
            gpio_mode = GPIO_INPUT_ANALOG;
            break;
        case INPUT_PULLUP:
            gpio_mode = GPIO_INPUT_PU;
            break;
        case INPUT_PULLDOWN:
            gpio_mode = GPIO_INPUT_PD;
            break;
        case PWM:
            gpio_mode = GPIO_AF_OUTPUT_PP;
            pwm = true;
            break;
        case PWM_OPEN_DRAIN:
            gpio_mode = GPIO_AF_OUTPUT_OD;
            pwm = true;
            break;
        default:
            return;
        }

        gPortC.pinMode(PIN, gpio_mode);
        (void)pwm; // TODO: implement timer start/stop
    }
};

typedef GPIOPortCPin<1> PB_1;
typedef GPIOPortCPin<2> PB_2;
// ... and on and on
typedef GPIOPortCPin<11> PB_15;
typedef GPIOPortCPin<13> PC_13;

#define fastWrite(pin,value) do { (value) ? pin.high() : pin.low(); } while(0)

//

// void setup() {
//     LED1.pinMode(OUTPUT);
// }

// void loop() {
//     LED1 = 1;
//     delay(50);
//     LED1 = 0;
//     delay(250);

//     LED1.high();
//     delay(50);
//     LED1.low();
//     delay(450);
    
//     fastWrite(LED1,1);
//     delay(50);
//     fastWrite(LED1,0);
//     delay(450);
// }