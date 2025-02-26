#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "inc/ssd1306.h"
#include "inc/ledMatrix.h"
#include "hardware/pio.h"


#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define address 0x3C
#define VRX 27
#define VRY 26
#define buttonJ 22
#define buttonA 5
#define DEADZONE 100
#define TEMP_MIN -50.0
#define TEMP_MAX 100.0
#define SAFE_RANGE_1 10
#define SAFE_RANGE_2 30
#define ADC_MAX 4095.0
#define YELLOW_FACTOR 3
#define RED_FACTOR 2
#define RED_BRIGHT 0
#define GREEN_BRIGHT 50
#define BLUE_BRIGHT 0

uint led_rgb[3] = {13,11,12};
uint x_center = 2047;   //centro padrão do joystick
uint y_center = 2047;   //centro padrão do joystick
int temp_ideal_min = 4;
int temp_ideal_max = 10;
uint wrap = 10000;
uint stepYellow = 0;
uint stepRed = 0;
bool yellowFlag = false;
bool redFlag = false;
char temp_str[10];
uint16_t vry_value = 0;
uint64_t volatile last_time = 0;    //variavel que indica o tempo da ultima demição
uint64_t volatile current_time = 0; //variavel que indica o tempo da atual medição
bool timer_flag = false;   //flag que habilita a chamada da função de contagem
bool button_flag = false;
bool color = true;  //variavel que indica que se o pixel está ligado ou desligado
ssd1306_t ssd; //inicializa a estrutura do display


uint init_pwm(uint gpio, uint wrap) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, wrap);
    
    pwm_set_enabled(slice_num, true);  
    return slice_num;  
}

void init_rgb(uint *led)
{
    gpio_init(led[0]);
    gpio_init(led[1]);
    gpio_init(led[2]);

    gpio_set_dir(led[0], GPIO_OUT);
    gpio_set_dir(led[1], GPIO_OUT);
    gpio_set_dir(led[2], GPIO_OUT);

    gpio_put(led[0], false);
    gpio_put(led[1], false);
    gpio_put(led[2], false);
}

void init_buttons()
{
    gpio_init(buttonJ);
    gpio_init(buttonA);

    gpio_set_dir(buttonJ, GPIO_IN);
    gpio_set_dir(buttonA, GPIO_IN);

    gpio_pull_up(buttonJ);
    gpio_pull_up(buttonA);
}

void init_display()
{
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);  //seta o pino gpio como i2c
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);  //seta o pino gpio como i2c
    gpio_pull_up(I2C_SDA);  //ativa o resistor de pull up para gantir o nível lógico alto
    gpio_pull_up(I2C_SCL);  //ativa o resistor de pull up para gantir o nível lógico alto


    ssd1306_init(&ssd, WIDTH, HEIGHT, false, address, I2C_PORT); //inicializa o display
    ssd1306_config(&ssd); //configura o display
    ssd1306_send_data(&ssd); //envia os dados para o display
  
    //limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    current_time = to_ms_since_boot(get_absolute_time());   //pega o tempo atual

    if(current_time - last_time > 200) button_flag = true;

}

void clear_screen() {
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif
}

float get_temperature() 
{
    adc_select_input(0);  // Seleciona a entrada do ADC para o eixo Y do joystick
    vry_value = adc_read();  // Lê o valor do ADC

    // Verifica se o joystick está dentro da zona neutra
    if (vry_value > (y_center - DEADZONE) && vry_value < (y_center + DEADZONE)) {
        return (temp_ideal_min + temp_ideal_max) / 2.0;  // Retorna a média entre os valores ideais
    }

    float delta;
    
    if (vry_value < y_center) { // Movimento para baixo
        delta = (float)(y_center - vry_value) / (float)(y_center - DEADZONE);
        delta = -delta; // Inverte para garantir que delta seja negativo para movimento para baixo
    } else { // Movimento para cima
        delta = (float)(vry_value - (y_center + DEADZONE)) / (float)((ADC_MAX) - (y_center + DEADZONE));
    }

    // Limitamos delta ao intervalo esperado [-1, 1]
    if (delta < -1.0) delta = -1.0;
    if (delta > 1.0) delta = 1.0;


    // Determina a amplitude com base na direção do movimento
    float amplitude = (delta < 0) ? (temp_ideal_min - TEMP_MIN) : (TEMP_MAX - temp_ideal_max);

    // Corrige o cálculo da temperatura para respeitar os valores mínimos e máximos corretamente
    float temperature = ((delta < 0) ? temp_ideal_min  : temp_ideal_max ) + (delta * amplitude);

    // Formata a temperatura como string para exibição
    sprintf(temp_str, "%.2f", temperature);

    return temperature;
}

void yellow_led_blink()
{

    if(stepYellow > wrap) yellowFlag = true;
    if(stepYellow <= wrap/YELLOW_FACTOR) yellowFlag = false;

    if(!yellowFlag)
    {
        stepYellow += wrap / YELLOW_FACTOR; // Incrementa de forma controlada
    }else{
        stepYellow -= wrap / YELLOW_FACTOR; // Decrementa de forma controlada
    }

    pwm_set_gpio_level(led_rgb[0], stepYellow);//atualiza o ciclo de trabalho do eixo x
    pwm_set_gpio_level(led_rgb[1], stepYellow);//atualiza o ciclo de trabalho do eixo x

}

void red_led_blink()
{

    if(stepRed > wrap) redFlag = true;
    if(stepRed <= wrap/RED_FACTOR) redFlag = false;

    if(!redFlag)
    {
        stepRed += wrap / RED_FACTOR; // Incrementa de forma controlada
    }else{
        stepRed -= wrap / RED_FACTOR; // Decrementa de forma controlada
    }
    
    pwm_set_gpio_level(led_rgb[0], stepRed);//atualiza o ciclo de trabalho do vermelho de acordo com o passo
    pwm_set_gpio_level(led_rgb[1], 0);//garante que o verde estará apagado

}

void draw_temperature()
{
    int x = 35;

    if(temp_str[0] != '-')
    {     
        x += 8;
    }   
    for(int i = 0; i < sizeof(temp_str); i++, x +=8)
    {   
        if(temp_str[i] == '\0') break;
        ssd1306_draw_char(&ssd, temp_str[i], x ,30);        
    }
}

void countdown()
{
    print_frame(frame9,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    print_frame(frame8,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    print_frame(frame7,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    print_frame(frame6,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    print_frame(frame5,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    print_frame(frame4,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    print_frame(frame3,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    print_frame(frame2,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    print_frame(frame1,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    print_frame(frame0,RED_BRIGHT,GREEN_BRIGHT,BLUE_BRIGHT);
    sleep_ms(500);
    npClear();
}

bool repeating_timer_callback(struct repeating_timer *t)
{
    printf("\nInterrupção de timer");
    timer_flag = true;
}

void temp_config()
{
    int temp_min, temp_max;

    clear_screen();  //limpa a tela

    printf("\nInforme a temperatura MINIMA ideal: ");
    scanf("%d", temp_min);

    while(temp_min < TEMP_MIN || temp_min > TEMP_MAX-1)
    {
        printf("\n\n\nA temperatura informada eh invalida. ");
        printf("\nInforme uma temperatura entre %d e %d: ", TEMP_MIN, TEMP_MAX-1);
        scanf("%d", temp_min);
    }

    printf("\n\nInforme a temperatura MAXIMA ideal: ");
    scanf("%d", temp_max);

    while(temp_max < temp_min || temp_max > TEMP_MAX)
    {
        printf("\n\n\nA temperatura informada eh invalida. ");
        printf("\nInforme uma temperatura entre %d e %d: ", temp_min, TEMP_MAX);
        scanf("%d", temp_max);
    }

    printf("\n\nIntervalo ideal ajustado para %d até %d", temp_min, temp_max);

    temp_ideal_min = temp_min;
    temp_ideal_max = temp_max;
}

int main()
{
    stdio_init_all();

    init_rgb(led_rgb);//inicializa o led rgb

    init_buttons();//inicializa os botões

    init_display();//inicializa o display

    npInit(LED_PIN);        //inicializa matriz de led
    npClear();              //limpa a matriz
    
    adc_init();//inicializa o adc
    adc_gpio_init(VRX);//inicializa o pino X do adc
    adc_gpio_init(VRY);//inicializa o pino Y do adc

    stepYellow = wrap/YELLOW_FACTOR;
    stepRed = wrap/RED_FACTOR;

    //bloco com leitura inicial do adc para definir onde é o centro do joystick
    sleep_ms(100); 

    adc_select_input(0);
    vry_value = adc_read();
    y_center = vry_value;
    printf("\nFirst Values vry, y_center: %d, %d", vry_value, y_center);

    sleep_ms(100);
    //fim da leitura inicial

    //configura o pwm com base na leitura do centro
    uint pwm_red = init_pwm(led_rgb[0],wrap);
    uint pwm_green = init_pwm(led_rgb[1],wrap);

    struct repeating_timer timer;
    
    add_repeating_timer_ms(30000, repeating_timer_callback, NULL, &timer);

    gpio_set_irq_enabled_with_callback(buttonA, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    //limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    while (true) {

        float temperature = get_temperature(); //recebe o valor da temperatura com base na leitura de Y
        printf("\n Y value: %d", vry_value);
        printf("\n temperature value: %.2f", temperature);


        //precisa ajustar a impressão
        if(temperature < temp_ideal_min-SAFE_RANGE_2 || temperature > temp_ideal_max+SAFE_RANGE_2)
        {
            red_led_blink();
        }else if(temperature < temp_ideal_min-SAFE_RANGE_1 || temperature > temp_ideal_max+SAFE_RANGE_1)
        {
            yellow_led_blink();      
        }else{
            pwm_set_gpio_level(led_rgb[0], 0);
            pwm_set_gpio_level(led_rgb[1], 0);
        }

        ssd1306_fill(&ssd, !color); //limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 58, color, !color); //desenha um retângulo nas bordas do display
        ssd1306_draw_string(&ssd, "Temperatura: ", 20, 15);
        draw_temperature();
        ssd1306_send_data(&ssd);    //atualiza o display

        current_time = to_ms_since_boot(get_absolute_time());   //pega o tempo atual

        if(!gpio_get(buttonJ) && (current_time - last_time > 200))//implementa o debouncing
        {
            temperature = (temp_ideal_min + temp_ideal_max) / 2.0;
            sprintf(temp_str, "%.2f", temperature);
            draw_temperature();
            ssd1306_send_data(&ssd);    //atualiza o display
            sleep_ms(100);
        }

        if(timer_flag)
        {
            countdown();
            timer_flag = false;
        }

        if(button_flag)
        {
            temp_config();
        }
        
       sleep_ms(10);
    }
}
