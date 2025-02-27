# Embarcatech_Final

Repositório criado para a tarefa relacionada à simulação de um sistema de monitoramento de temperatura para o transporte de frutas utilizando a plataforma Raspberry Pi Pico W.

O programa implementa um sistema de simulação de temperatura, onde um joystick é utilizado para representar a variação de temperatura. O valor de temperatura é exibido em um display de LEDs SSD1306. Um LED RGB indica visualmente o status da temperatura, enquanto um buzzer fornece alertas sonoros.

Além disso, uma matriz de LEDs WS2812 simula a ativação de um exaustor quando a temperatura ultrapassa um limite configurado pelo usuário. A configuração dos limites de temperatura máximo e mínimo é realizada via comunicação UART.

# Instruções de compilação

Para compilar o código, são necessárias as seguintes extensões:

* Wokwi Simulator
* Raspberry Pi Pico
* CMake

Após instalá-las, basta buildar o projeto pelo CMake. Em seguida, abra o arquivo diagram.json e clique no botão verde para iniciar a simulação.

# Instruções de utilização

Durante a simulação, o usuário pode interagir com o sistema utilizando o joystick e a interface UART.

- Movimentar o joystick altera a temperatura simulada.
- O display SSD1306 exibe a temperatura em tempo real.
- O LED RGB muda de cor conforme a temperatura:
  - Verde: temperatura dentro do limite.
  - Amarelo: temperatura levemente fora do limite.
  - Vermelho: temperatura muito longe do limite.
- O buzzer emite alertas sonoros quando a temperatura ultrapassa o limite.
- A matriz de LEDs WS2812 acende a cada 30 segundos através de interrupção.
- A interface UART permite configurar os valores de temperatura mínima e máxima.

# Vídeo demonstrativo
https://youtu.be/gW9G9rDFuIQ
