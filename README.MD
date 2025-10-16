// GPIO 10 e 4 ficam em HIGH fixo (velocidade máxima)
// OU em LOW (motor parado)
gpio_set_level(GPIO_MOTOR_FRONT_PWM, 1);  // GPIO 10 = HIGH
gpio_set_level(GPIO_MOTOR_REAR_PWM, 1);   // GPIO 4 = HIGH
```

---

## **Na L298N:**
```
     L298N
    ┌──────┐
ENA │ 10   │ ← GPIO 10 (PWM Front)
IN1 │ 7    │ ← GPIO 7  (Direção)
IN2 │ 8    │ ← GPIO 8  (Direção)
    │      │
IN3 │ 18   │ ← GPIO 18 (Direção)
IN4 │ 19   │ ← GPIO 19 (Direção)
ENB │ 4    │ ← GPIO 4  (PWM Rear)
    └──────┘



## **Mapeamento Físico:**
```
        [FRONT]              ← 1 motor (GPIO 7/8/10)
       /       \
      /         \
    [REAR]   [REAR]          ← 2 motores em paralelo (GPIO 18/19/4)
```

**FRONT** = Canal A da L298N (IN1, IN2, ENA)  
**REAR** = Canal B da L298N (IN3, IN4, ENB) → controla os 2 motores traseiros juntos

---

## **Sequência de Teste Recomendada:**

1. **Teste básico dianteira:**
```
   MOTOR FRONT FORWARD 3
```
   Observe: A roda da frente deve girar devagar

2. **Para:**
```
   MOTOR FRONT STOP
```

3. **Teste básico traseiras:**
```
   MOTOR REAR FORWARD 3
```
   Observe: As 2 rodas traseiras devem girar juntas, mesma velocidade

4. **Para:**
```
   MOTOR REAR STOP
```

5. **Teste completo - andar para frente:**
```
   MOTOR FRONT FORWARD 7
   MOTOR REAR FORWARD 7
```
   O carrinho deve andar reto!

6. **Para tudo:**
```
   MOTOR FRONT STOP
   MOTOR REAR STOP
```

---

## **Dica Importante:**

Como você mencionou que **"todos os 3 motores precisam ter a mesma velocidade"**, sempre use o **mesmo valor** de velocidade nos comandos:
```
MOTOR FRONT FORWARD 8
MOTOR REAR FORWARD 8


-90          // Servo vira 90° para ESQUERDA
-45          // Servo vira 45° para ESQUERDA
0            // Servo centralizado (RETO)
45           // Servo vira 45° para DIREITA
90           // Servo vira 90° para DIREITA
```
**Como usar:** Digite o ângulo e pressione Enter (ou espere 500ms)  
**Range:** -90 a +90 graus

---

### **Modo SWEEP (Varredura Automática)**
```
SWEEP START 5          // Inicia varredura automática, velocidade 5
SWEEP START 1          // Varredura LENTA (1 = mais lento)
SWEEP START 10         // Varredura RÁPIDA (10 = mais rápido)
SWEEP STOP             // Para a varredura e retorna ao centro
```
**Como usar:**  
- Servo vai varrer de -90° a +90° automaticamente
- Velocidade controla a suavidade do movimento
- Use para testar o servo ou fazer scanning

---

## 🚗 **CONTROLE INDIVIDUAL DOS MOTORES**

### **Motor DIANTEIRO (FRONT) - 1 motor**
```
MOTOR FRONT FORWARD 5     // Dianteiro para FRENTE, velocidade 5
MOTOR FRONT FORWARD 10    // Dianteiro para FRENTE, velocidade MÁXIMA
MOTOR FRONT BACKWARD 3    // Dianteiro para TRÁS, velocidade 3
MOTOR FRONT STOP          // PARA o motor dianteiro
```

### **Motores TRASEIROS (REAR) - 2 motores em paralelo**
```
MOTOR REAR FORWARD 5      // Traseiros para FRENTE, velocidade 5
MOTOR REAR FORWARD 10     // Traseiros para FRENTE, velocidade MÁXIMA
MOTOR REAR BACKWARD 3     // Traseiros para TRÁS, velocidade 3
MOTOR REAR STOP           // PARA os motores traseiros
```

**Velocidade:** 1 (mais lento) a 10 (mais rápido)

---

## 🔧 **CALIBRAÇÃO (Encontrar valores ideais)**

### **Ver Tabela de Conversão PWM**
```
MOTOR FRONT CALIB         // Mostra tabela Speed → PWM do motor dianteiro
MOTOR REAR CALIB          // Mostra tabela Speed → PWM dos motores traseiros
```

**Saída Exemplo:**
```
======================================
  CALIBRAÇÃO MOTOR FRONT
======================================
PWM_MIN: 70
PWM_MAX: 255
--------------------------------------
Speed | PWM  | Percentual
--------------------------------------
   1  |  70  |    27%
   2  |  91  |    36%
   3  | 111  |    44%
   ...
  10  | 255  |   100%
======================================
```

**Como usar:**  
1. Execute `MOTOR FRONT CALIB` e `MOTOR REAR CALIB`
2. Veja os valores PWM para cada speed
3. Ajuste `MOTOR_PWM_MIN_FRONT` e `MOTOR_PWM_MIN_REAR` no código se necessário

---

## ⚡ **SYNC - COMANDO SINCRONIZADO (RECOMENDADO)**

### **Controle Simplificado - Todos os Motores Juntos**
```
SYNC FORWARD 5            // Todos os motores para FRENTE, velocidade 5
SYNC FORWARD 10           // Todos os motores para FRENTE, MÁXIMA velocidade
SYNC BACKWARD 3           // Todos os motores para TRÁS, velocidade 3
SYNC STOP                 // PARA TODOS os motores
```

**Como funciona:**  
- Você digita apenas **1 velocidade**
- O código automaticamente aplica **PWM diferente** para FRONT e REAR
- Resultado: **Velocidade REAL sincronizada!** 🎯

**Exemplo Interno:**
```
SYNC FORWARD 5
  ↓
Motor FRONT recebe PWM=152 (baseado em calibração)
Motor REAR recebe PWM=172 (baseado em calibração)
  ↓
Carrinho anda RETO! ✅
```

---

## 📊 **EXEMPLOS PRÁTICOS**

### **Cenário 1: Teste Inicial**
```
MOTOR FRONT CALIB         // Ver configuração atual
MOTOR REAR CALIB

MOTOR FRONT FORWARD 1     // Testar velocidade mínima
MOTOR FRONT STOP

MOTOR REAR FORWARD 1      // Testar velocidade mínima
MOTOR REAR STOP
```

### **Cenário 2: Andar Reto (Modo Simples)**
```
SYNC FORWARD 7            // Liga tudo sincronizado
0                         // Servo reto
SYNC STOP                 // Para
```

### **Cenário 3: Curva para Direita**
```
SYNC FORWARD 5            // Motores em velocidade média
45                        // Servo vira 45° direita
SYNC STOP                 // Para
0                         // Servo volta ao centro
```

### **Cenário 4: Manobra Ré**
```
SYNC BACKWARD 4           // Ré em velocidade baixa
-30                       // Servo vira 30° esquerda
SYNC STOP                 // Para
```

### **Cenário 5: Teste de Varredura com Movimento**
```
SWEEP START 7             // Servo fazendo sweep
SYNC FORWARD 3            // Motores em velocidade baixa
SWEEP STOP                // Para sweep
SYNC STOP                 // Para motores
