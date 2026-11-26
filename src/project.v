/*
 * Supervisor CNC Dual - Tiny Tapeout (GF)
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// ============================================================
//  Wrapper de Tiny Tapeout (modulo top)
// ============================================================
//  Cada supervisor tiene sus PROPIAS lineas STEP.
//
//  ui_in[0] = step_x               uo_out[0] = falla_x       (cnc_v4, 2 s por eje)
//  ui_in[1] = step_y               uo_out[1] = falla_y       (cnc_v4)
//  ui_in[2] = vigilar_x            uo_out[2] = falla_general (cnc_v4)
//  ui_in[3] = vigilar_y            uo_out[3] = salida_segura (cnc_v4)
//  ui_in[4] = step2_x              uo_out[4] = falla_step    (supervisor_step, rapido)
//  ui_in[5] = step2_y
//  ui_in[6] = vigilar_movimiento
//  ui_in[7], uo_out[7:5], uio[7:0] = sin usar
//  reset de ambos = ~rst_n (Tiny Tapeout usa reset activo en bajo)
module tt_um_supervisor_cnc_dual (
    input  wire [7:0] ui_in,    // Entradas dedicadas
    output wire [7:0] uo_out,   // Salidas dedicadas
    input  wire [7:0] uio_in,   // Entradas de los pines bidireccionales
    output wire [7:0] uio_out,  // Salidas de los pines bidireccionales
    output wire [7:0] uio_oe,   // 1 = salida, 0 = entrada
    input  wire       ena,      // siempre 1 cuando el diseno esta alimentado
    input  wire       clk,      // reloj
    input  wire       rst_n     // reset activo en bajo
);

    wire rst = ~rst_n;

    // --- Supervisor 1: lento, por eje (2 s) ---
    wire falla_x, falla_y, falla_general, salida_segura;

    supervisor_cnc_v4 #(
        .PRESCALE(10000),
        .TIMEOUT_TICKS(2000)
    ) u_supervisor_cnc (
        .clk           (clk),
        .rst           (rst),
        .step_x        (ui_in[0]),
        .step_y        (ui_in[1]),
        .vigilar_x     (ui_in[2]),
        .vigilar_y     (ui_in[3]),
        .falla_x       (falla_x),
        .falla_y       (falla_y),
        .falla_general (falla_general),
        .salida_segura (salida_segura)
    );

    // --- Supervisor 2: rapido, X o Y (TIMEOUT ciclos de reloj) ---
    wire falla_step;

    supervisor_step #(
        .TIMEOUT(20)
    ) u_supervisor_step (
        .clk                (clk),
        .reset              (rst),
        .step_x             (ui_in[4]),
        .step_y             (ui_in[5]),
        .vigilar_movimiento (ui_in[6]),
        .falla              (falla_step)
    );

    assign uo_out  = {3'b000, falla_step, salida_segura, falla_general, falla_y, falla_x};
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;     // todos los uio como entradas (sin usar)

    // Evita advertencias por entradas sin usar
    wire _unused = &{ena, ui_in[7], uio_in, 1'b0};

endmodule


// ============================================================
//  Codigo original 1: supervisor_cnc_v4 (sin modificaciones)
// ============================================================

// Supervisor CNC V4 - Silicluster V3 (Verilog-2001)
// Entradas y salidas logicas activas en alto, salvo indicacion de rst.
// CLK = 10 MHz. Divisor compartido: 10,000 ciclos = 1 ms.
// Dos contadores de 2,000 ticks = 2 segundos sin pulsos.
module supervisor_cnc_v4 #(
    parameter integer PRESCALE = 10000,
    parameter integer TIMEOUT_TICKS = 2000
)(
    input  wire clk,
    input  wire rst,
    input  wire step_x,
    input  wire step_y,
    input  wire vigilar_x,
    input  wire vigilar_y,
    output wire falla_x,
    output wire falla_y,
    output wire falla_general,
    output wire salida_segura
);
    // Dimensionados para los valores por defecto de 10 MHz / 2 s.
    // Para la simulacion se modifican solo los parametros, no los anchos.
    reg [13:0] divisor;
    wire tick_ms = (divisor == PRESCALE - 1);

    reg [10:0] tiempo_x;
    reg [10:0] tiempo_y;
    reg fx, fy;

    // Sincronizacion de entradas asincronas al dominio clk.
    reg x_meta, x_sync, x_prev;
    reg y_meta, y_sync, y_prev;
    reg vx_meta, vx_sync;
    reg vy_meta, vy_sync;

    wire pulso_x = x_sync & ~x_prev;
    wire pulso_y = y_sync & ~y_prev;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            divisor <= 14'd0;
            tiempo_x <= 11'd0;
            tiempo_y <= 11'd0;
            fx <= 1'b0;
            fy <= 1'b0;
            x_meta <= 1'b0; x_sync <= 1'b0; x_prev <= 1'b0;
            y_meta <= 1'b0; y_sync <= 1'b0; y_prev <= 1'b0;
            vx_meta <= 1'b0; vx_sync <= 1'b0;
            vy_meta <= 1'b0; vy_sync <= 1'b0;
        end else begin
            x_meta <= step_x;  x_sync <= x_meta;  x_prev <= x_sync;
            y_meta <= step_y;  y_sync <= y_meta;  y_prev <= y_sync;
            vx_meta <= vigilar_x; vx_sync <= vx_meta;
            vy_meta <= vigilar_y; vy_sync <= vy_meta;

            if (tick_ms)
                divisor <= 14'd0;
            else
                divisor <= divisor + 1'b1;

            // Las fallas son retenidas hasta reset.
            if (!fx) begin
                if (!vx_sync || pulso_x)
                    tiempo_x <= 11'd0;
                else if (tick_ms) begin
                    if (tiempo_x == TIMEOUT_TICKS - 1)
                        fx <= 1'b1;
                    else
                        tiempo_x <= tiempo_x + 1'b1;
                end
            end

            if (!fy) begin
                if (!vy_sync || pulso_y)
                    tiempo_y <= 11'd0;
                else if (tick_ms) begin
                    if (tiempo_y == TIMEOUT_TICKS - 1)
                        fy <= 1'b1;
                    else
                        tiempo_y <= tiempo_y + 1'b1;
                end
            end
        end
    end

    assign falla_x = fx;
    assign falla_y = fy;
    assign falla_general = fx | fy;
    // 1 = sin falla detectada; 0 = alarma. SOLO INDICACION LOGICA.
    assign salida_segura = ~(fx | fy);
endmodule


// ============================================================
//  Codigo original 2: supervisor_step (sin modificaciones)
// ============================================================

// =============================================================
// BLOQUE 1. INTERFAZ EXTERNA Y PARAMETRO DE CONFIGURACION
// =============================================================
module supervisor_step #(
    parameter integer TIMEOUT = 20 // Limite de ciclos sin eventos STEP.
)(
    input wire clk,                 // Reloj: sincroniza el circuito.
    input wire reset,               // Reinicio activo en 1.
    input wire step_x,              // Ordenes STEP del eje X.
    input wire step_y,              // Ordenes STEP del eje Y.
    input wire vigilar_movimiento,  // 1: se espera movimiento; 0: pausa normal.
    output reg falla                // Salida de falla retenida hasta reset.
);
 
// BLOQUE 2. MEMORIAS INTERNAS
reg [4:0] contador;               // Cinco bits: cuenta de 0 a 31.
reg x_meta, x_sync, x_anterior;   // Captura, sincroniza y memoriza X.
reg y_meta, y_sync, y_anterior;   // Captura, sincroniza y memoriza Y.
 
// BLOQUE 3. SINCRONIZACION DE LOS DOS EJES
// Se ejecuta en cada flanco ascendente de clk o de reset.
always @(posedge clk or posedge reset) begin
    if (reset) begin                 // Con reset activo, poner registros en 0.
        x_meta     <= 1'b0;         // Limpia primera etapa X.
        x_sync     <= 1'b0;         // Limpia segunda etapa X.
        x_anterior <= 1'b0;         // Limpia muestra previa X.
        y_meta     <= 1'b0;         // Limpia primera etapa Y.
        y_sync     <= 1'b0;         // Limpia segunda etapa Y.
        y_anterior <= 1'b0;         // Limpia muestra previa Y.
    end else begin                  // Si no hay reset, capturar entradas.
        x_meta     <= step_x;       // Primera muestra de STEP X.
        x_sync     <= x_meta;       // Segunda muestra, retrasada un ciclo.
        x_anterior <= x_sync;       // Copia del estado sincronizado anterior.
        y_meta     <= step_y;       // Primera muestra de STEP Y.
        y_sync     <= y_meta;       // Segunda muestra, retrasada un ciclo.
        y_anterior <= y_sync;       // Copia del estado sincronizado anterior.
    end                             // Cierra if/else.
end                                 // Cierra bloque always.
 
// BLOQUE 4. DETECCION DE FLANCOS Y COMBINACION
// && = Y logico; ! = negacion. Detecta el cambio 0 -> 1.
wire pulso_x = x_sync && !x_anterior; // Flanco ascendente en X.
wire pulso_y = y_sync && !y_anterior; // Flanco ascendente en Y.
wire pulso_step = pulso_x || pulso_y; // || = O; basta uno de los ejes.
 
// BLOQUE 5. TEMPORIZADOR Y FALLA ENCLAVADA
// Las condiciones else if se revisan de arriba hacia abajo.
always @(posedge clk or posedge reset) begin
    if (reset) begin                 // Prioridad 1: reiniciar todo.
        contador <= 5'd0;           // Cero decimal expresado en 5 bits.
        falla    <= 1'b0;           // Apagar indicacion de falla.
    end else if (falla) begin       // Prioridad 2: falla ya registrada.
        falla <= 1'b1;              // Mantener falla activa hasta reset.
    end else if (!vigilar_movimiento) begin // Prioridad 3: no vigilar.
        contador <= 5'd0;           // No contar durante una pausa normal.
    end else if (pulso_step) begin  // Prioridad 4: se detecta actividad.
        contador <= 5'd0;           // Reiniciar tiempo de espera.
    end else if (contador >= TIMEOUT - 1) begin // Prioridad 5: limite.
        falla <= 1'b1;              // Activar y retener senal de falla.
    end else begin                  // Si nada anterior se cumple:
        contador <= contador + 1'b1;// Sumar un ciclo a la cuenta.
    end                             // Cierre de la cadena de condiciones.
end                                 // Cierre del temporizador.
endmodule                           // Fin del circuito supervisor_step.
