; Somma due numeri e salva il risultato in 0x2002
; Numero 1: 0x05
; Numero 2: 0x03

    ORG 0x1000

    ; Salva i due numeri da sommare in memoria
    LXI H, 0x2000       ; 0x2000 in HL
    MVI M, 0x05         ; Numero 1 (0x05) in 0x2000
    INX H               ; HL++ (0x2001)
    MVI M, 0x03         ; Numero 2 (0x03) in 0x2001

    ; Carica il primo numero in A
    LXI H, 0x2000
    MOV A, M

    ; Aggiungi il secondo numero
    INX H               ; HL = 0x2001
    ADD M               ; Aggiungi 0x2001 (Numero 2) ad A

    ; Memorizza il risultato
    INX H               ; HL = 0x2002
    MOV M, A            ; Salva il risultato

    HLT

END
