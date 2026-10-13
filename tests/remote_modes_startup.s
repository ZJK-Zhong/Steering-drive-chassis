        AREA RESET, DATA, READONLY
        EXPORT __Vectors
__Vectors
        DCD 0x20010000
        DCD Reset_Handler
        AREA |.text|, CODE, READONLY
        THUMB
        EXPORT Reset_Handler
        IMPORT __main
Reset_Handler PROC
        B __main
        ENDP
        END
