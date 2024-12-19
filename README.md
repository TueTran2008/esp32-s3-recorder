# Recoding WAV and WAV DES File to MicroSD Card Simultaneously

```
[mic] ---> codec_chip ---> i2s_stream ---> wav_encoder ---> fatfs_stream ---> [sdcard]
                                      |
                                       ---> raw_stream ---> wav_encoder ---> des encrypt ---> fatfs_stream ---> [sdcard]
                                                                                 ▲
                                                                         ┌───────┴────────┐
                                                                         │ DES Encrypt    │
                                                                         │                │
                                                                         └────────────────┘
```

## Environment Setup


