# Recoding WAV and WAV DES File to MicroSD Card Simultaneously
This is my target topology. But it currently doesn't work -> Change to another wav, will comeback with this in the feature
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

```
[mic] ---> codec_chip ---> i2s_stream ---> wav_encoder ---> fatfs_stream ---> [sdcard] ---> Read wav file -> des encrypt -> [sdcard]
                                      |

```
## Environment Setup


