# Armed failsafe test (props off, battery connected, FC on USB, read-only monitor)
Betaflight 4.5.4, failsafe settings left at defaults (none in `diff all`).

- 14.78 s armed (arm switch on AUX1), motors idle ~1102.
- 26.55 s last live frame; 26.75 s radio link lost -> throttle channel goes to 885.
- 27.99 s FC disarmed, motors 1000, failsafe flag set (flags 0x12) -> ~1.2-1.45 s from last live frame to disarm.
- 28.07 s throttle channel 1000 (failsafe stage value).
- 147.23 s radio switched on again -> throttle channel back to 989; failsafe flag cleared 148.17 s.
- FC stayed disarmed with the arm switch still ON (observed ~2 s after recovery, capture ended at 150 s).
- Caveat: the exact moment the radio was switched off is not logged; times are measured from the FC's side.
