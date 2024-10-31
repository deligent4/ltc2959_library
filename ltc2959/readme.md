## ACR_CALIBARITION_FACTOR
The value of this is calculated by flowing a known current through the sense resistor for known amount of time.
For this test, current of 1-Amp is passed through the sense resistor for 1-minute. The output of this is calculated by function
> LTC2959_Get_Acc_Charge()

The result was 13.55mAh. Again I ran 2-Amp of current for 1-minute, the expected result should be 33.33mAh but it was 27.1mAh,
and i got the same result with 2.5-Amp. The output from the sensor was off by a constant factor of around 1.23.
I used [this](https://mathda.com/convert/electriccharge/milliampere-min-to-milliampere-hour) calculator for the calculation.
Basically **1000mA-minute** is equal to **16.667mA-h** and it is linear.

The reason behind this is not clear, but this can be due to the capacitor value that is connected
between CFP and CFN pins of the sensor. The capacitor is not very accurate, it is some +-10% MLCC with X5R temperature coefficient
which was available to me. I think the sensor expects a very precise 470nF capacitor to do the integration or something related
to keeping the time accurate. So to compensate for this, there is this calibration factor multiplied with the result. 